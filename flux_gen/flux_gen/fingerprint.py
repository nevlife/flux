"""Schema fingerprint: a stable 64-bit digest of the top-level type name and the flat layout.

Computed once at generation time and emitted as a literal into both adapters, and part of the
segment name, so two ends that disagree never meet. Field names and nested type names are
absent on purpose: restructuring the nesting moves no byte, so it must not move the fingerprint
(docs/en/enumerations_msgs.en.md, docs/en/message_shapes.en.md).
"""

import hashlib

from .flatten import LeafKind

# 2: jagged leaves carry their element structure (and fixed count), so jagged schemas that
#    previously all collided on "jagged:-:1" fingerprint distinctly.
# 3: every leaf whose length is per-frame now tokenizes count as 0 (flatten.DYNAMIC_COUNT).
#    A dynamic jagged leaf used to keep the default 1 and so collided with the fixed T[1] form.
# 4: the top-level type name joins the digest. Layout alone let two different types with the
#    same shape share a segment, a pairing ROS 2 refuses outright.
# 5: HEADER and STAMP tokenize as the fields they lay out as. A top-level Header collapses to
#    one leaf and a nested one flattens to its members (message_shapes.md 6), but both occupy
#    the same bytes, so an opaque token made two identical layouts fingerprint differently --
#    exactly the drift this digest exists to catch.
FINGERPRINT_SCHEMA_VERSION = 5

# What HEADER and STAMP place, in the order layout places it. Tokenizing from the expanded form
# is what keeps "same wire bytes, same fingerprint" true across the top-level/nested split.
_STAMP_TOKENS = "fixed:i32:1;fixed:u32:1"
_HEADER_TOKENS = _STAMP_TOKENS + ";string:-:1"


def _leaf_token(leaf):
    if leaf.kind == LeafKind.HEADER:
        return _HEADER_TOKENS
    if leaf.kind == LeafKind.STAMP:
        return _STAMP_TOKENS
    dt = leaf.dtype.value if leaf.dtype is not None else "-"
    rec = ""
    if leaf.record_leaves:
        rec = "{" + ",".join(_leaf_token(r) for r in leaf.record_leaves) + "}"
    return f"{leaf.kind.value}:{dt}:{leaf.count}{rec}"


def canonical(leaves, type_name=None):
    """`type_name` is the outermost `pkg/Msg`. Omitting it digests the layout alone, for a
    caller carrying raw arrays rather than a .msg, which has no type to name."""
    head = f"v{FINGERPRINT_SCHEMA_VERSION}"
    if type_name is not None:
        head += "#" + type_name  # '#' cannot occur in a ROS type name or in a leaf token
    return ";".join([head] + [_leaf_token(l) for l in leaves])


def fingerprint(leaves, type_name=None):
    digest = hashlib.blake2b(
        canonical(leaves, type_name).encode("utf-8"), digest_size=8).digest()
    return int.from_bytes(digest, "little")
