"""Schema fingerprint: a stable 64-bit digest of a schema's identity and its byte layout.

Computed once at generation time and emitted as a literal into both the C++ and Python
adapters, so the two ends agree by construction. flux puts this number in the segment name, so
two ends that disagree open different files and never meet -- there is no post-attach check to
get wrong.

It covers two things, which is what ROS 2 needs two mechanisms for:

  the top-level type name   `geometry_msgs/Point` and `geometry_msgs/Vector3` are both
                            three float64 and would otherwise share a segment, letting one end
                            read a position as a direction with nothing to complain. DDS refuses
                            that pair on the type name; this refuses it on the fingerprint.
  the flattened layout      a field added or a dtype changed moves the fingerprint, which is what
                            the RIHS type hash catches for ROS: same name, skewed definition.

Field names and NESTED type names are deliberately absent. Nesting is a source-level convenience
over the same flat bytes -- pulling fields into a nested struct does not change what is on the
wire, so it must not change the fingerprint. Only the outermost name identifies the schema.
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
    """`type_name` is the outermost `pkg/Msg`. Omitting it digests the layout alone, which is
    what a caller carrying raw arrays rather than a .msg has to do -- it has no type to name."""
    head = f"v{FINGERPRINT_SCHEMA_VERSION}"
    if type_name is not None:
        head += "#" + type_name  # '#' cannot occur in a ROS type name or in a leaf token
    return ";".join([head] + [_leaf_token(l) for l in leaves])


def fingerprint(leaves, type_name=None):
    digest = hashlib.blake2b(
        canonical(leaves, type_name).encode("utf-8"), digest_size=8).digest()
    return int.from_bytes(digest, "little")
