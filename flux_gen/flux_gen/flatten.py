"""Flatten a parsed message into an ordered leaf layout and classify its tier.

The wire is one contiguous, pointer-free blob, so nesting is resolved at generation time:
fixed nesting is flattened into a scalar block, dynamic primitive/record fields become
columns, strings become a variable tail, and a dynamic array whose element itself varies in
length is jagged. A jagged array rides the same array-of-element-blocks layout a record column
does, with descriptors inside the element block (layout.py). Anything that cannot ride this
flat layout is rejected -- the caller falls back to plain ROS.
"""

from dataclasses import dataclass, field, replace
from enum import Enum

from .dtypes import ROS_PRIMITIVE, DType
from .model import ArrayKind
from .parse import resolve


# std_msgs/Header and builtin_interfaces/Time are frame metadata, not payload. A top-level
# Header (stamp + frame_id) or bare Time maps to the frame's stamp/frame_id and is not
# flattened as a struct. Nested deeper they are ordinary structs and flatten like any other --
# the frame carries one stamp, the top level's.
HEADER_TYPES = frozenset({"std_msgs/Header", "Header"})
TIME_TYPES = frozenset({"builtin_interfaces/Time", "Time"})

# A fixed array of a fixed struct is expanded leaf by leaf, so nesting multiplies: A[64] holding
# B[64] holding float64[64] is 262144 leaves. Each individual size passes the parser's per-field
# bound, so only a total cap stops the blowup. The largest message shipped with ROS jazzy
# flattens to 107 leaves, so this bound is far above any real schema; hitting it is a reject
# (fall back to plain ROS), not a crash.
MAX_LEAVES = 4096

# Leaf.count is a multiplicity, and this is the one value that means "per-frame length", for
# every leaf kind that can have one. Leaving a dynamic leaf at the default 1 makes T[1] and T[]
# fingerprint identically, so the two ends agree on a layout that differs in how it is read.
DYNAMIC_COUNT = 0


class LeafKind(Enum):
    FIXED = "fixed"          # compile-time offset scalar block entry (dtype x count)
    COLUMN = "column"        # per-frame dynamic primitive array (0-copy)
    RECORD_COLUMN = "record"  # per-frame dynamic array of a fixed struct (0-copy)
    STRING = "string"        # single string (variable tail)
    STRING_ARRAY = "sarray"  # array of strings (variable tail)
    JAGGED = "jagged"        # dynamic array of a variable-length element (bulk + structure)
    HEADER = "header"        # std_msgs/Header -> frame metadata (stamp + frame_id)
    STAMP = "stamp"          # bare builtin_interfaces/Time -> frame stamp


# Which side of the DYNAMIC_COUNT rule each kind is pinned to. STRING_ARRAY and JAGGED are on
# neither list: they exist in both a fixed T[N] and a dynamic T[] form.
ALWAYS_DYNAMIC = frozenset({LeafKind.COLUMN, LeafKind.RECORD_COLUMN})
ALWAYS_FIXED = frozenset({LeafKind.FIXED, LeafKind.STRING, LeafKind.HEADER, LeafKind.STAMP})


@dataclass(frozen=True)
class Leaf:
    kind: LeafKind
    dtype: DType = None  # set for FIXED/COLUMN; None for record/string/jagged
    count: int = 1  # multiplicity: N for a fixed T[N], DYNAMIC_COUNT when the length is per-frame
    record_leaves: tuple = ()  # flattened element layout for RECORD_COLUMN and JAGGED
    # Source path: ("pose", "position", "x"), or ("poses", 0, "x") through a fixed array. The
    # flat accessor name joins it with '_'; the ROS adapter walks it to reach m.pose.position.x.
    # The fingerprint hashes neither -- a rename does not move a byte.
    path: tuple = ()
    elem_type: str = None  # RECORD_COLUMN/JAGGED: the element's `pkg/Msg`, for the ROS adapter

    @property
    def name(self):
        return "_".join(str(part) for part in self.path)


class Reject(Exception):
    pass


@dataclass
class FlatResult:
    name: str
    leaves: list = field(default_factory=list)
    rejected: str = None  # reason string when the message cannot ride the fast path
    package: str = None  # owning ROS package; with `name` it identifies the schema
    constants: tuple = ()  # named symbols to re-emit; not wire data, not in the fingerprint

    @property
    def type_name(self):
        """`pkg/Msg` -- what the fingerprint identifies this schema by, beyond its layout."""
        return f"{self.package}/{self.name}" if self.package else self.name

    @property
    def tier(self):
        if self.rejected:
            return "reject"
        if any(l.kind == LeafKind.JAGGED for l in self.leaves):
            return "jagged"
        if any(l.kind != LeafKind.FIXED for l in self.leaves):
            return "columnar"
        return "fixed"


def _all_fixed(leaves):
    return all(l.kind == LeafKind.FIXED for l in leaves)


def _reparent(leaves, prefix):
    return [replace(l, path=prefix + l.path) for l in leaves]


def _expand(fld, registry, seen, package=None, prefix=()):
    """Expand one field into leaves. A single nested struct is hoisted -- its members flatten
    up (arrays -> columns, strings -> tail, fixed -> scalar block), since the wire is flat and
    nesting is only a name prefix. Raises Reject only for shapes that cannot ride the flat wire
    at all (bounded array, bool[], wstring, unknown or recursive type)."""
    base, kind, path = fld.type_name, fld.kind, prefix + (fld.name,)
    if kind == ArrayKind.BOUNDED:
        raise Reject(f"{fld.name}: bounded array [<=N] not supported")

    if base in ROS_PRIMITIVE:
        dt = ROS_PRIMITIVE[base]
        if kind == ArrayKind.SCALAR:
            return [Leaf(LeafKind.FIXED, dt, 1, path=path)]
        if kind == ArrayKind.FIXED:
            return [Leaf(LeafKind.FIXED, dt, fld.size, path=path)]
        if dt == DType.BOOL:
            raise Reject(f"{fld.name}: bool[] not supported")
        return [Leaf(LeafKind.COLUMN, dt, DYNAMIC_COUNT, path=path)]

    if base == "string":
        if kind == ArrayKind.SCALAR:
            return [Leaf(LeafKind.STRING, path=path)]
        return [Leaf(
            LeafKind.STRING_ARRAY,
            count=fld.size if kind == ArrayKind.FIXED else DYNAMIC_COUNT, path=path)]
    if base == "wstring":
        raise Reject(f"{fld.name}: wstring not supported")

    msg = resolve(registry, base, package)
    if msg is None:
        raise Reject(f"{fld.name}: unknown type '{base}'")
    # Key recursion on the resolved message, not on the basename: two packages can define the
    # same name without either containing the other.
    ident = f"{msg.package}/{msg.name}" if msg.package else msg.name
    if ident in seen:
        raise Reject(f"{fld.name}: recursive nested type '{ident}'")
    sub = _flatten(msg, registry, seen | {ident})
    if not sub:
        # A nested type with no fields has no element block, so an array of it has nothing to
        # index. Only the top-level form of this is worth a distinct message.
        raise Reject(f"{fld.name}: nested type '{ident}' has no publishable fields")

    if kind == ArrayKind.SCALAR:
        # hoist: the nested struct's members flatten into this message under a path prefix
        return _reparent(sub, path)
    if kind == ArrayKind.FIXED:
        if _all_fixed(sub):
            # Bound before materializing: the product is what blows up, not the operands.
            if len(sub) * fld.size > MAX_LEAVES:
                raise Reject(
                    f"{fld.name}: fixed nesting expands to {len(sub) * fld.size} leaves "
                    f"(limit {MAX_LEAVES})"
                )
            return [x for i in range(fld.size) for x in _reparent(sub, path + (i,))]
        # Carry the element's flattened layout (and the fixed count N) so the fingerprint
        # distinguishes jagged schemas by their element structure, not just "jagged".
        return [Leaf(
            LeafKind.JAGGED, count=fld.size, record_leaves=tuple(sub), path=path,
            elem_type=ident)]
    # DYNAMIC array of a struct: 0-copy record column iff the element is purely fixed
    # (rectangular), otherwise the element varies in length -> jagged.
    if _all_fixed(sub):
        return [Leaf(
            LeafKind.RECORD_COLUMN, count=DYNAMIC_COUNT, record_leaves=tuple(sub), path=path,
            elem_type=ident)]
    return [Leaf(
        LeafKind.JAGGED, count=DYNAMIC_COUNT, record_leaves=tuple(sub), path=path,
        elem_type=ident)]


def _flatten(msg, registry, seen):
    out = []
    for fld in msg.fields:
        out.extend(_expand(fld, registry, seen, msg.package))
        if len(out) > MAX_LEAVES:
            raise Reject(f"{msg.name}: flattens to more than {MAX_LEAVES} leaves")
    return out


def flatten_message(msg, registry):
    try:
        leaves = []
        for fld in msg.fields:
            if fld.kind == ArrayKind.SCALAR and fld.type_name in HEADER_TYPES:
                leaves.append(Leaf(LeafKind.HEADER, path=(fld.name,)))
                continue
            if fld.kind == ArrayKind.SCALAR and fld.type_name in TIME_TYPES:
                leaves.append(Leaf(LeafKind.STAMP, path=(fld.name,)))
                continue
            leaves.extend(_expand(fld, registry, frozenset(), msg.package))
            if len(leaves) > MAX_LEAVES:
                raise Reject(f"{msg.name}: flattens to more than {MAX_LEAVES} leaves")
        if not leaves:
            # Constant-only messages (std_msgs/Empty, rcl_interfaces/ParameterType) carry no
            # payload, so there is nothing for a frame to be. Plain ROS handles them.
            raise Reject(f"{msg.name}: no publishable fields")
        return FlatResult(
            msg.name, leaves, package=msg.package, constants=tuple(msg.constants))
    except Reject as exc:
        return FlatResult(
            msg.name, [], rejected=str(exc), package=msg.package,
            constants=tuple(msg.constants))
