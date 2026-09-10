"""Assign byte offsets: a leaf list becomes a concrete slot layout both languages agree on.

A frame is one pointer-free blob, so every variable part is reached by an offset from the blob's
own start, never by an address. The blob is two regions:

    [ scalar block: compile-time offsets ][ var region: bump-allocated per frame ]

The scalar block holds fixed values inline and, for every variable leaf, an 8-byte descriptor
`{uint32 off, uint32 len}` -- `off` measured from the blob start. Reading a column is: load the
descriptor, view `len` elements at `off`. Writing is: bump the cursor, store the descriptor.

Record and jagged arrays are the same mechanism one level down. Both are an array of fixed-stride
element blocks in the var region (the array-of-structs layout `message_shapes.md` gives a record
column); the only difference is that a jagged element's block itself holds descriptors pointing
further out. That makes nesting fall out of the same three rules instead of needing a separate
bulk-plus-offsets encoding per depth.

Offsets are uint32, so one frame of a generated schema is capped at 4 GiB regardless of slot_size.
"""

from dataclasses import dataclass

from .dtypes import SIZE, DType
from .fingerprint import fingerprint
from .flatten import DYNAMIC_COUNT, LeafKind

DESC_SIZE = 8   # {uint32 off, uint32 len}
DESC_ALIGN = 4
STAMP_SIZE = 8  # {int32 sec, uint32 nanosec}
MAX_ALIGN = 8

# Kinds whose scalar-block entry is a descriptor into the var region.
VARIABLE = frozenset({
    LeafKind.COLUMN, LeafKind.RECORD_COLUMN, LeafKind.JAGGED,
    LeafKind.STRING, LeafKind.STRING_ARRAY,
})


class LayoutError(Exception):
    pass


def _camel(name):
    return "".join(part[:1].upper() + part[1:] for part in name.split("_") if part)


def elem_class_name(path):
    """Class name both emitters give the element adapter of `path` (a tuple of Placed)."""
    return "".join(_camel(p.name) for p in path) + "Elem"


def _align_up(n, a):
    return (n + a - 1) // a * a


@dataclass(frozen=True)
class Placed:
    """One leaf, given an offset inside its enclosing block."""

    leaf: object
    offset: int
    elem: object = None  # Block for RECORD_COLUMN / JAGGED elements

    @property
    def kind(self):
        return self.leaf.kind

    @property
    def name(self):
        return self.leaf.name

    @property
    def path(self):
        return self.leaf.path

    @property
    def elem_type(self):
        return self.leaf.elem_type

    @property
    def dtype(self):
        return self.leaf.dtype

    @property
    def count(self):
        return self.leaf.count

    @property
    def is_dynamic(self):
        """True when the length rides in the frame rather than in the schema."""
        return self.leaf.count == DYNAMIC_COUNT

    @property
    def frame_id_offset(self):
        """Offset of a HEADER's frame_id descriptor; its stamp occupies the first 8 bytes."""
        return self.offset + STAMP_SIZE


@dataclass(frozen=True)
class Block:
    """A fixed-size run of placed leaves: a message's scalar block, or one element of an array."""

    placed: tuple
    stride: int  # size rounded up to align, so an array of these is addressable by index
    align: int

    @property
    def has_var(self):
        return any(p.kind in VARIABLE or p.kind == LeafKind.HEADER for p in self.placed)


@dataclass(frozen=True)
class Layout:
    name: str
    package: str
    tier: str
    root: Block = None
    fingerprint: int = 0
    rejected: str = None
    constants: tuple = ()

    @property
    def type_name(self):
        return f"{self.package}/{self.name}" if self.package else self.name

    @property
    def scalar_bytes(self):
        return self.root.stride if self.root else 0


def _entry_shape(leaf):
    """(align, size) this leaf occupies in its enclosing block."""
    if leaf.kind == LeafKind.FIXED:
        sz = SIZE[leaf.dtype]
        return sz, sz * leaf.count
    if leaf.kind == LeafKind.STAMP:
        return 4, STAMP_SIZE
    if leaf.kind == LeafKind.HEADER:
        return 4, STAMP_SIZE + DESC_SIZE  # stamp inline, frame_id as a descriptor
    return DESC_ALIGN, DESC_SIZE


def build_block(leaves):
    """Pack leaves in declaration order at natural alignment, C-struct style."""
    off, align, out = 0, 1, []
    for leaf in leaves:
        a, size = _entry_shape(leaf)
        off = _align_up(off, a)
        elem = build_block(leaf.record_leaves) if leaf.record_leaves else None
        out.append(Placed(leaf, off, elem))
        off += size
        align = max(align, a)
    return Block(tuple(out), _align_up(off, align), align)


def _check_names(block, path="", root=True):
    seen = set()
    for p in block.placed:
        if not p.name:
            raise LayoutError(f"{path}<unnamed>: leaf has no source name")
        if p.name in seen:
            raise LayoutError(
                f"{path}{p.name}: two fields flatten to the same name -- nesting is joined "
                f"with '_', so 'a_b' and 'a.b' collide")
        seen.add(p.name)
        if p.elem is not None:
            _check_names(p.elem, f"{path}{p.name}[].", root=False)


def _check_elem_classes(block):
    """Element class names camel-case their field path, which erases the '_' vs '.' boundary;
    two distinct paths landing on one class silently alias in Python and fail to compile in C++.
    """
    seen = {}

    def rec(b, prefix):
        for p in b.placed:
            if p.elem is None:
                continue
            path = prefix + (p,)
            cls = elem_class_name(path)
            dotted = ".".join(x.name for x in path)
            if cls in seen:
                raise LayoutError(
                    f"element class name collision: field paths '{seen[cls]}' and '{dotted}' "
                    f"both generate {cls}; rename one of the fields to disambiguate")
            seen[cls] = dotted
            rec(p.elem, path)

    rec(block, ())


def build_layout(flat):
    """FlatResult -> Layout. A rejected schema carries its reason through unchanged."""
    if flat.rejected:
        return Layout(flat.name, flat.package, "reject", rejected=flat.rejected,
                      constants=flat.constants)
    root = build_block(flat.leaves)
    _check_names(root)
    _check_elem_classes(root)
    if root.stride == 0:
        raise LayoutError(f"{flat.name}: empty schema has no bytes to publish")
    return Layout(
        flat.name, flat.package, flat.tier, root,
        fingerprint(flat.leaves, flat.type_name), constants=flat.constants)


def walk(block):
    """Yield every (path, Placed) in the block, descending into element blocks."""
    def rec(b, path):
        for p in b.placed:
            yield path + (p,)
            if p.elem is not None:
                yield from rec(p.elem, path + (p,))
    yield from rec(block, ())


def dtype_of(placed):
    return placed.dtype if placed.dtype is not None else DType.U8
