"""Layout tests: offsets, alignment, and the golden table that pins them.

The relational assertions here (aligned, packed in order, stride a multiple of align) hold for
any layout that satisfies them, so on their own they let offsets drift silently: reorder the
descriptor and the scalar fields and every offset moves while the suite stays green. Both ends
are generated from one tree, so nothing downstream notices either -- only a peer built from an
older tree does, at runtime, reading the wrong bytes out of a frame whose fingerprint still
matches. The golden table pins the actual offsets so that drift has to be an explicit edit.
"""

import os

import pytest

from flux_gen import LayoutError, build_layout, flatten_message, load_dir, walk
from flux_gen.dtypes import SIZE
from flux_gen.flatten import LeafKind
from flux_gen.layout import DESC_SIZE, MAX_ALIGN, STAMP_SIZE, VARIABLE
from flux_gen.model import ArrayKind, Field, Message

MSG_DIR = os.path.join(os.path.dirname(__file__), "msg")

# (message, scalar_bytes, [(name, offset)]) for the root block. Regenerate deliberately: these
# offsets are the wire contract between the C++ and Python adapters.
GOLDEN = [
    ("Vec3", 24, [("x", 0), ("y", 8), ("z", 16)]),
    ("Pose", 48, [("position_x", 0), ("orientation_z", 40)]),
    ("PointCloud", 32, [("x", 0), ("y", 8), ("z", 16), ("width", 24), ("height", 28)]),
    ("Labeled", 16, [("label", 0), ("values", 8)]),
    ("Trajectory", 8, [("poses", 0)]),
    ("TensorList", 8, [("tensors", 0)]),
    ("Tensor", 16, [("shape", 0), ("data", 8)]),
    ("Event", 24, [("stamp", 0), ("tags", 8), ("values", 16)]),
    ("Names", 16, [("fixed_tags", 0), ("dyn_tags", 8)]),
]


@pytest.fixture(scope="module")
def reg():
    return load_dir(MSG_DIR)


def lay(reg, name):
    return build_layout(flatten_message(reg[name], reg))


@pytest.mark.parametrize("name,scalar_bytes,offsets", GOLDEN, ids=[g[0] for g in GOLDEN])
def test_golden_offsets(reg, name, scalar_bytes, offsets):
    layout = lay(reg, name)
    assert layout.scalar_bytes == scalar_bytes
    placed = {p.name: p.offset for p in layout.root.placed}
    for field, off in offsets:
        assert placed[field] == off


def test_every_offset_is_naturally_aligned(reg):
    # An unaligned column would be undefined behaviour to read as T* on the C++ side, and the
    # reader rejects it rather than handing one out -- so a misaligned layout is not a slow
    # path, it is a frame that cannot be read at all.
    for name in [g[0] for g in GOLDEN]:
        for path in walk(lay(reg, name).root):
            p = path[-1]
            want = SIZE[p.dtype] if p.kind == LeafKind.FIXED else 4
            assert p.offset % want == 0, f"{name}.{p.name} at {p.offset}, needs {want}"


def test_blocks_are_addressable_as_arrays(reg):
    # Element blocks are indexed by i * stride, so stride has to be a multiple of the block's
    # own alignment or element 1 onward would be misaligned.
    for name in [g[0] for g in GOLDEN]:
        for path in walk(lay(reg, name).root):
            elem = path[-1].elem
            if elem is not None:
                assert elem.stride % elem.align == 0
                assert elem.align <= MAX_ALIGN


def test_variable_leaves_take_one_descriptor(reg):
    layout = lay(reg, "PointCloud")
    x, y = layout.root.placed[0], layout.root.placed[1]
    assert x.kind in VARIABLE
    assert y.offset - x.offset == DESC_SIZE


def test_header_is_a_stamp_plus_a_frame_id_descriptor(reg):
    msg = Message("H", [Field("header", "std_msgs/Header", ArrayKind.SCALAR)], package="test")
    layout = build_layout(flatten_message(msg, reg))
    header = layout.root.placed[0]
    assert header.kind == LeafKind.HEADER
    assert header.frame_id_offset == header.offset + STAMP_SIZE
    assert layout.scalar_bytes == STAMP_SIZE + DESC_SIZE


def test_nesting_does_not_move_a_byte(reg):
    # The fingerprint is invariant to restructuring a message's insides; the layout has to be
    # invariant the same way, or two ends that agree on the fingerprint would disagree on where
    # the bytes are -- the one failure the fingerprint cannot catch.
    pose, flat = lay(reg, "Pose"), lay(reg, "FlatPose")
    assert pose.scalar_bytes == flat.scalar_bytes
    assert [p.offset for p in pose.root.placed] == [p.offset for p in flat.root.placed]


def test_colliding_flattened_names_are_rejected(reg):
    # Nesting is joined with '_', so `a.b` and a sibling literally named `a_b` land on one
    # accessor. Rejecting beats emitting a header that will not compile.
    reg2 = dict(reg)
    reg2["test/Inner"] = Message("Inner", [Field("b", "float64", ArrayKind.SCALAR)],
                                 package="test")
    msg = Message("Clash", [
        Field("a", "test/Inner", ArrayKind.SCALAR),
        Field("a_b", "float64", ArrayKind.SCALAR),
    ], package="test")
    with pytest.raises(LayoutError, match="collide"):
        build_layout(flatten_message(msg, reg2))


def test_rejected_schema_carries_its_reason(reg):
    layout = build_layout(flatten_message(reg["BadBool"], reg))
    assert layout.tier == "reject"
    assert "bool[]" in layout.rejected
    assert layout.root is None


def test_a_constant_may_carry_a_name_the_adapter_defines(reg):
    # The adapter's own module-level names end in `__`, which no .msg constant can spell, so
    # FINGERPRINT__ sits beside FINGERPRINT__ rather than redeclaring it.
    from flux_gen.model import Constant
    msg = Message("C", [Field("x", "float64", ArrayKind.SCALAR)], package="test",
                  constants=[Constant("FINGERPRINT__", "uint8", 1)])
    assert build_layout(flatten_message(msg, reg)).constants[0].name == "FINGERPRINT__"


def test_fingerprint_travels_with_the_layout(reg):
    from flux_gen import fingerprint
    flat = flatten_message(reg["Pose"], reg)
    assert build_layout(flat).fingerprint == fingerprint(flat.leaves, flat.type_name)


def test_a_root_field_may_carry_a_name_the_adapter_defines(reg):
    # `uint32 size` used to be rejected at the root. The adapter now spells its own members
    # size__/ok__/commit__/build__, which _NAME_RE cannot produce, so the field is ordinary --
    # std_msgs/MultiArrayDimension publishes at the root for the same reason.
    msg = Message("F", [Field("size", "uint32", ArrayKind.SCALAR)], package="test")
    assert build_layout(flatten_message(msg, reg)).root.placed[0].name == "size"


def test_a_derived_accessor_cannot_collide_with_a_sibling_field(reg):
    # The Builder's allocator for `xs` is alloc__xs, so a sibling literally named alloc_xs is a
    # different name. Before the doubled underscore the two defined the same Python method and
    # the later one silently won.
    msg = Message("D", [Field("xs", "float32", ArrayKind.DYNAMIC),
                        Field("alloc_xs", "uint32", ArrayKind.SCALAR)], package="test")
    names = {p.name for p in build_layout(flatten_message(msg, reg)).root.placed}
    assert names == {"xs", "alloc_xs"}


def test_a_nested_dimension_field_still_flattens():
    # std_msgs/MultiArrayDimension puts a `size` field inside every MultiArray type; it has to
    # keep flattening whether it is nested or published on its own.
    from flux_gen import Registry, parse_msg
    r = Registry()
    dim = parse_msg("uint32 size\nfloat32[] xs\n", "Dim", package="t")
    arr = parse_msg("t/Dim[] dim\n", "Arr", package="t")
    r.add(dim)
    r.add(arr)
    assert build_layout(flatten_message(arr, r)).tier == "jagged"


def test_colliding_element_class_names_are_rejected():
    # Camel-casing erases '_' against '.', so `foo_bar` and `foo.bar` both name FooBarElem --
    # silent aliasing in Python, a redefinition error in C++.
    from flux_gen import Registry, parse_msg
    r = Registry()
    msgs = {}
    for name, text in (("Inner", "float32[] xs\n"), ("Foo", "t/Inner[] bar\n"),
                       ("Top", "t/Inner[] foo_bar\nt/Foo[] foo\n")):
        msgs[name] = parse_msg(text, name, package="t")
        r.add(msgs[name])
    with pytest.raises(LayoutError, match="FooBarElem"):
        build_layout(flatten_message(msgs["Top"], r))
