"""flux_gen flatten + tier classification tests, driven by test/msg/*.msg."""

import glob
import os

import pytest

from flux_gen import (
    DYNAMIC_COUNT, DType, LeafKind, Registry, flatten_message, load_dir, parse_msg)
from flux_gen.flatten import ALWAYS_DYNAMIC, ALWAYS_FIXED

MSG_DIR = os.path.join(os.path.dirname(__file__), "msg")
ROS_MSG_DIR_GLOB = "/opt/ros/*/share/*/msg"


@pytest.fixture(scope="module")
def reg():
    return load_dir(MSG_DIR)


def flat(reg, name):
    return flatten_message(reg[name], reg)


def test_pose_flattens_to_fixed_block(reg):
    r = flat(reg, "Pose")
    assert r.tier == "fixed"
    assert len(r.leaves) == 6
    assert all(l.kind == LeafKind.FIXED and l.dtype == DType.F64 and l.count == 1 for l in r.leaves)


def test_fixed_array_becomes_count_leaf(reg):
    from flux_gen import Field, Message
    from flux_gen.model import ArrayKind

    m = Message("Mat", [Field("matrix", "float64", ArrayKind.FIXED, 9)])
    r = flatten_message(m, reg)
    assert r.tier == "fixed"
    assert len(r.leaves) == 1
    assert r.leaves[0].kind == LeafKind.FIXED and r.leaves[0].count == 9


def test_header_is_frame_metadata(reg):
    from flux_gen import Field, Message
    from flux_gen.model import ArrayKind

    # A Header (stamp + string frame_id) must not reject: it maps to frame metadata, not a
    # flattened struct. The message stays supported with the payload column intact.
    m = Message("Stamped", [
        Field("header", "std_msgs/Header", ArrayKind.SCALAR),
        Field("data", "float32", ArrayKind.DYNAMIC),
    ])
    r = flatten_message(m, reg)
    assert r.tier != "reject"
    assert r.leaves[0].kind == LeafKind.HEADER
    assert r.leaves[1].kind == LeafKind.COLUMN


def test_trajectory_is_record_column(reg):
    r = flat(reg, "Trajectory")
    assert r.tier == "columnar"
    assert len(r.leaves) == 1
    leaf = r.leaves[0]
    assert leaf.kind == LeafKind.RECORD_COLUMN
    assert len(leaf.record_leaves) == 6
    assert all(rl.dtype == DType.F64 for rl in leaf.record_leaves)


def test_pointcloud_columns_and_scalars(reg):
    r = flat(reg, "PointCloud")
    assert r.tier == "columnar"
    kinds = [l.kind for l in r.leaves]
    assert kinds == [LeafKind.COLUMN, LeafKind.COLUMN, LeafKind.COLUMN, LeafKind.FIXED, LeafKind.FIXED]


def test_labeled_has_string_tail(reg):
    r = flat(reg, "Labeled")
    assert r.tier == "columnar"
    assert [l.kind for l in r.leaves] == [LeafKind.STRING, LeafKind.COLUMN]


def test_tensorlist_is_jagged(reg):
    r = flat(reg, "TensorList")
    assert r.tier == "jagged"
    assert r.leaves[0].kind == LeafKind.JAGGED


def test_jagged_carries_element_structure(reg):
    # A jagged leaf must record its element's flattened layout so distinct jagged schemas are
    # distinguishable (not all collapsed to a bare "jagged"). Tensor = {uint32[], float32[]}.
    r = flat(reg, "TensorList")
    leaf = r.leaves[0]
    assert leaf.kind == LeafKind.JAGGED
    assert [rl.kind for rl in leaf.record_leaves] == [LeafKind.COLUMN, LeafKind.COLUMN]
    assert [rl.dtype for rl in leaf.record_leaves] == [DType.U32, DType.F32]


def test_single_nested_hoists(reg):
    # A single nested struct with variable members is not rejected: it hoists up. Labeled item
    # (string label + float64[] values) -> [STRING tail, COLUMN].
    r = flat(reg, "NestedString")
    assert r.tier == "columnar"
    assert [l.kind for l in r.leaves] == [LeafKind.STRING, LeafKind.COLUMN]


def test_bare_time_is_a_frame_stamp(reg):
    # A bare builtin_interfaces/Time is frame metadata like Header, not a flattened sec/nanosec
    # pair, so it must not appear in the scalar block.
    r = flat(reg, "Event")
    assert [l.kind for l in r.leaves] == [LeafKind.STAMP, LeafKind.STRING_ARRAY, LeafKind.COLUMN]


def test_fixed_and_dynamic_string_arrays_carry_different_counts(reg):
    r = flat(reg, "Names")
    assert [l.kind for l in r.leaves] == [LeafKind.STRING_ARRAY, LeafKind.STRING_ARRAY]
    assert [l.count for l in r.leaves] == [4, DYNAMIC_COUNT]


@pytest.mark.parametrize(
    "name,needle",
    [
        ("BadBounded", "bounded"),
        ("BadBool", "bool[]"),
        ("BadWstring", "wstring"),
    ],
)
def test_rejections(reg, name, needle):
    r = flat(reg, name)
    assert r.tier == "reject"
    assert needle in r.rejected


def test_every_leaf_kind_has_a_declared_count_rule():
    # A new LeafKind has to be classified, not silently inherit the default count of 1 -- that
    # default is exactly what made dynamic JAGGED collide with the fixed T[1] form. STRING_ARRAY
    # and JAGGED are the only kinds with both a fixed and a dynamic form.
    undeclared = set(LeafKind) - ALWAYS_DYNAMIC - ALWAYS_FIXED
    assert undeclared == {LeafKind.STRING_ARRAY, LeafKind.JAGGED}


def _check_count_rule(leaf):
    """count == DYNAMIC_COUNT exactly when the leaf's length is decided per frame."""
    if leaf.kind in ALWAYS_DYNAMIC:
        assert leaf.count == DYNAMIC_COUNT, f"{leaf.kind} must carry the dynamic count"
    elif leaf.kind in ALWAYS_FIXED:
        assert leaf.count >= 1, f"{leaf.kind} must carry a fixed multiplicity"
    for sub in leaf.record_leaves:
        _check_count_rule(sub)


def test_dynamic_count_rule_holds_for_every_leaf(reg):
    # The rule has to be uniform across leaf kinds. It was not: STRING_ARRAY marked a dynamic
    # array with count 0 while a dynamic JAGGED kept the default 1, so T[] and T[1] flattened to
    # the same leaf and fingerprinted the same.
    for name in reg:
        for leaf in flatten_message(reg[name], reg).leaves:
            _check_count_rule(leaf)


@pytest.mark.skipif(not glob.glob(ROS_MSG_DIR_GLOB), reason="no installed ROS messages to sweep")
def test_dynamic_count_rule_holds_across_the_ros_corpus():
    corpus = Registry()
    for d in sorted(glob.glob(ROS_MSG_DIR_GLOB)):
        load_dir(d, into=corpus)
    for key, msg in corpus.items():
        if "/" not in key:
            continue  # bare alias of an entry already visited under its qualified key
        for leaf in flatten_message(msg, corpus).leaves:
            _check_count_rule(leaf)


def test_nested_fixed_arrays_cannot_blow_up_generation(reg):
    # Nesting multiplies: each level passes the parser's per-field size bound, but the product
    # does not. Without a total cap this materializes ~2e5 leaves (and deeper nesting far more),
    # so it must reject rather than hang the generator.
    from flux_gen import Field, Message
    from flux_gen.flatten import MAX_LEAVES
    from flux_gen.model import ArrayKind

    inner = Message("Inner", [Field(n, "float64", ArrayKind.SCALAR) for n in ("x", "y", "z")])
    mid = Message("Mid", [Field("a", "Inner", ArrayKind.FIXED, 64)])       # 192 leaves
    outer = Message("Outer", [Field("b", "Mid", ArrayKind.FIXED, 64)])     # 12288 leaves
    r = flatten_message(outer, {"Inner": inner, "Mid": mid, "Outer": outer})
    assert r.tier == "reject"
    assert str(MAX_LEAVES) in r.rejected


def test_a_large_but_sane_fixed_nesting_still_works(reg):
    # The cap must not catch schemas that are merely big. 16 x 6 = 96 leaves, the same order as
    # the largest message shipped with ROS jazzy (107).
    from flux_gen import Field, Message
    from flux_gen.model import ArrayKind

    m = Message("Poses", [Field("p", "Pose", ArrayKind.FIXED, 16)])
    r = flatten_message(m, reg)
    assert r.tier == "fixed"
    assert len(r.leaves) == 96


def test_an_array_of_a_fieldless_type_is_rejected(reg):
    # A constants-only nested type flattens to no leaves, so its element block has no bytes and
    # `items[i]` has no stride to step by. Rejecting beats emitting an adapter that divides by
    # a zero stride.
    reg2 = Registry()
    reg2.add(parse_msg("uint8 K = 1\n", "Nothing", package="test"))
    reg2.add(parse_msg("test/Nothing[] items\n", "Carrier", package="test"))
    result = flatten_message(reg2["test/Carrier"], reg2)
    assert result.tier == "reject"
    assert "no publishable fields" in result.rejected
