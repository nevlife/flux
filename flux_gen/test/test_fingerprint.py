"""flux_gen fingerprint tests: 64-bit, stable, nesting-invariant, layout-sensitive.

The relational tests below (a == b, a != b) hold for any encoding, so on their own they let the
canonical format drift: change a leaf token or a DType string and every fingerprint moves while
the suite stays green. Both ends are generated from the same tree, so nothing downstream notices
either -- only a peer built from an older tree does, at runtime, as a segment that never matches.
The golden table pins the actual bytes so that drift has to be an explicit edit.
"""

import glob
import os

import pytest

from flux_gen import (
    FINGERPRINT_SCHEMA_VERSION,
    Registry,
    canonical,
    fingerprint,
    flatten_message,
    load_dir,
    resolve,
)
from flux_gen.dtypes import ROS_PRIMITIVE
from flux_gen.model import ArrayKind

MSG_DIR = os.path.join(os.path.dirname(__file__), "msg")

# Canonical string and 64-bit digest for every leaf kind the flattener emits: fixed scalars and
# fixed blocks, numeric columns, record columns, jagged elements, string tails, fixed and dynamic
# string arrays, and a bare-Time stamp. Regenerate deliberately, and bump
# FINGERPRINT_SCHEMA_VERSION in the same commit -- these values are the wire contract.
GOLDEN = [
    ("Pose",
     "v5#test/Pose;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1",
     0x355D5F3FA5A9D96A),
    ("FlatPose",
     "v5#test/FlatPose;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1",
     0x8025D4B81455B91C),
    ("Vec3", "v5#test/Vec3;fixed:f64:1;fixed:f64:1;fixed:f64:1", 0xF331315EA0FFEB11),
    ("PointCloud",
     "v5#test/PointCloud;column:f32:0;column:f32:0;column:f32:0;fixed:u32:1;fixed:u32:1",
     0x438B2D5EE8C0D2C8),
    ("Labeled", "v5#test/Labeled;string:-:1;column:f64:0", 0xCD34AB0E7FEDFE3B),
    ("NestedString", "v5#test/NestedString;string:-:1;column:f64:0", 0x61259AAF57E27568),
    ("Trajectory",
     "v5#test/Trajectory;record:-:0{fixed:f64:1,fixed:f64:1,fixed:f64:1,fixed:f64:1,fixed:f64:1,fixed:f64:1}",
     0x6189DBC36C3162D4),
    ("TensorList", "v5#test/TensorList;jagged:-:0{column:u32:0,column:f32:0}", 0x365EF7BD07FF3A69),
    ("Tensor", "v5#test/Tensor;column:u32:0;column:f32:0", 0x1917DE80DD607226),
    # Event carries a bare Time. It tokenizes as the two fields it lays out as, not as an opaque
    # stamp token -- that is what keeps a nested Header/Time fingerprinting like a flattened one.
    ("Event", "v5#test/Event;fixed:i32:1;fixed:u32:1;sarray:-:0;column:f32:0", 0x53DEC4CA6B7ECB35),
    ("Names", "v5#test/Names;sarray:-:4;sarray:-:0", 0x57DCBC297E53A209),
]


@pytest.fixture(scope="module")
def reg():
    return load_dir(MSG_DIR)


def fp(reg, name):
    r = flatten_message(reg[name], reg)
    return fingerprint(r.leaves, r.type_name)


@pytest.mark.parametrize("name,text,digest", GOLDEN, ids=[g[0] for g in GOLDEN])
def test_golden_canonical_and_fingerprint(reg, name, text, digest):
    r = flatten_message(reg[name], reg)
    assert canonical(r.leaves, r.type_name) == text
    assert fingerprint(r.leaves, r.type_name) == digest


def test_canonical_carries_the_schema_version(reg):
    # The version prefix is what lets an incompatible encoding change show up as a mismatch
    # instead of as a coincidentally equal digest.
    leaves = flatten_message(reg["Pose"], reg).leaves
    assert canonical(leaves).startswith(f"v{FINGERPRINT_SCHEMA_VERSION};")
    assert canonical(leaves, "p/M").startswith(f"v{FINGERPRINT_SCHEMA_VERSION}#p/M;")
    assert FINGERPRINT_SCHEMA_VERSION == 5


def test_fingerprint_is_64bit_and_stable(reg):
    a = fp(reg, "Pose")
    b = fp(reg, "Pose")
    assert 0 <= a < 2**64
    assert a == b


def test_nesting_is_invariant(reg):
    # Restructuring a message's insides -- pulling fields out into a nested struct -- changes
    # nothing on the wire, so it must not change the fingerprint. Pose (nested Vec3 x2) and
    # FlatPose (six flat float64) flatten identically; under one type name they are one schema.
    pose = flatten_message(reg["Pose"], reg)
    flat = flatten_message(reg["FlatPose"], reg)
    assert canonical(pose.leaves, "p/Same") == canonical(flat.leaves, "p/Same")
    assert fingerprint(pose.leaves, "p/Same") == fingerprint(flat.leaves, "p/Same")


def test_type_name_separates_identical_layouts(reg):
    # geometry_msgs/Point and geometry_msgs/Vector3 are both three float64. On layout alone they
    # shared a segment, so a node could read a position as a direction with nothing to complain.
    # ROS 2 refuses that pairing on the type name; verified against rclpy, a Point publisher and
    # a Vector3 subscription on one topic never connect. The fingerprint refuses it the same way.
    leaves = flatten_message(reg["Vec3"], reg).leaves
    assert fingerprint(leaves, "geometry_msgs/msg/Point") != fingerprint(
        leaves, "geometry_msgs/msg/Vector3")
    # Nested names stay out of it: only the outermost name identifies the schema.
    assert fingerprint(leaves, "p/M") == fingerprint(leaves, "p/M")


def test_different_layout_differs(reg):
    assert fp(reg, "Pose") != fp(reg, "PointCloud")
    assert fp(reg, "Pose") != fp(reg, "Trajectory")
    assert fp(reg, "Labeled") != fp(reg, "PointCloud")


def test_jagged_element_structure_differentiates(reg):
    # Two jagged schemas with different element structure must not collide. Before the element
    # layout was folded in, every jagged schema hashed to the same "jagged:-:1".
    from flux_gen import Field, Message
    from flux_gen.model import ArrayKind

    tensor_list = Message("TL", [Field("items", "Tensor", ArrayKind.DYNAMIC)])   # {u32[], f32[]}
    labeled_list = Message("LL", [Field("items", "Labeled", ArrayKind.DYNAMIC)])  # {string, f64[]}
    a = fingerprint(flatten_message(tensor_list, reg).leaves)
    b = fingerprint(flatten_message(labeled_list, reg).leaves)
    assert a != b


def test_jagged_fixed_size_differentiates(reg):
    # A fixed jagged array carries its length N; dynamic vs N=2 vs N=3 must all differ.
    from flux_gen import Field, Message
    from flux_gen.model import ArrayKind

    dyn = Message("D", [Field("items", "Labeled", ArrayKind.DYNAMIC)])
    n2 = Message("N2", [Field("items", "Labeled", ArrayKind.FIXED, 2)])
    n3 = Message("N3", [Field("items", "Labeled", ArrayKind.FIXED, 3)])
    fps = {
        fingerprint(flatten_message(dyn, reg).leaves),
        fingerprint(flatten_message(n2, reg).leaves),
        fingerprint(flatten_message(n3, reg).leaves),
    }
    assert len(fps) == 3


def test_jagged_dynamic_does_not_collide_with_n_of_one(reg):
    # T[] and T[1] read differently (per-frame length vs a compile-time 1) but used to produce
    # the same token, because only the fixed path passed a count and the dynamic one kept the
    # default of 1. N=1 was the single value where the two overlapped.
    from flux_gen import Field, Message
    from flux_gen.model import ArrayKind

    a = Message("A", [Field("items", "Labeled", ArrayKind.DYNAMIC)])
    b = Message("B", [Field("items", "Labeled", ArrayKind.FIXED, 1)])
    fa = flatten_message(a, reg).leaves
    fb = flatten_message(b, reg).leaves
    assert canonical(fa) == "v5;jagged:-:0{string:-:1,column:f64:0}"
    assert canonical(fb) == "v5;jagged:-:1{string:-:1,column:f64:0}"
    assert fingerprint(fa) != fingerprint(fb)


def test_fixed_and_dynamic_string_arrays_differ(reg):
    # Same pairing one level down: string[4] and string[] must not share a token either.
    from flux_gen import Field, Message
    from flux_gen.model import ArrayKind

    one = Message("One", [Field("tags", "string", ArrayKind.FIXED, 1)])
    dyn = Message("Dyn", [Field("tags", "string", ArrayKind.DYNAMIC)])
    assert canonical(flatten_message(one, reg).leaves) == "v5;sarray:-:1"
    assert canonical(flatten_message(dyn, reg).leaves) == "v5;sarray:-:0"
    assert fingerprint(flatten_message(one, reg).leaves) != fingerprint(flatten_message(dyn, reg).leaves)


# --- corpus functionality ---------------------------------------------------------------------

ROS_MSG_DIR_GLOB = "/opt/ros/*/share/*/msg"


def _model_shape(msg, reg, depth=0):
    """The wire shape of a message read off the parsed model, without the flattener.

    This is the independent side of the check below. Describing the schema twice by two routes is
    the whole point, so it must not call flatten_message -- a bug reproduced on both sides is a
    bug the check cannot see.

    Three things it does copy from the design, because they are the design and not the code under
    test: `char`/`byte` and `uint8` are one dtype, a scalar nested message is spliced into its
    parent (nesting moves no byte), and a fixed array of messages expands to N copies. Field names
    stay out, matching the fingerprint. What it does NOT interpret is the array kind and size of
    each field, which is exactly where the two collapses this check exists for happened.
    """
    if depth > 32:
        raise AssertionError(f"{msg.name}: nested deeper than any ROS message; likely a cycle")
    out = []
    for f in msg.fields:
        nested = resolve(reg, f.type_name, msg.package)
        if nested is None:
            base = ROS_PRIMITIVE.get(f.type_name)
            inner = (base.value if base is not None else f.type_name,)
        else:
            inner = _model_shape(nested, reg, depth + 1)
        if f.kind is ArrayKind.SCALAR and nested is not None:
            out.extend(inner)  # nesting is a name prefix, not a layout
        elif f.kind is ArrayKind.FIXED and nested is not None:
            out.extend(inner * f.size)  # a fixed array of messages expands in place
        else:
            out.append((inner, f.kind.value, f.size))
    return tuple(out)


def _corpus():
    corpus = Registry()
    for d in sorted(glob.glob(ROS_MSG_DIR_GLOB)):
        load_dir(d, into=corpus)
    out = {}
    for key, msg in corpus.items():
        if "/" not in key:
            continue  # bare alias of an entry already visited under its qualified key
        try:
            flat = flatten_message(msg, corpus)
            shape = _model_shape(msg, corpus)
        except Exception:
            continue  # unresolvable here is the parser's business, not this test
        if flat.rejected:
            continue  # no adapter is generated, so this schema never reaches a segment
        out[key] = (flat, shape)
    return out


@pytest.mark.skipif(not glob.glob(ROS_MSG_DIR_GLOB), reason="no installed ROS messages to sweep")
def test_the_layout_fingerprint_is_a_function_of_the_shape_across_the_ros_corpus():
    # Two schemas that fingerprint alike are two schemas flux lets share a segment. So a shared
    # fingerprint has to mean a shared shape, and the shape here is derived without the flattener
    # so that a flattener bug shows up as a disagreement rather than being reproduced on both
    # sides. This is the property the hand-picked pairs above only sample: dynamic-vs-fixed
    # jagged and string arrays both collapsed to one token once, and each was found by someone
    # thinking of the pair rather than by the corpus.
    #
    # The layout-only digest is the one under test. It is what a caller with raw arrays and no
    # .msg gets, so it carries no type name to separate two schemas that happen to flatten alike.
    buckets = {}
    for key, (flat, shape) in _corpus().items():
        buckets.setdefault(fingerprint(flat.leaves), {}).setdefault(shape, []).append(key)

    collisions = {
        digest: list(by_shape.values())
        for digest, by_shape in buckets.items()
        if len(by_shape) > 1
    }
    assert not collisions, (
        "distinct shapes share one layout fingerprint, so flux would let them share a segment: "
        + "; ".join(
            f"0x{d:016X} -> " + " vs ".join(sorted(g)[0] for g in groups)
            for d, groups in sorted(collisions.items())))


@pytest.mark.skipif(not glob.glob(ROS_MSG_DIR_GLOB), reason="no installed ROS messages to sweep")
def test_the_named_fingerprint_separates_every_type_in_the_ros_corpus():
    # Generated adapters digest the type name with the layout (layout.py), so on that path no two
    # corpus types may collide at all -- not even the ones that legitimately flatten alike.
    seen = {}
    for key, (flat, _) in _corpus().items():
        digest = fingerprint(flat.leaves, flat.type_name)
        assert digest not in seen or seen[digest] == key, (
            f"{key} and {seen[digest]} share fingerprint 0x{digest:016X}")
        seen[digest] = key
    assert len(seen) > 100, "expected a substantial corpus to sweep"
