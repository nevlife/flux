//! Flatten and tier classification, driven by test/msg/*.msg.

mod common;

use common::{field, message, ros_corpus, ros_msg_dirs, test_registry};
use flux_gen::dtypes::DType;
use flux_gen::flatten::{DYNAMIC_COUNT, FlatResult, Leaf, LeafKind, MAX_LEAVES, flatten_message};
use flux_gen::model::ArrayKind;
use flux_gen::parse::{Registry, parse_msg};

fn flat(reg: &Registry, name: &str) -> FlatResult {
    flatten_message(reg.get(name).unwrap(), reg).unwrap()
}

fn kinds(leaves: &[Leaf]) -> Vec<LeafKind> {
    leaves.iter().map(|l| l.kind).collect()
}

#[test]
fn pose_flattens_to_fixed_block() {
    let r = flat(&test_registry(), "Pose");
    assert_eq!(r.tier(), "fixed");
    assert_eq!(r.leaves.len(), 6);
    assert!(
        r.leaves
            .iter()
            .all(|l| l.kind == LeafKind::Fixed && l.dtype == Some(DType::F64) && l.count == 1)
    );
}

#[test]
fn fixed_array_becomes_count_leaf() {
    let m = message(
        "Mat",
        vec![field("matrix", "float64", ArrayKind::Fixed, 9)],
        None,
    );
    let r = flatten_message(&m, &test_registry()).unwrap();
    assert_eq!(r.tier(), "fixed");
    assert_eq!(r.leaves.len(), 1);
    assert_eq!(r.leaves[0].kind, LeafKind::Fixed);
    assert_eq!(r.leaves[0].count, 9);
}

#[test]
fn header_is_frame_metadata() {
    // A Header (stamp + string frame_id) maps to frame metadata, not a flattened struct, so the
    // message stays supported with the payload column intact.
    let m = message(
        "Stamped",
        vec![
            field("header", "std_msgs/Header", ArrayKind::Scalar, 0),
            field("data", "float32", ArrayKind::Dynamic, 0),
        ],
        None,
    );
    let r = flatten_message(&m, &test_registry()).unwrap();
    assert_ne!(r.tier(), "reject");
    assert_eq!(r.leaves[0].kind, LeafKind::Header);
    assert_eq!(r.leaves[1].kind, LeafKind::Column);
}

#[test]
fn trajectory_is_record_column() {
    let r = flat(&test_registry(), "Trajectory");
    assert_eq!(r.tier(), "columnar");
    assert_eq!(r.leaves.len(), 1);
    let leaf = &r.leaves[0];
    assert_eq!(leaf.kind, LeafKind::RecordColumn);
    assert_eq!(leaf.record_leaves.len(), 6);
    assert!(
        leaf.record_leaves
            .iter()
            .all(|rl| rl.dtype == Some(DType::F64))
    );
}

#[test]
fn pointcloud_columns_and_scalars() {
    let r = flat(&test_registry(), "PointCloud");
    assert_eq!(r.tier(), "columnar");
    assert_eq!(
        kinds(&r.leaves),
        [
            LeafKind::Column,
            LeafKind::Column,
            LeafKind::Column,
            LeafKind::Fixed,
            LeafKind::Fixed,
        ]
    );
}

#[test]
fn labeled_has_string_tail() {
    let r = flat(&test_registry(), "Labeled");
    assert_eq!(r.tier(), "columnar");
    assert_eq!(kinds(&r.leaves), [LeafKind::String, LeafKind::Column]);
}

#[test]
fn tensorlist_is_jagged() {
    let r = flat(&test_registry(), "TensorList");
    assert_eq!(r.tier(), "jagged");
    assert_eq!(r.leaves[0].kind, LeafKind::Jagged);
}

#[test]
fn jagged_carries_element_structure() {
    // Distinct jagged schemas must stay distinguishable, not collapse to a bare "jagged".
    // Tensor = {uint32[], float32[]}.
    let r = flat(&test_registry(), "TensorList");
    let leaf = &r.leaves[0];
    assert_eq!(leaf.kind, LeafKind::Jagged);
    assert_eq!(
        kinds(&leaf.record_leaves),
        [LeafKind::Column, LeafKind::Column]
    );
    let dtypes: Vec<_> = leaf.record_leaves.iter().map(|rl| rl.dtype).collect();
    assert_eq!(dtypes, [Some(DType::U32), Some(DType::F32)]);
}

#[test]
fn single_nested_hoists() {
    // A single nested struct with variable members hoists up instead of being rejected.
    let r = flat(&test_registry(), "NestedString");
    assert_eq!(r.tier(), "columnar");
    assert_eq!(kinds(&r.leaves), [LeafKind::String, LeafKind::Column]);
}

#[test]
fn bare_time_is_a_frame_stamp() {
    // A bare builtin_interfaces/Time is frame metadata like Header, not a flattened
    // sec/nanosec pair.
    let r = flat(&test_registry(), "Event");
    assert_eq!(
        kinds(&r.leaves),
        [LeafKind::Stamp, LeafKind::StringArray, LeafKind::Column]
    );
}

#[test]
fn fixed_and_dynamic_string_arrays_carry_different_counts() {
    let r = flat(&test_registry(), "Names");
    assert_eq!(
        kinds(&r.leaves),
        [LeafKind::StringArray, LeafKind::StringArray]
    );
    let counts: Vec<_> = r.leaves.iter().map(|l| l.count).collect();
    assert_eq!(counts, [4, DYNAMIC_COUNT]);
}

#[test]
fn rejections() {
    let reg = test_registry();
    for (name, needle) in [
        ("BadBounded", "bounded"),
        ("BadBool", "bool[]"),
        ("BadWstring", "wstring"),
    ] {
        let r = flat(&reg, name);
        assert_eq!(r.tier(), "reject", "{name}");
        let why = r.rejected.unwrap();
        assert!(why.contains(needle), "{name}: {why}");
    }
}

/// `Some(true)` always dynamic, `Some(false)` always fixed, `None` both forms. Mirrors the
/// Python `ALWAYS_DYNAMIC`/`ALWAYS_FIXED`; exhaustive so a new kind must be classified.
fn count_rule(kind: LeafKind) -> Option<bool> {
    match kind {
        LeafKind::Column | LeafKind::RecordColumn => Some(true),
        LeafKind::Fixed | LeafKind::String | LeafKind::Header | LeafKind::Stamp => Some(false),
        LeafKind::StringArray | LeafKind::Jagged => None,
    }
}

fn check_count_rule(leaf: &Leaf) {
    match count_rule(leaf.kind) {
        Some(true) => assert_eq!(
            leaf.count, DYNAMIC_COUNT,
            "{:?} must carry the dynamic count",
            leaf.kind
        ),
        Some(false) => assert!(
            leaf.count >= 1,
            "{:?} must carry a fixed multiplicity",
            leaf.kind
        ),
        None => {}
    }
    for sub in &leaf.record_leaves {
        check_count_rule(sub);
    }
}

#[test]
fn dynamic_count_rule_holds_for_every_leaf() {
    // A dynamic JAGGED that kept the default count 1 flattened T[] and T[1] to the same leaf.
    let reg = test_registry();
    for (_, msg) in reg.messages() {
        for leaf in &flatten_message(msg, &reg).unwrap().leaves {
            check_count_rule(leaf);
        }
    }
}

#[test]
fn dynamic_count_rule_holds_across_the_ros_corpus() {
    if ros_msg_dirs().is_empty() {
        eprintln!("skipped: no installed ROS messages to sweep");
        return;
    }
    let corpus = ros_corpus();
    // A bare key aliases an entry already visited under its qualified key.
    for (_, msg) in corpus.messages().filter(|(k, _)| k.contains('/')) {
        for leaf in &flatten_message(msg, &corpus).unwrap().leaves {
            check_count_rule(leaf);
        }
    }
}

#[test]
fn nested_fixed_arrays_cannot_blow_up_generation() {
    // Each level passes the parser's per-field bound but the product does not: without a total
    // cap this materializes ~2e5 leaves, so it must reject rather than hang the generator.
    let xyz = ["x", "y", "z"].map(|n| field(n, "float64", ArrayKind::Scalar, 0));
    let inner = message("Inner", xyz.to_vec(), None);
    let mid = message("Mid", vec![field("a", "Inner", ArrayKind::Fixed, 64)], None);
    let outer = message("Outer", vec![field("b", "Mid", ArrayKind::Fixed, 64)], None);
    let mut reg = Registry::new();
    for m in [inner, mid, outer.clone()] {
        reg.add(m, false).unwrap();
    }
    let r = flatten_message(&outer, &reg).unwrap();
    assert_eq!(r.tier(), "reject");
    assert!(r.rejected.unwrap().contains(&MAX_LEAVES.to_string()));
}

#[test]
fn a_large_but_sane_fixed_nesting_still_works() {
    // 16 x 6 = 96 leaves, the same order as the largest message shipped with ROS jazzy (107).
    let m = message(
        "Poses",
        vec![field("p", "Pose", ArrayKind::Fixed, 16)],
        None,
    );
    let r = flatten_message(&m, &test_registry()).unwrap();
    assert_eq!(r.tier(), "fixed");
    assert_eq!(r.leaves.len(), 96);
}

#[test]
fn an_array_of_a_fieldless_type_is_rejected() {
    // A constants-only nested type has no element bytes, so `items[i]` has no stride to step by.
    let mut reg = Registry::new();
    let nothing = parse_msg("uint8 K = 1\n", "Nothing", None, Some("test")).unwrap();
    let carrier = parse_msg("test/Nothing[] items\n", "Carrier", None, Some("test")).unwrap();
    reg.add(nothing, false).unwrap();
    reg.add(carrier, false).unwrap();
    let r = flatten_message(reg.get("test/Carrier").unwrap(), &reg).unwrap();
    assert_eq!(r.tier(), "reject");
    assert!(r.rejected.unwrap().contains("no publishable fields"));
}

fn flat_of(reg: &mut Registry, text: &str) -> FlatResult {
    let msg = parse_msg(text, "Carrier", None, Some("p")).unwrap();
    reg.add(msg, false).unwrap();
    flatten_message(reg.get("p/Carrier").unwrap(), reg).unwrap()
}

#[test]
fn what_the_wire_cannot_carry_is_a_rejection_not_a_parse_error() {
    // rosidl takes these. A parse error stopped every message that shared a registry with them;
    // a rejection stops only this one.
    for (text, why) in [
        ("uint8[70000] x\n", "exceeds the 65536 limit"),
        ("uint8[99999999999999999999] x\n", "exceeds"),
        ("@optional int32 x\n", "@optional"),
    ] {
        let r = flat_of(&mut Registry::new(), text);
        assert!(r.rejected.unwrap().contains(why), "{text}");
    }
}

#[test]
fn a_package_time_or_header_is_an_ordinary_struct() {
    // Matching the field's type text took a package's own Time.msg for builtin_interfaces/Time:
    // the C++ bridge did not compile and the Python one wrote the wrong fields.
    let mut reg = Registry::new();
    for (name, text) in [("Time", "float64 x\n"), ("Header", "uint32 seq\n")] {
        reg.add(parse_msg(text, name, None, Some("p")).unwrap(), false)
            .unwrap();
    }
    let r = flat_of(&mut reg, "Time t\nHeader h\nint32 y\n");
    assert_eq!(kinds(&r.leaves), [LeafKind::Fixed; 3]);
    assert_eq!(r.leaves[0].dtype, Some(DType::F64));
}

#[test]
fn the_builtin_time_and_header_are_the_frame_stamp() {
    let mut reg = Registry::new();
    let header = parse_msg("int32 sec\n", "Header", None, Some("std_msgs")).unwrap();
    reg.add(header, false).unwrap();
    let r = flat_of(
        &mut reg,
        "builtin_interfaces/Time t\nstd_msgs/Header h\nHeader bare\n",
    );
    assert_eq!(
        kinds(&r.leaves),
        [LeafKind::Stamp, LeafKind::Header, LeafKind::Header]
    );
}

#[test]
fn a_ros_byte_is_marked_for_the_ros_bridge() {
    let r = flat_of(&mut Registry::new(), "byte b\nuint8 u\nchar c\nbyte[] bs\n");
    let marks: Vec<bool> = r.leaves.iter().map(|l| l.ros_byte).collect();
    assert_eq!(marks, [true, false, false, true]);
}
