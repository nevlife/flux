//! Layout: offsets, alignment, and the golden table that pins them.
//!
//! Relational assertions hold for any layout; GOLDEN pins the offsets, since a drift shows only
//! to a peer built from an older tree, reading wrong bytes under a matching fingerprint.

mod common;

use std::collections::HashSet;

use common::{field, message, test_registry};
use flux_gen::fingerprint::fingerprint;
use flux_gen::flatten::{LeafKind, flatten_message};
use flux_gen::layout::{DESC_SIZE, Layout, LayoutError, Placed, STAMP_SIZE, build_layout, walk};
use flux_gen::model::{ArrayKind, ConstValue, Constant, Message};
use flux_gen::parse::{Registry, parse_msg};

/// (message, `scalar_bytes`, [(name, offset)]) for the root block. These offsets are the wire
/// contract between the C++ and Python adapters: regenerate deliberately.
type Golden = (&'static str, usize, &'static [(&'static str, usize)]);

const GOLDEN: &[Golden] = &[
    ("Vec3", 24, &[("x", 0), ("y", 8), ("z", 16)]),
    ("Pose", 48, &[("position_x", 0), ("orientation_z", 40)]),
    (
        "PointCloud",
        32,
        &[("x", 0), ("y", 8), ("z", 16), ("width", 24), ("height", 28)],
    ),
    ("Labeled", 16, &[("label", 0), ("values", 8)]),
    ("Trajectory", 8, &[("poses", 0)]),
    ("TensorList", 8, &[("tensors", 0)]),
    ("Tensor", 16, &[("shape", 0), ("data", 8)]),
    ("Event", 24, &[("stamp", 0), ("tags", 8), ("values", 16)]),
    ("Names", 16, &[("fixed_tags", 0), ("dyn_tags", 8)]),
];

fn try_build(msg: &Message, reg: &Registry) -> Result<Layout, LayoutError> {
    build_layout(&flatten_message(msg, reg).unwrap())
}

fn lay(reg: &Registry, name: &str) -> Layout {
    try_build(reg.get(name).unwrap(), reg).unwrap()
}

#[test]
fn golden_offsets() {
    let reg = test_registry();
    for &(name, scalar_bytes, offsets) in GOLDEN {
        let layout = lay(&reg, name);
        assert_eq!(layout.scalar_bytes(), scalar_bytes, "{name}");
        for &(fname, off) in offsets {
            let placed = layout.root.placed.iter().find(|p| p.name() == fname);
            assert_eq!(placed.unwrap().offset, off, "{name}.{fname}");
        }
    }
}

#[test]
fn every_offset_is_naturally_aligned() {
    // The reader rejects an unaligned column (UB as T* in C++), so the frame would be unreadable.
    let reg = test_registry();
    for &(name, _, _) in GOLDEN {
        let layout = lay(&reg, name);
        for path in walk(&layout.root) {
            let p = path[path.len() - 1];
            let want = match p.kind() {
                LeafKind::Fixed => p.leaf.dtype.unwrap().size(),
                _ => 4,
            };
            assert_eq!(
                p.offset % want,
                0,
                "{name}.{} at {}, needs {want}",
                p.name(),
                p.offset
            );
        }
    }
}

#[test]
fn blocks_are_addressable_as_arrays() {
    let reg = test_registry();
    for &(name, _, _) in GOLDEN {
        let layout = lay(&reg, name);
        for path in walk(&layout.root) {
            if let Some(elem) = &path[path.len() - 1].elem {
                assert_eq!(elem.stride % elem.align, 0, "{name}");
                assert!(elem.align <= 8, "{name}"); // the widest scalar
            }
        }
    }
}

#[test]
fn variable_leaves_take_one_descriptor() {
    let layout = lay(&test_registry(), "PointCloud");
    let (x, y) = (&layout.root.placed[0], &layout.root.placed[1]);
    // The Python VARIABLE set: kinds whose scalar-block entry is a descriptor.
    assert!(matches!(
        x.kind(),
        LeafKind::Column
            | LeafKind::RecordColumn
            | LeafKind::Jagged
            | LeafKind::String
            | LeafKind::StringArray
    ));
    assert_eq!(y.offset - x.offset, DESC_SIZE);
}

#[test]
fn header_is_a_stamp_plus_a_frame_id_descriptor() {
    let msg = message(
        "H",
        vec![field("header", "std_msgs/Header", ArrayKind::Scalar, 0)],
        Some("test"),
    );
    let layout = try_build(&msg, &test_registry()).unwrap();
    let header = &layout.root.placed[0];
    assert_eq!(header.kind(), LeafKind::Header);
    assert_eq!(header.frame_id_offset(), header.offset + STAMP_SIZE);
    assert_eq!(layout.scalar_bytes(), STAMP_SIZE + DESC_SIZE);
}

#[test]
fn the_stamp_offset_is_the_top_level_headers() {
    let msg = message(
        "H",
        vec![
            field("count", "uint8", ArrayKind::Scalar, 0),
            field("header", "std_msgs/Header", ArrayKind::Scalar, 0),
        ],
        Some("test"),
    );
    let layout = try_build(&msg, &test_registry()).unwrap();
    let header = layout
        .root
        .placed
        .iter()
        .find(|p| p.kind() == LeafKind::Header);
    assert_eq!(
        layout.stamp_offset(),
        i32::try_from(header.unwrap().offset).unwrap()
    );
    assert_eq!(lay(&test_registry(), "Vec3").stamp_offset(), -1);
}

#[test]
fn nesting_does_not_move_a_byte() {
    // The fingerprint ignores nesting, so the layout must too: a mismatch here goes uncaught.
    let reg = test_registry();
    let (pose, flat) = (lay(&reg, "Pose"), lay(&reg, "FlatPose"));
    assert_eq!(pose.scalar_bytes(), flat.scalar_bytes());
    let offsets = |l: &Layout| l.root.placed.iter().map(|p| p.offset).collect::<Vec<_>>();
    assert_eq!(offsets(&pose), offsets(&flat));
}

#[test]
fn colliding_flattened_names_are_rejected() {
    // Nesting is joined with '_', so `a.b` and a sibling literally named `a_b` land on one
    // accessor. Rejecting beats emitting a header that will not compile.
    let mut reg = test_registry();
    let inner = message(
        "Inner",
        vec![field("b", "float64", ArrayKind::Scalar, 0)],
        Some("test"),
    );
    reg.add(inner, false).unwrap();
    let msg = message(
        "Clash",
        vec![
            field("a", "test/Inner", ArrayKind::Scalar, 0),
            field("a_b", "float64", ArrayKind::Scalar, 0),
        ],
        Some("test"),
    );
    let err = try_build(&msg, &reg).unwrap_err().to_string();
    assert!(err.contains("collide"), "{err}");
}

#[test]
fn rejected_schema_carries_its_reason() {
    let reg = test_registry();
    let err = try_build(reg.get("BadBool").unwrap(), &reg).unwrap_err();
    assert!(err.to_string().contains("bool[]"), "{err}");
}

#[test]
fn a_constant_may_carry_a_name_the_adapter_defines() {
    let mut msg = message(
        "C",
        vec![field("x", "float64", ArrayKind::Scalar, 0)],
        Some("test"),
    );
    msg.constants.push(Constant {
        name: "FINGERPRINT__".into(),
        type_name: "uint8".into(),
        value: ConstValue::Int(1),
    });
    let layout = try_build(&msg, &test_registry()).unwrap();
    assert_eq!(layout.constants[0].name, "FINGERPRINT__");
}

#[test]
fn fingerprint_travels_with_the_layout() {
    let reg = test_registry();
    let flat = flatten_message(reg.get("Pose").unwrap(), &reg).unwrap();
    assert_eq!(
        build_layout(&flat).unwrap().fingerprint,
        fingerprint(&flat.leaves, &flat.type_name())
    );
}

#[test]
fn a_root_field_may_carry_a_name_the_adapter_defines() {
    // The adapter's own members end in `__`, which a .msg field name cannot produce.
    let msg = message(
        "F",
        vec![field("size", "uint32", ArrayKind::Scalar, 0)],
        Some("test"),
    );
    let layout = try_build(&msg, &test_registry()).unwrap();
    assert_eq!(layout.root.placed[0].name(), "size");
}

#[test]
fn a_derived_accessor_cannot_collide_with_a_sibling_field() {
    // The allocator for `xs` is alloc__xs, distinct from a sibling named alloc_xs.
    let msg = message(
        "D",
        vec![
            field("xs", "float32", ArrayKind::Dynamic, 0),
            field("alloc_xs", "uint32", ArrayKind::Scalar, 0),
        ],
        Some("test"),
    );
    let layout = try_build(&msg, &test_registry()).unwrap();
    let names: HashSet<String> = layout.root.placed.iter().map(Placed::name).collect();
    assert_eq!(names, HashSet::from(["xs".into(), "alloc_xs".into()]));
}

#[test]
fn a_nested_dimension_field_still_flattens() {
    let mut reg = Registry::new();
    let dim = parse_msg("uint32 size\nfloat32[] xs\n", "Dim", None, Some("t")).unwrap();
    let arr = parse_msg("t/Dim[] dim\n", "Arr", None, Some("t")).unwrap();
    reg.add(dim, false).unwrap();
    reg.add(arr.clone(), false).unwrap();
    assert_eq!(try_build(&arr, &reg).unwrap().tier, "jagged");
}

#[test]
fn colliding_element_class_names_are_rejected() {
    // Camel-casing erases '_' against '.', so `foo_bar` and `foo.bar` both name FooBarElem:
    // silent aliasing in Python, a redefinition error in C++.
    let mut reg = Registry::new();
    for (name, text) in [
        ("Inner", "float32[] xs\n"),
        ("Foo", "t/Inner[] bar\n"),
        ("Top", "t/Inner[] foo_bar\nt/Foo[] foo\n"),
    ] {
        reg.add(parse_msg(text, name, None, Some("t")).unwrap(), false)
            .unwrap();
    }
    let err = try_build(reg.get("t/Top").unwrap(), &reg)
        .unwrap_err()
        .to_string();
    assert!(err.contains("FooBarElem"), "{err}");
}

/// `name` with `text` as its body, beside a one-field `t/Item` for it to nest.
fn layout_of(text: &str, name: &str) -> Result<Layout, LayoutError> {
    let mut reg = Registry::new();
    for (text, name) in [("uint8 v\n", "Item"), (text, name)] {
        reg.add(parse_msg(text, name, None, Some("t")).unwrap(), false)
            .unwrap();
    }
    try_build(reg.get(&format!("t/{name}")).unwrap(), &reg)
}

#[test]
fn a_name_the_adapter_defines_is_refused() {
    // C++ nests View, Builder and the element classes in a struct named after the message, and
    // the Python module rebinds them: either fails to compile or calls the wrong class.
    for (text, name) in [
        ("uint8 x\n", "View"),
        ("uint8 x\n", "Builder"),
        ("t/Item[] points\n", "PointsElem"),
        ("t/Item[] points\n", "PointsElemArray"),
        ("uint8 ABC=1\nuint8 value\n", "ABC"),
    ] {
        let err = layout_of(text, name).unwrap_err();
        assert!(err.0.contains(name), "{name}: {err}");
    }
    for name in ["Reader", "Writer", "Published", "NullWriter", "ElemArray"] {
        assert!(layout_of("t/Item[] points\n", name).is_ok(), "{name}");
    }
}
