//! Fingerprint: 64-bit, stable, nesting-invariant, layout-sensitive.
//!
//! Relational tests hold for any encoding; GOLDEN pins the bytes so that drift, visible only to
//! a peer built from an older tree, has to be an explicit edit.

mod common;

use std::collections::HashMap;
use std::fmt::Write;

use common::{field, message, ros_corpus, ros_msg_dirs, test_registry};
use flux2_gen::dtypes::ros_primitive;
use flux2_gen::fingerprint::{FINGERPRINT_SCHEMA_VERSION, canonical, fingerprint};
use flux2_gen::flatten::{FlatResult, Leaf, flatten_message};
use flux2_gen::model::{ArrayKind, Message};
use flux2_gen::parse::{Registry, resolve};

/// One entry per leaf kind. These values are the wire contract: regenerate only together with
/// a `FINGERPRINT_SCHEMA_VERSION` bump.
const GOLDEN: &[(&str, &str, u64)] = &[
    (
        "Pose",
        "v5#test/Pose;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1",
        0x355D_5F3F_A5A9_D96A,
    ),
    (
        "FlatPose",
        "v5#test/FlatPose;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1;fixed:f64:1",
        0x8025_D4B8_1455_B91C,
    ),
    (
        "Vec3",
        "v5#test/Vec3;fixed:f64:1;fixed:f64:1;fixed:f64:1",
        0xF331_315E_A0FF_EB11,
    ),
    (
        "PointCloud",
        "v5#test/PointCloud;column:f32:0;column:f32:0;column:f32:0;fixed:u32:1;fixed:u32:1",
        0x438B_2D5E_E8C0_D2C8,
    ),
    (
        "Labeled",
        "v5#test/Labeled;string:-:1;column:f64:0",
        0xCD34_AB0E_7FED_FE3B,
    ),
    (
        "NestedString",
        "v5#test/NestedString;string:-:1;column:f64:0",
        0x6125_9AAF_57E2_7568,
    ),
    (
        "Trajectory",
        "v5#test/Trajectory;record:-:0{fixed:f64:1,fixed:f64:1,fixed:f64:1,fixed:f64:1,fixed:f64:1,fixed:f64:1}",
        0x6189_DBC3_6C31_62D4,
    ),
    (
        "TensorList",
        "v5#test/TensorList;jagged:-:0{column:u32:0,column:f32:0}",
        0x365E_F7BD_07FF_3A69,
    ),
    (
        "Tensor",
        "v5#test/Tensor;column:u32:0;column:f32:0",
        0x1917_DE80_DD60_7226,
    ),
    // A bare Time tokenizes as its two fields, so nested and top-level stamps hash alike.
    (
        "Event",
        "v5#test/Event;fixed:i32:1;fixed:u32:1;sarray:-:0;column:f32:0",
        0x53DE_C4CA_6B7E_CB35,
    ),
    (
        "Names",
        "v5#test/Names;sarray:-:4;sarray:-:0",
        0x57DC_BC29_7E53_A209,
    ),
];

fn flat(reg: &Registry, name: &str) -> FlatResult {
    flatten_message(reg.get(name).unwrap(), reg).unwrap()
}

fn fp(reg: &Registry, name: &str) -> u64 {
    let r = flat(reg, name);
    fingerprint(&r.leaves, &r.type_name())
}

fn leaves_of(msg: &Message, reg: &Registry) -> Vec<Leaf> {
    flatten_message(msg, reg).unwrap().leaves
}

fn items(type_name: &str, kind: ArrayKind, size: usize) -> Message {
    message("M", vec![field("items", type_name, kind, size)], None)
}

#[test]
fn golden_canonical_and_fingerprint() {
    let reg = test_registry();
    for &(name, text, digest) in GOLDEN {
        let r = flat(&reg, name);
        assert_eq!(canonical(&r.leaves, &r.type_name()), text, "{name}");
        assert_eq!(fingerprint(&r.leaves, &r.type_name()), digest, "{name}");
    }
}

#[test]
fn canonical_carries_the_schema_version() {
    let leaves = flat(&test_registry(), "Pose").leaves;
    let prefix = format!("v{FINGERPRINT_SCHEMA_VERSION}#p/M;");
    assert!(canonical(&leaves, "p/M").starts_with(&prefix));
    assert_eq!(FINGERPRINT_SCHEMA_VERSION, 5);
}

#[test]
fn fingerprint_is_64bit_and_stable() {
    let reg = test_registry();
    assert_eq!(fp(&reg, "Pose"), fp(&reg, "Pose"));
}

#[test]
fn nesting_is_invariant() {
    // Pose (nested Vec3 x2) and FlatPose (six flat float64) put the same bytes on the wire.
    let reg = test_registry();
    let (pose, flat) = (flat(&reg, "Pose").leaves, flat(&reg, "FlatPose").leaves);
    assert_eq!(canonical(&pose, "p/Same"), canonical(&flat, "p/Same"));
    assert_eq!(fingerprint(&pose, "p/Same"), fingerprint(&flat, "p/Same"));
}

#[test]
fn type_name_separates_identical_layouts() {
    // Point and Vector3 share a layout, but ROS 2 never connects them on one topic.
    let leaves = flat(&test_registry(), "Vec3").leaves;
    assert_ne!(
        fingerprint(&leaves, "geometry_msgs/msg/Point"),
        fingerprint(&leaves, "geometry_msgs/msg/Vector3")
    );
    assert_eq!(fingerprint(&leaves, "p/M"), fingerprint(&leaves, "p/M"));
}

#[test]
fn different_layout_differs() {
    let reg = test_registry();
    assert_ne!(fp(&reg, "Pose"), fp(&reg, "PointCloud"));
    assert_ne!(fp(&reg, "Pose"), fp(&reg, "Trajectory"));
    assert_ne!(fp(&reg, "Labeled"), fp(&reg, "PointCloud"));
}

#[test]
fn jagged_element_structure_differentiates() {
    let reg = test_registry();
    // {u32[], f32[]} against {string, f64[]}.
    let a = leaves_of(&items("Tensor", ArrayKind::Dynamic, 0), &reg);
    let b = leaves_of(&items("Labeled", ArrayKind::Dynamic, 0), &reg);
    assert_ne!(fingerprint(&a, "p/M"), fingerprint(&b, "p/M"));
}

#[test]
fn jagged_fixed_size_differentiates() {
    let reg = test_registry();
    let mut fps: Vec<u64> = [
        (ArrayKind::Dynamic, 0),
        (ArrayKind::Fixed, 2),
        (ArrayKind::Fixed, 3),
    ]
    .into_iter()
    .map(|(kind, size)| fingerprint(&leaves_of(&items("Labeled", kind, size), &reg), "p/M"))
    .collect();
    fps.sort_unstable();
    fps.dedup();
    assert_eq!(fps.len(), 3);
}

#[test]
fn jagged_dynamic_does_not_collide_with_n_of_one() {
    let reg = test_registry();
    let fa = leaves_of(&items("Labeled", ArrayKind::Dynamic, 0), &reg);
    let fb = leaves_of(&items("Labeled", ArrayKind::Fixed, 1), &reg);
    assert_eq!(
        canonical(&fa, "p/M"),
        "v5#p/M;jagged:-:0{string:-:1,column:f64:0}"
    );
    assert_eq!(
        canonical(&fb, "p/M"),
        "v5#p/M;jagged:-:1{string:-:1,column:f64:0}"
    );
    assert_ne!(fingerprint(&fa, "p/M"), fingerprint(&fb, "p/M"));
}

#[test]
fn fixed_and_dynamic_string_arrays_differ() {
    let reg = test_registry();
    let tags = |kind, size| message("M", vec![field("tags", "string", kind, size)], None);
    let fo = leaves_of(&tags(ArrayKind::Fixed, 1), &reg);
    let fd = leaves_of(&tags(ArrayKind::Dynamic, 0), &reg);
    assert_eq!(canonical(&fo, "p/M"), "v5#p/M;sarray:-:1");
    assert_eq!(canonical(&fd, "p/M"), "v5#p/M;sarray:-:0");
    assert_ne!(fingerprint(&fo, "p/M"), fingerprint(&fd, "p/M"));
}

/// Element type of a shape item: a primitive token or base name, or a nested message's items.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
enum Inner {
    Prim(String),
    Msg(Vec<Item>),
}

/// (element, array kind, size)
type Item = (Inner, String, usize);

/// The wire shape read off the parsed model, never via `flatten_message` (a bug on both sides
/// is invisible). Copies only design rules: char/byte = uint8, scalar nesting splices, fixed
/// message arrays expand. Array kind and size stay uninterpreted. `None` is unresolvable here,
/// which is the parser's business, not this test's.
fn model_shape(msg: &Message, reg: &Registry, depth: usize) -> Option<Vec<Item>> {
    if depth > 32 {
        return None;
    }
    let mut out = Vec::new();
    for f in &msg.fields {
        let inner = match resolve(reg, &f.type_name, msg.package.as_deref()).ok()? {
            None => Inner::Prim(
                ros_primitive(&f.type_name)
                    .map_or_else(|| f.type_name.clone(), |d| d.token().into()),
            ),
            Some(nested) => Inner::Msg(model_shape(&nested, reg, depth + 1)?),
        };
        match (f.kind, inner) {
            (ArrayKind::Scalar, Inner::Msg(sub)) => out.extend(sub),
            (ArrayKind::Fixed, Inner::Msg(sub)) => {
                for _ in 0..f.size {
                    out.extend(sub.iter().cloned());
                }
            }
            (kind, inner) => out.push((inner, format!("{kind:?}"), f.size)),
        }
    }
    Some(out)
}

/// Qualified key -> (flattened, model shape), for every corpus schema that gets an adapter.
fn corpus() -> HashMap<String, (FlatResult, Vec<Item>)> {
    let reg = ros_corpus();
    let mut out = HashMap::new();
    // A bare key aliases an entry already visited under its qualified key.
    for (key, msg) in reg.messages().filter(|(k, _)| k.contains('/')) {
        let (Ok(flat), Some(shape)) = (flatten_message(msg, &reg), model_shape(msg, &reg, 0))
        else {
            continue;
        };
        if flat.rejected.is_none() {
            out.insert(key.clone(), (flat, shape));
        }
    }
    out
}

#[test]
fn the_layout_fingerprint_is_a_function_of_the_shape_across_the_ros_corpus() {
    // A shared fingerprint shares a segment, so it must mean a shared shape. One type name for
    // all, so only the layout separates them.
    if ros_msg_dirs().is_empty() {
        eprintln!("skipped: no installed ROS messages to sweep");
        return;
    }
    let mut buckets: HashMap<u64, HashMap<Vec<Item>, Vec<String>>> = HashMap::new();
    for (key, (flat, shape)) in corpus() {
        buckets
            .entry(fingerprint(&flat.leaves, "p/M"))
            .or_default()
            .entry(shape)
            .or_default()
            .push(key);
    }
    let mut report = String::new();
    for (digest, by_shape) in &buckets {
        if by_shape.len() > 1 {
            let firsts: Vec<&String> = by_shape.values().filter_map(|g| g.iter().min()).collect();
            let _ = write!(report, "0x{digest:016X} -> {firsts:?}; ");
        }
    }
    assert!(
        report.is_empty(),
        "distinct shapes share one layout fingerprint, so flux would let them share a segment: \
         {report}"
    );
}

#[test]
fn the_named_fingerprint_separates_every_type_in_the_ros_corpus() {
    if ros_msg_dirs().is_empty() {
        eprintln!("skipped: no installed ROS messages to sweep");
        return;
    }
    let mut seen: HashMap<u64, String> = HashMap::new();
    for (key, (flat, _)) in corpus() {
        let digest = fingerprint(&flat.leaves, &flat.type_name());
        if let Some(prev) = seen.get(&digest) {
            assert_eq!(
                prev, &key,
                "{key} and {prev} share fingerprint 0x{digest:016X}"
            );
        }
        seen.insert(digest, key);
    }
    assert!(seen.len() > 100, "expected a substantial corpus to sweep");
}
