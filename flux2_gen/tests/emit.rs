//! Generator logic behind the emitted adapters. The generated-code round trips live in Python.

mod common;

use common::test_registry;
use flux2_gen::emit_cpp::guard;
use flux2_gen::emit_ros::{emit_cpp, emit_py};
use flux2_gen::fingerprint::fingerprint;
use flux2_gen::flatten::flatten_message;
use flux2_gen::layout::build_layout;
use flux2_gen::parse::parse_msg;
use flux2_gen::text::{cpp_ident, py_ident, py_module};

#[test]
fn ros_bridge_refuses_a_fixed_length_mismatch() {
    // The frame's length is data; the schema's T[N] is not. The generated bridge must refuse a
    // frame that disagrees instead of clamping or half-copying. One throw/raise per
    // length-carrying fixed leaf: names and pair.
    let reg = test_registry();
    let flat = flatten_message(reg.get("FixedNames").unwrap(), &reg).unwrap();
    let layout = build_layout(&flat).unwrap();
    let cpp = emit_cpp(&layout, "test/msg/FixedNames.msg");
    assert!(!cpp.contains("std::min"));
    assert_eq!(cpp.matches("throw std::length_error").count(), 2);
    assert!(cpp.contains("#include <stdexcept>"));
    let py = emit_py(&layout, "test/msg/FixedNames.msg");
    assert_eq!(py.matches("raise ValueError").count(), 2);
}

#[test]
fn ros_bridge_checks_the_bytes_of_fixed_arrays_before_it_builds_elements() {
    // An element's string[2] and Tensor[2] are one descriptor each in the frame but two strings
    // and two tensors once built, so a stride-only budget lets a short frame build far more than
    // it holds. Elements without fixed arrays need no such check.
    let reg = test_registry();
    let emitted = |text: &str| {
        let msg = parse_msg(text, "Outer", None, Some("test")).unwrap();
        let layout = build_layout(&flatten_message(&msg, &reg).unwrap()).unwrap();
        (
            emit_cpp(&layout, "Outer.msg"),
            emit_py(&layout, "Outer.msg"),
        )
    };
    let (cpp, py) = emitted("FixedNames[] rosters\n");
    assert_eq!(cpp.matches("require(n").count(), 1);
    assert!(cpp.contains("const auto require"));
    assert_eq!(py.matches("require(len(").count(), 1);
    assert!(py.contains("def require"));
    let (cpp, py) = emitted("Tensor[] tensors\n");
    assert!(!cpp.contains("require"));
    assert!(!py.contains("require"));
}

#[test]
fn infinite_and_nan_constants_are_spelled_so_they_compile() {
    // rosidl reads `inf` and `nan`; Python's repr() spells them as names neither language has.
    let msg = parse_msg(
        "float64 UP=inf\nfloat32 DOWN=-inf\nfloat64 NONE=nan\nuint8 x\n",
        "Limits",
        None,
        Some("test"),
    )
    .unwrap();
    let layout = build_layout(&flatten_message(&msg, &test_registry()).unwrap()).unwrap();
    let cpp = flux2_gen::emit_cpp::emit(&layout, "Limits.msg");
    assert!(cpp.contains("UP = std::numeric_limits<double>::infinity();"));
    assert!(cpp.contains("DOWN = -std::numeric_limits<float>::infinity();"));
    assert!(cpp.contains("NONE = std::numeric_limits<double>::quiet_NaN();"));
    assert!(cpp.contains("#include <limits>"));
    let py = flux2_gen::emit_py::emit(&layout, "Limits.msg");
    assert!(py.contains("UP = float('inf')"));
    assert!(py.contains("DOWN = -float('inf')"));
    assert!(py.contains("NONE = float('nan')"));
}

#[test]
fn constants_stay_out_of_the_fingerprint() {
    // Adding a constant must not move the fingerprint: it is not wire data, so a peer built
    // before it was added still reads the same bytes and must still meet this one.
    let reg = test_registry();
    let body = "uint16 code\nfloat64[] values\n";
    let with_extra = format!("uint16 EXTRA = 9\n{body}");
    let flat_of = |text: &str| {
        let msg = parse_msg(text, "Status", None, Some("test")).unwrap();
        flatten_message(&msg, &reg).unwrap()
    };
    let (bare, withc) = (flat_of(body), flat_of(&with_extra));
    assert!(!withc.constants.is_empty() && bare.constants.is_empty());
    assert_eq!(bare.leaves, withc.leaves);
    assert_eq!(
        fingerprint(&bare.leaves, &bare.type_name()),
        fingerprint(&withc.leaves, &withc.type_name())
    );
}

#[test]
fn header_guards_keep_package_and_message_apart() {
    // Upper-casing the snake name gave a/Flux2Pose and a_flux2/Pose one guard, so including both
    // silently dropped the second header.
    let layout = |text: &str, name: &str, pkg: &str| {
        let msg = parse_msg(text, name, None, Some(pkg)).unwrap();
        build_layout(&flatten_message(&msg, &test_registry()).unwrap()).unwrap()
    };
    let (a, b) = (
        layout("uint8 x\n", "Flux2Pose", "a"),
        layout("uint8 x\n", "Pose", "a_flux2"),
    );
    assert_ne!(guard(&a, ""), guard(&b, ""));
    let (foo, foo_ros) = (
        layout("uint8 x\n", "Foo", "p"),
        layout("uint8 x\n", "FooRos", "p"),
    );
    assert_ne!(guard(&foo, "_ROS"), guard(&foo_ros, ""));
}

#[test]
fn names_are_escaped_where_the_language_needs_it_and_nowhere_else() {
    for kw in [
        "static_cast",
        "thread_local",
        "and_eq",
        "co_await",
        "requires",
        "linux",
        "errno",
    ] {
        assert_eq!(cpp_ident(kw), format!("{kw}_"));
    }
    assert_eq!(cpp_ident("data"), "data");
    // A field np or self needs no escape in Python: renaming it there alone made the Python
    // attribute differ from the C++ accessor and the ROS field.
    assert_eq!(py_ident("np"), "np");
    assert_eq!(py_ident("self"), "self");
    assert_eq!(py_ident("lambda"), "lambda_");
    assert_eq!(py_module("Global"), "global_");
    assert_eq!(py_module("PointCloud2"), "point_cloud2");
}

#[test]
fn a_tensor_message_is_matched_by_its_fields_alone() {
    // DLPack's fields in any order and under any package get the tensor accessors; one field
    // off in name, type or nesting, or one field more, and the message is an ordinary one.
    const BODY: &str = "uint8 dtype_code\nuint8 dtype_bits\nuint16 dtype_lanes\nint64[] shape\n\
                        int64[] strides\nuint64 byte_offset\nuint8[] data\n";
    let mut reg = test_registry();
    let dtype = parse_msg("uint8 code\n", "Dtype", None, Some("mine")).unwrap();
    reg.add(dtype, false).unwrap();
    let layout = |text: &str| {
        let msg = parse_msg(text, "T", None, Some("mine")).unwrap();
        build_layout(&flatten_message(&msg, &reg).unwrap()).unwrap()
    };
    let fixture = build_layout(&flatten_message(reg.get("DlTensor").unwrap(), &reg).unwrap());
    assert_eq!(fixture.unwrap().tensor_at(), layout(BODY).tensor_at());
    let mut reordered: Vec<&str> = BODY.lines().collect();
    reordered.reverse();
    let at = layout(&reordered.join("\n")).tensor_at().unwrap();
    assert_eq!(at.len(), 7);
    for off in [
        BODY.replace("uint8[] data", "byte[] data"),
        BODY.replace("uint16 dtype_lanes", "uint32 dtype_lanes"),
        BODY.replace("int64[] strides", "int64[2] strides"),
        BODY.replace("uint64 byte_offset", "uint64 offset"),
        format!("{BODY}uint32 extra\n"),
        BODY.replace("uint8 dtype_code\n", "Dtype dtype\n"),
    ] {
        assert_eq!(layout(&off).tensor_at(), None, "{off}");
    }
    let (py, cpp) = (
        flux2_gen::emit_py::emit(&layout(BODY), "T.msg"),
        flux2_gen::emit_cpp::emit(&layout(BODY), "T.msg"),
    );
    assert!(
        py.contains("TENSOR_AT__ = (") && py.contains("def alloc_tensor__(self, dtype, shape)")
    );
    assert!(cpp.contains("kTensorAt{") && cpp.contains("as_tensor__() const noexcept"));
    let plain = layout("int64[] shape\nuint8[] data\n");
    assert!(!flux2_gen::emit_py::emit(&plain, "T.msg").contains("tensor"));
    assert!(!flux2_gen::emit_cpp::emit(&plain, "T.msg").contains("Tensor"));
}
