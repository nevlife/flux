//! The optional ROS-message bridge for one schema: `msg_to_frame` / `frame_to_msg`.
//!
//! Kept in its own header and module because it is the one part of a generated adapter that has
//! to include the rosidl-generated message type. Both directions copy every field: this is for
//! code that already holds a ROS message object.

use std::collections::HashMap;
use std::fmt::Write;

use crate::dtypes::DType;
use crate::emit_cpp::{at, guard, namespace};
use crate::flatten::{DYNAMIC_COUNT, LeafKind, PathPart};
use crate::layout::{Block, DESC_ALIGN, DESC_SIZE, Layout, walk};
use crate::text::{cpp_ident, py_ident, py_module, snake};

/// `array.array` typecodes `rosidl_generator_py` uses for a primitive sequence. Bool sequences
/// are plain lists there.
fn array_typecode(dtype: DType) -> Option<char> {
    Some(match dtype {
        DType::U8 => 'B',
        DType::I8 => 'b',
        DType::U16 => 'H',
        DType::I16 => 'h',
        DType::U32 => 'I',
        DType::I32 => 'i',
        DType::U64 => 'Q',
        DType::I64 => 'q',
        DType::F32 => 'f',
        DType::F64 => 'd',
        DType::Bool => return None,
    })
}

fn split_type(type_name: &str) -> (&str, &str) {
    type_name.split_once('/').expect("a ROS type is pkg/Name")
}

fn access(base: &str, path: &[PathPart]) -> String {
    let mut out = base.to_owned();
    for part in path {
        match part {
            PathPart::Index(i) => write!(out, "[{i}]").expect("writing to a String"),
            PathPart::Name(n) => {
                out.push('.');
                out.push_str(n);
            }
        }
    }
    out
}

fn label(path: &[PathPart]) -> String {
    access("", path).trim_start_matches('.').to_owned()
}

/// Every nested ROS type the bridge has to name, after the message itself.
fn element_types(layout: &Layout) -> Vec<String> {
    let mut order = vec![layout.type_name()];
    for path in walk(&layout.root) {
        if let Some(et) = &path[path.len() - 1].leaf.elem_type
            && !order.contains(et)
        {
            order.push(et.clone());
        }
    }
    order
}

fn cpp_to_frame(block: &Block, out: &mut Vec<String>, ind: &str, msg: &str, bld: &str, d: usize) {
    for p in &block.placed {
        let n = cpp_ident(&p.name());
        let src = access(msg, &p.leaf.path);
        let count = if p.is_dynamic() {
            format!("{src}.size()")
        } else {
            String::new()
        };
        match p.kind() {
            LeafKind::Fixed if p.leaf.count == 1 => {
                out.push(format!("{ind}{bld}.set__{n}({src});"));
            }
            LeafKind::Fixed => out.push(format!(
                "{ind}{{ auto s = {bld}.{n}(); std::copy_n({src}.begin(), s.size(), s.begin()); }}"
            )),
            LeafKind::Column => out.push(format!(
                "{ind}{{ auto s = {bld}.alloc__{n}({src}.size()); \
                 std::copy_n({src}.begin(), s.size(), s.begin()); }}"
            )),
            LeafKind::String => out.push(format!("{ind}{bld}.set__{n}({src});")),
            LeafKind::StringArray => {
                let i = format!("i{d}");
                out.push(format!("{ind}{bld}.alloc__{n}({count});"));
                out.push(format!(
                    "{ind}for (std::size_t {i} = 0; {i} < {src}.size(); ++{i}) {{"
                ));
                out.push(format!("{ind}  {bld}.set__{n}({i}, {src}[{i}]);"));
                out.push(format!("{ind}}}"));
            }
            LeafKind::Stamp => out.push(format!(
                "{ind}{bld}.set__{n}__stamp({src}.sec, {src}.nanosec);"
            )),
            LeafKind::Header => {
                out.push(format!(
                    "{ind}{bld}.set__{n}__stamp({src}.stamp.sec, {src}.stamp.nanosec);"
                ));
                out.push(format!("{ind}{bld}.set__{n}__frame_id({src}.frame_id);"));
            }
            LeafKind::RecordColumn | LeafKind::Jagged => {
                let (i, arr, elem) = (format!("i{d}"), format!("a{d}"), format!("e{d}"));
                out.push(format!("{ind}{{"));
                out.push(format!("{ind}  auto {arr} = {bld}.alloc__{n}({count});"));
                out.push(format!(
                    "{ind}  for (std::size_t {i} = 0; {i} < {src}.size(); ++{i}) {{"
                ));
                out.push(format!("{ind}    auto {elem} = {arr}[{i}];"));
                let inner = format!("{ind}    ");
                cpp_to_frame(p.elem(), out, &inner, &format!("{src}[{i}]"), &elem, d + 1);
                out.push(format!("{ind}  }}"));
                out.push(format!("{ind}}}"));
            }
        }
    }
}

/// The bytes a writer lays out for one element: its stride, and the slots of every fixed array in
/// it. Building elements builds those arrays before any byte of them is read, so the copy checks
/// this first. Saturates.
fn min_bytes(block: &Block) -> u64 {
    block.placed.iter().fold(block.stride as u64, |sum, p| {
        let n = p.leaf.count as u64;
        let extra = match p.kind() {
            _ if p.is_dynamic() => 0,
            LeafKind::StringArray => n.saturating_mul(DESC_SIZE as u64),
            LeafKind::RecordColumn | LeafKind::Jagged => n.saturating_mul(min_bytes(p.elem())),
            _ => 0,
        };
        sum.saturating_add(extra)
    })
}

/// `min_bytes` of an element block when it is more than its stride, which the copy spends anyway.
fn fixed_arrays_need(elem: &Block) -> Option<u64> {
    let need = min_bytes(elem);
    (need > elem.stride as u64).then_some(need)
}

struct ToMsg<'a> {
    out: &'a mut Vec<String>,
    type_name: &'a str,
    /// Whether any copy spends the read budget, which is then declared.
    spends: bool,
    /// Whether any element copy checks the bytes its fixed arrays need, which is then declared.
    requires: bool,
}

impl ToMsg<'_> {
    fn spend(&mut self, ind: &str, bytes: &str) {
        self.out.push(format!("{ind}spend({bytes});"));
        self.spends = true;
    }

    fn string(&mut self, ind: &str, dst: &str, src: &str) {
        self.out.push(format!("{ind}{{"));
        self.out
            .push(format!("{ind}  const std::string_view s = {src};"));
        self.spend(&format!("{ind}  "), "s.size()");
        self.out.push(format!(
            "{ind}  {dst} = std::string(flux2::wire::until_nul(s));"
        ));
        self.out.push(format!("{ind}}}"));
    }

    fn cpp(&mut self, block: &Block, ind: &str, msg: &str, view: &str, depth: usize) {
        for p in &block.placed {
            let n = cpp_ident(&p.name());
            let dst = access(msg, &p.leaf.path);
            match p.kind() {
                LeafKind::Fixed if p.leaf.count == 1 => {
                    self.out.push(format!("{ind}{dst} = {view}.{n}();"));
                }
                // A frame byte is a bool only through as_bool: it may hold any value.
                LeafKind::Fixed if p.leaf.dtype == Some(DType::Bool) => self.out.push(format!(
                    "{ind}{{ auto s = {view}.{n}(); \
                     std::transform(s.begin(), s.end(), {dst}.begin(), flux2::wire::as_bool); }}"
                )),
                LeafKind::Fixed => self.out.push(format!(
                    "{ind}{{ auto s = {view}.{n}(); std::copy(s.begin(), s.end(), {dst}.begin()); }}"
                )),
                LeafKind::Column => {
                    let size = p.leaf.dtype.expect("columns have a dtype").size();
                    self.out.push(format!("{ind}{{"));
                    self.out.push(format!("{ind}  auto s = {view}.{n}();"));
                    let bytes = match size {
                        1 => "s.size()".to_owned(),
                        _ => format!("s.size() * {size}"),
                    };
                    self.spend(&format!("{ind}  "), &bytes);
                    self.out.push(format!("{ind}  {dst}.assign(s.begin(), s.end());"));
                    self.out.push(format!("{ind}}}"));
                }
                LeafKind::String => self.string(ind, &dst, &format!("{view}.{n}()")),
                LeafKind::Stamp => {
                    self.out.push(format!("{ind}{dst}.sec = {view}.{n}__sec();"));
                    self.out.push(format!("{ind}{dst}.nanosec = {view}.{n}__nanosec();"));
                }
                LeafKind::Header => {
                    self.out.push(format!("{ind}{dst}.stamp.sec = {view}.{n}__sec();"));
                    self.out.push(format!("{ind}{dst}.stamp.nanosec = {view}.{n}__nanosec();"));
                    let (dst, src) = (format!("{dst}.frame_id"), format!("{view}.{n}__frame_id()"));
                    self.string(ind, &dst, &src);
                }
                LeafKind::StringArray | LeafKind::RecordColumn | LeafKind::Jagged => {
                    let (i, cnt) = (format!("i{depth}"), format!("n{depth}"));
                    let inner = format!("{ind}  ");
                    self.out.push(format!("{ind}{{"));
                    self.out.push(format!("{ind}  const std::size_t {cnt} = {view}.{n}__size();"));
                    let stride = match p.kind() {
                        LeafKind::StringArray => DESC_SIZE,
                        _ => p.elem().stride,
                    };
                    if p.kind() != LeafKind::StringArray
                        && let Some(need) = fixed_arrays_need(p.elem())
                    {
                        self.out.push(format!("{inner}require({cnt}, {need}U);"));
                        self.requires = true;
                    }
                    self.spend(&inner, &format!("{cnt} * {stride}"));
                    if p.is_dynamic() {
                        self.out.push(format!("{ind}  {dst}.resize({cnt});"));
                    } else {
                        // A std::array has no resize, and a clamped copy is a silently wrong
                        // message: refuse a frame whose length disagrees with the schema's N.
                        let (count, tn, lbl) = (p.leaf.count, self.type_name, label(&p.leaf.path));
                        self.out.push(format!("{ind}  if ({cnt} != {count}) {{"));
                        self.out.push(format!(
                            "{ind}    throw std::length_error(\"flux2: {tn}.{lbl}: frame holds \" \
                             + std::to_string({cnt}) + \" elements, schema fixes {count}\");"
                        ));
                        self.out.push(format!("{ind}  }}"));
                    }
                    self.out.push(format!(
                        "{ind}  for (std::size_t {i} = 0; {i} < {cnt}; ++{i}) {{"
                    ));
                    let body = format!("{ind}    ");
                    if p.kind() == LeafKind::StringArray {
                        self.string(&body, &format!("{dst}[{i}]"), &format!("{view}.{n}({i})"));
                    } else {
                        let elem = format!("e{depth}");
                        self.out.push(format!("{body}auto {elem} = {view}.{n}({i});"));
                        self.cpp(p.elem(), &body, &format!("{dst}[{i}]"), &elem, depth + 1);
                    }
                    self.out.push(format!("{ind}  }}"));
                    self.out.push(format!("{ind}}}"));
                }
            }
        }
    }
}

pub fn emit_cpp(layout: &Layout, source: &str) -> String {
    let ns = namespace(layout);
    let snake_name = snake(&layout.name);
    let guard = guard(layout, "_ROS");
    let type_name = layout.type_name();
    let (pkg, name) = split_type(&type_name);
    let ros = format!("{pkg}::msg::{name}");
    let includes = element_types(layout)
        .iter()
        .fold(String::new(), |mut s, t| {
            let (p, n) = split_type(t);
            writeln!(s, "#include \"{p}/msg/{}.hpp\"", snake(n)).expect("writing to a String");
            s
        });
    let mut to_frame = Vec::new();
    cpp_to_frame(&layout.root, &mut to_frame, "  ", "m", "b", 0);
    let mut to_msg = Vec::new();
    let mut emitter = ToMsg {
        out: &mut to_msg,
        type_name: &type_name,
        spends: false,
        requires: false,
    };
    emitter.cpp(&layout.root, "  ", "m", "v", 0);
    let require = if emitter.requires {
        format!(
            r#"  const auto require = [&unread](std::size_t n, std::size_t each) {{
    if (n > unread / each) {{
      throw std::runtime_error("flux2: {type_name}: the frame is too short for its elements");
    }}
  }};
"#
        )
    } else {
        String::new()
    };
    let budget = if emitter.spends {
        format!(
            r#"  // A writer never points two descriptors at the same bytes, so a frame whose copy
  // would read more than it holds is built to multiply the copy: refuse it first.
  std::size_t unread = v.size__();
  const auto spend = [&unread](std::size_t n) {{
    if (n > unread) {{
      throw std::runtime_error("flux2: {type_name}: the frame reads the same bytes twice");
    }}
    unread -= n;
  }};
{require}"#
        )
    } else {
        String::new()
    };
    let to_frame = to_frame.join("\n");
    let to_msg = to_msg.join("\n");
    let name = &layout.name;
    format!(
        r#"// generated by flux2_gen from {source} -- do not edit.
#ifndef {guard}
#define {guard}

#include "{ns}/flux2/{snake_name}.hpp"
{includes}
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace {ns}::flux2_msg
{{

// Copy every field of a {type_name} into a loaned slot. This is the 1-copy path: the
// message owns its own buffers, so nothing can be aliased. Prefer writing into {name}::Builder
// directly when the data has not been built as a ROS message yet.
inline void msg_to_frame(const {ros} & m, {name}::Builder & b)
{{
{to_frame}
}}

// Copy a received frame back out into a ROS message. One copy per subscriber, which is the cost
// flux exists to avoid -- use {name}::View directly unless an existing signature needs
// the message object. A string stops at its first NUL, as rclpy's would.
inline void frame_to_msg(const {name}::View & v, {ros} & m)
{{
{budget}{to_msg}
  if (!v.ok__()) {{
    throw std::runtime_error("flux2: {type_name}: the frame does not match its schema");
  }}
}}

inline {ros} frame_to_msg(const {name}::View & v)
{{
  {ros} m;
  frame_to_msg(v, m);
  return m;
}}

}}  // namespace {ns}::flux2_msg

#endif  // {guard}
"#
    )
}

fn py_to_frame(block: &Block, out: &mut Vec<String>, ind: &str, msg: &str, bld: &str, d: usize) {
    for p in &block.placed {
        let n = py_ident(&p.name());
        let src = access(msg, &p.leaf.path);
        // rclpy holds a byte as a one-byte bytes object.
        let byte = p.leaf.ros_byte;
        match p.kind() {
            LeafKind::Fixed if p.leaf.count == 1 && byte => {
                out.push(format!("{ind}{bld}.{n} = {src}[0]"));
            }
            LeafKind::Fixed if p.leaf.count == 1 => out.push(format!("{ind}{bld}.{n} = {src}")),
            LeafKind::Fixed if byte => {
                out.push(format!("{ind}{bld}.{n}[:] = memoryview(b\"\".join({src}))"));
            }
            LeafKind::Fixed => out.push(format!("{ind}{bld}.{n}[:] = {src}")),
            LeafKind::Column if byte => out.push(format!(
                "{ind}{bld}.alloc__{n}(len({src}))[:] = memoryview(b\"\".join({src}))"
            )),
            LeafKind::Column => out.push(format!("{ind}{bld}.alloc__{n}(len({src}))[:] = {src}")),
            LeafKind::String | LeafKind::StringArray => {
                out.push(format!("{ind}{bld}.{n} = {src}"));
            }
            LeafKind::Stamp => out.push(format!(
                "{ind}{bld}.set__{n}__stamp({src}.sec, {src}.nanosec)"
            )),
            LeafKind::Header => {
                out.push(format!(
                    "{ind}{bld}.set__{n}__stamp({src}.stamp.sec, {src}.stamp.nanosec)"
                ));
                out.push(format!("{ind}{bld}.{n}__frame_id = {src}.frame_id"));
            }
            LeafKind::RecordColumn | LeafKind::Jagged => {
                let (src_v, arr, i, elem) = (
                    format!("_s{d}"),
                    format!("_a{d}"),
                    format!("_i{d}"),
                    format!("_e{d}"),
                );
                out.push(format!("{ind}{src_v} = {src}"));
                let count = if p.leaf.count == DYNAMIC_COUNT {
                    format!("len({src_v})")
                } else {
                    String::new()
                };
                out.push(format!("{ind}{arr} = {bld}.alloc__{n}({count})"));
                out.push(format!("{ind}for {i}, {elem} in enumerate({arr}):"));
                let inner = format!("{ind}    ");
                py_to_frame(
                    p.elem(),
                    out,
                    &inner,
                    &format!("{src_v}[{i}]"),
                    &elem,
                    d + 1,
                );
            }
        }
    }
}

struct PyToMsg<'a> {
    out: &'a mut Vec<String>,
    alias: &'a HashMap<String, String>,
    type_name: &'a str,
    /// Whether any copy spends the read budget, which is then declared.
    spends: bool,
    /// Whether any element copy checks the bytes its fixed arrays need, which is then declared.
    requires: bool,
}

impl PyToMsg<'_> {
    /// Spends what the copy that follows reads. `Reader.elems` checks the region and copies
    /// nothing, so the budget is spent before the bytes are.
    fn spend(&mut self, ind: &str, bytes: &str) {
        self.out.push(format!("{ind}spend({bytes})"));
        self.spends = true;
    }

    /// Bytes of the region the descriptor at `off` names, in elements of `size` bytes.
    fn region(&mut self, ind: &str, off: &str, size: usize) {
        let n = format!("r.elems({off}, {size}, {size})[1]");
        let bytes = if size == 1 {
            n
        } else {
            format!("{size} * {n}")
        };
        self.spend(ind, &bytes);
    }

    fn string_array(&mut self, ind: &str, off: &str, d: usize) {
        let desc = format!("_d{d}");
        self.out.push(format!(
            "{ind}{desc} = r.elems({off}, {DESC_SIZE}, {DESC_ALIGN})"
        ));
        self.spend(ind, &format!("{DESC_SIZE} * {desc}[1]"));
        self.spend(
            ind,
            &format!(
                "sum(r.elems({desc}[0] + {DESC_SIZE} * i, 1, 1)[1] for i in range({desc}[1]))"
            ),
        );
    }

    fn length_check(&mut self, ind: &str, seq: &str, count: usize, path: &[PathPart]) {
        let (tn, l) = (self.type_name, label(path));
        self.out.push(format!("{ind}if len({seq}) != {count}:"));
        self.out.push(format!(
            "{ind}    raise ValueError(f\"flux2: {tn}.{l}: frame holds {{len({seq})}} elements, \
             schema fixes {count}\")"
        ));
    }

    /// `base` reaches the block from the frame start: "" for the root, `_e0._at + ` in an element.
    fn emit(&mut self, block: &Block, ind: &str, msg: &str, view: &str, base: &str, d: usize) {
        for p in &block.placed {
            let n = py_ident(&p.name());
            let dst = access(msg, &p.leaf.path);
            let off = at(base, p.offset);
            // rclpy's C converter asserts on a byte that is not a one-byte bytes object, and on
            // a bool[N] item that is not a Python bool.
            let byte = p.leaf.ros_byte;
            let bytes_of = |raw: &str| format!("[bytes((x,)) for x in {raw}]");
            match p.kind() {
                // Same rule as the C++ side: a fixed-size array cannot hold a disagreeing frame
                // length, and rclpy's own assertion would fire too late and say too little.
                LeafKind::StringArray if !p.is_dynamic() => {
                    let tmp = format!("_t{d}");
                    self.string_array(ind, &off, d);
                    self.out.push(format!("{ind}{tmp} = {view}.{n}"));
                    self.length_check(ind, &tmp, p.leaf.count, &p.leaf.path);
                    self.out.push(format!("{ind}{dst} = {tmp}"));
                }
                LeafKind::Fixed if p.leaf.count == 1 && byte => {
                    self.out
                        .push(format!("{ind}{dst} = bytes(({view}.{n}.item(),))"));
                }
                // A numpy integer is not a PyLong; rosidl's C converter asserts on it.
                LeafKind::Fixed if p.leaf.count == 1 => {
                    self.out.push(format!("{ind}{dst} = {view}.{n}.item()"));
                }
                LeafKind::Column if byte => {
                    self.region(ind, &off, 1);
                    let raw = format!("{view}.{n}.tobytes()");
                    self.out.push(format!("{ind}{dst} = {}", bytes_of(&raw)));
                }
                LeafKind::Fixed if byte => {
                    let raw = format!("{view}.{n}.tobytes()");
                    self.out.push(format!("{ind}{dst} = {}", bytes_of(&raw)));
                }
                LeafKind::Fixed if p.leaf.dtype == Some(DType::Bool) => {
                    self.out.push(format!("{ind}{dst} = {view}.{n}.tolist()"));
                }
                LeafKind::Column => {
                    let dt = p.leaf.dtype.expect("columns have a dtype");
                    self.region(ind, &off, dt.size());
                    // From an ndarray rclpy's setter makes one Python call per element; from
                    // bytes it is one memcpy.
                    self.out.push(match array_typecode(dt) {
                        Some(tc) => {
                            format!("{ind}{dst} = array.array(\"{tc}\", {view}.{n}.tobytes())")
                        }
                        None => format!("{ind}{dst} = {view}.{n}.tolist()"),
                    });
                }
                // A view would pin the slot for as long as the message lives.
                LeafKind::Fixed => self.out.push(format!("{ind}{dst} = {view}.{n}.copy()")),
                LeafKind::String => {
                    self.region(ind, &off, 1);
                    self.out.push(format!("{ind}{dst} = {view}.{n}"));
                }
                LeafKind::StringArray => {
                    self.string_array(ind, &off, d);
                    self.out.push(format!("{ind}{dst} = {view}.{n}"));
                }
                LeafKind::Stamp => self
                    .out
                    .push(format!("{ind}{dst}.sec, {dst}.nanosec = {view}.{n}__stamp")),
                LeafKind::Header => {
                    self.out.push(format!(
                        "{ind}{dst}.stamp.sec, {dst}.stamp.nanosec = {view}.{n}__stamp"
                    ));
                    self.region(ind, &at(base, p.frame_id_offset()), 1);
                    self.out
                        .push(format!("{ind}{dst}.frame_id = {view}.{n}__frame_id"));
                }
                LeafKind::RecordColumn | LeafKind::Jagged => {
                    let (seq, lst, i, elem) = (
                        format!("_q{d}"),
                        format!("_l{d}"),
                        format!("_i{d}"),
                        format!("_e{d}"),
                    );
                    let et = p.leaf.elem_type.as_ref().expect("elements have a type");
                    let cls = &self.alias[et];
                    self.out.push(format!("{ind}{seq} = {view}.{n}"));
                    if let Some(need) = fixed_arrays_need(p.elem()) {
                        self.out.push(format!("{ind}require(len({seq}), {need})"));
                        self.requires = true;
                    }
                    self.spend(ind, &format!("len({seq}) * {}", p.elem().stride));
                    if !p.is_dynamic() {
                        self.length_check(ind, &seq, p.leaf.count, &p.leaf.path);
                    }
                    self.out
                        .push(format!("{ind}{lst} = [{cls}() for _ in range(len({seq}))]"));
                    self.out
                        .push(format!("{ind}for {i}, {elem} in enumerate({seq}):"));
                    let inner = format!("{ind}    ");
                    let (item, base) = (format!("{lst}[{i}]"), format!("{elem}._at + "));
                    self.emit(p.elem(), &inner, &item, &elem, &base, d + 1);
                    self.out.push(format!("{ind}{dst} = {lst}"));
                }
            }
        }
    }
}

pub fn emit_py(layout: &Layout, source: &str) -> String {
    let types = element_types(layout);
    let alias: HashMap<String, String> = types
        .iter()
        .enumerate()
        .map(|(i, t)| (t.clone(), format!("_Ros{i}")))
        .collect();
    let imports = types
        .iter()
        .map(|t| {
            let (pkg, name) = split_type(t);
            format!("from {pkg}.msg import {name} as {}", alias[t])
        })
        .collect::<Vec<_>>()
        .join("\n");
    let type_name = layout.type_name();
    let mut to_frame = Vec::new();
    py_to_frame(&layout.root, &mut to_frame, "    ", "m", "b", 0);
    let mut to_msg = Vec::new();
    let mut emitter = PyToMsg {
        out: &mut to_msg,
        alias: &alias,
        type_name: &type_name,
        spends: false,
        requires: false,
    };
    emitter.emit(&layout.root, "    ", "m", "v", "", 0);
    let require = if emitter.requires {
        format!(
            r#"
    def require(n, each):
        if n * each > unread:
            raise WireError("flux2: {type_name}: the frame is too short for its elements")
"#
        )
    } else {
        String::new()
    };
    let (wire_error, budget) = if emitter.spends {
        (
            "\nfrom flux2.wire import WireError\n".to_owned(),
            format!(
                r#"    # A writer never points two descriptors at the same bytes, so a frame whose copy
    # would read more than it holds is built to multiply the copy: refuse it first.
    r, unread = v._r, len(v._r)

    def spend(n):
        nonlocal unread
        if n > unread:
            raise WireError("flux2: {type_name}: the frame reads the same bytes twice")
        unread -= n
{require}
"#
            ),
        )
    } else {
        (String::new(), String::new())
    };
    let to_frame = to_frame.join("\n");
    let to_msg = to_msg.join("\n");
    let root = &alias[&type_name];
    let module = py_module(&layout.name);
    format!(
        r#""""ROS message bridge for {type_name}.

generated by flux2_gen from {source} -- do not edit.

Importing this module pulls in the rclpy message type; the base adapter does not, so code that
never converts a ROS message object needs neither this module nor rclpy.
"""

import array
{wire_error}
{imports}

from .{module} import Builder, View  # noqa: F401

MESSAGE = {root}


def msg_to_frame(m, b):
    """Copy every field of a {type_name} into a Builder over a loaned slot.

    The 1-copy path: the message owns its own buffers, so nothing can be aliased. Write into the
    Builder directly when the data has not been built as a ROS message yet.
    """
{to_frame}
    return b


def frame_to_msg(v, m=None):
    """Copy a received frame back out into a ROS message.

    One copy per subscriber, which is the cost flux exists to avoid -- read the View directly
    unless an existing signature needs the message object.
    """
    if m is None:
        m = {root}()
{budget}{to_msg}
    return m
"#
    )
}
