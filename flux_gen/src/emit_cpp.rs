//! A header-only C++ adapter for one schema.
//!
//! The header defines `<pkg>::flux_msg::<Msg>` carrying the fingerprint, a View over a received
//! frame and a Builder over a loaned slot. Every accessor is one call into flux/wire.hpp at a
//! constant offset computed here, so there is nothing to keep in sync but the offsets.

use std::fmt::Write;

use crate::dtypes::{DType, ros_primitive};
use crate::flatten::{DYNAMIC_COUNT, LeafKind};
use crate::layout::{Block, Layout, Placed, elem_class_name};
use crate::model::{ConstValue, Constant};
use crate::text::{cpp_ident, py_float};

pub fn cpp_type(dtype: DType) -> &'static str {
    match dtype {
        DType::Bool => "bool",
        DType::U8 => "std::uint8_t",
        DType::I8 => "std::int8_t",
        DType::U16 => "std::uint16_t",
        DType::I16 => "std::int16_t",
        DType::U32 => "std::uint32_t",
        DType::I32 => "std::int32_t",
        DType::U64 => "std::uint64_t",
        DType::I64 => "std::int64_t",
        DType::F32 => "float",
        DType::F64 => "double",
    }
}

/// The offset expression of `off` inside a block reached through `base` ("" for the root).
pub fn at(base: &str, off: usize) -> String {
    if base.is_empty() {
        off.to_string()
    } else {
        format!("{base}{off}")
    }
}

fn cpp_literal(c: &Constant) -> (String, String) {
    match &c.value {
        ConstValue::Str(s) => (
            "const char *".into(),
            format!("\"{}\"", s.replace('\\', "\\\\").replace('"', "\\\"")),
        ),
        ConstValue::Bool(b) => ("bool".into(), b.to_string()),
        value => {
            let dt = ros_primitive(&c.type_name).expect("a numeric constant has a primitive type");
            // An unsuffixed decimal literal is signed: 2^63 and up does not fit, and -2^63 is the
            // negation of one that does not.
            let lit = match value {
                ConstValue::Float(f) if !f.is_finite() => {
                    let t = cpp_type(dt);
                    if f.is_nan() {
                        format!("std::numeric_limits<{t}>::quiet_NaN()")
                    } else {
                        let sign = if *f < 0.0 { "-" } else { "" };
                        format!("{sign}std::numeric_limits<{t}>::infinity()")
                    }
                }
                ConstValue::Float(f) if c.type_name == "float32" => format!("{}f", py_float(*f)),
                ConstValue::Float(f) => py_float(*f),
                ConstValue::Int(i) if dt == DType::U64 => format!("{i}ull"),
                ConstValue::Int(i) if dt == DType::I64 && *i == i128::from(i64::MIN) => {
                    "(-9223372036854775807ll - 1)".into()
                }
                ConstValue::Int(i) => i.to_string(),
                _ => unreachable!("strings and bools are matched above"),
            };
            (cpp_type(dt).to_owned(), lit)
        }
    }
}

fn constants(layout: &Layout) -> String {
    if layout.constants.is_empty() {
        return String::new();
    }
    let mut lines = Vec::new();
    for c in &layout.constants {
        let (t, lit) = cpp_literal(c);
        lines.push(format!(
            "  static constexpr {t} {} = {lit};",
            cpp_ident(&c.name)
        ));
    }
    format!("\n{}\n", lines.join("\n"))
}

/// How a block's class reaches its Reader or Writer: through a member value (root) or a
/// pointer (element blocks).
struct Through<'a> {
    base: &'a str,
    recv: &'a str,
    owner: &'a str,
}

const IND: &str = "    ";

fn view_accessors(block: &Block, r: &Through, out: &mut Vec<String>, prefix: &[&Placed]) {
    let Through { base, recv, owner } = r;
    for p in &block.placed {
        let n = cpp_ident(&p.name());
        let o = p.offset;
        match p.kind() {
            LeafKind::Fixed => {
                let t = cpp_type(p.leaf.dtype.expect("fixed leaves have a dtype"));
                if p.leaf.count == 1 {
                    out.push(format!(
                        "{IND}{t} {n}() const noexcept {{ return {recv}get<{t}>({}); }}",
                        at(base, o)
                    ));
                } else {
                    out.push(format!(
                        "{IND}flux::wire::Span<const {t}> {n}() const noexcept \
                         {{ return {recv}block<{t}>({}, {}); }}",
                        at(base, o),
                        p.leaf.count
                    ));
                }
            }
            LeafKind::Column => {
                let t = cpp_type(p.leaf.dtype.expect("columns have a dtype"));
                out.push(format!(
                    "{IND}flux::wire::Span<const {t}> {n}() const noexcept \
                     {{ return {recv}span<{t}>({}); }}",
                    at(base, o)
                ));
                // Where the GPU reads it, when the publisher wrote it there (alloc_device__).
                out.push(format!(
                    "{IND}flux::wire::DeviceSpan<const {t}> {n}__device() const noexcept \
                     {{ return {recv}device<{t}>({}); }}",
                    at(base, o)
                ));
            }
            LeafKind::String => out.push(format!(
                "{IND}std::string_view {n}() const noexcept {{ return {recv}str({}); }}",
                at(base, o)
            )),
            LeafKind::StringArray => {
                out.push(format!(
                    "{IND}std::size_t {n}__size() const noexcept \
                     {{ return {recv}len({}, sizeof(flux::wire::Desc), \
                     alignof(flux::wire::Desc)); }}",
                    at(base, o)
                ));
                out.push(format!(
                    "{IND}std::string_view {n}(std::size_t i) const noexcept \
                     {{ return {recv}str_at({}, i); }}",
                    at(base, o)
                ));
            }
            LeafKind::Stamp | LeafKind::Header => {
                out.push(format!(
                    "{IND}std::int32_t {n}__sec() const noexcept \
                     {{ return {recv}get<std::int32_t>({}); }}",
                    at(base, o)
                ));
                out.push(format!(
                    "{IND}std::uint32_t {n}__nanosec() const noexcept \
                     {{ return {recv}get<std::uint32_t>({}); }}",
                    at(base, o + 4)
                ));
                if p.kind() == LeafKind::Header {
                    out.push(format!(
                        "{IND}std::string_view {n}__frame_id() const noexcept \
                         {{ return {recv}str({}); }}",
                        at(base, p.frame_id_offset())
                    ));
                }
            }
            LeafKind::RecordColumn | LeafKind::Jagged => {
                let mut path = prefix.to_vec();
                path.push(p);
                let cls = elem_class_name(&path);
                let e = p.elem();
                out.push(format!(
                    "{IND}std::size_t {n}__size() const noexcept \
                     {{ return {recv}len({}, {}, {}); }}",
                    at(base, o),
                    e.stride,
                    e.align
                ));
                out.push(format!(
                    "{IND}{cls} {n}(std::size_t i) const noexcept \
                     {{ return {cls}({owner}, {recv}elem({}, i, {}, {})); }}",
                    at(base, o),
                    e.stride,
                    e.align
                ));
            }
        }
    }
}

/// A column's Builder methods: in the slot or the arena, in the GPU arena, or adopted.
fn column_builder(p: &Placed, n: &str, at: &str, recv: &str, out: &mut Vec<String>) {
    let t = cpp_type(p.leaf.dtype.expect("columns have a dtype"));
    out.push(format!(
        "{IND}flux::wire::Span<{t}> alloc__{n}(std::size_t n) noexcept \
         {{ return {recv}alloc<{t}>({at}, n); }}"
    ));
    // In the GPU arena, on a publisher that declared a stream: write it on that stream.
    out.push(format!(
        "{IND}flux::wire::DeviceSpan<{t}> alloc_device__{n}(std::size_t n) noexcept \
         {{ return {recv}alloc_device<{t}>({at}, n); }}"
    ));
    // A container filled under flux::ArenaScope, without a copy.
    out.push(format!(
        "{IND}template <class C> bool adopt__{n}(C && c) \
         {{ static_assert(std::is_same_v<typename std::decay_t<C>::value_type, {t}>); \
         return {recv}adopt({at}, std::forward<C>(c)); }}"
    ));
}

fn builder_accessors(block: &Block, w: &Through, out: &mut Vec<String>, prefix: &[&Placed]) {
    let Through { base, recv, owner } = w;
    for p in &block.placed {
        let n = cpp_ident(&p.name());
        let o = p.offset;
        match p.kind() {
            LeafKind::Fixed => {
                let t = cpp_type(p.leaf.dtype.expect("fixed leaves have a dtype"));
                if p.leaf.count == 1 {
                    out.push(format!(
                        "{IND}void set__{n}({t} v) noexcept {{ {recv}put<{t}>({}, v); }}",
                        at(base, o)
                    ));
                } else {
                    out.push(format!(
                        "{IND}flux::wire::Span<{t}> {n}() noexcept \
                         {{ return {recv}block<{t}>({}, {}); }}",
                        at(base, o),
                        p.leaf.count
                    ));
                }
            }
            LeafKind::Column => column_builder(p, &n, &at(base, o), recv, out),
            LeafKind::String => out.push(format!(
                "{IND}void set__{n}(std::string_view s) noexcept {{ {recv}put_str({}, s); }}",
                at(base, o)
            )),
            LeafKind::StringArray => {
                // A fixed T[N] takes no count: N is the schema's, so a wrong one cannot be
                // written.
                if p.leaf.count == DYNAMIC_COUNT {
                    out.push(format!(
                        "{IND}void alloc__{n}(std::size_t n) noexcept \
                         {{ {recv}alloc_strs({}, n); }}",
                        at(base, o)
                    ));
                } else {
                    out.push(format!(
                        "{IND}void alloc__{n}() noexcept {{ {recv}alloc_strs({}, {}); }}",
                        at(base, o),
                        p.leaf.count
                    ));
                }
                out.push(format!(
                    "{IND}void set__{n}(std::size_t i, std::string_view s) noexcept \
                     {{ {recv}put_str_at({}, i, s); }}",
                    at(base, o)
                ));
            }
            LeafKind::Stamp | LeafKind::Header => {
                out.push(format!(
                    "{IND}void set__{n}__stamp(std::int32_t sec, std::uint32_t nanosec) \
                     noexcept {{ {recv}put<std::int32_t>({}, sec); \
                     {recv}put<std::uint32_t>({}, nanosec); }}",
                    at(base, o),
                    at(base, o + 4)
                ));
                if p.kind() == LeafKind::Header {
                    out.push(format!(
                        "{IND}void set__{n}__frame_id(std::string_view s) noexcept \
                         {{ {recv}put_str({}, s); }}",
                        at(base, p.frame_id_offset())
                    ));
                }
            }
            LeafKind::RecordColumn | LeafKind::Jagged => {
                let mut path = prefix.to_vec();
                path.push(p);
                let cls = elem_class_name(&path);
                let e = p.elem();
                let (param, n_arg) = if p.leaf.count == DYNAMIC_COUNT {
                    ("std::size_t n".to_owned(), "n".to_owned())
                } else {
                    (String::new(), p.leaf.count.to_string())
                };
                out.push(format!(
                    "{IND}{cls}Array alloc__{n}({param}) noexcept \
                     {{ return {cls}Array({owner}, {recv}alloc_elems({}, {n_arg}, {}, {}), \
                     {n_arg}); }}",
                    at(base, o),
                    e.stride,
                    e.align
                ));
            }
        }
    }
}

/// Element View/Builder pairs, innermost first so each is declared before its user.
fn elem_classes(block: &Block, out: &mut String, prefix: &[&Placed]) {
    for p in &block.placed {
        let Some(e) = &p.elem else { continue };
        let mut path = prefix.to_vec();
        path.push(p);
        elem_classes(e, out, &path);
        let cls = elem_class_name(&path);
        let dotted = path.iter().map(|x| x.name()).collect::<Vec<_>>().join(".");
        let (stride, align) = (e.stride, e.align);
        let mut body = Vec::new();
        let r = Through {
            base: "at_ + ",
            recv: "r_->",
            owner: "*r_",
        };
        view_accessors(e, &r, &mut body, &path);
        let body = body.join("\n");
        write!(
            out,
            r"
  // one element of `{dotted}`
  class {cls}
  {{
public:
    static constexpr std::size_t kStride = {stride};
    static constexpr std::size_t kAlign = {align};
    {cls}(const flux::wire::Reader & r, std::size_t at) noexcept : r_(&r), at_(at) {{}}
{body}

private:
    const flux::wire::Reader * r_;
    std::size_t at_;
  }};"
        )
        .expect("writing to a String");
        let mut bbody = Vec::new();
        let w = Through {
            base: "at_ + ",
            recv: "w_->",
            owner: "*w_",
        };
        builder_accessors(e, &w, &mut bbody, &path);
        let bbody = bbody.join("\n");
        write!(
            out,
            r"
  class {cls}Builder
  {{
public:
    {cls}Builder(flux::wire::Writer & w, std::size_t at) noexcept : w_(&w), at_(at) {{}}
{bbody}

private:
    flux::wire::Writer * w_;
    std::size_t at_;
  }};

  class {cls}Array
  {{
public:
    {cls}Array() = default;
    {cls}Array(flux::wire::Writer & w, std::size_t at, std::size_t n) noexcept
    : w_(&w), at_(at), n_(n) {{}}
    std::size_t size() const noexcept {{ return n_; }}
    {cls}Builder operator[](std::size_t i) const noexcept
    {{ return {cls}Builder(*w_, at_ + i * {stride}); }}

private:
    flux::wire::Writer * w_ = nullptr;
    std::size_t at_ = 0;
    std::size_t n_ = 0;
  }};"
        )
        .expect("writing to a String");
    }
}

pub fn namespace(layout: &Layout) -> &str {
    layout.package.as_deref().unwrap_or("flux_msg")
}

/// Include guard of a generated header. The message name keeps its case: upper-casing its snake
/// form would give `a/FluxPose` and `a_flux/Pose` the same guard.
pub fn guard(layout: &Layout, suffix: &str) -> String {
    format!(
        "{}_FLUX_{}{suffix}_HPP",
        namespace(layout).to_uppercase(),
        layout.name
    )
}

/// Registered as the header loads, so a channel of the type knows where `header.stamp` sits from the
/// fingerprint alone.
fn stamp_registration(layout: &Layout) -> &'static str {
    // The first line lets a publisher of this type that declares a stream keep host slots, for
    // alloc_device__; the second lets a channel of it report header.stamp to trace collectors.
    if layout.stamp_offset() < 0 {
        return "\n  inline static const bool kLayoutRegistered__ =\n    \
                (flux_layout_register(kFingerprint), true);";
    }
    "\n  inline static const bool kLayoutRegistered__ =\n    \
     (flux_layout_register(kFingerprint), true);\n  inline static const bool kStampRegistered__ \
     =\n    (flux_stamp_offset_register(kFingerprint, kStampOffset), true);"
}

/// A tensor message's View accessor and Builder methods onto theirs, and its struct constant;
/// empty for the rest.
fn tensor(layout: &Layout, view: &mut Vec<String>, builder: &mut Vec<String>) -> String {
    let Some(at) = layout.tensor_at() else {
        return String::new();
    };
    view.push(
        "    // `data` as the tensor the other fields describe. Null data (and !ok__()) if they put an
    // element outside `data` or off its alignment, or name a dtype flux does not read.
    flux::wire::Tensor as_tensor__() const noexcept { return r_.tensor(kTensorAt); }"
            .into(),
    );
    builder.push(
        "    // Lays `data` out as a C-contiguous tensor of `shape` on a 64-byte boundary and writes the
    // fields that describe it. Null (and !ok__()) if it does not fit.
    void * alloc_tensor__(
      flux::wire::DlType t, flux::wire::Span<const std::int64_t> shape) noexcept
    {
      return w_.tensor(kTensorAt, t, shape);
    }
    template <class T>
    flux::wire::Span<T> alloc_tensor__(flux::wire::Span<const std::int64_t> shape) noexcept
    {
      return w_.tensor<T>(kTensorAt, shape);
    }"
        .into(),
    );
    let at = at.map(|o| o.to_string()).join(", ");
    format!("\n  static constexpr flux::wire::TensorAt kTensorAt{{{at}}};")
}

/// The C++ header text for a layout. `source` is the .msg path, for the banner.
#[allow(clippy::too_many_lines)]
pub fn emit(layout: &Layout, source: &str) -> String {
    let (ns, guard) = (namespace(layout), guard(layout, ""));
    let mut elems = String::new();
    elem_classes(&layout.root, &mut elems, &[]);
    let (mut view, mut builder) = (Vec::new(), Vec::new());
    let r = Through {
        base: "",
        recv: "r_.",
        owner: "r_",
    };
    view_accessors(&layout.root, &r, &mut view, &[]);
    let w = Through {
        base: "",
        recv: "w_.",
        owner: "w_",
    };
    builder_accessors(&layout.root, &w, &mut builder, &[]);
    let tensor_at = tensor(layout, &mut view, &mut builder);
    let (view, builder) = (view.join("\n"), builder.join("\n"));
    let consts = constants(layout);
    let (name, type_name, tier) = (&layout.name, layout.type_name(), layout.tier);
    let fp = layout.fingerprint;
    let scalar = layout.scalar_bytes();
    let (stamp, register) = (layout.stamp_offset(), stamp_registration(layout));
    format!(
        r#"// generated by flux_gen from {source} -- do not edit.
#ifndef {guard}
#define {guard}

#include "flux/channel.hpp"
#include "flux/wire.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string_view>
#include <utility>

namespace {ns}::flux_msg
{{

// {type_name} as a flux flat frame ({tier} tier).
struct {name}
{{
  static constexpr const char * kTypeName = "{type_name}";
  static constexpr std::uint64_t kFingerprint = 0x{fp:016X}ull;
  static constexpr std::size_t kScalarBytes = {scalar};
  static constexpr std::int32_t kStampOffset = {stamp};{register}{tensor_at}
{consts}{elems}

  // Read side. Aliases the frame: valid only while the FrameView it came from is alive.
  class View
  {{
public:
    View() = default;
    View(const void * data, std::size_t size) noexcept : r_(data, size) {{}}
    explicit View(const flux::FrameView & f) noexcept
    : r_(f.data(), f.size(), f.heap(), f.gpu_heap()) {{}}

    // False once any accessor has been asked for something the frame does not contain.
    bool ok__() const noexcept {{ return !r_.bad(); }}
    explicit operator bool() const noexcept {{ return ok__(); }}
    std::size_t size__() const noexcept {{ return r_.size(); }}
    std::size_t budget__() const noexcept {{ return r_.budget(); }}

{view}

private:
    flux::wire::Reader r_;
  }};

  // Write side over a loaned slot: fields are written straight into shared memory.
  class Builder
  {{
public:
    Builder(void * data, std::size_t capacity) noexcept : w_(data, capacity, kScalarBytes) {{}}
    explicit Builder(flux::WriteSlot & slot) noexcept
    : ext_(&slot), w_(slot.data(), slot.capacity(), kScalarBytes, slot.heap()) {{}}
    // Owning form, for build__(). The slot travels with the Builder, so the caller has one object
    // to keep alive instead of two.
    explicit Builder(flux::WriteSlot && slot) noexcept
    : owned_(std::move(slot)), w_(owned_->data(), owned_->capacity(), kScalarBytes, owned_->heap()) {{}}

    // False once a field did not fit; commit__() then refuses rather than publishing a torn frame.
    bool ok__() const noexcept {{ return !w_.bad(); }}
    explicit operator bool() const noexcept {{ return ok__(); }}
    std::size_t size__() const noexcept {{ return w_.size(); }}

{builder}

    // Publish what has been written. The slot was loaned as one u8 run, so committing the built
    // prefix is the whole statement; the schema lives in the fingerprint, not in FrameMeta.
    flux::Published commit__() noexcept
    {{
      flux::WriteSlot * s = owned_ ? &*owned_ : ext_;
      if (s == nullptr) {{
        return flux::Published::TooLarge;  // built into a caller's buffer: nothing to publish
      }}
      if (!s->valid()) {{
        return flux::Published::Backpressure;  // the loan never happened
      }}
      if (!ok__()) {{
        return flux::Published::TooLarge;
      }}
      const flux::Published p = s->commit(w_.size());
      w_ = flux::wire::Writer(nullptr, 0, kScalarBytes);  // the slot is the subscribers' now
      return p;
    }}

private:
    std::optional<flux::WriteSlot> owned_;
    flux::WriteSlot * ext_ = nullptr;
    flux::wire::Writer w_;
  }};

  // Loan a slot from `pub` and build into it -- the whole write side in one line. Templated so a
  // generated header needs only flux/channel.hpp: any publisher with loan() fits.
  //
  // Check it before writing. A Builder whose loan found no free slot is false, and writing into
  // it is harmless (the writer has no buffer) -- but its commit__() reports Backpressure rather
  // than pretending the frame went out.
  template <typename Pub>
  static Builder build__(Pub & pub) noexcept
  {{
    return Builder(pub.loan());
  }}
}};

}}  // namespace {ns}::flux_msg

#endif  // {guard}
"#
    )
}
