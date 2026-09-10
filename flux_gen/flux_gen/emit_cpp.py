"""Emit a header-only C++ adapter for one schema.

The header defines `<Pkg>::flux_msg::<Msg>` carrying the fingerprint, a View over a received
frame and a Builder over a loaned slot. Both are thin: every accessor is one call into
flux/wire.hpp at a constant offset this module computed, so there is no per-message runtime and
nothing to keep in sync but the offsets themselves.
"""

from .dtypes import ROS_PRIMITIVE, DType
from .flatten import DYNAMIC_COUNT, LeafKind
from .layout import STAMP_SIZE, elem_class_name as _elem_class

CPP_TYPE = {
    DType.BOOL: "bool",
    DType.U8: "std::uint8_t", DType.I8: "std::int8_t",
    DType.U16: "std::uint16_t", DType.I16: "std::int16_t",
    DType.U32: "std::uint32_t", DType.I32: "std::int32_t",
    DType.U64: "std::uint64_t", DType.I64: "std::int64_t",
    DType.F32: "float", DType.F64: "double",
}

CPP_KEYWORDS = frozenset("""
alignas alignof and asm auto bool break case catch char class const consteval constexpr
continue decltype default delete do double dynamic_cast else enum explicit export extern false
float for friend goto if inline int long mutable namespace new noexcept not nullptr operator or
private protected public register return short signed sizeof static struct switch template this
throw true try typedef typeid typename union unsigned using virtual void volatile wchar_t while
xor
""".split())


def ident(name):
    return name + "_" if name in CPP_KEYWORDS else name


def snake(name):
    out = []
    for i, c in enumerate(name):
        if c.isupper() and i and (not name[i - 1].isupper() or
                                  (i + 1 < len(name) and name[i + 1].islower())):
            out.append("_")
        out.append(c.lower())
    return "".join(out)


def cpp_type(dtype):
    # Refused rather than mapped: C++17 has no standard 16-bit float type, and raw uint16 bits
    # would read like numbers while meaning none of them. Revisit when C++23 is the floor.
    if dtype is DType.F16:
        raise ValueError(
            "f16 has no standard C++17 type, so the C++ adapter refuses it; "
            "use float32 in the schema, or uint16 if raw bits are intended")
    if dtype not in CPP_TYPE:
        raise ValueError(f"no C++ type for {dtype}")
    return CPP_TYPE[dtype]


def cpp_literal(const):
    if const.type_name in ("string", "wstring"):
        body = const.value.replace("\\", "\\\\").replace('"', '\\"')
        return "const char *", f'"{body}"'
    if const.type_name == "bool":
        return "bool", "true" if const.value else "false"
    t = cpp_type(ROS_PRIMITIVE[const.type_name])
    if const.type_name == "float32":
        return t, f"{const.value!r}f"
    return t, repr(const.value)


def _constants(layout):
    if not layout.constants:
        return ""
    lines = []
    for c in layout.constants:
        t, lit = cpp_literal(c)
        lines.append(f"  static constexpr {t} {ident(c.name)} = {lit};")
    return "\n" + "\n".join(lines) + "\n"


def _view_accessors(block, base, out, prefix=(), recv="r_.", ind="    "):
    """`base` is the C++ expression for the block's frame offset ("" for the root block);
    `recv` is how that block's class reaches its Reader (a member value, or a pointer)."""
    reader = "r_" if recv == "r_." else "*r_"

    def at(off):
        return f"{base}{off}" if base else str(off)

    for p in block.placed:
        n, o = ident(p.name), p.offset
        if p.kind == LeafKind.FIXED:
            t = cpp_type(p.dtype)
            if p.count == 1:
                out.append(f"{ind}{t} {n}() const noexcept {{ return {recv}get<{t}>({at(o)}); }}")
            else:
                out.append(
                    f"{ind}flux::wire::Span<const {t}> {n}() const noexcept "
                    f"{{ return {recv}block<{t}>({at(o)}, {p.count}); }}")
        elif p.kind == LeafKind.COLUMN:
            t = cpp_type(p.dtype)
            out.append(
                f"{ind}flux::wire::Span<const {t}> {n}() const noexcept "
                f"{{ return {recv}span<{t}>({at(o)}); }}")
        elif p.kind == LeafKind.STRING:
            out.append(
                f"{ind}std::string_view {n}() const noexcept {{ return {recv}str({at(o)}); }}")
        elif p.kind == LeafKind.STRING_ARRAY:
            out.append(f"{ind}std::size_t {n}__size() const noexcept {{ return {recv}len({at(o)}); }}")
            out.append(
                f"{ind}std::string_view {n}(std::size_t i) const noexcept "
                f"{{ return {recv}str_at({at(o)}, i); }}")
        elif p.kind == LeafKind.STAMP:
            out.append(
                f"{ind}std::int32_t {n}__sec() const noexcept "
                f"{{ return {recv}get<std::int32_t>({at(o)}); }}")
            out.append(
                f"{ind}std::uint32_t {n}__nanosec() const noexcept "
                f"{{ return {recv}get<std::uint32_t>({at(o + 4)}); }}")
        elif p.kind == LeafKind.HEADER:
            out.append(
                f"{ind}std::int32_t {n}__sec() const noexcept "
                f"{{ return {recv}get<std::int32_t>({at(o)}); }}")
            out.append(
                f"{ind}std::uint32_t {n}__nanosec() const noexcept "
                f"{{ return {recv}get<std::uint32_t>({at(o + 4)}); }}")
            out.append(
                f"{ind}std::string_view {n}__frame_id() const noexcept "
                f"{{ return {recv}str({at(o + STAMP_SIZE)}); }}")
        else:  # RECORD_COLUMN / JAGGED
            cls = _elem_class(prefix + (p,))
            out.append(f"{ind}std::size_t {n}__size() const noexcept {{ return {recv}len({at(o)}); }}")
            out.append(
                f"{ind}{cls} {n}(std::size_t i) const noexcept "
                f"{{ return {cls}({reader}, {recv}elem({at(o)}, i, {p.elem.stride}, {p.elem.align})); }}")


def _builder_accessors(block, base, out, prefix=(), recv="w_.", ind="    "):
    writer = "w_" if recv == "w_." else "*w_"

    def at(off):
        return f"{base}{off}" if base else str(off)

    for p in block.placed:
        n, o = ident(p.name), p.offset
        if p.kind == LeafKind.FIXED:
            t = cpp_type(p.dtype)
            if p.count == 1:
                out.append(
                    f"{ind}void set__{n}({t} v) noexcept {{ {recv}put<{t}>({at(o)}, v); }}")
            else:
                out.append(
                    f"{ind}flux::wire::Span<{t}> {n}() noexcept "
                    f"{{ return {recv}block<{t}>({at(o)}, {p.count}); }}")
        elif p.kind == LeafKind.COLUMN:
            t = cpp_type(p.dtype)
            out.append(
                f"{ind}flux::wire::Span<{t}> alloc__{n}(std::size_t n) noexcept "
                f"{{ return {recv}alloc<{t}>({at(o)}, n); }}")
        elif p.kind == LeafKind.STRING:
            out.append(
                f"{ind}void set__{n}(std::string_view s) noexcept "
                f"{{ {recv}put_str({at(o)}, s); }}")
        elif p.kind == LeafKind.STRING_ARRAY:
            guard = (f"if (n != {p.count}) {{ {recv}poison(); return; }} "
                     if p.count != DYNAMIC_COUNT else "")
            out.append(
                f"{ind}void alloc__{n}(std::size_t n) noexcept "
                f"{{ {guard}{recv}alloc_strs({at(o)}, n); }}")
            out.append(
                f"{ind}void set__{n}(std::size_t i, std::string_view s) noexcept "
                f"{{ {recv}put_str_at({at(o)}, i, s); }}")
        elif p.kind in (LeafKind.STAMP, LeafKind.HEADER):
            out.append(
                f"{ind}void set__{n}__stamp(std::int32_t sec, std::uint32_t nanosec) noexcept "
                f"{{ {recv}put<std::int32_t>({at(o)}, sec); "
                f"{recv}put<std::uint32_t>({at(o + 4)}, nanosec); }}")
            if p.kind == LeafKind.HEADER:
                out.append(
                    f"{ind}void set__{n}__frame_id(std::string_view s) noexcept "
                    f"{{ {recv}put_str({at(o + STAMP_SIZE)}, s); }}")
        else:  # RECORD_COLUMN / JAGGED
            cls = _elem_class(prefix + (p,))
            guard = (f"if (n != {p.count}) {{ {recv}poison(); return {cls}Array(); }} "
                     if p.count != DYNAMIC_COUNT else "")
            out.append(
                f"{ind}{cls}Array alloc__{n}(std::size_t n) noexcept "
                f"{{ {guard}return {cls}Array({writer}, {recv}alloc_elems("
                f"{at(o)}, n, {p.elem.stride}, {p.elem.align}), n); }}")


def _elem_classes(block, out, prefix=()):
    """Emit element View/Builder pairs, innermost first so each is declared before its user."""
    for p in block.placed:
        if p.elem is None:
            continue
        path = prefix + (p,)
        _elem_classes(p.elem, out, path)
        cls = _elem_class(path)
        body = []
        _view_accessors(p.elem, "at_ + ", body, path, "r_->", "    ")
        out.append(f"""
  // one element of `{'.'.join(x.name for x in path)}`
  class {cls}
  {{
public:
    static constexpr std::size_t kStride = {p.elem.stride};
    static constexpr std::size_t kAlign = {p.elem.align};
    {cls}(const flux::wire::Reader & r, std::size_t at) noexcept : r_(&r), at_(at) {{}}
{chr(10).join(body)}

private:
    const flux::wire::Reader * r_;
    std::size_t at_;
  }};""")
        bbody = []
        _builder_accessors(p.elem, "at_ + ", bbody, path, "w_->", "    ")
        out.append(f"""
  class {cls}Builder
  {{
public:
    {cls}Builder(flux::wire::Writer & w, std::size_t at) noexcept : w_(&w), at_(at) {{}}
{chr(10).join(bbody)}

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
    {{ return {cls}Builder(*w_, at_ + i * {p.elem.stride}); }}

private:
    flux::wire::Writer * w_ = nullptr;
    std::size_t at_ = 0;
    std::size_t n_ = 0;
  }};""")


def emit(layout, source):
    """Return the C++ header text for a Layout. `source` is the .msg path, for the banner."""
    if layout.rejected:
        raise ValueError(f"{layout.type_name}: {layout.rejected}")
    ns = layout.package or "flux_msg"
    guard = f"{ns.upper()}__FLUX__{snake(layout.name).upper()}_HPP_"

    elems = []
    _elem_classes(layout.root, elems)
    view, builder = [], []
    _view_accessors(layout.root, "", view)
    _builder_accessors(layout.root, "", builder)

    return f"""// generated by flux_gen from {source} -- do not edit.
#ifndef {guard}
#define {guard}

#include "flux/channel.hpp"
#include "flux/wire.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <utility>

namespace {ns}::flux_msg
{{

// {layout.type_name} as a flux flat frame ({layout.tier} tier).
struct {layout.name}
{{
  static constexpr const char * kTypeName = "{layout.type_name}";
  static constexpr std::uint64_t kFingerprint = 0x{layout.fingerprint:016X}ull;
  static constexpr std::size_t kScalarBytes = {layout.scalar_bytes};
{_constants(layout)}{"".join(elems)}

  // Read side. Aliases the frame: valid only while the FrameView it came from is alive.
  class View
  {{
public:
    View() = default;
    View(const void * data, std::size_t size) noexcept : r_(data, size) {{}}
    explicit View(const flux::FrameView & f) noexcept : r_(f.data(), f.size()) {{}}

    // False once any accessor has been asked for something the frame does not contain.
    bool ok__() const noexcept {{ return !r_.bad(); }}
    explicit operator bool() const noexcept {{ return ok__(); }}

{chr(10).join(view)}

private:
    flux::wire::Reader r_;
  }};

  // Write side over a loaned slot: fields are written straight into shared memory.
  class Builder
  {{
public:
    Builder(void * data, std::size_t capacity) noexcept : w_(data, capacity, kScalarBytes) {{}}
    explicit Builder(flux::WriteSlot & slot) noexcept
    : ext_(&slot), w_(slot.data(), slot.capacity(), kScalarBytes) {{}}
    // Owning form, for build__(). The slot travels with the Builder, so the caller has one object
    // to keep alive instead of two.
    explicit Builder(flux::WriteSlot && slot) noexcept
    : owned_(std::move(slot)), w_(owned_->data(), owned_->capacity(), kScalarBytes) {{}}

    // False once a field did not fit; commit__() then refuses rather than publishing a torn frame.
    bool ok__() const noexcept {{ return !w_.bad(); }}
    explicit operator bool() const noexcept {{ return ok__(); }}
    std::size_t size__() const noexcept {{ return w_.size(); }}

{chr(10).join(builder)}

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
      return s->commit(w_.size());
    }}

private:
    std::optional<flux::WriteSlot> owned_;
    flux::WriteSlot * ext_ = nullptr;
    flux::wire::Writer w_;
  }};

  // Loan a slot from `pub` and build into it -- the whole write side in one line. Templated so a
  // generated header depends on flux_core alone: any publisher with loan() fits, which is
  // flux::Channel and flux::ros::Publisher both.
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
"""
