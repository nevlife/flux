// Reader and Writer of the flat frame layout for generated Python adapters: flux/wire.hpp, the
// runtime C++ adapters use, with numpy views out. A violation raises WireError, and a Reader
// starts each read afresh, as a Python caller that catches it expects. A Writer call that raises
// also latches bad(), so commit refuses the torn frame as the C++ Builder does.

#include "flux/wire.hpp"

#include "bind_wire.hpp"
#include "flux/channel.hpp"
#include "numpy_api.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace
{

struct WireError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

// A contiguous buffer held for the life of the object that read it.
class Buffer
{
public:
  Buffer(nb::handle obj, bool writable)
  {
    if (PyObject_GetBuffer(obj.ptr(), &b_, writable ? PyBUF_WRITABLE : PyBUF_SIMPLE) != 0) {
      throw nb::python_error();
    }
  }
  ~Buffer() { PyBuffer_Release(&b_); }
  Buffer(const Buffer &) = delete;
  auto operator=(const Buffer &) -> Buffer & = delete;
  Buffer(Buffer &&) = delete;
  auto operator=(Buffer &&) -> Buffer & = delete;

  [[nodiscard]] auto data() const noexcept -> std::byte *
  {
    return static_cast<std::byte *>(b_.buf);
  }
  [[nodiscard]] auto size() const noexcept -> std::size_t
  {
    return static_cast<std::size_t>(b_.len);
  }

private:
  Py_buffer b_{};
};

struct DType
{
  PyArray_Descr * descr;
  std::size_t size;
  // Owns `descr` for the call, unless the dtype is cached.
  nb::object held;
};

auto resolve(nb::handle t) -> DType
{
  PyArray_Descr * d = nullptr;
  if (PyArray_DescrConverter(t.ptr(), &d) != NPY_SUCCEED) {
    throw nb::python_error();
  }
  DType v{d, flux_py::elsize(d), flux_py::own(d)};
  // The frame holds plain numbers: an object dtype would turn its bytes into pointers.
  const bool number = d->kind == 'b' || d->kind == 'i' || d->kind == 'u' || d->kind == 'f';
  if (!number || (v.size != 1 && v.size != 2 && v.size != 4 && v.size != 8)) {
    throw nb::type_error("flux: a frame field is a bool, integer or float of 1, 2, 4 or 8 bytes");
  }
  return v;
}

// The numpy scalar types generated code passes are resolved once each and held for good. Other
// dtype arguments, such as names built at run time, are resolved on each call.
auto dtype_of(nb::handle t) -> DType
{
  static std::vector<std::pair<PyObject *, DType>> types;
  if (!PyType_Check(t.ptr())) {
    return resolve(t);
  }
  for (const auto & [k, v] : types) {
    if (k == t.ptr()) {
      return v;
    }
  }
  DType v = resolve(t);
  v.held.release();
  return types.emplace_back(t.inc_ref().ptr(), std::move(v)).second;
}

// Offsets come from the generated adapter, so only the element size matters to wire.hpp.
template <class F>
auto by_size(std::size_t size, F f)
{
  switch (size) {
    case 1:
      return f(std::uint8_t{});
    case 2:
      return f(std::uint16_t{});
    case 4:
      return f(std::uint32_t{});
    default:
      return f(std::uint64_t{});
  }
}

// region() divides by the stride and aligns up by `align`.
void check_layout(std::size_t stride, std::size_t align)
{
  if (stride == 0 || align == 0) {
    throw nb::value_error("flux: stride and align must be positive");
  }
}

// Views alias the buffer; `owner` keeps it alive while any of them does.
auto ro_array(const void * p, std::size_t n, const DType & t, nb::handle owner) -> nb::object
{
  return flux_py::numpy_view(p, n, t.descr, owner, false);
}

auto rw_array(void * p, std::size_t n, const DType & t, nb::handle owner) -> nb::object
{
  return flux_py::numpy_view(p, n, t.descr, owner, true);
}

class Reader
{
public:
  explicit Reader(nb::handle buf) : buf_(buf, false)
  {
    const std::uint8_t * base = nullptr;
    std::size_t n = 0;
    arena_ = flux_arena_acquire(
      reinterpret_cast<const std::uint8_t *>(buf_.data()), buf_.size(), &base, &n);  // NOLINT
    heap_ = {reinterpret_cast<const std::byte *>(base), arena_ != nullptr ? n : 0};  // NOLINT
    // A take result reaches across PCIe too: its device fields, or a GPU reader's copies.
    if (const flux::FrameView * v = flux_py::frame_of(buf, &device_)) {
      gpu_ = v->gpu_heap();
    }
  }
  ~Reader() { flux_arena_release(arena_); }
  Reader(const Reader &) = delete;
  auto operator=(const Reader &) -> Reader & = delete;
  Reader(Reader &&) = delete;
  auto operator=(Reader &&) -> Reader & = delete;

  [[nodiscard]] auto size() const noexcept -> std::size_t { return buf_.size(); }
  [[nodiscard]] auto reader() const noexcept -> flux::wire::Reader
  {
    return {buf_.data(), buf_.size(), heap_, gpu_};
  }
  [[nodiscard]] auto device() const noexcept -> int { return device_; }

  static void check(const flux::wire::Reader & r, const char * what, std::size_t off)
  {
    if (r.bad()) {
      throw WireError(
        "flux: " + std::string(what) + " at " + std::to_string(off) + " escapes the frame");
    }
  }

private:
  Buffer buf_;
  // The arena the frame's trailer names, mapped while this reader and its views live.
  const void * arena_ = nullptr;
  flux::wire::Span<const std::byte> heap_;
  flux::wire::GpuHeap gpu_;
  int device_ = -1;
};

auto reader_block(nb::handle self, std::size_t off, nb::handle dtype, std::size_t n) -> nb::object
{
  const Reader & r = *nb::inst_ptr<Reader>(self);
  const DType t = dtype_of(dtype);
  const flux::wire::Reader w = r.reader();
  const void * p =
    by_size(t.size, [&](auto e) -> const void * { return w.block<decltype(e)>(off, n).data(); });
  Reader::check(w, "block", off);
  return ro_array(p, n, t, self);
}

auto reader_get(nb::handle self, std::size_t off, nb::handle dtype) -> nb::object
{
  const Reader & r = *nb::inst_ptr<Reader>(self);
  const DType t = dtype_of(dtype);
  if (!flux::wire::detail::fits(off, t.size, r.size())) {
    throw WireError("flux: scalar at " + std::to_string(off) + " escapes the frame");
  }
  const flux::wire::Reader w = r.reader();
  return flux_py::numpy_scalar(w.base() + off, t.descr);
}

auto reader_span(nb::handle self, std::size_t off, nb::handle dtype) -> nb::object
{
  const Reader & r = *nb::inst_ptr<Reader>(self);
  const DType t = dtype_of(dtype);
  const flux::wire::Reader w = r.reader();
  const auto [p, n] = by_size(t.size, [&](auto e) {
    const auto s = w.span<decltype(e)>(off);
    return std::pair<const void *, std::size_t>(s.data(), s.size());
  });
  Reader::check(w, "span", off);
  return ro_array(p, n, t, self);
}

// As rclpy decodes a string field: up to the first NUL, and bytes that are not UTF-8 become
// U+FFFD so the frame stays readable. C++ hands the bytes over unchecked, as rclcpp does.
auto decode(std::string_view s) -> nb::str
{
  s = s.substr(0, s.find('\0'));
  return nb::steal<nb::str>(
    PyUnicode_DecodeUTF8(s.data(), static_cast<Py_ssize_t>(s.size()), "replace"));
}

auto reader_string(const Reader & r, std::size_t off) -> nb::str
{
  const flux::wire::Reader w = r.reader();
  const std::string_view s = w.str(off);
  Reader::check(w, "string", off);
  return decode(s);
}

// A writer never points two descriptors at the same bytes, so the strings cannot add up to more
// than the frame: one that does would make a small frame decode to gigabytes.
auto reader_strings(const Reader & r, std::size_t off) -> nb::list
{
  const flux::wire::Reader w = r.reader();
  const std::size_t n = w.len(off, sizeof(flux::wire::Desc), alignof(flux::wire::Desc));
  Reader::check(w, "string array", off);
  if (n > w.budget() / sizeof(flux::wire::Desc)) {
    throw WireError("flux: string array at " + std::to_string(off) + " reads the same bytes twice");
  }
  std::size_t unread = w.budget() - n * sizeof(flux::wire::Desc);
  nb::list out;
  for (std::size_t i = 0; i < n; ++i) {
    const std::string_view s = w.str_at(off, i);
    Reader::check(w, "string", off);
    if (s.size() > unread) {
      throw WireError(
        "flux: string array at " + std::to_string(off) + " reads the same bytes twice");
    }
    unread -= s.size();
    out.append(decode(s));
  }
  return out;
}

auto reader_elems(const Reader & r, std::size_t off, std::size_t stride, std::size_t align)
  -> std::pair<std::size_t, std::size_t>
{
  check_layout(stride, align);
  const flux::wire::Reader w = r.reader();
  const std::size_t n = w.len(off, stride, align);
  Reader::check(w, "element array", off);
  return {w.desc(off).off, n};
}

// A loan's `_heap`: the arena, and the device of a GPU arena (-1 for none).
auto heap_of(nb::handle h, int * device) -> flux::wire::Heap
{
  *device = -1;
  if (h.is_none()) {
    return {};
  }
  const auto t =
    nb::cast<std::tuple<std::uintptr_t, std::size_t, std::uintptr_t, std::uint32_t, int>>(h);
  *device = std::get<4>(t);
  // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr)
  return flux::detail::heap_of(
    {reinterpret_cast<std::uint8_t *>(std::get<0>(t)), std::get<1>(t),
     reinterpret_cast<const void *>(std::get<2>(t)), std::get<3>(t), *device >= 0});
  // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr)
}

class Writer
{
public:
  // `heap` is a loan's `_heap`, or None.
  Writer(nb::handle buf, std::size_t scalar_bytes, nb::handle heap)
  : buf_(buf, true), w_(buf_.data(), buf_.size(), scalar_bytes, heap_of(heap, &device_))
  {
    if (w_.bad()) {
      throw WireError(
        "flux: scalar block (" + std::to_string(scalar_bytes) + "B) exceeds the slot (" +
        std::to_string(buf_.size()) + "B)");
    }
  }

  [[nodiscard]] auto bad() const noexcept -> bool { return w_.bad() || poisoned_; }
  void poison() noexcept { poisoned_ = true; }
  [[nodiscard]] auto size() const noexcept -> std::size_t { return w_.size(); }
  [[nodiscard]] auto device() const noexcept -> int { return device_; }
  auto w() noexcept -> flux::wire::Writer & { return w_; }

  // A write that does not fit has latched bad() in wire.hpp before this raises.
  void check(const char * what, std::size_t off) const
  {
    if (w_.bad()) {
      throw WireError(
        "flux: " + std::string(what) + " at " + std::to_string(off) + " does not fit the slot");
    }
  }

private:
  Buffer buf_;
  int device_ = -1;
  flux::wire::Writer w_;
  bool poisoned_ = false;
};

template <class F>
auto poisoning(Writer & w, F f)
{
  try {
    return f();
  } catch (...) {
    w.poison();
    throw;
  }
}

auto count_of(nb::handle n) -> std::size_t
{
  const nb::object i = nb::steal(PyNumber_Index(n.ptr()));
  if (!i.is_valid()) {
    throw nb::python_error();
  }
  const std::size_t v = PyLong_AsSize_t(i.ptr());
  if (v == static_cast<std::size_t>(-1) && PyErr_Occurred() != nullptr) {
    throw nb::python_error();
  }
  return v;
}

auto writer_block(nb::handle self, std::size_t off, nb::handle dtype, std::size_t n) -> nb::object
{
  Writer & w = *nb::inst_ptr<Writer>(self);
  const DType t = dtype_of(dtype);
  void * p =
    by_size(t.size, [&](auto e) -> void * { return w.w().block<decltype(e)>(off, n).data(); });
  w.check("block", off);
  return rw_array(p, n, t, self);
}

// numpy 1.x wraps an int that does not fit, and numpy 2 a numpy integer: both raise
// OverflowError here, as rclpy refuses a field value out of its range.
void check_range(nb::handle value, const nb::object & a)
{
  PyArray_Descr * d = PyArray_DESCR(flux_py::as_array(a));
  const char kind = d->kind;
  if ((kind != 'i' && kind != 'u') || PyIndex_Check(value.ptr()) == 0) {
    return;
  }
  const nb::object i = nb::steal(PyNumber_Index(value.ptr()));
  if (!i.is_valid()) {
    throw nb::python_error();
  }
  int over = 0;
  const long long v = PyLong_AsLongLongAndOverflow(i.ptr(), &over);
  if (v == -1 && PyErr_Occurred() != nullptr) {
    throw nb::python_error();
  }
  const std::size_t bits = 8 * flux_py::elsize(d);
  bool fits = false;
  if (kind == 'i') {
    fits = over == 0 && (bits == 64 || (v >= -(1LL << (bits - 1)) && v < (1LL << (bits - 1))));
  } else if (over == 0) {
    fits = v >= 0 && (bits == 64 || v < (1LL << bits));
  } else if (over > 0 && bits == 64) {
    (void)PyLong_AsUnsignedLongLong(i.ptr());
    fits = PyErr_Occurred() == nullptr;
    PyErr_Clear();
  }
  if (!fits) {
    const std::string what = "flux: " + nb::cast<std::string>(nb::repr(i)) +
                             " is out of range for " +
                             nb::cast<std::string>(a.attr("dtype").attr("name"));
    PyErr_SetString(PyExc_OverflowError, what.c_str());
    throw nb::python_error();
  }
}

void writer_put(nb::handle self, std::size_t off, nb::handle dtype, nb::handle value)
{
  poisoning(*nb::inst_ptr<Writer>(self), [&] {
    const nb::object a = writer_block(self, off, dtype, 1);
    check_range(value, a);
    PyArrayObject * arr = flux_py::as_array(a);
    if (PyArray_SETITEM(arr, static_cast<char *>(PyArray_DATA(arr)), value.ptr()) != 0) {
      throw nb::python_error();
    }
  });
}

auto writer_alloc(nb::handle self, std::size_t off, nb::handle count, nb::handle dtype)
  -> nb::object
{
  Writer & w = *nb::inst_ptr<Writer>(self);
  return poisoning(w, [&] {
    const std::size_t n = count_of(count);
    const DType t = dtype_of(dtype);
    void * p =
      by_size(t.size, [&](auto e) -> void * { return w.w().alloc<decltype(e)>(off, n).data(); });
    w.check("array", off);
    return rw_array(p, n, t, self);
  });
}

// A C-contiguous array whose memory the malloc hook placed in the arena becomes the
// field without a copy, and the frame keeps it alive; any other is copied. True when not copied.
auto writer_adopt(nb::handle self, std::size_t off, nb::handle a, nb::handle dtype) -> bool
{
  Writer & w = *nb::inst_ptr<Writer>(self);
  return poisoning(w, [&] {
    const DType t = dtype_of(dtype);
    if (PyArray_Check(a.ptr()) == 0) {
      throw nb::type_error("flux: adopt takes a numpy array");
    }
    PyArrayObject * arr = flux_py::as_array(a);
    if (
      PyArray_IS_C_CONTIGUOUS(arr) == 0 ||
      static_cast<std::size_t>(PyArray_ITEMSIZE(arr)) != t.size) {
      throw nb::type_error("flux: adopt takes a C-contiguous array of the field's dtype");
    }
    const auto n = static_cast<std::size_t>(PyArray_SIZE(arr));
    const auto release = [](void * o) {
      const PyGILState_STATE g = PyGILState_Ensure();
      Py_DECREF(static_cast<PyObject *>(o));
      PyGILState_Release(g);
    };
    if (w.w().adopt(off, PyArray_DATA(arr), n, t.size, release, a.inc_ref().ptr())) {
      return true;
    }
    a.dec_ref();
    void * p =
      by_size(t.size, [&](auto e) -> void * { return w.w().alloc<decltype(e)>(off, n).data(); });
    w.check("array", off);
    if (n != 0) {
      std::memcpy(p, PyArray_DATA(arr), n * t.size);
    }
    return false;
  });
}

// A str is stored as rclpy stores it: UTF-8, cut at the first NUL. Like rclpy, nothing else is
// taken, and a lone surrogate is a UnicodeEncodeError.
auto text_of(nb::handle s) -> std::string_view
{
  if (!PyUnicode_Check(s.ptr())) {
    throw nb::type_error("flux: a string field takes a str");
  }
  Py_ssize_t n = 0;
  const char * p = PyUnicode_AsUTF8AndSize(s.ptr(), &n);
  if (p == nullptr) {
    throw nb::python_error();
  }
  const std::string_view v(p, static_cast<std::size_t>(n));
  return v.substr(0, v.find('\0'));
}

void writer_put_str(Writer & w, std::size_t off, nb::handle s)
{
  poisoning(w, [&] {
    w.w().put_str(off, text_of(s));
    w.check("string", off);
  });
}

// `count` is a fixed string[N]'s N: the frame carries the length, which must agree with it.
void writer_put_strs(
  Writer & w, std::size_t off, nb::handle items, std::optional<std::size_t> count)
{
  poisoning(w, [&] {
    if (PyUnicode_Check(items.ptr())) {
      throw nb::type_error("flux: a string array takes a sequence of str, not a str");
    }
    nb::list list;
    for (nb::handle h : nb::iter(items)) {
      list.append(h);
    }
    if (count && list.size() != *count) {
      throw nb::value_error(("flux: a string[" + std::to_string(*count) + "] takes exactly " +
                             std::to_string(*count) + " strings, got " +
                             std::to_string(list.size()))
                              .c_str());
    }
    w.w().alloc_strs(off, list.size());
    w.check("string array", off);
    for (std::size_t i = 0; i < list.size(); ++i) {
      w.w().put_str_at(off, i, text_of(list[i]));
      w.check("string", off);
    }
  });
}

auto writer_alloc_elems(
  Writer & w, std::size_t off, nb::handle count, std::size_t stride,
  std::size_t align) -> std::size_t
{
  return poisoning(w, [&] {
    check_layout(stride, align);
    const std::size_t at = w.w().alloc_elems(off, count_of(count), stride, align);
    w.check("element array", off);
    return at;
  });
}

auto at_of(const flux_py::TensorAt & a) -> flux::wire::TensorAt
{
  return {a[0], a[1], a[2], a[3], a[4], a[5], a[6]};
}

// The dtypes numpy names, -1 for the rest.
auto npy_of(flux::wire::DlType t) -> int
{
  if (flux::wire::dl_itemsize(t) == 0) {
    return -1;
  }
  switch (t.code) {
    case flux::wire::kDlInt:
      return t.bits == 8    ? NPY_INT8
             : t.bits == 16 ? NPY_INT16
             : t.bits == 32 ? NPY_INT32
                            : NPY_INT64;
    case flux::wire::kDlUInt:
      return t.bits == 8    ? NPY_UINT8
             : t.bits == 16 ? NPY_UINT16
             : t.bits == 32 ? NPY_UINT32
                            : NPY_UINT64;
    case flux::wire::kDlFloat:
      return t.bits == 16 ? NPY_HALF : t.bits == 32 ? NPY_FLOAT32 : NPY_FLOAT64;
    case flux::wire::kDlComplex:
      return t.bits == 64 ? NPY_COMPLEX64 : NPY_COMPLEX128;
    case flux::wire::kDlBool:
      return NPY_BOOL;
    default:
      return -1;
  }
}

// DLPack's names for the dtypes numpy lacks, which ml_dtypes gives numpy under the same names.
auto dl_named(nb::handle dtype) -> flux::wire::DlType
{
  static const std::array<std::pair<const char *, std::uint8_t>, 9> named{{
    {"bfloat16", flux::wire::kDlBfloat},
    {"float8_e3m4", 7},
    {"float8_e4m3", 8},
    {"float8_e4m3b11fnuz", 9},
    {"float8_e4m3fn", 10},
    {"float8_e4m3fnuz", 11},
    {"float8_e5m2", 12},
    {"float8_e5m2fnuz", 13},
    {"float8_e8m0fnu", 14},
  }};
  PyArray_Descr * d = nullptr;
  std::string name;
  nb::object held;
  if (PyUnicode_Check(dtype.ptr())) {
    name = nb::cast<std::string>(dtype);
  }
  for (const auto & [n, code] : named) {
    if (name == n) {
      return {code, static_cast<std::uint8_t>(code == 4 ? 16 : 8), 1};
    }
  }
  if (PyArray_DescrConverter(dtype.ptr(), &d) != NPY_SUCCEED) {
    throw nb::python_error();
  }
  held = flux_py::own(d);
  if (d->type_num >= NPY_USERDEF) {
    name = nb::cast<std::string>(held.attr("name"));
    for (const auto & [n, code] : named) {
      if (name == n) {
        return {code, static_cast<std::uint8_t>(8 * flux_py::elsize(d)), 1};
      }
    }
  }
  if (!PyArray_ISNBO(d->byteorder)) {
    throw nb::type_error("flux: frames are in native byte order; use a native dtype");
  }
  const auto bits = static_cast<std::uint8_t>(8 * flux_py::elsize(d));
  const char kind = d->kind;
  const flux::wire::DlType t{
    kind == 'i'   ? flux::wire::kDlInt
    : kind == 'u' ? flux::wire::kDlUInt
    : kind == 'f' ? flux::wire::kDlFloat
    : kind == 'c' ? flux::wire::kDlComplex
    : kind == 'b' ? flux::wire::kDlBool
                  : std::uint8_t{3},
    bits, 1};
  if (flux::wire::dl_itemsize(t) == 0) {
    throw nb::type_error("flux: a tensor is a bool, integer, float, complex, bfloat16 or float8");
  }
  return t;
}

auto dims_of(nb::handle shape) -> std::vector<std::int64_t>
{
  std::vector<std::int64_t> dims;
  for (nb::handle h : nb::iter(shape)) {
    const nb::object i = nb::steal(PyNumber_Index(h.ptr()));
    if (!i.is_valid()) {
      throw nb::python_error();
    }
    const long long v = PyLong_AsLongLong(i.ptr());
    if (v == -1 && PyErr_Occurred() != nullptr) {
      throw nb::python_error();
    }
    if (v < 0) {
      throw nb::value_error("flux: a tensor's shape has no negative extent");
    }
    dims.push_back(v);
  }
  return dims;
}

// A tensor field in place. Handed out as itself only when numpy has no dtype for it (bfloat16,
// float8): its bytes as same-width unsigned integers, or a DLPack capsule.
class Tensor
{
public:
  Tensor(nb::handle owner, const flux_py::TensorFields & f, bool writable)
  : owner_(nb::borrow(owner)), f_(f), writable_(writable)
  {
  }

  [[nodiscard]] auto named() const -> bool { return npy_of(f_.dtype) >= 0; }

  [[nodiscard]] auto array() const -> nb::object
  {
    if (!named()) {
      throw nb::type_error(
        "flux: numpy has no dtype for this tensor (bfloat16 or float8): read .bits, or take it "
        "with __dlpack__ (torch.from_dlpack)");
    }
    return view(npy_of(f_.dtype));
  }

  [[nodiscard]] auto bits() const -> nb::object
  {
    const std::size_t size = flux::wire::dl_itemsize(f_.dtype);
    if (size == 16) {
      throw nb::type_error("flux: numpy has no 128-bit unsigned integer");
    }
    return view(
      size == 1   ? NPY_UINT8
      : size == 2 ? NPY_UINT16
      : size == 4 ? NPY_UINT32
                  : NPY_UINT64);
  }

  [[nodiscard]] auto dlpack() const -> nb::object
  {
    const auto addr = reinterpret_cast<std::uintptr_t>(f_.data);  // NOLINT(*-reinterpret-cast)
    return flux_py::tensor_dlpack(f_, addr, owner_, 1, 0);
  }

  [[nodiscard]] auto shape() const -> nb::tuple
  {
    nb::list l;
    for (const std::int64_t s : f_.shape) {
      l.append(s);
    }
    return nb::tuple(l);
  }

  [[nodiscard]] auto dtype() const -> nb::tuple
  {
    return nb::make_tuple(f_.dtype.code, f_.dtype.bits, f_.dtype.lanes);
  }

private:
  [[nodiscard]] auto view(int type) const -> nb::object
  {
    const std::size_t size = flux::wire::dl_itemsize(f_.dtype);
    std::vector<npy_intp> shape(f_.shape.begin(), f_.shape.end());
    std::vector<npy_intp> strides;
    bool empty = false;
    for (const std::int64_t n : f_.shape) {
      empty = empty || n == 0;
    }
    // A stride under an extent of 0 or 1 is never stepped, so tensor_fault() left it unbounded.
    for (std::size_t i = 0; i < f_.strides.size(); ++i) {
      const bool stepped = !empty && f_.shape[i] > 1;
      strides.push_back(stepped ? static_cast<npy_intp>(f_.strides[i] * size) : 0);
    }
    const nb::object descr = flux_py::own(PyArray_DescrFromType(type));
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast): read-only unless `writable_`
    void * p = const_cast<std::uint8_t *>(f_.data);
    return flux_py::numpy_view(
      p, static_cast<int>(shape.size()), shape.data(), flux_py::as_descr(descr), owner_, writable_,
      strides.empty() ? nullptr : strides.data());
  }

  nb::object owner_;
  flux_py::TensorFields f_;
  bool writable_;
};

auto handed_out(Tensor && t) -> nb::object
{
  return t.named() ? t.array() : nb::cast(std::move(t));
}

auto reader_tensor(nb::handle self, const flux_py::TensorAt & at, bool array) -> nb::object
{
  Tensor t(self, flux_py::read_tensor(nb::inst_ptr<Reader>(self)->reader(), at), false);
  return array ? handed_out(std::move(t)) : nb::cast(std::move(t));
}

auto writer_tensor(
  nb::handle self, const flux_py::TensorAt & at, nb::handle dtype, nb::handle shape) -> nb::object
{
  Writer & w = *nb::inst_ptr<Writer>(self);
  return poisoning(w, [&] {
    const flux::wire::DlType t = dl_named(dtype);
    std::vector<std::int64_t> dims = dims_of(shape);
    void * p = w.w().tensor(at_of(at), t, {dims.data(), dims.size()});
    w.check("tensor", at[6]);
    return handed_out(Tensor(self, {static_cast<std::uint8_t *>(p), t, std::move(dims), {}}, true));
  });
}

// What a Builder with no slot hands out: the tensor it asked for, in memory of its own.
auto scratch_tensor(nb::handle dtype, nb::handle shape) -> nb::object
{
  const flux::wire::DlType t = dl_named(dtype);
  std::vector<std::int64_t> dims = dims_of(shape);
  std::size_t bytes = flux::wire::dl_itemsize(t);
  for (const std::int64_t n : dims) {
    if (__builtin_mul_overflow(bytes, static_cast<std::size_t>(n), &bytes)) {
      throw nb::value_error("flux: tensor shape overflows");
    }
  }
  npy_intp n = static_cast<npy_intp>(bytes + flux::wire::kTensorAlign);
  const nb::object buf = nb::steal(PyArray_SimpleNew(1, &n, NPY_UINT8));
  if (!buf.is_valid()) {
    throw nb::python_error();
  }
  auto * p = static_cast<std::uint8_t *>(PyArray_DATA(flux_py::as_array(buf)));
  const auto a = reinterpret_cast<std::uintptr_t>(p);  // NOLINT(*-reinterpret-cast)
  p += flux::wire::detail::align_up(a, flux::wire::kTensorAlign) - a;
  return handed_out(Tensor(buf, {p, t, std::move(dims), {}}, true));
}

// A numeric array where the device reads it, as a DLPack tensor holding the reader; None where
// this reader has no device address for it.
auto reader_device(nb::handle self, std::size_t off, nb::handle dtype) -> nb::object
{
  const Reader & r = *nb::inst_ptr<Reader>(self);
  const DType t = dtype_of(dtype);
  const flux::wire::Reader w = r.reader();
  const auto [p, n] = by_size(t.size, [&](auto e) {
    const auto s = w.device<decltype(e)>(off);
    return std::pair<const void *, std::size_t>(s.data(), s.size());
  });
  Reader::check(w, "device array", off);
  return p == nullptr ? nb::none() : flux_py::device_span(self, p, n, dtype, r.device(), true);
}

// n elements in the GPU arena on the loan's device, as a DLPack tensor to write on the loan's
// stream; the commit waits for it.
auto writer_alloc_device(nb::handle self, std::size_t off, nb::handle count, nb::handle dtype)
  -> nb::object
{
  Writer & w = *nb::inst_ptr<Writer>(self);
  return poisoning(w, [&] {
    const std::size_t n = count_of(count);
    const DType t = dtype_of(dtype);
    const void * p = by_size(t.size, [&](auto e) -> const void * {
      return w.w().alloc_device<decltype(e)>(off, n).data();
    });
    if (w.bad()) {
      throw WireError(
        "flux: device array at " + std::to_string(off) +
        " found no room in the GPU arena, or the publisher declared no device");
    }
    return flux_py::device_span(self, p, n, dtype, w.device(), false);
  });
}

}  // namespace

auto flux_py::read_tensor(const flux::wire::Reader & w, const TensorAt & at)
  -> flux_py::TensorFields
{
  const flux::wire::DlType t{
    w.get<std::uint8_t>(at[0]), w.get<std::uint8_t>(at[1]), w.get<std::uint16_t>(at[2])};
  const auto shape = w.span<std::int64_t>(at[3]);
  const auto strides = w.span<std::int64_t>(at[4]);
  const auto byte_offset = w.get<std::uint64_t>(at[5]);
  const auto data = w.span<std::uint8_t>(at[6]);
  Reader::check(w, "tensor", at[6]);
  TensorFields f{nullptr, t, {shape.begin(), shape.end()}, {strides.begin(), strides.end()}};
  const char * fault = flux::wire::tensor_fault(
    data.data(), data.size(), t, {f.shape.data(), f.shape.size()},
    {f.strides.data(), f.strides.size()}, byte_offset);
  if (fault != nullptr) {
    throw WireError(
      "flux: tensor at " + std::to_string(at[6]) + " has " + fault + " (dtype code " +
      std::to_string(t.code) + ", bits " + std::to_string(t.bits) + ", lanes " +
      std::to_string(t.lanes) + ")");
  }
  f.data = data.data() + byte_offset;
  return f;
}

auto flux_py::tensor_dlpack(
  const TensorFields & t, std::uintptr_t addr, nb::handle owner, int device_type,
  int device_id) -> nb::object
{
  const std::vector<std::size_t> shape(t.shape.begin(), t.shape.end());
  const nb::ndarray<> arr(
    reinterpret_cast<void *>(addr),  // NOLINT(*-reinterpret-cast,performance-no-int-to-ptr)
    shape.size(), shape.data(), owner, t.strides.empty() ? nullptr : t.strides.data(),
    nb::dlpack::dtype{t.dtype.code, t.dtype.bits, 1}, device_type, device_id);
  return nb::cast(arr);
}

void bind_wire(nb::module_ & m)
{
  static nb::exception<WireError> wire_error(m, "WireError", PyExc_ValueError);

  nb::class_<Reader>(m, "Reader")
    .def(nb::init<nb::handle>(), "buf"_a)
    .def("__len__", &Reader::size)
    .def("budget", [](const Reader & r) { return r.reader().budget(); })
    .def("get", &reader_get, "off"_a, "dtype"_a)
    .def("block", &reader_block, "off"_a, "dtype"_a, "count"_a)
    .def(
      "desc",
      [](const Reader & r, std::size_t off) {
        const flux::wire::Reader w = r.reader();
        const flux::wire::Ref d = w.desc(off);
        Reader::check(w, "descriptor", off);
        return std::pair<std::size_t, std::size_t>(d.off, d.len);
      },
      "off"_a)
    .def("span", &reader_span, "off"_a, "dtype"_a)
    .def("string", &reader_string, "off"_a)
    .def("strings", &reader_strings, "off"_a)
    .def(
      "elem",
      [](const Reader & r, std::size_t off, std::size_t i, std::size_t stride, std::size_t align) {
        check_layout(stride, align);
        const flux::wire::Reader w = r.reader();
        const std::size_t at = w.elem(off, i, stride, align);
        Reader::check(w, "element", off);
        return at;
      },
      "off"_a, "i"_a, "stride"_a, "align"_a)
    .def("elems", &reader_elems, "off"_a, "stride"_a, "align"_a)
    .def("device", &reader_device, "off"_a, "dtype"_a)
    .def("tensor", &reader_tensor, "at"_a, "array"_a = true);

  nb::class_<Tensor>(m, "Tensor")
    .def_prop_ro("array", &Tensor::array)
    .def_prop_ro("bits", &Tensor::bits)
    .def_prop_ro("shape", &Tensor::shape)
    .def_prop_ro("dtype", &Tensor::dtype)
    .def("__dlpack__", [](const Tensor & t, const nb::kwargs &) { return t.dlpack(); })
    .def("__dlpack_device__", [](const Tensor &) { return nb::make_tuple(1, 0); });
  m.def("_scratch_tensor", &scratch_tensor, "dtype"_a, "shape"_a);

  nb::class_<Writer>(m, "Writer")
    .def(
      nb::init<nb::handle, std::size_t, nb::handle>(), "buf"_a, "scalar_bytes"_a,
      "heap"_a = nb::none())
    .def_prop_ro("bad", &Writer::bad)
    .def_prop_ro("size", &Writer::size)
    .def("poison", &Writer::poison)
    // .none(): a None value is refused, and latches, inside the call rather than by nanobind.
    .def("put", &writer_put, "off"_a, "dtype"_a, "value"_a.none())
    .def("block_view", &writer_block, "off"_a, "dtype"_a, "count"_a)
    .def("alloc", &writer_alloc, "off"_a, "n"_a.none(), "dtype"_a)
    .def("alloc_device", &writer_alloc_device, "off"_a, "n"_a.none(), "dtype"_a)
    .def("adopt", &writer_adopt, "off"_a, "array"_a, "dtype"_a)
    .def("tensor", &writer_tensor, "at"_a, "dtype"_a, "shape"_a.none())
    .def("put_str", &writer_put_str, "off"_a, "s"_a.none())
    .def("put_strs", &writer_put_strs, "off"_a, "items"_a.none(), "count"_a = nb::none())
    .def("alloc_elems", &writer_alloc_elems, "off"_a, "n"_a.none(), "stride"_a, "align"_a);
}
