// Python over flux's C ABI, through the same RAII layer as flux_cpp (flux::detail). The ROS
// half (node names, the rclpy executor bridge) is Python, in flux/__init__.py.
#define FLUX_PY_IMPORT_NUMPY
#include "bind_wire.hpp"
#include "flux/channel.hpp"
#include "flux/detail/executor.hpp"
#include "flux/detail/manifest.hpp"
#include "flux/detail/ros_mirror.hpp"
#include "flux/detail/subscriber.hpp"
#include "flux/schemas.hpp"
#include "flux/topics.hpp"
#include "flux/trace.h"
#include "numpy_api.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;
using nb::dlpack::dtype_code;

namespace
{

auto finalizing() -> bool
{
#if PY_VERSION_HEX >= 0x030D0000
  return Py_IsFinalizing() != 0;
#else
  return _Py_IsFinalizing() != 0;
#endif
}

// Threads that have decided to take the GIL back and do not hold it yet, and whether the
// interpreter is on its way out (set at exit, before finalization begins).
auto regaining() -> std::atomic<int> &
{
  static std::atomic<int> n{0};
  return n;
}

auto exiting() -> std::atomic<bool> &
{
  static std::atomic<bool> on{false};
  return on;
}

// Before 3.14, a thread that takes the GIL back while Python finalizes is ended by pthread_exit,
// which aborts as it unwinds through noexcept frames. 3.14 parks the thread; so does this. A thread
// already waiting for the GIL when finalization begins would still be ended, so before_exit()
// lets every such thread through first.
void take_gil_back(PyThreadState * ts)
{
  flux_trace_py_gil(0);
  regaining().fetch_add(1);
  if (exiting().load() || (PY_VERSION_HEX < 0x030E0000 && finalizing())) {
    regaining().fetch_sub(1);
    for (;;) {
      pause();
    }
  }
  PyEval_RestoreThread(ts);
  flux_trace_py_gil(1);
  regaining().fetch_sub(1);
}

// From atexit, with the GIL.
void before_exit()
{
  exiting().store(true);
  Py_BEGIN_ALLOW_THREADS;
  while (regaining().load() != 0) {
    std::this_thread::yield();
  }
  Py_END_ALLOW_THREADS;
}

class Unlocked
{
public:
  Unlocked() noexcept : ts_(PyEval_SaveThread()) {}
  ~Unlocked() { take_gil_back(ts_); }

  // Runs f with the GIL, which f must not leave by an exception.
  template <class F>
  auto with_gil(F f)
  {
    take_gil_back(ts_);
    const auto r = f();
    ts_ = PyEval_SaveThread();
    return r;
  }
  Unlocked(const Unlocked &) = delete;
  auto operator=(const Unlocked &) -> Unlocked & = delete;
  Unlocked(Unlocked &&) = delete;
  auto operator=(Unlocked &&) -> Unlocked & = delete;

private:
  PyThreadState * ts_;
};

// Runs f without the GIL, as a wait on a stream must: work queued on it may need the GIL. Not on
// the way out, where taking the GIL back would park this thread for good.
template <class F>
auto without_gil(F f)
{
  if (exiting().load() || finalizing()) {
    return f();
  }
  const Unlocked unlocked;
  return f();
}

auto dl(dtype_code code, std::uint8_t bits) -> nb::dlpack::dtype
{
  return nb::dlpack::dtype{static_cast<std::uint8_t>(code), bits, 1};
}

auto dtype_of(nb::dlpack::dtype t) -> flux::DType
{
  const auto code = static_cast<dtype_code>(t.code);
  const std::uint8_t bits = t.bits;
  if (t.lanes == 1) {
    if (code == dtype_code::UInt) {
      if (bits == 8) {
        return flux::DType::U8;
      }
      if (bits == 16) {
        return flux::DType::U16;
      }
      if (bits == 32) {
        return flux::DType::U32;
      }
      if (bits == 64) {
        return flux::DType::U64;
      }
    } else if (code == dtype_code::Int) {
      if (bits == 8) {
        return flux::DType::I8;
      }
      if (bits == 16) {
        return flux::DType::I16;
      }
      if (bits == 32) {
        return flux::DType::I32;
      }
      if (bits == 64) {
        return flux::DType::I64;
      }
    } else if (code == dtype_code::Float) {
      if (bits == 16) {
        return flux::DType::F16;
      }
      if (bits == 32) {
        return flux::DType::F32;
      }
      if (bits == 64) {
        return flux::DType::F64;
      }
    } else if (code == dtype_code::Bfloat && bits == 16) {
      return flux::DType::BF16;
    }
  }
  throw nb::type_error("flux: unsupported dtype; use a numpy integer or float dtype");
}

auto dtype_named(const nb::handle & dtype) -> flux::DType
{
  PyArray_Descr * raw = nullptr;
  if (PyArray_DescrConverter(dtype.ptr(), &raw) != NPY_SUCCEED) {
    throw nb::python_error();
  }
  const nb::object d = flux_py::own(raw);
  // A TypeError, as publish() gives for such an array.
  if (!PyArray_ISNBO(raw->byteorder)) {
    throw nb::type_error("flux: frames are in native byte order; use a native dtype");
  }
  // bfloat16 is a user dtype (ml_dtypes). Builtins skip the name, which numpy builds in Python.
  if (raw->type_num >= NPY_USERDEF && nb::cast<std::string>(d.attr("name")) == "bfloat16") {
    return flux::DType::BF16;
  }
  const char kind = raw->kind;
  const auto bits = static_cast<std::uint8_t>(8 * flux_py::elsize(raw));
  const dtype_code code = kind == 'u'   ? dtype_code::UInt
                          : kind == 'i' ? dtype_code::Int
                          : kind == 'f' ? dtype_code::Float
                                        : dtype_code::Complex;
  return dtype_of(dl(code, bits));
}

auto descr_of(flux::DType d) -> nb::object
{
  return flux_py::own(PyArray_DescrFromType(flux_py::npy_type(d)));
}

// Names the capsule a host view's array holds its FrameView in, so aux_of can find it.
constexpr const char * kViewCapsule = "flux.view";
// And the one a Frame's arrays hold its FrameHold in, so a wire Reader can find the view.
constexpr const char * kFrameCapsule = "flux.frame";

// The core's take already checked that the shape covers exactly the frame's bytes.
auto view_of(flux::FrameView && v) -> nb::object
{
  const flux::FrameMeta m = v.meta();
  std::array<npy_intp, flux::kMaxDims> shape{};
  for (std::size_t i = 0; i < m.ndim; ++i) {
    shape.at(i) = static_cast<npy_intp>(m.shape[i]);
  }
  auto held = std::make_unique<flux::FrameView>(std::move(v));
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast): numpy's signature; the view is read-only
  void * data = const_cast<void *>(held->data());
  const nb::capsule owner(held.release(), kViewCapsule, [](void * p) noexcept {
    const std::unique_ptr<flux::FrameView> gone(static_cast<flux::FrameView *>(p));
  });
  const nb::object descr = descr_of(static_cast<flux::DType>(m.dtype));
  return flux_py::numpy_view(data, m.ndim, shape.data(), flux_py::as_descr(descr), owner, false);
}

auto view_or_none(flux::FrameView && v) -> nb::object
{
  if (!v) {
    return nb::none();
  }
  return view_of(std::move(v));
}

// Device addresses travel as integers through both GPU protocols.
auto addr_of(const void * p) noexcept -> std::uintptr_t
{
  return reinterpret_cast<std::uintptr_t>(p);  // NOLINT(*-reinterpret-cast)
}

auto ptr_of(std::uint64_t a) noexcept -> void *
{
  return reinterpret_cast<void *>(  // NOLINT(*-reinterpret-cast,performance-no-int-to-ptr)
    static_cast<std::uintptr_t>(a));
}

// numpy's spelling, which the CUDA array interface takes; bfloat16 has none.
auto typestr(flux::DType d) -> const char *
{
  switch (d) {
    case flux::DType::U8:
      return "|u1";
    case flux::DType::I8:
      return "|i1";
    case flux::DType::U16:
      return "<u2";
    case flux::DType::I16:
      return "<i2";
    case flux::DType::U32:
      return "<u4";
    case flux::DType::I32:
      return "<i4";
    case flux::DType::U64:
      return "<u8";
    case flux::DType::I64:
      return "<i8";
    case flux::DType::F16:
      return "<f2";
    case flux::DType::F32:
      return "<f4";
    case flux::DType::F64:
      return "<f8";
    case flux::DType::BF16:
      break;
  }
  throw nb::type_error(
    "flux: bfloat16 has no __cuda_array_interface__ typestr; take it with __dlpack__ "
    "(torch.from_dlpack)");
}

auto dl_of(flux::DType d) -> nb::dlpack::dtype
{
  const auto bits = static_cast<std::uint8_t>(8 * flux::dtype_size(d));
  switch (d) {
    case flux::DType::U8:
    case flux::DType::U16:
    case flux::DType::U32:
    case flux::DType::U64:
      return dl(dtype_code::UInt, bits);
    case flux::DType::I8:
    case flux::DType::I16:
    case flux::DType::I32:
    case flux::DType::I64:
      return dl(dtype_code::Int, bits);
    case flux::DType::BF16:
      return dl(dtype_code::Bfloat, bits);
    case flux::DType::F16:
    case flux::DType::F32:
    case flux::DType::F64:
      break;
  }
  return dl(dtype_code::Float, bits);
}

// Device memory of a frame or loan, as the two GPU protocols describe it. No stream to sync on:
// commit and release wait for the channel's stream, so the bytes are complete when handed out.
struct DeviceArray
{
  std::uint64_t ptr;
  flux::DType dtype;
  std::vector<std::size_t> shape;
  int device;
  bool readonly;
};

auto cuda_array_interface(const DeviceArray & a) -> nb::dict
{
  nb::list shape;
  for (const std::size_t s : a.shape) {
    shape.append(s);
  }
  nb::dict d;
  d["shape"] = nb::tuple(shape);
  d["typestr"] = typestr(a.dtype);
  d["data"] = nb::make_tuple(a.ptr, a.readonly);
  d["strides"] = nb::none();
  d["stream"] = nb::none();
  d["version"] = 3;
  return d;
}

// A "dltensor" capsule, which nanobind makes of an ndarray cast with no framework. It holds
// `owner` as long as the consumer's tensor lives. Not marked read-only, which torch refuses: a
// subscriber's mapping is read-only on the device already.
auto dlpack_of(const DeviceArray & a, nb::handle owner) -> nb::object
{
  const nb::ndarray<> arr(
    ptr_of(a.ptr), a.shape.size(), a.shape.data(), owner, nullptr, dl_of(a.dtype),
    nb::device::cuda::value, a.device);
  return nb::cast(arr);
}

// The stream a DLPack consumer names, which the channel's own fence does not cover. None and -1
// ask for no ordering, and CUDA orders the legacy default stream (1) with the channel's blocking
// stream itself.
void remember_stream(std::vector<flux::gpu::Stream> & streams, const nb::kwargs & kw)
{
  if (!kw.contains("stream") || kw["stream"].is_none()) {
    return;
  }
  const auto handle = nb::cast<std::intptr_t>(kw["stream"]);
  if (handle == -1 || handle == 0 || handle == 1) {
    return;
  }
  void * native = ptr_of(static_cast<std::uint64_t>(handle));
  for (const flux::gpu::Stream & s : streams) {
    if (s.native() == native) {
      return;
    }
  }
  streams.push_back(flux::gpu::Stream::from_native(native));
}

auto wait_all(const std::vector<flux::gpu::Stream> & streams) -> bool
{
  for (const flux::gpu::Stream & s : streams) {
    if (!s.wait()) {
      return false;
    }
  }
  return true;
}

// A frame's view and the streams its memory was handed to. It goes when its last holder does: the
// Frame's scope, or a tensor that outlived it. The release waits for those streams first; one that
// fails keeps the borrow, as a failed fence does.
struct FrameHold
{
  flux::FrameView v;
  std::vector<flux::gpu::Stream> streams;
  // The channel defers releases: the consumer streams are ordered before the channel's on the
  // device instead, and the view's own release returns at once.
  bool deferred;
  // The channel stream's device.
  int device;

  FrameHold(const FrameHold &) = delete;
  auto operator=(const FrameHold &) -> FrameHold & = delete;
  FrameHold(FrameHold &&) = delete;
  auto operator=(FrameHold &&) -> FrameHold & = delete;
  FrameHold(flux::FrameView && view, bool defer, int dev)
  : v(std::move(view)), deferred(defer), device(dev)
  {
  }
  ~FrameHold()
  {
    if (deferred && ordered()) {
      return;
    }
    if (wait_all(streams)) {
      return;
    }
    // NOLINTNEXTLINE(bugprone-unused-return-value)
    std::make_unique<flux::FrameView>(std::move(v)).release();
  }
  [[nodiscard]] auto ordered() const -> bool
  {
    for (const flux::gpu::Stream & s : streams) {
      if (!v.after(s)) {
        return false;
      }
    }
    return true;
  }
};

auto shape_of(const flux::FrameMeta & m) -> std::vector<std::size_t>
{
  return {m.shape, m.shape + m.ndim};
}

// A frame of a channel that declared a stream. Its device memory is handed out only inside a
// `with` block or a callback, and the release waits for the stream where the scope ends, unless
// the memory is still held: a DLPack tensor holds it until the tensor goes, and a CUDA array
// interface consumer, which holds the Frame itself, until the Frame goes. Shape, dtype and stream
// are the frame's and the channel's, readable at any time.
class Frame
{
public:
  Frame(flux::FrameView && v, int device, bool scoped, bool deferred)
  : meta_(v.meta()),
    stream_(addr_of(v.stream())),
    held_(std::make_shared<FrameHold>(std::move(v), deferred, device)),
    device_(device),
    scoped_(scoped)
  {
  }
  ~Frame()
  {
    if (held_) {
      without_gil([this] { held_.reset(); });
    }
  }
  Frame(Frame &&) noexcept = default;
  auto operator=(Frame &&) noexcept -> Frame & = default;
  Frame(const Frame &) = delete;
  auto operator=(const Frame &) -> Frame & = delete;

  void enter()
  {
    if (!valid()) {
      throw std::runtime_error("flux: the frame is already released");
    }
    scoped_ = true;
  }

  void release()
  {
    scoped_ = false;
    if (!held_ || cai_out_) {
      return;
    }
    without_gil([this] { held_.reset(); });
  }

  auto cai() -> nb::dict
  {
    nb::dict d = cuda_array_interface(array());
    cai_out_ = true;
    return d;
  }

  auto dlpack(const nb::kwargs & kw) -> nb::object
  {
    const DeviceArray a = array();
    remember_stream(held_->streams, kw);
    return dlpack_of(a, hold());
  }

  // Holds the frame as a DLPack tensor does.
  auto host() -> nb::object
  {
    const flux::FrameView & v = host_view();
    const nb::object descr = descr_of(flux::DType::U8);
    return flux_py::numpy_view(v.data(), v.size(), flux_py::as_descr(descr), hold(), false);
  }

  // A tensor message's data on the device: its fields are read through the host mapping, and the
  // device address is where the frame's data field is on the device.
  auto tensor_dlpack(const flux_py::TensorAt & at, const nb::kwargs & kw) -> nb::object
  {
    const flux::FrameView & v = host_view();
    const flux::wire::Reader r(v.data(), v.size(), v.heap(), v.gpu_heap());
    const flux_py::TensorFields f = flux_py::read_tensor(r, at);
    const auto host = r.span<std::uint8_t>(at[6]);
    const auto dev = r.device<std::uint8_t>(at[6]);
    if (dev.data() == nullptr) {
      throw std::runtime_error(
        "flux: this subscriber has no device address for the tensor's data (an integrated GPU "
        "reads a field in the publisher's arena on the host only)");
    }
    remember_stream(held_->streams, kw);
    const auto addr = addr_of(dev.data() + (f.data - host.data()));
    return flux_py::tensor_dlpack(f, addr, hold(), nb::device::cuda::value, device_);
  }

  [[nodiscard]] auto dlpack_device() const -> nb::tuple
  {
    return nb::make_tuple(nb::device::cuda::value, device_);
  }

  // Host memory whatever the payload is, so readable outside a `with` block too.
  [[nodiscard]] auto aux() const -> nb::object
  {
    if (!valid()) {
      throw std::runtime_error("flux: the frame is already released");
    }
    const auto a = held_->v.aux();
    const nb::object descr = descr_of(flux::DType::U8);
    return flux_py::numpy_view(a.data(), a.size(), flux_py::as_descr(descr), hold(), false);
  }

  [[nodiscard]] auto valid() const noexcept -> bool { return held_ && held_->v.valid(); }
  [[nodiscard]] auto shape() const -> nb::tuple
  {
    nb::list l;
    for (const std::size_t s : shape_of(meta_)) {
      l.append(s);
    }
    return nb::tuple(l);
  }
  [[nodiscard]] auto dtype() const -> nb::object
  {
    return descr_of(static_cast<flux::DType>(meta_.dtype));
  }
  [[nodiscard]] auto nbytes() const noexcept -> std::uint64_t { return meta_.nbytes; }
  [[nodiscard]] auto stream() const noexcept -> std::uintptr_t { return stream_; }

private:
  [[nodiscard]] auto hold() const -> nb::capsule
  {
    return nb::capsule(new std::shared_ptr<FrameHold>(held_), kFrameCapsule, [](void * p) noexcept {
      auto * hold = static_cast<std::shared_ptr<FrameHold> *>(p);
      // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
      without_gil([hold] { delete hold; });
    });
  }

  // The host mapping a generated View reads a frame's fields through, which a route without one
  // (DeviceHandle) cannot give.
  [[nodiscard]] auto host_view() const -> const flux::FrameView &
  {
    in_scope();
    if (held_->v.data() == nullptr) {
      throw std::runtime_error(
        "flux: this GPU route has no host mapping to read a frame's fields through");
    }
    return held_->v;
  }

  void in_scope() const
  {
    if (!valid()) {
      throw std::runtime_error("flux: the frame is already released");
    }
    if (!scoped_) {
      throw std::runtime_error(
        "flux: a CUDA frame's memory is handed out only inside `with sub.take() as frame:` or a "
        "callback, since its release waits for the stream");
    }
  }

  [[nodiscard]] auto array() const -> DeviceArray
  {
    in_scope();
    return {
      addr_of(held_->v.device_ptr()), static_cast<flux::DType>(meta_.dtype), shape_of(meta_),
      device_, true};
  }

  flux::FrameMeta meta_;
  std::uintptr_t stream_;
  std::shared_ptr<FrameHold> held_;
  int device_;
  bool scoped_;
  bool cai_out_ = false;
};

auto frame_or_none(flux::FrameView && v, int device, bool deferred) -> nb::object
{
  if (!v) {
    return nb::none();
  }
  return nb::cast(Frame(std::move(v), device, false, deferred));
}

// The aux bytes of what a take returned, a host view or a Frame: read-only uint8, holding it.
auto aux_of(nb::handle taken) -> nb::object
{
  if (nb::isinstance<Frame>(taken)) {
    return nb::inst_ptr<Frame>(taken)->aux();
  }
  PyObject * base =
    PyArray_Check(taken.ptr()) != 0 ? PyArray_BASE(flux_py::as_array(taken)) : nullptr;
  if (base == nullptr || PyCapsule_IsValid(base, kViewCapsule) == 0) {
    throw nb::type_error("flux: aux() takes a frame that a subscription's take returned");
  }
  const auto * v = static_cast<const flux::FrameView *>(PyCapsule_GetPointer(base, kViewCapsule));
  const auto a = v->aux();
  const nb::object descr = descr_of(flux::DType::U8);
  return flux_py::numpy_view(a.data(), a.size(), flux_py::as_descr(descr), taken, false);
}

}  // namespace

namespace
{

// A field's device memory, from a generated View's `<field>__device` or a Builder's
// `alloc_device__<field>`: both GPU protocols over it, holding `owner` (the reader, which holds the
// frame, or the writer) while a consumer's tensor lives.
class DeviceSpan
{
public:
  DeviceSpan(const DeviceArray & a, nb::handle owner) : a_(a), owner_(nb::borrow(owner)) {}

  [[nodiscard]] auto cai() const -> nb::dict { return cuda_array_interface(a_); }
  [[nodiscard]] auto dlpack(const nb::kwargs & /*kw*/) const -> nb::object
  {
    return dlpack_of(a_, owner_);
  }
  [[nodiscard]] auto dlpack_device() const -> nb::tuple
  {
    return nb::make_tuple(nb::device::cuda::value, a_.device);
  }
  [[nodiscard]] auto size() const noexcept -> std::size_t { return a_.shape[0]; }
  [[nodiscard]] auto nbytes() const noexcept -> std::size_t
  {
    return a_.shape[0] * flux::dtype_size(a_.dtype);
  }
  [[nodiscard]] auto dtype() const -> nb::object { return descr_of(a_.dtype); }

private:
  DeviceArray a_;
  nb::object owner_;
};

}  // namespace

auto flux_py::device_span(
  nb::handle owner, const void * p, std::size_t n, nb::handle dtype, int device,
  bool readonly) -> nb::object
{
  return nb::cast(DeviceSpan({addr_of(p), dtype_named(dtype), {n}, device, readonly}, owner));
}

auto flux_py::frame_of(nb::handle buf, int * device) -> const flux::FrameView *
{
  PyObject * base = PyArray_Check(buf.ptr()) != 0 ? PyArray_BASE(flux_py::as_array(buf)) : nullptr;
  if (base != nullptr && PyCapsule_IsValid(base, kViewCapsule) != 0) {
    *device = -1;
    return static_cast<const flux::FrameView *>(PyCapsule_GetPointer(base, kViewCapsule));
  }
  if (base != nullptr && PyCapsule_IsValid(base, kFrameCapsule) != 0) {
    const auto * h =
      static_cast<const std::shared_ptr<FrameHold> *>(PyCapsule_GetPointer(base, kFrameCapsule));
    *device = (*h)->device;
    return &(*h)->v;
  }
  return nullptr;
}

namespace
{

// Writable numpy views of a loaned slot. commit and abort make the arrays `array` handed out
// read-only; a slice or memoryview taken from one before then can still write to the slot.
class Loan
{
public:
  Loan(flux::WriteSlot && w, flux::DType dtype, const std::vector<std::size_t> & shape, int device)
  : w_(std::move(w)), dtype_(dtype), shape_(shape.begin(), shape.end()), device_(device)
  {
  }
  // A loan dropped unfinished is aborted only once its tensors' writes are done.
  ~Loan() { end(); }
  Loan(Loan &&) noexcept = default;
  auto operator=(Loan &&) noexcept -> Loan & = default;
  Loan(const Loan &) = delete;
  auto operator=(const Loan &) -> Loan & = delete;

  auto array(nb::handle self) -> nb::object
  {
    if (!w_) {
      throw std::runtime_error("flux: the loan is already committed or aborted");
    }
    if (w_.data() == nullptr) {
      throw std::runtime_error(
        "flux: the loan's payload is device memory; write it through __cuda_array_interface__ "
        "or __dlpack__");
    }
    const nb::object descr = descr_of(dtype_);
    nb::object a = flux_py::numpy_view(
      w_.data(), static_cast<int>(shape_.size()), shape_.data(), flux_py::as_descr(descr), self,
      true);
    issued_.emplace_back(a);
    return a;
  }

  // The slot's whole aux region, sealed with the arrays at commit.
  auto aux(nb::handle self) -> nb::object
  {
    if (!w_) {
      throw std::runtime_error("flux: the loan is already committed or aborted");
    }
    const auto a = w_.aux();
    const nb::object descr = descr_of(flux::DType::U8);
    nb::object o = flux_py::numpy_view(a.data(), a.size(), flux_py::as_descr(descr), self, true);
    issued_.emplace_back(o);
    return o;
  }
  [[nodiscard]] auto aux_len() const noexcept -> std::size_t { return aux_len_; }
  // Past the aux region, the commit is TOO_LARGE.
  void set_aux_len(std::size_t n)
  {
    if (!w_) {
      throw std::runtime_error("flux: the loan is already committed or aborted");
    }
    w_.set_aux_len(n);
    aux_len_ = n;
  }

  // A refused commit ends the loan too, as in C++: commit(nbytes) on a loan that is not 1-D, or
  // with nbytes past it or not a whole number of items, is TOO_LARGE.
  auto commit(std::optional<std::size_t> nbytes) -> flux::Published
  {
    const std::optional<flux::Published> p = without_gil([&]() -> std::optional<flux::Published> {
      if (!writers_done()) {
        return std::nullopt;
      }
      return nbytes ? w_.commit(*nbytes) : w_.commit();
    });
    if (!p) {
      return flux::Published::FenceFailed;
    }
    seal();
    return *p;
  }

  // After a commit, w_ keeps the mapping that sealed arrays still read; only the Loan's end
  // lets go of it.
  void abort()
  {
    end();
    seal();
  }

  [[nodiscard]] auto valid() const noexcept -> bool { return w_.valid(); }
  auto capacity() noexcept -> std::size_t { return w_.capacity(); }

  // The loan's arena handle for a wire Writer, None without an arena.
  [[nodiscard]] auto heap() const -> nb::object
  {
    const flux::wire::Heap h = w_.heap();
    if (h.channel == nullptr) {
      return nb::none();
    }
    return nb::make_tuple(
      reinterpret_cast<std::uintptr_t>(h.base), h.size,     // NOLINT(*-reinterpret-cast)
      reinterpret_cast<std::uintptr_t>(h.channel), h.slot,  // NOLINT(*-reinterpret-cast)
      h.alloc_device != nullptr ? device_ : -1);
  }

  auto cai() -> nb::dict { return cuda_array_interface(device_array()); }
  auto dlpack(nb::handle self, const nb::kwargs & kw) -> nb::object
  {
    const DeviceArray a = device_array();
    remember_stream(writers_, kw);
    return dlpack_of(a, self);
  }
  [[nodiscard]] auto dlpack_device() const -> nb::tuple
  {
    return nb::make_tuple(nb::device::cuda::value, device_);
  }
  [[nodiscard]] auto stream() const noexcept -> std::uintptr_t { return addr_of(w_.stream()); }

private:
  void end()
  {
    without_gil([this] {
      if (w_.valid() && writers_done()) {
        w_.abort();
      }
    });
  }

  // The streams the loan's tensors were handed to, waited for before the slot is published or
  // given back. One that fails keeps the claim, as a failed commit fence does.
  auto writers_done() -> bool
  {
    if (wait_all(writers_)) {
      return true;
    }
    // NOLINTNEXTLINE(bugprone-unused-return-value)
    std::make_unique<flux::WriteSlot>(std::move(w_)).release();
    return false;
  }

  auto device_array() -> DeviceArray
  {
    if (!w_) {
      throw std::runtime_error("flux: the loan is already committed or aborted");
    }
    if (w_.device_ptr() == nullptr) {
      throw std::runtime_error(
        "flux: the publisher declared no device; write the loan through .array");
    }
    return {
      addr_of(w_.device_ptr()), dtype_, std::vector<std::size_t>(shape_.begin(), shape_.end()),
      device_, false};
  }

  void seal()
  {
    for (const nb::weakref & ref : issued_) {
      const nb::object a = ref();
      if (!a.is_none()) {
        PyArray_CLEARFLAGS(flux_py::as_array(a), NPY_ARRAY_WRITEABLE);
      }
    }
    issued_.clear();
  }

  flux::WriteSlot w_;
  flux::DType dtype_;
  std::vector<npy_intp> shape_;
  int device_;
  std::vector<nb::weakref> issued_;
  std::vector<flux::gpu::Stream> writers_;
  std::size_t aux_len_ = 0;
};

class Publisher
{
public:
  Publisher(
    const std::string & signpost, const std::string & topic, const std::string & label,
    std::uint64_t fingerprint, std::uint32_t slot_size, std::uint32_t slot_count,
    flux::MemoryPolicy mem, std::uint32_t max_subscribers, bool cuda, std::uint32_t aux_size)
  : stream_(flux::gpu::stream_for(cuda ? flux::Device::Cuda : flux::Device::Cpu)),
    ch_(flux::detail::Channel::create(
      signpost, slot_size, slot_count, fingerprint, mem, max_subscribers, stream_, aux_size)),
    announced_(signpost, topic, label, true)
  {
    flux_trace_endpoint_init(
      this, label.c_str(), topic.c_str(), signpost.c_str(), fingerprint, FLUX_TRACE_ROLE_PUBLISHER,
      FLUX_TRACE_LANG_PYTHON);
  }

  // More than kMaxDims dimensions is the channel's TOO_LARGE, as in C++.
  auto publish(nb::ndarray<nb::ro, nb::c_contig, nb::device::cpu> a) -> flux::Published
  {
    const flux::DType dtype = dtype_of(a.dtype());
    std::vector<std::uint64_t> shape(a.ndim());
    for (std::size_t i = 0; i < a.ndim(); ++i) {
      shape[i] = a.shape(i);
    }
    const Unlocked unlocked;
    return ch_.publish(a.data(), dtype, shape.data(), a.ndim());
  }

  // None while every slot is borrowed. Without a shape the loan is the whole slot.
  auto loan(std::optional<std::vector<std::size_t>> shape, nb::handle dtype) -> std::optional<Loan>
  {
    const flux::DType d = dtype_named(dtype);
    std::vector<std::size_t> dims =
      shape ? *shape : std::vector<std::size_t>{ch_.slot_size() / flux::dtype_size(d)};
    if (dims.size() > flux::kMaxDims) {
      throw nb::value_error("flux: more than 8 dimensions");
    }
    // The numpy view spans the shape, so a shape past the slot would write into the next one.
    std::uint64_t nbytes = flux::dtype_size(d);
    for (const std::size_t s : dims) {
      if (s != 0 && nbytes > UINT64_MAX / s) {
        throw nb::value_error("flux: loan shape overflows");
      }
      nbytes *= s;
    }
    if (nbytes > ch_.slot_size()) {
      throw nb::value_error(("flux: a loan of " + std::to_string(nbytes) +
                             " bytes exceeds slot_size " + std::to_string(ch_.slot_size()))
                              .c_str());
    }
    const std::vector<std::uint64_t> wide(dims.begin(), dims.end());
    flux::WriteSlot w = ch_.loan(d, wide.data(), wide.size());
    if (!w) {
      return std::nullopt;
    }
    return Loan(std::move(w), d, dims, stream_.device());
  }

  [[nodiscard]] auto dropped() const noexcept -> std::uint64_t { return ch_.dropped(); }
  [[nodiscard]] auto subscriber_count() const noexcept -> std::uint32_t
  {
    return ch_.subscriber_count();
  }
  [[nodiscard]] auto slot_size() const noexcept -> std::uint32_t { return ch_.slot_size(); }
  auto channel() noexcept -> flux::detail::Channel & { return ch_; }
  [[nodiscard]] auto stream() const noexcept -> const flux::gpu::Stream & { return stream_; }

private:
  flux::gpu::Stream stream_;
  flux::detail::Channel ch_;
  flux::detail::Announcement announced_;
};

class Subscription
{
public:
  Subscription(
    const std::string & signpost, const std::string & topic, const std::string & label,
    std::uint64_t fingerprint, const flux::QoS & qos, flux::MemoryPolicy mem, bool cuda)
  : sub_(
      signpost, fingerprint, qos, mem,
      flux::gpu::stream_for(cuda ? flux::Device::Cuda : flux::Device::Cpu)),
    announced_(signpost, topic, label, false)
  {
    flux_trace_endpoint_init(
      this, label.c_str(), topic.c_str(), signpost.c_str(), fingerprint,
      FLUX_TRACE_ROLE_SUBSCRIPTION, FLUX_TRACE_LANG_PYTHON);
    sub_.attach();
  }

  auto take() -> nb::object { return wrap(sub_.take()); }

  // Ctrl-C, and `running` once it returns False, end the wait between its slices.
  auto take_blocking(std::int64_t timeout_ns, const nb::object & running) -> nb::object
  {
    // With the GIL. An error is left set, to be raised once the wait is over.
    const auto stop = [&] {
      try {
        if (PyErr_CheckSignals() != 0) {
          return true;
        }
        return !running.is_none() && PyObject_IsTrue(running().ptr()) != 1;
      } catch (nb::python_error & e) {
        e.restore();
        return true;
      }
    };
    flux::FrameView v;
    if (!stop()) {
      Unlocked unlocked;
      v = sub_.take_blocking(timeout_ns, [&] { return unlocked.with_gil(stop); });
    }
    if (PyErr_Occurred() != nullptr) {
      throw nb::python_error();
    }
    return wrap(std::move(v));
  }

  // A numpy view, or a Frame on a channel that declared a stream.
  auto wrap(flux::FrameView && v) -> nb::object
  {
    const flux::gpu::Stream & s = sub_.stream();
    const bool deferred = sub_.release_mode() == flux::gpu::ReleaseMode::Deferred;
    return s.declared() ? frame_or_none(std::move(v), s.device(), deferred)
                        : view_or_none(std::move(v));
  }

  auto core() noexcept -> flux::detail::Subscriber & { return sub_; }

private:
  flux::detail::Subscriber sub_;
  flux::detail::Announcement announced_;
};

// Publisher(mirror_to_ros=True)'s reader: the thread that publishes on rclpy is Python's.
class RosMirror
{
public:
  RosMirror(Publisher & publisher, const std::string & signpost, std::uint64_t fingerprint)
  : core_(publisher.channel(), signpost, fingerprint)
  {
  }

  // The CDR bytes of the next frame, or None after `timeout_ns`.
  auto next(std::int64_t timeout_ns) -> nb::object
  {
    std::size_t n = 0;
    {
      const Unlocked unlocked;
      n = core_.next(
        timeout_ns, [] { return false; },
        [this](std::size_t size) {
          out_.resize(size);
          return out_.data();
        });
    }
    if (n == 0) {
      return nb::none();
    }
    return nb::bytes(out_.data(), n);
  }

  void detach()
  {
    const Unlocked unlocked;
    core_.detach();
  }

  [[nodiscard]] auto ros_type() const -> std::string { return core_.ros_type(); }

private:
  flux::detail::RosMirror core_;
  std::vector<std::uint8_t> out_;
};

// The thread state of the serve() on this thread while it runs without the GIL. Its callbacks
// take the GIL back with it: PyGILState_Ensure would look the thread up again, through state a
// daemon thread finds freed once Python has finalized, where take_gil_back parks it.
auto released() -> PyThreadState *&
{
  thread_local PyThreadState * ts = nullptr;
  return ts;
}

class Released
{
public:
  Released() noexcept : outer_(released()) { released() = PyEval_SaveThread(); }
  ~Released()
  {
    take_gil_back(released());
    released() = outer_;
  }
  Released(const Released &) = delete;
  auto operator=(const Released &) -> Released & = delete;
  Released(Released &&) = delete;
  auto operator=(Released &&) -> Released & = delete;

private:
  PyThreadState * outer_;
};

// Threads in serve() waiting for the GIL, which a PartitionedExecutor node thread lets in after
// each rclpy callback. CPython forces a hand-over only after 5 ms without any switch, and a busy
// rclpy thread drops and retakes the GIL more often than that, so a waiting thread can starve.
class Turnstile
{
public:
  void arrive()
  {
    const std::lock_guard<std::mutex> l(m_);
    ++waiting_;
  }

  void enter()
  {
    {
      const std::lock_guard<std::mutex> l(m_);
      --waiting_;
      ++entered_;
    }
    cv_.notify_all();
  }

  // With the GIL: gives it up until a waiting thread has taken it. The GIL comes back after the
  // mutex is let go, as the thread that took it needs the mutex to say so.
  void step_aside()
  {
    std::unique_lock<std::mutex> l(m_);
    if (waiting_ == 0) {
      return;
    }
    const std::uint64_t seen = entered_;
    const Unlocked unlocked;
    cv_.wait(l, [&] { return entered_ != seen; });
    l.unlock();
  }

private:
  std::mutex m_;
  std::condition_variable cv_;
  int waiting_ = 0;
  std::uint64_t entered_ = 0;
};

// Never destroyed: a daemon thread may still wait in it as the process exits.
auto turnstile() -> Turnstile &
{
  static auto * t = new Turnstile;  // NOLINT(cppcoreguidelines-owning-memory)
  return *t;
}

// The GIL for a callback under serve(). Under dispatch() the caller holds it already.
class Held
{
public:
  Held() noexcept : ts_(std::exchange(released(), nullptr))
  {
    if (ts_ == nullptr) {
      return;
    }
    turnstile().arrive();
    take_gil_back(ts_);
    turnstile().enter();
  }
  ~Held()
  {
    if (ts_ == nullptr) {
      return;
    }
    PyEval_SaveThread();
    released() = ts_;
  }
  Held(const Held &) = delete;
  auto operator=(const Held &) -> Held & = delete;
  Held(Held &&) = delete;
  auto operator=(Held &&) -> Held & = delete;

private:
  PyThreadState * ts_;
};

class Source final : public flux::detail::Source
{
public:
  Source(nb::object sub, nb::callable cb)
  : sub_(std::move(sub)),
    endpoint_(&nb::cast<Subscription &>(sub_)),
    core_(&nb::cast<Subscription &>(sub_).core()),
    cb_(std::move(cb))
  {
  }

  auto attach() -> bool override
  {
    const bool attached = core_->attach_for_executor();
    report_refusal();
    return attached;
  }
  auto deliver_one() -> bool override
  {
    flux::FrameView v = core_->take_for_executor();
    report_refusal();
    if (!v) {
      return false;
    }
    const Held held;
    const void * channel = v.trace_channel();
    const std::uint64_t ticket = v.ticket();
    const flux::gpu::Stream & s = core_->stream();
    if (!s.declared()) {
      const nb::object view = view_of(std::move(v));
      flux_trace_py_ready(channel, ticket);
      cb_(view);
      return true;
    }
    // The callback is the frame's scope, as a `with` block is for take().
    const bool deferred = core_->release_mode() == flux::gpu::ReleaseMode::Deferred;
    const nb::object frame = nb::cast(Frame(std::move(v), s.device(), true, deferred));
    flux_trace_py_ready(channel, ticket);
    try {
      cb_(frame);
    } catch (...) {
      nb::cast<Frame &>(frame).release();
      throw;
    }
    nb::cast<Frame &>(frame).release();
    return true;
  }
  [[nodiscard]] auto channel() const -> const flux::detail::Channel * override
  {
    return core_->channel();
  }
  [[nodiscard]] auto watch() const -> const flux::detail::Watch * override
  {
    return core_->watch();
  }
  [[nodiscard]] auto endpoint() const -> const void * override { return endpoint_; }

  auto traverse(visitproc visit, void * arg) const -> int
  {
    Py_VISIT(sub_.ptr());
    Py_VISIT(cb_.ptr());
    return 0;
  }

private:
  void report_refusal()
  {
    const std::string * why = core_->new_refusal();
    if (why == nullptr) {
      return;
    }
    const Held held;
    sub_.attr("_left_out")(*why);
  }

  nb::object sub_;
  const void * endpoint_;
  flux::detail::Subscriber * core_;
  nb::callable cb_;
};

// Runs alone in serve(), or in ROS mode across three threads. There the bridge thread waits for
// frames without ever taking the GIL, the post thread hands each pass to rclpy as a task, and the
// spin thread runs that task, or between rclpy callbacks a pass no task was queued for yet: a
// frame does not wait for another thread to win the GIL. One thread at a time is on the core:
// the bridge waits only once a pass is over, and a pass is run by whoever claims it.
class Executor
{
public:
  Executor() : core_(std::in_place), handle_(core_->handle()) {}

  void add(nb::object sub, nb::callable cb, std::int32_t priority)
  {
    auto src = std::make_unique<Source>(std::move(sub), std::move(cb));
    core_->add(*src, priority);
    sources_.push_back(std::move(src));
  }

  // Held if serve() has not started, so a stop racing its start is not lost.
  void stop() const noexcept { handle_.stop(); }

  // dispatch and wait in turn until stop(). The GIL is released for the whole loop and taken
  // per callback, and once a tick to raise a pending signal such as Ctrl-C and to ask `running`,
  // whose False ends the loop.
  void serve(const nb::object & running)
  {
    auto next = std::chrono::steady_clock::now();
    core_->on_pass([&] {
      const auto now = std::chrono::steady_clock::now();
      if (now < next) {
        return;
      }
      next = now + std::chrono::nanoseconds(FLUX_DEFAULT_TICK_NS);
      const Held held;
      if (PyErr_CheckSignals() != 0) {
        throw nb::python_error();
      }
      if (!running.is_none() && !nb::cast<bool>(running())) {
        handle_.stop();
      }
    });
    std::exception_ptr error;
    {
      const Released unlocked;
      try {
        core_->spin(FLUX_DEFAULT_TICK_NS);
      } catch (...) {
        error = std::current_exception();
      }
    }
    if (error) {
      std::rethrow_exception(error);
    }
  }

  // Returns the generation that tasks pass back to dispatch_posted.
  auto open_bridge() -> std::uint64_t
  {
    const std::lock_guard<std::mutex> l(m_);
    // The spin thread's first pass attaches and samples, which the bridge's first wait relies on.
    pending_ = true;
    posted_ = false;
    closed_ = false;
    return ++generation_;
  }

  void close_bridge()
  {
    {
      const std::lock_guard<std::mutex> l(m_);
      closed_ = true;
    }
    cv_.notify_all();
    handle_.wake();
  }

  // The bridge thread, until close_bridge().
  void bridge()
  {
    const Unlocked unlocked;
    std::unique_lock<std::mutex> l(m_);
    for (;;) {
      cv_.wait(l, [this] { return closed_ || (!pending_ && !dispatching_); });
      if (closed_) {
        return;
      }
      l.unlock();
      // A pass cut short by its budget leaves frames the wait would not see again.
      if (!core_->has_more()) {
        core_->wait_for_work(FLUX_DEFAULT_TICK_NS);
      }
      l.lock();
      if (closed_) {
        return;
      }
      pending_ = true;
      cv_.notify_all();
    }
  }

  // The post thread, until close_bridge(). `post` queues a task that calls dispatch_posted.
  void poster(const nb::callable & post)
  {
    for (;;) {
      {
        const Unlocked unlocked;
        std::unique_lock<std::mutex> l(m_);
        cv_.wait(l, [this] { return closed_ || (pending_ && !posted_); });
        if (closed_) {
          return;
        }
      }
      // The spin thread may have claimed the pass while this thread waited for the GIL.
      bool post_it = false;
      {
        const std::lock_guard<std::mutex> l(m_);
        post_it = pending_ && !posted_ && !closed_;
        posted_ = posted_ || post_it;
      }
      if (post_it) {
        post();
      }
    }
  }

  // The spin thread, between rclpy callbacks: a pass no task was queued for.
  auto dispatch_pending(std::int64_t budget_ns) -> std::size_t
  {
    {
      const std::lock_guard<std::mutex> l(m_);
      if (!pending_ || posted_) {
        return 0;
      }
      pending_ = false;
      dispatching_ = true;
    }
    return run_pass(budget_ns);
  }

  // A task the bridge queued. One left over from an earlier spin does nothing.
  auto dispatch_posted(std::int64_t budget_ns, std::uint64_t generation) -> std::size_t
  {
    {
      const std::lock_guard<std::mutex> l(m_);
      if (closed_ || generation != generation_ || !pending_) {
        return 0;
      }
      pending_ = false;
      posted_ = false;
      dispatching_ = true;
    }
    return run_pass(budget_ns);
  }

  [[nodiscard]] auto size() const noexcept -> std::size_t { return sources_.size(); }

  // A callback that closes over its executor is the usual case, so the cycle must be
  // collectable.
  auto traverse(visitproc visit, void * arg) const -> int
  {
    for (const auto & s : sources_) {
      if (const int r = s->traverse(visit, arg); r != 0) {
        return r;
      }
    }
    return 0;
  }

  // The core goes first: it points at the sources.
  void clear() noexcept
  {
    core_.reset();
    sources_.clear();
  }

private:
  // A pass ends once budget_ns has gone by, after at least one frame. The bridge may wait again
  // once it is over, also after a callback raised.
  auto run_pass(std::int64_t budget_ns) -> std::size_t
  {
    const Done done(*this);
    const auto end = std::chrono::steady_clock::now() + std::chrono::nanoseconds(budget_ns);
    auto over = [&]() noexcept { return std::chrono::steady_clock::now() >= end; };
    return core_->dispatch(over);
  }

  class Done
  {
  public:
    explicit Done(Executor & e) noexcept : e_(e) {}
    ~Done()
    {
      {
        const std::lock_guard<std::mutex> l(e_.m_);
        e_.dispatching_ = false;
      }
      e_.cv_.notify_all();
    }
    Done(const Done &) = delete;
    auto operator=(const Done &) -> Done & = delete;
    Done(Done &&) = delete;
    auto operator=(Done &&) -> Done & = delete;

  private:
    Executor & e_;
  };

  std::optional<flux::detail::Executor> core_;
  flux::detail::ExecutorHandle handle_;
  std::vector<std::unique_ptr<Source>> sources_;
  std::mutex m_;
  std::condition_variable cv_;
  bool pending_ = false;
  bool posted_ = false;
  bool dispatching_ = false;
  bool closed_ = false;
  std::uint64_t generation_ = 0;
};

auto executor_traverse(PyObject * self, visitproc visit, void * arg) -> int
{
  if (nb::inst_ready(self)) {
    if (const int r = nb::inst_ptr<Executor>(self)->traverse(visit, arg); r != 0) {
      return r;
    }
  }
  Py_VISIT(Py_TYPE(self));
  return 0;
}

auto executor_clear(PyObject * self) -> int
{
  if (nb::inst_ready(self)) {
    nb::inst_ptr<Executor>(self)->clear();
  }
  return 0;
}

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast): PyType_Slot takes a function as void *.
const std::array<PyType_Slot, 3> executor_slots{{
  {Py_tp_traverse, reinterpret_cast<void *>(executor_traverse)},
  {Py_tp_clear, reinterpret_cast<void *>(executor_clear)},
  {0, nullptr},
}};
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

}  // namespace

NB_MODULE(_flux, m)
{
  if (_import_array() < 0) {
    throw nb::python_error();
  }
  // Objects a daemon thread still holds at exit are not a reference counting bug.
  nb::set_leak_warnings(false);
  m.attr("NO_SCHEMA") = flux::kNoSchema;
  m.attr("DEFAULT_SLOT_SIZE") = FLUX_DEFAULT_SLOT_SIZE;
  m.attr("DEFAULT_SLOT_COUNT") = FLUX_DEFAULT_SLOT_COUNT;
  m.attr("DEFAULT_MAX_SUBSCRIBERS") = FLUX_DEFAULT_MAX_SUBSCRIBERS;
  m.attr("DEFAULT_TICK_NS") = FLUX_DEFAULT_TICK_NS;
  m.attr("DEFAULT_TURN_NS") = FLUX_DEFAULT_TURN_NS;

  static nb::exception<flux::Error> error(m, "Error", PyExc_RuntimeError);
  nb::register_exception_translator([](const std::exception_ptr & p, void *) {
    try {
      std::rethrow_exception(p);
    } catch (const flux::Error & e) {
      const nb::object err = error(e.what());
      err.attr("status") = nb::cast(e.status());
      PyErr_SetObject(error.ptr(), err.ptr());
    }
  });

  nb::enum_<flux::Status>(m, "Status")
    .value("OK", FLUX_OK)
    .value("INVALID_ARGUMENT", FLUX_INVALID_ARGUMENT)
    .value("MISMATCH", FLUX_MISMATCH)
    .value("NOT_READY", FLUX_NOT_READY)
    .value("PUBLISHER_EXISTS", FLUX_PUBLISHER_EXISTS)
    .value("SUBSCRIBERS_FULL", FLUX_SUBSCRIBERS_FULL)
    .value("UNSUPPORTED", FLUX_UNSUPPORTED)
    .value("CALLBACK", FLUX_CALLBACK)
    .value("IO", FLUX_IO)
    .value("IN_USE", FLUX_IN_USE);

  nb::enum_<flux::Published>(m, "Published")
    .value("OK", flux::Published::Ok)
    .value("BACKPRESSURE", flux::Published::Backpressure)
    .value("TOO_LARGE", flux::Published::TooLarge)
    .value("NOT_PUBLISHER", flux::Published::NotPublisher)
    .value("INVALID_ARGUMENT", flux::Published::InvalidArgument)
    .value("IN_USE", flux::Published::InUse)
    .value("WRONG_DEVICE", flux::Published::WrongDevice)
    .value("FENCE_FAILED", flux::Published::FenceFailed)
    .value("GPU_WEDGED", flux::Published::GpuWedged)
    .def("__bool__", [](flux::Published p) { return p == flux::Published::Ok; });
  m.def("faulted", &flux::faulted, "published"_a);

  nb::class_<flux::QoS>(m, "QoS")
    .def(
      "__init__",
      [](
        flux::QoS * q, std::optional<std::uint32_t> depth, std::optional<std::uint32_t> replay,
        std::optional<std::uint32_t> max_borrow) {
        new (q) flux::QoS(depth ? flux::QoS(*depth) : flux::QoS());
        if (replay) {
          q->transient_local(*replay);
        }
        if (max_borrow) {
          q->max_borrow(*max_borrow);
        }
      },
      "depth"_a = nb::none(), "replay"_a = nb::none(), "max_borrow"_a = nb::none())
    .def_prop_ro("depth", &flux::QoS::depth)
    .def_prop_ro("replay", &flux::QoS::replay)
    .def_prop_ro("max_borrow", &flux::QoS::borrow_limit);

  nb::class_<flux::MemoryPolicy>(m, "MemoryPolicy")
    .def(
      "__init__",
      [](flux::MemoryPolicy * p, bool precommit, bool lock) {
        new (p) flux::MemoryPolicy{precommit, lock};
      },
      "precommit"_a = false, "lock"_a = false)
    .def_ro("precommit", &flux::MemoryPolicy::precommit)
    .def_ro("lock", &flux::MemoryPolicy::lock);

  nb::class_<flux::Refused>(m, "Refused")
    .def_ro("max_borrow", &flux::Refused::max_borrow)
    .def_ro("no_reader", &flux::Refused::no_reader)
    .def_ro("contended", &flux::Refused::contended)
    .def_ro("bad_frame", &flux::Refused::bad_frame)
    .def_ro("fence", &flux::Refused::fence);

  nb::class_<flux::gpu::FenceWait>(m, "FenceWait")
    .def_ro("commit_ns", &flux::gpu::FenceWait::commit_ns)
    .def_ro("commit_waits", &flux::gpu::FenceWait::commit_waits)
    .def_ro("commit_max_ns", &flux::gpu::FenceWait::commit_max_ns)
    .def_ro("release_ns", &flux::gpu::FenceWait::release_ns)
    .def_ro("release_waits", &flux::gpu::FenceWait::release_waits)
    .def_ro("release_max_ns", &flux::gpu::FenceWait::release_max_ns);

  nb::enum_<flux::gpu::Route>(m, "Route")
    .value("NONE", flux::gpu::Route::None)
    .value("SHM_DIRECT", flux::gpu::Route::ShmDirect)
    .value("SHM_REGISTERED", flux::gpu::Route::ShmRegistered)
    .value("DEVICE_HANDLE", flux::gpu::Route::DeviceHandle);

  nb::class_<flux::gpu::Platform>(m, "Platform")
    .def_ro("route", &flux::gpu::Platform::route)
    .def_ro("device", &flux::gpu::Platform::device)
    .def_ro("integrated", &flux::gpu::Platform::integrated)
    .def_ro("pageable_access", &flux::gpu::Platform::pageable_access)
    .def_ro("host_register", &flux::gpu::Platform::host_register)
    .def_ro("vmm", &flux::gpu::Platform::vmm)
    .def_ro("posix_fd_handle", &flux::gpu::Platform::posix_fd_handle)
    .def_ro("reason", &flux::gpu::Platform::reason);
  m.def("probe", &flux::gpu::probe, "device"_a = 0);
  m.def("gpu_wedged", &flux::gpu::wedged);

  nb::class_<DeviceSpan>(m, "DeviceSpan")
    .def_prop_ro("__cuda_array_interface__", &DeviceSpan::cai)
    .def("__dlpack__", &DeviceSpan::dlpack)
    .def("__dlpack_device__", &DeviceSpan::dlpack_device)
    .def("__len__", &DeviceSpan::size)
    .def_prop_ro("nbytes", &DeviceSpan::nbytes)
    .def_prop_ro("dtype", &DeviceSpan::dtype);

  nb::class_<Frame>(m, "Frame")
    .def(
      "__enter__",
      [](nb::handle self) {
        nb::inst_ptr<Frame>(self)->enter();
        return nb::borrow(self);
      })
    .def("__exit__", [](Frame & f, const nb::args &) { f.release(); })
    .def("release", &Frame::release)
    .def_prop_ro("__cuda_array_interface__", &Frame::cai)
    .def("__dlpack__", [](Frame & f, const nb::kwargs & kw) { return f.dlpack(kw); })
    .def("__dlpack_device__", &Frame::dlpack_device)
    .def("_host", &Frame::host)
    .def(
      "_tensor_dlpack",
      [](Frame & f, const flux_py::TensorAt & at, const nb::kwargs & kw) {
        return f.tensor_dlpack(at, kw);
      })
    .def_prop_ro("valid", &Frame::valid)
    .def("__bool__", &Frame::valid)
    .def_prop_ro("shape", &Frame::shape)
    .def_prop_ro("dtype", &Frame::dtype)
    .def_prop_ro("nbytes", &Frame::nbytes)
    .def_prop_ro("stream", &Frame::stream);

  nb::class_<Loan>(m, "Loan")
    .def_prop_ro("array", [](nb::handle self) { return nb::inst_ptr<Loan>(self)->array(self); })
    .def("commit", &Loan::commit, "nbytes"_a = nb::none())
    .def("abort", &Loan::abort)
    .def_prop_ro("valid", &Loan::valid)
    .def("__bool__", &Loan::valid)
    .def_prop_ro("capacity", &Loan::capacity)
    .def_prop_ro("aux", [](nb::handle self) { return nb::inst_ptr<Loan>(self)->aux(self); })
    .def_prop_rw("aux_len", &Loan::aux_len, &Loan::set_aux_len)
    .def_prop_ro("_heap", &Loan::heap)
    .def_prop_ro("__cuda_array_interface__", &Loan::cai)
    .def(
      "__dlpack__",
      [](nb::handle self, const nb::kwargs & kw) {
        return nb::inst_ptr<Loan>(self)->dlpack(self, kw);
      })
    .def("__dlpack_device__", &Loan::dlpack_device)
    .def_prop_ro("stream", &Loan::stream);

  nb::class_<Publisher>(m, "_Publisher")
    .def(
      nb::init<
        const std::string &, const std::string &, const std::string &, std::uint64_t, std::uint32_t,
        std::uint32_t, flux::MemoryPolicy, std::uint32_t, bool, std::uint32_t>(),
      "signpost"_a, "topic"_a, "label"_a, "fingerprint"_a, "slot_size"_a, "slot_count"_a,
      "memory"_a, "max_subscribers"_a = FLUX_DEFAULT_MAX_SUBSCRIBERS, "cuda"_a = false,
      "aux_size"_a = 0)
    .def("publish", &Publisher::publish, "array"_a)
    .def("loan", &Publisher::loan, "shape"_a = nb::none(), "dtype"_a = "uint8")
    .def_prop_ro("dropped", &Publisher::dropped)
    .def_prop_ro("subscriber_count", &Publisher::subscriber_count)
    .def_prop_ro("slot_size", &Publisher::slot_size)
    .def_prop_ro("aux_size", [](Publisher & p) { return p.channel().aux_size(); })
    .def_prop_ro("fence_failed", [](Publisher & p) { return p.channel().fence_failed(); })
    .def_prop_ro("fence_wait", [](Publisher & p) { return p.channel().fence_wait(); })
    .def_prop_ro("device_backed", [](Publisher & p) { return p.channel().device_backed(); })
    .def(
      "_set_fence_timeout",
      [](Publisher & p, std::int64_t ns) {
        p.channel().set_fence_timeout(std::chrono::nanoseconds(ns));
      })
    .def_prop_ro("stream", [](Publisher & p) { return addr_of(p.stream().native()); });

  nb::class_<Subscription>(m, "_Subscription")
    .def(
      nb::init<
        const std::string &, const std::string &, const std::string &, std::uint64_t,
        const flux::QoS &, flux::MemoryPolicy, bool>(),
      "signpost"_a, "topic"_a, "label"_a, "fingerprint"_a, "qos"_a, "memory"_a, "cuda"_a = false)
    .def("take", &Subscription::take)
    .def(
      "take_blocking", &Subscription::take_blocking, "timeout_ns"_a = -1, "running"_a = nb::none())
    .def_prop_ro("attached", [](Subscription & s) { return s.core().attached(); })
    .def_prop_ro("lost", [](Subscription & s) { return s.core().lost(); })
    .def_prop_ro("attach_generation", [](Subscription & s) { return s.core().attach_generation(); })
    .def_prop_ro("refused", [](Subscription & s) { return s.core().refused(); })
    .def_prop_ro("fence_failed", [](Subscription & s) { return s.core().fence_failed(); })
    .def_prop_ro("fence_wait", [](Subscription & s) { return s.core().fence_wait(); })
    .def_prop_ro("stream", [](Subscription & s) { return addr_of(s.core().stream().native()); })
    .def_prop_ro("qos", [](Subscription & s) { return s.core().qos(); })
    .def(
      "_set_fence_timeout",
      [](Subscription & s, std::int64_t ns) {
        s.core().set_fence_timeout(std::chrono::nanoseconds(ns));
      })
    .def(
      "_set_release_deferred",
      [](Subscription & s, bool deferred) {
        s.core().set_release_mode(
          deferred ? flux::gpu::ReleaseMode::Deferred : flux::gpu::ReleaseMode::Wait);
      })
    .def_prop_ro("signpost_name", [](Subscription & s) { return s.core().signpost_name(); })
    .def("_require_slots", [](Subscription & s, std::uint32_t n) { s.core().require_slots(n); });

  nb::class_<RosMirror>(m, "_RosMirror")
    .def(
      nb::init<Publisher &, const std::string &, std::uint64_t>(), "publisher"_a, "signpost"_a,
      "fingerprint"_a)
    .def("next", &RosMirror::next, "timeout_ns"_a)
    .def("detach", &RosMirror::detach)
    .def_prop_ro("ros_type", &RosMirror::ros_type);

  nb::class_<Executor>(m, "_Executor", nb::type_slots(executor_slots.data()))
    .def(nb::init<>())
    .def("add", &Executor::add, "sub"_a, "callback"_a, "priority"_a = 0)
    .def("stop", &Executor::stop)
    .def("serve", &Executor::serve, "running"_a = nb::none())
    .def("open_bridge", &Executor::open_bridge)
    .def("close_bridge", &Executor::close_bridge)
    .def("bridge", &Executor::bridge)
    .def("poster", &Executor::poster, "post"_a)
    .def("dispatch_pending", &Executor::dispatch_pending, "budget_ns"_a)
    .def("dispatch_posted", &Executor::dispatch_posted, "budget_ns"_a, "generation"_a)
    .def("__len__", &Executor::size);

  m.def("_step_aside", [] { turnstile().step_aside(); });
  m.def("_before_exit", &before_exit);
  m.def("signpost_name", &flux::detail::signpost_name, "key"_a, "fingerprint"_a);
  m.def("aux", &aux_of, "frame"_a);
  m.def("_register_stamp_offset", &flux_stamp_offset_register, "fingerprint"_a, "offset"_a);
  m.def("_register_layout", &flux_layout_register, "fingerprint"_a);

  nb::class_<flux::Endpoint>(m, "Endpoint")
    .def_ro("pid", &flux::Endpoint::pid)
    .def_ro("publisher", &flux::Endpoint::publisher)
    .def_ro("label", &flux::Endpoint::label);
  nb::class_<flux::Topic>(m, "Topic")
    .def_ro("signpost", &flux::Topic::signpost)
    .def_ro("key", &flux::Topic::key)
    .def_ro("fingerprint", &flux::Topic::fingerprint)
    .def_ro("key_exact", &flux::Topic::key_exact)
    .def_ro("endpoints", &flux::Topic::endpoints);
  nb::class_<flux::ChannelStats>(m, "ChannelStats")
    .def_ro("live", &flux::ChannelStats::live)
    .def_ro("slot_size", &flux::ChannelStats::slot_size)
    .def_ro("slot_count", &flux::ChannelStats::slot_count)
    .def_ro("max_subscribers", &flux::ChannelStats::max_subscribers)
    .def_ro("epoch", &flux::ChannelStats::epoch)
    .def_ro("fingerprint", &flux::ChannelStats::fingerprint)
    .def_ro("publish_seq", &flux::ChannelStats::publish_seq)
    .def_ro("waiters", &flux::ChannelStats::waiters);
  m.def("enumerate_topics", &flux::enumerate_topics);
  m.def("read_channel_stats", &flux::read_channel_stats, "signpost"_a);
  m.def(
    "ros_type_of",
    [](std::uint64_t fingerprint) -> std::optional<std::string> {
      const flux::Schemas schemas;
      if (!schemas.contains(fingerprint)) {
        return std::nullopt;
      }
      return schemas.at(fingerprint).ros_type();
    },
    "fingerprint"_a);

  bind_wire(m);
  m.def("_arena_scope_enter", &flux_arena_scope_enter);
  m.def("_arena_scope_leave", &flux_arena_scope_leave);
}
