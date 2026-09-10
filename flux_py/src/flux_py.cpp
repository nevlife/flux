// flux Python binding (nanobind). Publisher copies a numpy array into a slot; Subscription
// returns the latest frame as a READ-ONLY zero-copy numpy view. The view's data points into
// the shared segment; a capsule owns the FrameView so the borrow (refcount) is held until
// Python garbage-collects the array. This is flux's reason to exist: C++ <-> Python bulk data
// with no copy on the read side.

#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/executor.hpp"
#include "flux/owner.hpp"
#include "flux/rt.hpp"
#include "flux/segment.hpp"
#include "flux/segment_layout.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace
{

std::int64_t mono_ns()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// flux names the shm segment from the topic. The C++ ros wrapper resolves the topic through
// its node first (namespace/remap), so the segment name is built from a fully-qualified name.
// flux_py has no node, so the caller must pass that already-resolved absolute name; requiring
// the leading '/' rejects a relative name that would otherwise silently name a different
// segment than the matching C++ node. flux_py cannot apply a namespace itself -- pass the same
// absolute name a C++ node resolves to.

// Enumeration metadata only. flux_py has no node, so the label stays empty
// unless the caller passes one; the key is what makes a tool able to print the real topic name
// rather than the name's punctuation-flattened spelling.
void announce(
  const std::string & signpost, const std::string & key, bool publisher, const std::string & label)
{
  flux::ManifestEntry e;
  e.signpost = signpost;
  e.key = key;
  e.label = label;
  e.publisher = publisher;
  flux::OwnerFile::announce(e);
}

const std::string & require_absolute(const std::string & topic)
{
  if (topic.empty() || topic.front() != '/') {
    throw std::invalid_argument(
      "flux: topic must be an absolute ROS name starting with '/', got '" + topic +
      "' (pass the fully-resolved name a C++ node would use)");
  }
  return topic;
}

// A GPU declaration named the way Python names things. The C++ enum is bound as flux.Device
// too, so both `device="cuda"` and `device=flux.Device.Cuda` reach the same value; a string is
// what the ROS-facing call sites read best and the enum is what survives a typo.
flux::Device device_from(nb::handle d)
{
  if (d.is_none()) return flux::Device::Cpu;
  if (nb::isinstance<flux::Device>(d)) return nb::cast<flux::Device>(d);
  if (nb::isinstance<nb::str>(d)) {
    const std::string name = nb::cast<std::string>(d);
    if (name == "cpu") return flux::Device::Cpu;
    if (name == "cuda") return flux::Device::Cuda;
    throw std::invalid_argument(
      "flux: device must be 'cpu' or 'cuda' (or flux.Device), got '" + name + "'");
  }
  throw std::invalid_argument("flux: device must be a string or flux.Device");
}

// The declared stream as the integer handle every CUDA library in Python speaks (cupy's
// ExternalStream, numba's external_stream). None when the channel declared no stream: there is
// nothing to launch on, and handing back 0 would name CUDA's legacy default stream instead --
// the one declaration flux refuses outright.
nb::object stream_object(const flux::gpu::Stream & st)
{
  if (!st.declared()) return nb::none();
  return nb::cast(reinterpret_cast<std::uintptr_t>(st.native()));
}

// __cuda_array_interface__ typestr. Same-host transport, so the byte order is the host's and
// never needs to be negotiated; single-byte types carry '|' because they have no order to state.
const char * cai_typestr(flux::DType dt)
{
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  constexpr char e = '>';
#else
  constexpr char e = '<';
#endif
  static thread_local char buf[4];
  const char * two = nullptr;
  switch (dt) {
    case flux::DType::U8:
      return "|u1";
    case flux::DType::I8:
      return "|i1";
    case flux::DType::U16:
      two = "u2";
      break;
    case flux::DType::I16:
      two = "i2";
      break;
    case flux::DType::F16:
      two = "f2";
      break;
    case flux::DType::U32:
      two = "u4";
      break;
    case flux::DType::I32:
      two = "i4";
      break;
    case flux::DType::F32:
      two = "f4";
      break;
    case flux::DType::U64:
      two = "u8";
      break;
    case flux::DType::I64:
      two = "i8";
      break;
    case flux::DType::F64:
      two = "f8";
      break;
    case flux::DType::BF16:
      // The protocol's typestr is numpy's, and numpy has no bfloat16. Refused by name rather
      // than spelled 'u2' or 'f2': either would hand a consumer numbers that are not the ones
      // in the slot. bf16 travels through __dlpack__ instead.
      throw std::runtime_error(
        "flux: bfloat16 has no __cuda_array_interface__ typestr; read this frame through "
        "__dlpack__ (torch.from_dlpack) instead");
  }
  if (two == nullptr) throw std::runtime_error("flux: unknown DType in __cuda_array_interface__");
  buf[0] = e;
  buf[1] = two[0];
  buf[2] = two[1];
  buf[3] = '\0';
  return buf;
}

// The frame's dtype as a name, readable whether or not the type has a typestr. Every type numpy
// can express reports its typestr, unchanged; bfloat16 has none to report and carries the name
// DLPack and ml_dtypes spell it with.
const char * dtype_name(flux::DType dt)
{
  return dt == flux::DType::BF16 ? "bfloat16" : cai_typestr(dt);
}

// The DLPack dtype for a flux DType. Needed wherever the numpy scalar type cannot carry it: f16
// has no portable C++ scalar to template on, bf16 has no numpy dtype at all, and the device
// views below are built from bytes with the dtype supplied rather than deduced.
nb::dlpack::dtype dl_dtype(nb::dlpack::dtype_code code, std::uint8_t bits)
{
  return nb::dlpack::dtype{static_cast<std::uint8_t>(code), bits, 1};
}

nb::dlpack::dtype dl_dtype_of(flux::DType dt)
{
  using C = nb::dlpack::dtype_code;
  switch (dt) {
    case flux::DType::U8:
      return dl_dtype(C::UInt, 8);
    case flux::DType::I8:
      return dl_dtype(C::Int, 8);
    case flux::DType::U16:
      return dl_dtype(C::UInt, 16);
    case flux::DType::I16:
      return dl_dtype(C::Int, 16);
    case flux::DType::U32:
      return dl_dtype(C::UInt, 32);
    case flux::DType::I32:
      return dl_dtype(C::Int, 32);
    case flux::DType::U64:
      return dl_dtype(C::UInt, 64);
    case flux::DType::I64:
      return dl_dtype(C::Int, 64);
    case flux::DType::F16:
      return dl_dtype(C::Float, 16);
    case flux::DType::F32:
      return dl_dtype(C::Float, 32);
    case flux::DType::F64:
      return dl_dtype(C::Float, 64);
    case flux::DType::BF16:
      return dl_dtype(C::Bfloat, 16);
  }
  throw std::runtime_error("flux: unknown DType in __dlpack__");
}

// The CUDA ordinal a device view names. A channel declares its device through gpu::stream_for,
// which probes device 0, so this is that probe's answer. Read once:
// it cannot change within a process, and __dlpack__ is on the per-frame path.
int cuda_device_id()
{
  static const int id = flux::gpu::probe().device;
  return id < 0 ? 0 : id;
}

// The "dltensor" capsule __dlpack__ must return, over borrowed bytes. `owner` pins the borrow
// for the capsule's lifetime exactly as the numpy path's capsule does; the dtype is supplied
// rather than deduced so one entry point serves every DType, including the two numpy cannot name.
//
// Cast without a framework tag, which is what makes nanobind hand back the capsule itself rather
// than wrap it in a numpy array or import torch to build a tensor.
nb::object dlpack_capsule(
  const void * data, std::size_t ndim, const std::size_t * shape, flux::DType dtype,
  nb::handle owner, int device_type, int device_id)
{
  return nb::cast(nb::ndarray<const std::uint8_t>(
    data, ndim, shape, owner, nullptr, dl_dtype_of(dtype), device_type, device_id));
}

// Writable overload, selected by the pointer's constness: a loan is filled through its capsule,
// a borrowed frame is only read through one.
nb::object dlpack_capsule(
  void * data, std::size_t ndim, const std::size_t * shape, flux::DType dtype, nb::handle owner,
  int device_type, int device_id)
{
  return nb::cast(nb::ndarray<std::uint8_t>(
    data, ndim, shape, owner, nullptr, dl_dtype_of(dtype), device_type, device_id));
}

// The v3 __cuda_array_interface__ dict for `ptr`. `stream` is None
// on purpose: both seams are closed synchronously, so a frame handed out is
// already complete and a consumer needs no cross-stream wait to read it. Naming a stream here
// would make every consumer library insert a wait that flux has already done.
nb::dict cuda_array_interface(
  const void * ptr, std::size_t ndim, const std::size_t * shape, flux::DType dtype, bool read_only)
{
  nb::list shp;
  for (std::size_t i = 0; i < ndim; ++i) shp.append(shape[i]);
  nb::dict d;
  d["shape"] = nb::tuple(shp);
  d["typestr"] = cai_typestr(dtype);
  d["data"] = nb::make_tuple(reinterpret_cast<std::uintptr_t>(ptr), read_only);
  d["strides"] = nb::none();  // C-contiguous
  d["stream"] = nb::none();
  d["version"] = 3;
  return d;
}

flux::DType flux_from_dl(nb::dlpack::dtype dt)
{
  using C = nb::dlpack::dtype_code;
  const auto code = static_cast<C>(dt.code);
  if (code == C::UInt) {
    switch (dt.bits) {
      case 8:
        return flux::DType::U8;
      case 16:
        return flux::DType::U16;
      case 32:
        return flux::DType::U32;
      case 64:
        return flux::DType::U64;
    }
  } else if (code == C::Int) {
    switch (dt.bits) {
      case 8:
        return flux::DType::I8;
      case 16:
        return flux::DType::I16;
      case 32:
        return flux::DType::I32;
      case 64:
        return flux::DType::I64;
    }
  } else if (code == C::Float) {
    switch (dt.bits) {
      case 16:
        return flux::DType::F16;
      case 32:
        return flux::DType::F32;
      case 64:
        return flux::DType::F64;
    }
  } else if (code == C::Bfloat && dt.bits == 16) {
    // Reached from any DLPack producer, numpy or not -- a torch bf16 CPU tensor publishes here
    // even though numpy could never have held it.
    return flux::DType::BF16;
  }
  throw std::invalid_argument(
    "flux: unsupported array dtype (code=" + std::to_string(dt.code) +
    " bits=" + std::to_string(dt.bits) + ")");
}

// Owner of a returned view. The borrowed FrameView aliases the subscriber's mmap, so the
// view must not outlive that mapping: `keepalive` pins the Subscription python object (and
// thus its Channel/Segment) for the view's lifetime. Declaration order matters -- members
// destroy in reverse, so `view` (the borrow) releases BEFORE `keepalive` drops the mapping.
struct Held
{
  nb::object keepalive;
  flux::FrameView view;
};

// Read-only numpy view over already-borrowed bytes. `owner` (a capsule holding the Held)
// keeps both the borrow and the mapping alive until the array is collected.
template <typename T>
nb::object typed_view(
  const void * data, std::size_t ndim, const std::size_t * shape, nb::handle owner)
{
  return nb::cast(nb::ndarray<nb::numpy, const T>(data, ndim, shape, owner));
}

// The same bytes as a read-only unsigned numpy view of equal width. The escape hatch for a dtype
// numpy cannot name: a caller that knows the frame is bf16 reinterprets these bits itself
// (arr.view(ml_dtypes.bfloat16)), which keeps that spelling the caller's dependency rather than
// flux's.
nb::object bits_view(
  const void * data, std::size_t ndim, const std::size_t * shape, flux::DType dtype,
  nb::handle owner)
{
  switch (flux::dtype_size(dtype)) {
    case 1:
      return typed_view<std::uint8_t>(data, ndim, shape, owner);
    case 2:
      return typed_view<std::uint16_t>(data, ndim, shape, owner);
    case 4:
      return typed_view<std::uint32_t>(data, ndim, shape, owner);
    case 8:
      return typed_view<std::uint64_t>(data, ndim, shape, owner);
  }
  throw std::runtime_error("flux: unknown DType width in bits");
}

// Read the frame's shape out of meta another process wrote, refusing anything that would let a
// view span past the payload. Both consumer paths go through this: the
// numpy view below and the device view a GPU consumer gets, which are sized the same way and so
// must be bounded the same way. Returns ndim and fills `shape`.
std::size_t checked_frame_shape(const flux::FrameView & v, std::size_t * shape)
{
  const flux::FrameMeta & m = v.meta();
  const std::size_t ndim = m.ndim;
  if (ndim > flux::kMaxDims) throw std::runtime_error("flux: frame ndim exceeds kMaxDims");
  for (std::size_t i = 0; i < ndim; ++i) shape[i] = static_cast<std::size_t>(m.shape[i]);

  // A view spans prod(shape) * itemsize bytes. Require it to equal the frame's byte count so a
  // shape/dtype from another (non-conforming) publisher cannot make a consumer read past the
  // payload. Multiplications are overflow-checked before the equality test.
  std::size_t elems = 1;
  for (std::size_t i = 0; i < ndim; ++i) {
    if (shape[i] != 0 && elems > SIZE_MAX / shape[i])
      throw std::runtime_error("flux: frame shape overflows size_t");
    elems *= shape[i];
  }
  // Size the check from the dtype, because that is what the view is sized from. Validating
  // against m.itemsize instead let a publisher that set the two inconsistently slip a view past
  // the end of the frame -- and past the mapping.
  const std::size_t itemsize = flux::dtype_size(m.dtype);
  if (itemsize == 0 || m.itemsize != itemsize)
    throw std::runtime_error("flux: frame itemsize disagrees with its dtype");
  if (elems > SIZE_MAX / itemsize) throw std::runtime_error("flux: frame shape overflows size_t");
  if (elems * itemsize != m.nbytes)
    throw std::runtime_error("flux: frame shape/itemsize inconsistent with nbytes");
  if (m.nbytes > v.size()) throw std::runtime_error("flux: frame nbytes exceeds the borrowed view");
  return ndim;
}

nb::object view_from_frame(flux::FrameView && v, nb::handle keepalive)
{
  const flux::DType dtype = v.meta().dtype;
  std::size_t shape[flux::kMaxDims];
  const std::size_t ndim = checked_frame_shape(v, shape);

  auto * held = new Held{nb::borrow(keepalive), std::move(v)};
  const void * data = held->view.data();
  nb::capsule owner(held, [](void * p) noexcept { delete static_cast<Held *>(p); });

  switch (dtype) {
    case flux::DType::U8:
      return typed_view<std::uint8_t>(data, ndim, shape, owner);
    case flux::DType::I8:
      return typed_view<std::int8_t>(data, ndim, shape, owner);
    case flux::DType::U16:
      return typed_view<std::uint16_t>(data, ndim, shape, owner);
    case flux::DType::I16:
      return typed_view<std::int16_t>(data, ndim, shape, owner);
    case flux::DType::U32:
      return typed_view<std::uint32_t>(data, ndim, shape, owner);
    case flux::DType::I32:
      return typed_view<std::int32_t>(data, ndim, shape, owner);
    case flux::DType::U64:
      return typed_view<std::uint64_t>(data, ndim, shape, owner);
    case flux::DType::I64:
      return typed_view<std::int64_t>(data, ndim, shape, owner);
    case flux::DType::F32:
      return typed_view<float>(data, ndim, shape, owner);
    case flux::DType::F64:
      return typed_view<double>(data, ndim, shape, owner);
    case flux::DType::F16:  // no portable half scalar: carry as u16, override to float16
      return nb::cast(nb::ndarray<nb::numpy, const std::uint16_t>(
        data, ndim, shape, owner, nullptr, dl_dtype(nb::dlpack::dtype_code::Float, 16)));
    case flux::DType::BF16:
      // Unreachable: numpy has no bfloat16, so such a frame is wrapped as a flux.Frame before
      // it gets here (Subscription::wrap). Stated rather than defaulted, so adding a
      // DType that numpy also cannot name is a compile error here instead of this message.
      throw std::runtime_error(
        "flux: bfloat16 has no numpy dtype; read this frame through __dlpack__");
  }
  throw std::runtime_error("flux: unknown DType in frame");
}

// Map a numpy dtype (object, type, or string -- anything np.dtype() accepts) to a flux DType.
std::pair<flux::DType, std::uint32_t> flux_from_np_dtype(nb::handle dt)
{
  // bfloat16 first, and by name. np.dtype("bfloat16") raises unless ml_dtypes is installed, so
  // the plain string is the spelling that works everywhere; the name check below is what lets an
  // ml_dtypes.bfloat16 object through, since its numpy kind is 'V' and 'V2' alone means nothing.
  if (nb::isinstance<nb::str>(dt) && nb::cast<std::string>(dt) == "bfloat16") {
    return {flux::DType::BF16, 2};
  }
  nb::object np = nb::module_::import_("numpy");
  nb::object d = np.attr("dtype")(dt);
  if (nb::cast<std::string>(d.attr("name")) == "bfloat16") return {flux::DType::BF16, 2};
  const std::uint32_t itemsize = nb::cast<std::uint32_t>(d.attr("itemsize"));
  const std::string kind = nb::cast<std::string>(d.attr("kind"));
  const int bits = static_cast<int>(itemsize) * 8;
  auto bad = [&]() {
    throw std::invalid_argument(
      "flux: unsupported loan dtype (kind=" + kind + " bits=" + std::to_string(bits) + ")");
  };
  if (kind == "u") {
    switch (bits) {
      case 8:
        return {flux::DType::U8, itemsize};
      case 16:
        return {flux::DType::U16, itemsize};
      case 32:
        return {flux::DType::U32, itemsize};
      case 64:
        return {flux::DType::U64, itemsize};
    }
  } else if (kind == "i") {
    switch (bits) {
      case 8:
        return {flux::DType::I8, itemsize};
      case 16:
        return {flux::DType::I16, itemsize};
      case 32:
        return {flux::DType::I32, itemsize};
      case 64:
        return {flux::DType::I64, itemsize};
    }
  } else if (kind == "f") {
    switch (bits) {
      case 16:
        return {flux::DType::F16, itemsize};
      case 32:
        return {flux::DType::F32, itemsize};
      case 64:
        return {flux::DType::F64, itemsize};
    }
  }
  bad();
  return {flux::DType::U8, 1};  // unreachable
}

// numpy can only view host memory. On a dGPU channel the slot is a device address, so building
// the view would hand Python a pointer whose first store faults -- and it faulted, before this
// guard existed. Refused by name, with the two paths that do work.
[[noreturn]] void device_payload_has_no_host_view(const char * what)
{
  throw std::runtime_error(
    std::string("flux: ") + what +
    " is a numpy view, which needs host memory; this channel's payload is GPU memory. Use "
    "__cuda_array_interface__ (cupy.asarray) or __dlpack__ (torch.from_dlpack) instead");
}

template <typename T>
nb::object typed_writable(
  void * data, std::size_t ndim, const std::size_t * shape, nb::handle owner)
{
  return nb::cast(nb::ndarray<nb::numpy, T>(data, ndim, shape, owner));
}

// Write-side dual of bits_view: fill a loan whose dtype numpy cannot name through your own
// spelling of it (loan.bits.view(ml_dtypes.bfloat16)[:] = x).
nb::object bits_writable(
  void * data, std::size_t ndim, const std::size_t * shape, flux::DType dtype, nb::handle owner)
{
  switch (flux::dtype_size(dtype)) {
    case 1:
      return typed_writable<std::uint8_t>(data, ndim, shape, owner);
    case 2:
      return typed_writable<std::uint16_t>(data, ndim, shape, owner);
    case 4:
      return typed_writable<std::uint32_t>(data, ndim, shape, owner);
    case 8:
      return typed_writable<std::uint64_t>(data, ndim, shape, owner);
  }
  throw std::runtime_error("flux: unknown DType width in bits");
}

// Writable numpy view aliasing slot bytes (the write-side dual of typed_view). `owner` pins the
// Loan (and through it the Publisher mapping) for the array's lifetime.
nb::object writable_view(
  void * data, std::size_t ndim, const std::size_t * shape, flux::DType dtype, nb::handle owner)
{
  switch (dtype) {
    case flux::DType::U8:
      return typed_writable<std::uint8_t>(data, ndim, shape, owner);
    case flux::DType::I8:
      return typed_writable<std::int8_t>(data, ndim, shape, owner);
    case flux::DType::U16:
      return typed_writable<std::uint16_t>(data, ndim, shape, owner);
    case flux::DType::I16:
      return typed_writable<std::int16_t>(data, ndim, shape, owner);
    case flux::DType::U32:
      return typed_writable<std::uint32_t>(data, ndim, shape, owner);
    case flux::DType::I32:
      return typed_writable<std::int32_t>(data, ndim, shape, owner);
    case flux::DType::U64:
      return typed_writable<std::uint64_t>(data, ndim, shape, owner);
    case flux::DType::I64:
      return typed_writable<std::int64_t>(data, ndim, shape, owner);
    case flux::DType::F32:
      return typed_writable<float>(data, ndim, shape, owner);
    case flux::DType::F64:
      return typed_writable<double>(data, ndim, shape, owner);
    case flux::DType::F16:
      return nb::cast(nb::ndarray<nb::numpy, std::uint16_t>(
        data, ndim, shape, owner, nullptr, dl_dtype(nb::dlpack::dtype_code::Float, 16)));
    case flux::DType::BF16:  // .array promises numpy, which cannot name this one
      throw std::runtime_error(
        "flux: Loan.array is numpy, which has no bfloat16; fill this loan through __dlpack__ "
        "(torch.from_dlpack(loan)) instead");
  }
  throw std::runtime_error("flux: unknown DType in loan");
}

// 0-copy publish handle exposed to Python. Holds the claimed slot; `array` is a writable numpy
// view into it (write the frame there), `commit()` publishes. Dropping it uncommitted reverts
// the claim; the frame that slot held is dropped, not restored.
class Loan
{
public:
  Loan(
    flux::WriteSlot && ws, nb::object keepalive, std::vector<std::size_t> shape, flux::DType dtype,
    std::uint32_t itemsize, std::uint64_t nbytes)
  : keepalive_(std::move(keepalive)),
    ws_(std::move(ws)),
    shape_(std::move(shape)),
    dtype_(dtype),
    itemsize_(itemsize),
    nbytes_(nbytes)
  {
  }

  // A fresh writable view each call, owned by `self` (this Loan). Not cached: caching it would
  // make the array's owner point back to the Loan that holds it -- a reference cycle nanobind
  // does not collect. Each view pins the Loan (and its claimed slot) for its own lifetime.
  //
  // Reject once the slot is gone: after commit()/abort() the WriteSlot is consumed and its
  // data pointer is null/stale, so building a writable array over it would alias a null base
  // or a slot that is now published and may be borrowed by readers -- writing through it would
  // corrupt a live frame. (An array handed out BEFORE commit stays writable; drop it before
  // commit, mirroring the read-side FrameView contract.)
  nb::object array(nb::handle self)
  {
    if (!ws_.valid()) {
      throw std::runtime_error(
        "flux: Loan.array is unavailable after commit()/abort() -- the slot is no longer yours "
        "to write; obtain and fill the array before committing");
    }
    if (!ws_.host_addressable()) device_payload_has_no_host_view("Loan.array");
    return track(writable_view(ws_.data(), shape_.size(), shape_.data(), dtype_, self));
  }

  // `nbytes` publishes a prefix of the loan. A generated adapter loans the whole slot and only
  // knows the frame's real size once it is built, so without this it would publish slot_size
  // bytes on every frame. Only meaningful for a 1-D loan, where shrinking the byte count is the
  // same statement as shrinking the shape.
  flux::Published commit(std::optional<std::uint64_t> nbytes)
  {
    // Same answer WriteSlot::commit gives a spent handle, so the two languages read alike.
    if (!ws_.valid()) return flux::Published::TooLarge;
    std::uint64_t n = nbytes_;
    if (nbytes) {
      if (shape_.size() != 1) {
        throw std::invalid_argument(
          "flux: commit(nbytes) needs a 1-D loan -- a shorter byte count does not determine a "
          "multi-dimensional shape");
      }
      if (*nbytes > nbytes_ || *nbytes % itemsize_ != 0) {
        throw std::invalid_argument(
          "flux: commit(nbytes=" + std::to_string(*nbytes) + ") must be a multiple of " +
          std::to_string(itemsize_) + " and at most the " + std::to_string(nbytes_) +
          " bytes loaned");
      }
      n = *nbytes;
    }
    revoke_issued();
    return nbytes ? ws_.commit(n) : ws_.commit();
  }

  void abort()
  {
    revoke_issued();
    ws_.abort();
  }
  bool valid() const { return ws_.valid(); }

private:
  // A handed-out array aliases the claimed slot, so a write after commit() corrupts a published
  // frame. Clearing numpy's writeable flag makes that a ValueError. Weak, or the cycle leaks.
  nb::object track(nb::object arr)
  {
    issued_.push_back(nb::module_::import_("weakref").attr("ref")(arr));
    return arr;
  }

  void revoke_issued() noexcept
  {
    for (auto & w : issued_) {
      try {
        nb::object arr = w();
        if (!arr.is_none()) arr.attr("setflags")(nb::arg("write") = false);
      } catch (...) {
      }
    }
    issued_.clear();
  }

  std::vector<nb::object> issued_;

public:
  // The stream a producing kernel must be launched on. commit() waits on exactly this one, so
  // taking it from here rather than keeping a copy is what stops the two from diverging.
  // None on a publisher that declared no device.
  nb::object stream() const { return stream_object(ws_.stream()); }
  bool host_addressable() const { return ws_.host_addressable(); }

  // Device-side view of the same reserved slot, for cp.asarray(loan).
  // .array (numpy, host) stays alongside it -- on ShmDirect both name the same bytes, and which
  // one a caller reaches for says whether the frame is built by a kernel or by the CPU.
  nb::dict cai()
  {
    if (!ws_.valid()) {
      throw std::runtime_error(
        "flux: Loan.__cuda_array_interface__ is unavailable after commit()/abort() -- the slot "
        "is no longer yours to write");
    }
    void * p = ws_.device_ptr();
    if (p == nullptr) {
      throw std::runtime_error(
        "flux: this Publisher declared no device, so the slot has no device address; construct "
        "it with device='cuda' to publish from a kernel");
    }
    return cuda_array_interface(p, shape_.size(), shape_.data(), dtype_, false);
  }

  // The write-side dual of Frame.__dlpack__: the reserved slot as a DLPack capsule, which is the
  // only way to fill a loan whose dtype numpy cannot name. Device address when the
  // publisher declared one, host address otherwise -- on ShmDirect they are the same bytes.
  nb::object dlpack(nb::handle self)
  {
    if (!ws_.valid()) {
      throw std::runtime_error(
        "flux: Loan.__dlpack__ is unavailable after commit()/abort() -- the slot is no longer "
        "yours to write");
    }
    void * dev = ws_.device_ptr();
    return dlpack_capsule(
      dev != nullptr ? dev : ws_.data(), shape_.size(), shape_.data(), dtype_, self,
      dev != nullptr ? nb::device::cuda::value : nb::device::cpu::value,
      dev != nullptr ? cuda_device_id() : 0);
  }

  // Write-side dual of Frame.bits: the reserved slot as an unsigned numpy view of equal width,
  // so a caller can fill a bf16 loan through its own spelling of the type.
  nb::object bits(nb::handle self)
  {
    if (!ws_.valid()) {
      throw std::runtime_error(
        "flux: Loan.bits is unavailable after commit()/abort() -- the slot is no longer yours "
        "to write");
    }
    if (!ws_.host_addressable()) device_payload_has_no_host_view("Loan.bits");
    return track(bits_writable(ws_.data(), shape_.size(), shape_.data(), dtype_, self));
  }

  nb::object dlpack_device() const
  {
    const bool dev = ws_.stream().declared();
    return nb::make_tuple(
      static_cast<int>(dev ? nb::device::cuda::value : nb::device::cpu::value),
      dev ? cuda_device_id() : 0);
  }

private:
  // Declaration order matters -- members destroy in reverse, so `ws_` (which writes to the slot
  // on abort) must go BEFORE `keepalive_` drops the publisher's mapping. Same rule as Held.
  nb::object keepalive_;
  flux::WriteSlot ws_;
  std::vector<std::size_t> shape_;
  flux::DType dtype_;
  std::uint32_t itemsize_;
  std::uint64_t nbytes_;
};

// Read side of a GPU channel. The CPU path hands back a numpy array
// whose capsule ends the borrow whenever Python collects it, and that is fine because ending a
// host borrow is a decrement. With a declared stream it is a GPU synchronization instead, and
// letting it fall on an arbitrary thread at an arbitrary time is the non-determinism the fence
// exists to remove -- so a GPU frame is scoped.
//
// The object is inert until entered: __cuda_array_interface__ refuses outside a `with`, which is
// what makes "cp.asarray(v) without a domain" fail loudly rather than work until the day the
// collector runs somewhere inconvenient.
class Frame
{
public:
  Frame(flux::FrameView && v, nb::object keepalive)
  : held_(std::make_shared<Held>(Held{std::move(keepalive), std::move(v)}))
  {
    ndim_ = checked_frame_shape(held_->view, shape_);
    dtype_ = held_->view.meta().dtype;
    nbytes_ = held_->view.meta().nbytes;
    stream_ = held_->view.stream();
    host_addressable_ = held_->view.host_addressable();
  }

  nb::object enter(nb::handle self)
  {
    if (released_) throw std::runtime_error("flux: this frame was already released");
    entered_ = true;
    return nb::borrow(self);
  }

  // Ends the borrow: with a declared stream the wait happens first, so the publisher cannot
  // regain the slot while a consuming kernel still reads it. Returns false so an exception
  // inside the block still propagates.
  bool exit(nb::handle, nb::handle, nb::handle)
  {
    entered_ = false;
    if (!released_) {
      released_ = true;
      // A scoped borrow ends here whatever holds it: its release fences the declared stream and
      // that must land where the caller put it. A host borrow ends when the last share dies.
      if (scoped()) held_->view.release();
      held_.reset();
    }
    return false;
  }

  nb::dict cai() const
  {
    require_readable();
    const void * p = held_->view.device_ptr();
    if (p == nullptr) throw std::runtime_error("flux: this frame has no device address");
    return cuda_array_interface(p, ndim_, shape_, dtype_, /*read_only=*/true);
  }

  // The borrowed bytes as a DLPack capsule, for consumers that read DLPack rather than
  // __cuda_array_interface__ -- and the only way to hand out a dtype the CAI typestr and numpy
  // both fail to spell, which is what bf16 needs.
  //
  // Guarded like cai(): the tensor a consumer builds from this capsule aliases the slot, so on a
  // GPU channel it must not outlive the `with`. Consumer keywords (stream, max_version,
  // dl_device, copy) are accepted and ignored -- flux closes both seams synchronously,
  // so there is no stream to negotiate, and it has one address to give.
  nb::object dlpack(nb::handle self) const
  {
    require_readable();
    const void * dev = held_->view.device_ptr();
    return dlpack_capsule(
      dev != nullptr ? dev : held_->view.data(), ndim_, shape_, dtype_, anchor(self),
      dev != nullptr ? nb::device::cuda::value : nb::device::cpu::value,
      dev != nullptr ? cuda_device_id() : 0);
  }

  // The frame's bits as an unsigned numpy view of equal width, for a caller that wants to
  // reinterpret them itself. Built on the host address, so it exists only where that address is
  // one a host load may follow -- an iGPU slot is host memory too, a dGPU slot is not.
  nb::object bits(nb::handle self) const
  {
    require_readable();
    if (!host_addressable_) device_payload_has_no_host_view("Frame.bits");
    return bits_view(held_->view.data(), ndim_, shape_, dtype_, anchor(self));
  }

  // Readable outside the domain: which device holds the bytes is a property of the channel, and a
  // consumer asks this before deciding whether to enter at all.
  nb::object dlpack_device() const
  {
    const bool dev = scoped();
    return nb::make_tuple(
      static_cast<int>(dev ? nb::device::cuda::value : nb::device::cpu::value),
      dev ? cuda_device_id() : 0);
  }

  // The stream a consuming kernel must be launched on -- the one release() waits for. Readable
  // outside the domain: it is a property of the channel, not of the borrow.
  nb::object stream() const { return stream_object(stream_); }
  bool host_addressable() const { return host_addressable_; }

  nb::object shape() const
  {
    nb::list s;
    for (std::size_t i = 0; i < ndim_; ++i) s.append(shape_[i]);
    return nb::tuple(s);
  }
  const char * dtype() const { return dtype_name(dtype_); }
  std::uint64_t nbytes() const { return nbytes_; }
  bool released() const { return released_; }

private:
  // Whether ending this borrow is a stream synchronization rather than a decrement. That is what
  // the `with` domain exists for, so it is also what decides whether one is required: a host
  // frame (bf16) releases on collection exactly as the numpy view does.
  bool scoped() const { return stream_.declared(); }

  // A host view is owned by the borrow itself, so it keeps its bytes after this object leaves the
  // `with`. A scoped view stays owned by this object: that borrow ends at exit() by design.
  nb::object anchor(nb::handle self) const
  {
    if (scoped()) return nb::borrow(self);
    auto * share = new std::shared_ptr<Held>(held_);
    return nb::capsule(
      share, [](void * p) noexcept { delete static_cast<std::shared_ptr<Held> *>(p); });
  }

  void require_readable() const
  {
    if (released_) throw std::runtime_error("flux: this frame was already released");
    if (scoped() && !entered_) {
      throw std::runtime_error(
        "flux: a GPU frame is only readable inside `with` -- releasing it fences the declared "
        "stream, which must not happen whenever the collector gets around to it. "
        "Use: with sub.take() as v: ...");
    }
  }

  // Declaration order matters -- members destroy in reverse, so `view_` (whose release touches
  // the segment) must go before `keepalive_` drops the mapping. Same rule as Held and Loan.
  std::shared_ptr<Held> held_;
  flux::gpu::Stream stream_;
  bool host_addressable_ = false;
  std::size_t shape_[flux::kMaxDims];
  std::size_t ndim_ = 0;
  flux::DType dtype_ = flux::DType::U8;
  std::uint64_t nbytes_ = 0;
  bool entered_ = false;
  bool released_ = false;
};

class Publisher
{
public:
  Publisher(
    const std::string & topic, std::uint64_t fingerprint, std::uint32_t slot_size,
    std::uint32_t slot_count, nb::handle device, const flux::MemoryPolicy & mem)
  : seg_name_(flux::signpost_name(require_absolute(topic), fingerprint)),
    slot_size_(slot_size),
    // Declared at construction, not through a setter: a setter leaves a window in which the
    // channel exists with no stream, and a frame published in it goes out unfenced.
    // An unusable declaration throws here rather than per frame.
    ch_(
      flux::open_publisher_segment(
        seg_name_, slot_size, slot_count, fingerprint, device_from(device)),
      flux::gpu::stream_for(device_from(device)), mem)
  {
    announce(seg_name_, topic, /*publisher=*/true, /*label=*/"");
  }

  // The device restriction is stated here rather than in the signature so the refusal can name
  // the path that does work. flux copies host bytes into the slot; moving device bytes there is
  // a CUDA copy flux_core does not offer, and loan() already publishes from a kernel with no
  // copy at all -- which is the point of a GPU channel.
  flux::Published publish(nb::ndarray<nb::c_contig> arr)
  {
    if (arr.device_type() != nb::device::cpu::value) {
      throw std::invalid_argument(
        "flux: publish() copies from host memory and cannot take a device array; use loan() and "
        "write into it with a kernel");
    }
    // The mirror of the line above on a channel whose slots are GPU memory: the source is fine
    // and the destination is not. Raised rather than left to the engine's drop counter, because
    // a publish that can never succeed is a wiring mistake, not backpressure.
    if (!ch_.host_addressable()) {
      throw std::invalid_argument(
        "flux: this channel's slots are GPU memory, so publish() has nowhere to copy to; use "
        "loan() and write into it with a kernel");
    }
    // Rank, like size and device, is a shape flux cannot carry rather than a slot it has to wait
    // for. Raised for the same reason as the two above: waiting never makes it succeed.
    if (arr.ndim() > flux::kMaxDims) {
      throw std::invalid_argument(
        "flux: array of rank " + std::to_string(arr.ndim()) + " exceeds the " +
        std::to_string(flux::kMaxDims) + " dimensions a frame descriptor holds");
    }
    if (arr.nbytes() > slot_size_) {
      throw std::invalid_argument(
        "flux: array of " + std::to_string(arr.nbytes()) + " bytes exceeds slot_size " +
        std::to_string(slot_size_) + " (raise slot_size or send a smaller array)");
    }
    std::vector<std::uint64_t> dims;
    dims.reserve(arr.ndim());
    for (std::size_t i = 0; i < arr.ndim(); ++i) dims.push_back(arr.shape(i));
    // Same flux::Published flux_cpp returns. The checks above raise instead, because they are
    // about the argument and the message can name the path that does work (docs/en/contracts.en.md
    // 3); what is left is the channel's own answer and it is reported, not collapsed.
    return ch_.publish(arr.data(), flux_from_dl(arr.dtype()), dims.data(), dims.size());
  }

  // 0-copy publish: reserve a free slot and return a Loan whose .array writes straight into
  // shared memory. commit() publishes. Returns None if every slot is borrowed (dropped).
  nb::object loan(nb::handle self, nb::object shape, nb::handle dtype)
  {
    std::vector<std::size_t> shp;
    if (nb::isinstance<nb::int_>(shape)) {
      shp.push_back(nb::cast<std::size_t>(shape));
    } else {
      const std::size_t n = nb::len(shape);
      for (std::size_t i = 0; i < n; ++i) shp.push_back(nb::cast<std::size_t>(shape[i]));
    }
    if (shp.empty() || shp.size() > flux::kMaxDims) {
      throw std::invalid_argument(
        "flux: loan shape rank must be 1.." + std::to_string(flux::kMaxDims));
    }
    auto [dt, itemsize] = flux_from_np_dtype(dtype);
    std::uint64_t nbytes = itemsize;
    for (std::size_t s : shp) {  // unchecked, this wraps to a small value and passes the cap below
      if (s != 0 && nbytes > UINT64_MAX / s) {
        throw std::invalid_argument("flux: loan shape overflows a 64-bit byte count");
      }
      nbytes *= s;
    }
    if (nbytes > slot_size_) {
      throw std::invalid_argument(
        "flux: loan of " + std::to_string(nbytes) + " bytes exceeds slot_size " +
        std::to_string(slot_size_));
    }
    std::vector<std::uint64_t> dims(shp.begin(), shp.end());
    flux::WriteSlot ws = ch_.loan(dt, dims.data(), dims.size());
    if (!ws) return nb::none();  // all slots borrowed -> dropped
    return nb::cast(Loan(std::move(ws), nb::borrow(self), std::move(shp), dt, itemsize, nbytes));
  }

  std::uint64_t dropped() const { return ch_.dropped(); }
  std::uint64_t fence_failed() const { return ch_.fence_failed(); }
  bool host_addressable() const { return ch_.host_addressable(); }
  flux::Channel::FenceWait fence_wait() const { return ch_.fence_wait(); }
  nb::object stream() const { return stream_object(ch_.stream()); }
  std::uint32_t slot_size() const { return slot_size_; }
  const std::string & segment_name() const { return seg_name_; }
  const std::string & domain() const { return flux::process_domain(); }

private:
  std::string seg_name_;
  std::uint32_t slot_size_;
  flux::Channel ch_;
};

class Subscription
{
public:
  Subscription(
    const std::string & topic, std::uint64_t fingerprint, const flux::QoS & qos, nb::handle device,
    const flux::MemoryPolicy & mem)
  : seg_name_(flux::signpost_name(require_absolute(topic), fingerprint)),
    fp_(fingerprint),
    qos_(qos),
    mem_(mem),
    // Same reason as qos_.validate() below: a declaration this host cannot serve is refused
    // where it is made, not once per frame on whichever thread ends up holding a view.
    stream_(flux::gpu::stream_for(device_from(device)))
  {
    announce(seg_name_, topic, /*publisher=*/false, /*label=*/"");
    qos_.validate();  // fail at construction, not on the first take
    attach();  // join the stream here if the publisher is already up: volatile is measured from
               // where this subscription joined, and attaching lazily would move that to the
               // first take. Failure is normal (late publisher) and retried on every call.
  }

  // `self` is the python Subscription object; the returned view pins it so the mmap the
  // view aliases cannot be unmapped while the view is alive.

  // Current state: the newest frame, without consuming it. Returns the same frame again when
  // nothing new was published, so it is None only if nothing has ever been published.
  nb::object peek(nb::handle self)
  {
    if (!attach()) return nb::none();
    flux::FrameView v = ch_->peek();
    if (!v) return nb::none();
    return wrap(std::move(v), self);
  }

  // The next frame in publish order, consumed. None once caught up. qos.depth bounds how far
  // behind this may sit; frames dropped by that window (or lapped by the ring) land in .lost.
  nb::object take(nb::handle self)
  {
    if (!attach()) return nb::none();
    flux::FrameView v = ch_->take();
    if (!v) return nb::none();
    return wrap(std::move(v), self);
  }

  // take(), parking on the futex wake word until a frame arrives or timeout_ns elapses
  // (negative = forever). Near-zero CPU, unlike polling take() in a loop. Returns None on
  // timeout / if the publisher segment is not up yet. The GIL is released while parked.
  // Nanoseconds, like every other flux timeout -- one unit across both languages.
  nb::object take_blocking(nb::handle self, std::int64_t timeout_ns)
  {
    if (!attach()) return nb::none();
    const bool infinite = timeout_ns < 0;
    const std::int64_t deadline = infinite ? 0 : mono_ns() + timeout_ns;
    for (;;) {
      const std::uint32_t wseq = ch_->wake_seq();  // sample, then look for a frame
      flux::FrameView v = ch_->take();
      if (v) return wrap(std::move(v), self);
      if (!ch_->can_borrow()) return nb::none();  // only the caller can free a lease
      std::int64_t rel = -1;
      if (!infinite) {
        rel = deadline - mono_ns();
        if (rel <= 0) return nb::none();  // timeout
      }
      {
        // Bounded park: re-running take() is what lets a re-attach happen after a publisher
        // restart, whose new segment nothing will bump the old wake word for.
        constexpr std::int64_t kMaxParkNs = 100'000'000;
        nb::gil_scoped_release unlocked;
        ch_->wait(wseq, (rel < 0 || rel > kMaxParkNs) ? kMaxParkNs : rel);
      }
      // Deliver Ctrl-C between parks. Without this a pending SIGINT waits for the next frame
      // and then corrupts the view construction instead of raising KeyboardInterrupt.
      if (PyErr_CheckSignals() != 0) throw nb::python_error();
    }
  }

  const flux::QoS & qos() const { return qos_; }
  std::uint64_t lost() const { return ch_ ? ch_->lost() : 0; }
  std::uint64_t fence_failed() const { return ch_ ? ch_->fence_failed() : 0; }
  // True until proven otherwise: an unattached subscription has no channel to ask, and a host
  // channel is the answer that costs nothing to be wrong about here (the guards that matter sit
  // on Frame, which only exists once a channel does).
  bool host_addressable() const { return ch_ ? ch_->host_addressable() : true; }
  flux::Channel::FenceWait fence_wait() const
  {
    return ch_ ? ch_->fence_wait() : flux::Channel::FenceWait{};
  }
  nb::object stream() const { return stream_object(stream_); }
  const std::string & segment_name() const { return seg_name_; }
  const std::string & domain() const { return flux::process_domain(); }
  bool attached() const { return ch_.has_value(); }
  bool can_borrow() const { return ch_ ? ch_->can_borrow() : true; }
  flux::Channel::Refused refused() const { return ch_ ? ch_->refused() : flux::Channel::Refused{}; }

  // Executor hooks, not exposed to Python. The executor drives the lazy attach itself so it can
  // arm its wait layer on this subscription's wake word as soon as the publisher shows up.
  bool ensure_attached() { return attach(); }
  flux::Channel * channel() { return ch_ ? &*ch_ : nullptr; }

private:
  // Lazily attach to the publisher's segment, applying the QoS chosen at construction.
  bool attach()
  {
    if (ch_ && ch_->orphaned()) ch_.reset();  // dead stream: drop the mapping, re-attach lazily
    if (ch_) return true;
    try {
      // Channel::open, not open_subscriber_segment: only the former records the signpost
      // name + epoch, and without those a publisher restart is never detected.
      ch_.emplace(flux::Channel::open(seg_name_, fp_, stream_, mem_));
    } catch (const flux::SegmentMismatch &) {
      throw;  // wrong fingerprint/version/config: retrying can never fix it, so say so
    } catch (...) {
      return false;  // publisher segment not up yet; caller retries
    }
    ch_->qos(qos_);  // validated in the constructor, so this cannot throw here
    return true;
  }

  // A host frame is a numpy array whose capsule ends the borrow on collection; a device frame
  // is scoped, because ending that borrow fences a stream. The
  // declaration decides, so the two never mix on one subscription.
  // A frame numpy cannot name comes back as flux.Frame for the same reason a GPU frame does:
  // the object is what carries __dlpack__. Such a host frame needs no `with` -- ending
  // a host borrow is a decrement, so it releases on collection like the numpy view.
  nb::object wrap(flux::FrameView && v, nb::handle self)
  {
    if (stream_.declared() || v.meta().dtype == flux::DType::BF16) {
      return nb::cast(Frame(std::move(v), nb::borrow(self)));
    }
    return view_from_frame(std::move(v), self);
  }

  std::string seg_name_;
  std::uint64_t fp_;
  flux::QoS qos_;
  flux::gpu::Stream stream_;
  flux::MemoryPolicy mem_;  // same reason as stream_: declared once, re-applied per attach
  std::optional<flux::Channel> ch_;
};

// One wait for many subscriptions -- the Python counterpart of flux::ros::Executor.
// The wait itself is flux::Executor, shared with flux_cpp so the ordering
// rules that make a wake safe exist in one place; this adds only what Python needs on top.
//
// The GIL is released across the blocking wait and held for every callback, so callbacks run on
// the thread that called spin()/spin_once() and never concurrently with each other.
class PyExecutor : public flux::Executor
{
public:
  using flux::Executor::Executor;

  void wait_for_work(std::int64_t timeout_ns) override
  {
    nb::gil_scoped_release unlocked;
    flux::Executor::wait_for_work(timeout_ns);
  }

protected:
  // Deliver Ctrl-C between passes. Without this a pending SIGINT waits for the next frame.
  void on_pass() override
  {
    if (PyErr_CheckSignals() != 0) throw nb::python_error();
  }
};

// One registered subscription plus the callback to run per frame. Holds the python objects so
// the mapping a view aliases cannot be unmapped while the executor still drives it.
class PySource : public flux::Source
{
public:
  PySource(nb::object obj, Subscription * sub, nb::callable cb)
  : obj_(std::move(obj)), sub_(sub), cb_(std::move(cb))
  {
  }

  bool attach() override { return sub_->ensure_attached(); }

  int deliver_one() override
  {
    nb::object v = sub_->take(obj_);
    if (v.is_none()) return 0;
    cb_(v);
    return 1;
  }

  flux::Channel * channel() noexcept override { return sub_->channel(); }

  nb::object obj_;
  Subscription * sub_ = nullptr;
  nb::callable cb_;
};

class Executor
{
public:
  explicit Executor(unsigned max_channels, std::int64_t poll_tick_ns)
  : core_(max_channels, poll_tick_ns)
  {
  }

  Executor(const Executor &) = delete;
  Executor & operator=(const Executor &) = delete;

  void add(nb::object sub_obj, nb::callable cb, int priority)
  {
    if (!nb::isinstance<Subscription>(sub_obj)) {
      throw nb::type_error("flux: Executor.add expects a flux.Subscription as its first argument");
    }
    Subscription * s = &nb::cast<Subscription &>(sub_obj);
    auto src = std::make_unique<PySource>(std::move(sub_obj), s, std::move(cb));
    core_.add(*src, priority);  // throws past max_channels, before this source is kept
    sources_.push_back(std::move(src));
  }

  int spin_once(std::int64_t timeout_ns) { return core_.spin_once(timeout_ns); }
  void spin(std::int64_t tick_ns) { core_.spin(tick_ns); }
  void stop() noexcept { core_.stop(); }
  void interrupt() noexcept { core_.interrupt(); }
  bool is_spinning() const noexcept { return core_.is_spinning(); }
  int dispatch() { return core_.dispatch(); }
  bool has_more() const noexcept { return core_.has_more(); }
  void set_pass_budget(int n) { core_.set_pass_budget(n); }
  int pass_budget() const noexcept { return core_.pass_budget(); }
  void wait_for_work(std::int64_t timeout_ns) { core_.wait_for_work(timeout_ns); }
  bool uses_io_uring() const noexcept { return core_.uses_io_uring(); }
  std::size_t size() const noexcept { return sources_.size(); }

  // ---- cyclic GC (paired with the tp_traverse/tp_clear slots at the binding) ----
  //
  // A callback almost always closes over the executor itself (`lambda v: ex.stop()`), so
  // Executor -> callback -> Executor is the normal case, not an edge case. Without these the
  // cycle is uncollectable and it pins every registered Subscription -- and therefore its shm
  // mapping -- for the life of the process.
  int traverse(visitproc visit, void * arg) noexcept
  {
    for (auto & s : sources_) {
      if (s->obj_.ptr() != nullptr) {
        if (const int r = visit(s->obj_.ptr(), arg); r != 0) return r;
      }
      if (s->cb_.ptr() != nullptr) {
        if (const int r = visit(s->cb_.ptr(), arg); r != 0) return r;
      }
    }
    return 0;
  }

  // Drop every Python reference. The core is emptied first: it holds raw Source pointers, and
  // clearing the other order would leave it driving objects that are going away.
  void clear_refs() noexcept
  {
    core_.clear();
    sources_.clear();
  }

private:
  PyExecutor core_;
  std::vector<std::unique_ptr<PySource>> sources_;
};

// GC slots for Executor. nanobind flips on Py_TPFLAGS_HAVE_GC once a tp_traverse slot is
// supplied, which is what lets the interpreter collect the Executor <-> callback cycle.
int executor_tp_traverse(PyObject * self, visitproc visit, void * arg)
{
  if (nb::inst_ready(self)) {
    if (const int r = nb::inst_ptr<Executor>(self)->traverse(visit, arg); r != 0) return r;
  }
#if PY_VERSION_HEX >= 0x03090000
  Py_VISIT(Py_TYPE(self));  // heap types are GC-tracked from 3.9 on
#endif
  return 0;
}

int executor_tp_clear(PyObject * self)
{
  if (nb::inst_ready(self)) nb::inst_ptr<Executor>(self)->clear_refs();
  return 0;
}

PyType_Slot kExecutorSlots[] = {
  {Py_tp_traverse, reinterpret_cast<void *>(executor_tp_traverse)},
  {Py_tp_clear, reinterpret_cast<void *>(executor_tp_clear)},
  {0, nullptr},
};

}  // namespace

NB_MODULE(_flux, m)
{
  m.doc() = "flux: same-host zero-copy shared-memory transport (C++ <-> Python)";

  // No schema contract: any publisher on the same topic name attaches. Named rather than spelled
  // 0 at the call site, where a literal does not say whether it is a decision or a hole.
  m.attr("NO_SCHEMA") = flux::kNoSchema;

  // Derives from RuntimeError so code that predates this type still catches it. Without its own
  // translator, an attach that can never succeed would arrive as a bare RuntimeError -- the
  // distinction between "not up yet" and "will never work" that C++ callers get for free
  // (docs/en/contracts.en.md 1, X-019).
  nb::exception<flux::SegmentMismatch>(m, "SegmentMismatch", PyExc_RuntimeError);

  // A refused mlock or page commit carries an errno, and errno failures are OSError in Python.
  // Without this nanobind lands them on RuntimeError, where `except OSError` -- the thing a
  // caller writes for a resource limit -- does not catch them.
  nb::register_exception_translator(
    [](const std::exception_ptr & p, void *) {
      try {
        std::rethrow_exception(p);
      } catch (const std::system_error & e) {
        PyErr_SetString(PyExc_OSError, e.what());
      }
    },
    nullptr);

  // Bound as an enum and accepted as a string: device="cuda" reads best at a call site, and
  // flux.Device.Cuda is what a typo cannot survive.
  nb::enum_<flux::Device>(m, "Device")
    .value("Cpu", flux::Device::Cpu, "Host path. The default -- nothing here touches CUDA.")
    .value(
      "Cuda", flux::Device::Cuda,
      "CUDA path. flux creates the stream both seams fence on. Raises ValueError when this host "
      "cannot serve the declaration rather than quietly running on the host path.");

  nb::class_<flux::Durability>(m, "Durability")
    .def_prop_ro("replay", [](const flux::Durability & d) { return d.replay; })
    .def_prop_ro("is_volatile", &flux::Durability::is_volatile)
    .def("__repr__", [](const flux::Durability & d) {
      return d.is_volatile() ? std::string("Volatile()")
                             : "TransientLocal(" + std::to_string(d.replay) + ")";
    });

  m.def(
    "Volatile", &flux::Durability::Volatile,
    "Durability: only frames published after this subscription attaches (default).");
  m.def(
    "TransientLocal", &flux::Durability::TransientLocal, nb::arg("n"),
    "Durability: replay up to n frames already in the ring, then follow live. n must not exceed "
    "qos.depth, and is capped by what the publisher's ring still holds.");

  nb::enum_<flux::Reliability>(m, "Reliability")
    .value("BEST_EFFORT", flux::Reliability::BestEffort)
    .value("RELIABLE", flux::Reliability::Reliable);

  // Same five outcomes flux_cpp returns. Not collapsed to a bool: Backpressure is a dropped
  // frame on a best-effort transport, while FenceFailed leaks the slot for good, and a caller
  // that cannot tell them apart watches the ring shrink with nothing in the log.
  nb::enum_<flux::Published>(m, "Published")
    .value("Ok", flux::Published::Ok, "Published.")
    .value(
      "Backpressure", flux::Published::Backpressure,
      "Every slot was borrowed, so the frame was dropped. A rate, not a fault: delivery is "
      "best-effort and .dropped counts these.")
    .value(
      "TooLarge", flux::Published::TooLarge,
      "The frame does not fit the slot, or this handle had already been committed or aborted.")
    .value(
      "WrongDevice", flux::Published::WrongDevice,
      "A host publish into a device-backed channel. loan() plus a kernel is the path there.")
    .value(
      "FenceFailed", flux::Published::FenceFailed,
      "The declared stream could not be waited on. The slot is never reused, so a nonzero "
      ".fence_failed is a fault, not a rate.")
    // A nanobind enum is truthy for every value, Ok included -- and Ok is 0, so neither the
    // default nor an int cast says what a caller means by `if not p`. Spell it: true means the
    // frame went out.
    .def("__bool__", [](flux::Published p) { return p == flux::Published::Ok; });

  m.def(
    "faulted", &flux::faulted, nb::arg("published"),
    "One question instead of five. False for Ok and for Backpressure (a dropped frame on a "
    "best-effort transport); True for the outcomes that do not clear on their own and are worth "
    "a log. Same function flux_cpp exposes.");

  nb::class_<flux::Channel::Refused>(m, "Refused")
    .def_ro("max_borrow", &flux::Channel::Refused::max_borrow)
    .def_ro("holder_table", &flux::Channel::Refused::holder_table)
    .def_ro("not_ready", &flux::Channel::Refused::not_ready)
    .def_ro("contended", &flux::Channel::Refused::contended)
    .def_ro("bad_frame", &flux::Channel::Refused::bad_frame)
    .def_ro("no_owner_file", &flux::Channel::Refused::no_owner_file)
    .def_ro("fence", &flux::Channel::Refused::fence)
    .def_prop_ro("total", &flux::Channel::Refused::total)
    .def("__repr__", [](const flux::Channel::Refused & r) {
      return "Refused(max_borrow=" + std::to_string(r.max_borrow) +
             ", holder_table=" + std::to_string(r.holder_table) +
             ", not_ready=" + std::to_string(r.not_ready) +
             ", contended=" + std::to_string(r.contended) +
             ", bad_frame=" + std::to_string(r.bad_frame) +
             ", no_owner_file=" + std::to_string(r.no_owner_file) +
             ", fence=" + std::to_string(r.fence) + ")";
    });

  // How long the two GPU seams blocked, kept apart because they block different threads: a
  // publisher owns the gap between launching a kernel and committing, while a callback consumer
  // owns nothing -- its frame dies when the callback returns.
  nb::class_<flux::Channel::FenceWait>(m, "FenceWait")
    .def_ro("commit_ns", &flux::Channel::FenceWait::commit_ns)
    .def_ro("commit_count", &flux::Channel::FenceWait::commit_count)
    .def_ro("commit_max_ns", &flux::Channel::FenceWait::commit_max_ns)
    .def_ro("release_ns", &flux::Channel::FenceWait::release_ns)
    .def_ro("release_count", &flux::Channel::FenceWait::release_count)
    .def_ro("release_max_ns", &flux::Channel::FenceWait::release_max_ns)
    .def("__repr__", [](const flux::Channel::FenceWait & w) {
      return "FenceWait(commit_ns=" + std::to_string(w.commit_ns) +
             ", commit_count=" + std::to_string(w.commit_count) +
             ", commit_max_ns=" + std::to_string(w.commit_max_ns) +
             ", release_ns=" + std::to_string(w.release_ns) +
             ", release_count=" + std::to_string(w.release_count) +
             ", release_max_ns=" + std::to_string(w.release_max_ns) + ")";
    });

  nb::class_<flux::MemoryPolicy>(m, "MemoryPolicy")
    .def(
      "__init__",
      [](flux::MemoryPolicy * self, bool precommit, bool lock) {
        flux::MemoryPolicy p;
        p.precommit = precommit;
        p.lock = lock;
        new (self) flux::MemoryPolicy(p);
      },
      nb::arg("precommit") = false, nb::arg("lock") = false,
      "Page residency for this process's mapping of a channel. Off by default: "
      "flux maps a sparse segment so a large slot_size costs almost no idle RAM, and the price "
      "is a page fault the first time each page is touched. precommit faults them all in at "
      "attach; lock also mlocks the mapping, which needs RLIMIT_MEMLOCK to cover the segment. "
      "A refusal raises rather than coming back as a weaker channel.")
    .def_ro("precommit", &flux::MemoryPolicy::precommit)
    .def_ro("lock", &flux::MemoryPolicy::lock)
    .def("__repr__", [](const flux::MemoryPolicy & p) {
      return "MemoryPolicy(precommit=" + std::string(p.precommit ? "True" : "False") +
             ", lock=" + (p.lock ? "True" : "False") + ")";
    });

  nb::class_<flux::QoS>(m, "QoS")
    .def(
      "__init__",
      [](
        flux::QoS * self, std::uint32_t depth, const flux::Durability & durability,
        std::uint32_t max_borrow, flux::Reliability reliability) {
        flux::QoS q;
        q.depth = depth;
        q.durability = durability;
        q.max_borrow = max_borrow;
        q.reliability = reliability;
        q.validate();  // ValueError here rather than a surprise at the first take
        new (self) flux::QoS(q);
      },
      nb::arg("depth") = 1u, nb::arg("durability") = flux::Durability::Volatile(),
      nb::arg("max_borrow") = 2u, nb::arg("reliability") = flux::Reliability::BestEffort,
      "Consumer QoS. depth: frames this subscription may fall behind the newest (1 = newest "
      "only). durability: Volatile() or TransientLocal(n) for the backlog on attach. "
      "max_borrow: views held at once. reliability: BEST_EFFORT only.")
    .def_ro("depth", &flux::QoS::depth)
    .def_ro("durability", &flux::QoS::durability)
    .def_ro("max_borrow", &flux::QoS::max_borrow)
    .def_ro("reliability", &flux::QoS::reliability)
    .def("__repr__", [](const flux::QoS & q) {
      return "QoS(depth=" + std::to_string(q.depth) + ", durability=" +
             (q.durability.is_volatile()
                ? std::string("Volatile()")
                : "TransientLocal(" + std::to_string(q.durability.replay) + ")") +
             ", max_borrow=" + std::to_string(q.max_borrow) + ")";
    });

  nb::class_<Loan>(m, "Loan")
    .def_prop_ro(
      "array", [](nb::handle self) { return nb::cast<Loan &>(self).array(self); },
      "Writable numpy array aliasing the reserved slot. Write your frame here, then commit(). "
      "Do not write after commit() -- the frame is then live and may be borrowed by readers.")
    .def(
      "commit", &Loan::commit, nb::arg("nbytes") = nb::none(),
      "Publish the bytes written into array() with no copy. Returns a flux.Published: Ok, "
      "Backpressure (dropped, a rate), or a fault -- flux.faulted(p) is the one check. Truthy "
      "only for Ok, so `if not loan.commit()` still reads. nbytes publishes only the first "
      "nbytes of a 1-D loan -- what a generated adapter uses, since it loans the whole slot and "
      "sizes the frame as it builds it.")
    .def(
      "abort", &Loan::abort,
      "Discard without publishing. The frame that slot held is dropped, not restored -- array "
      "aliases it, so it is gone as soon as you write. Lagging consumers count it in .lost.")
    .def_prop_ro("valid", &Loan::valid)
    .def_prop_ro(
      "__cuda_array_interface__", &Loan::cai,
      "Device-side view of the reserved slot, so cp.asarray(loan) writes straight into shared "
      "memory. Raises unless the Publisher declared device='cuda'.")
    .def(
      "__dlpack__", [](nb::handle self, nb::kwargs) { return nb::cast<Loan &>(self).dlpack(self); },
      "Writable DLPack capsule over the reserved slot, for torch.from_dlpack(loan). The only "
      "route for a dtype numpy cannot name (bfloat16); .array covers the rest.")
    .def("__dlpack_device__", &Loan::dlpack_device, "(device_type, device_id) for DLPack.")
    .def_prop_ro(
      "bits", [](nb::handle self) { return nb::cast<Loan &>(self).bits(self); },
      "The reserved slot as a writable unsigned numpy view of the same element width. For a "
      "dtype numpy cannot name: bring your own spelling, e.g. "
      "loan.bits.view(ml_dtypes.bfloat16)[:] = x.")
    .def_prop_ro(
      "host_addressable", &Loan::host_addressable,
      "Whether .array and .bits exist here. False on a discrete GPU, where these bytes are VRAM: "
      "reach them through __cuda_array_interface__ or __dlpack__ instead.")
    .def_prop_ro(
      "stream", &Loan::stream,
      "Stream handle a producing kernel must be launched on -- the one commit() waits for. None "
      "on a host publisher.");

  // Returned by a device subscription's peek/take instead of a numpy array. Scoped rather than
  // collected, because releasing it fences the declared stream.
  nb::class_<Frame>(m, "Frame")
    .def("__enter__", [](nb::handle self) { return nb::cast<Frame &>(self).enter(self); })
    // .none() on all three: a `with` block that exits without an exception passes None, and a
    // nanobind argument refuses None unless it says so.
    .def(
      "__exit__", &Frame::exit, nb::arg("exc_type").none(), nb::arg("exc_value").none(),
      nb::arg("traceback").none())
    .def_prop_ro(
      "__cuda_array_interface__", &Frame::cai,
      "Read-only device view of the borrowed slot, for cp.asarray(frame). Raises outside `with`: "
      "the release that ends this borrow is a stream synchronization and must happen at a point "
      "you chose, not whenever Python collects the object.")
    // kwargs are swallowed: consumers pass stream=/max_version=/dl_device=/copy=, and flux has
    // one address and no stream to negotiate. Raises outside `with` for the same reason as
    // __cuda_array_interface__ -- the capsule aliases the slot this borrow holds.
    .def(
      "__dlpack__",
      [](nb::handle self, nb::kwargs) { return nb::cast<Frame &>(self).dlpack(self); },
      "DLPack capsule over the borrowed device bytes, for torch.from_dlpack(frame). The only "
      "route for a dtype the __cuda_array_interface__ typestr cannot spell (bfloat16).")
    .def(
      "__dlpack_device__", &Frame::dlpack_device,
      "(device_type, device_id) for DLPack. Readable outside `with`.")
    .def_prop_ro(
      "bits", [](nb::handle self) { return nb::cast<Frame &>(self).bits(self); },
      "The frame as a read-only unsigned numpy view of the same element width. For a dtype "
      "numpy cannot name: bring your own spelling, e.g. v.bits.view(ml_dtypes.bfloat16).")
    .def_prop_ro(
      "host_addressable", &Frame::host_addressable,
      "Whether .array and .bits exist here. False on a discrete GPU, where these bytes are VRAM: "
      "reach them through __cuda_array_interface__ or __dlpack__ instead.")
    .def_prop_ro(
      "stream", &Frame::stream,
      "Stream handle a consuming kernel must be launched on -- the one the release waits for.")
    .def_prop_ro("shape", &Frame::shape)
    .def_prop_ro(
      "dtype", &Frame::dtype,
      "numpy typestr of the frame's element type, or \"bfloat16\", which has no typestr.")
    .def_prop_ro("nbytes", &Frame::nbytes)
    .def_prop_ro("released", &Frame::released);

  nb::class_<Publisher>(m, "Publisher")
    .def(
      nb::init<
        const std::string &, std::uint64_t, std::uint32_t, std::uint32_t, nb::handle,
        const flux::MemoryPolicy &>(),
      nb::arg("topic"), nb::arg("fingerprint") = flux::kNoSchema,
      nb::arg("slot_size") = (16u << 20), nb::arg("slot_count") = 16u,
      nb::arg("device") = nb::none(), nb::arg("memory") = flux::MemoryPolicy{})
    .def(
      "publish", &Publisher::publish, nb::arg("array"),
      "Copy a C-contiguous numpy array into the next free slot (1-copy). Returns a "
      "flux.Published: Ok, or Backpressure when every slot is currently borrowed (frame dropped; "
      "delivery is best-effort). Truthy only for Ok. Raises ValueError before it gets that far "
      "if the array exceeds slot_size, is on a device, or the topic is not an absolute name.")
    .def(
      "loan",
      [](nb::handle self, nb::object shape, nb::handle dtype) {
        return nb::cast<Publisher &>(self).loan(self, shape, dtype);
      },
      nb::arg("shape"), nb::arg("dtype") = "uint8",
      "Reserve a free slot and return a Loan whose .array writes straight into shared memory "
      "(0-copy publish). Call commit() to publish. Returns None if every slot is borrowed "
      "(dropped; delivery is best-effort).")
    .def_prop_ro(
      "dropped", &Publisher::dropped,
      "Frames this publisher discarded because every slot was borrowed. A property, like "
      "Subscription.lost -- both are counters you read, not actions you perform.")
    .def_prop_ro(
      "fence_failed", &Publisher::fence_failed,
      "Commits refused because the declared stream could not be waited on. Each costs a slot "
      "that is never reused, so a nonzero value is a fault, not a rate. Always 0 with device "
      "'cpu'.")
    .def_prop_ro(
      "fence_wait", &Publisher::fence_wait,
      "How long commit blocked on the declared stream. The release fields stay 0 -- a publisher "
      "holds no frames.")
    .def_prop_ro(
      "stream", &Publisher::stream,
      "Stream handle a producing kernel must be launched on. None on a host publisher.")
    .def_prop_ro(
      "host_addressable", &Publisher::host_addressable,
      "Whether this channel's payload may be touched by host code. False on a discrete GPU, where "
      "the slot is VRAM: publish() and .array/.bits are refused there and the paths are loan() "
      "plus __cuda_array_interface__ / __dlpack__.")
    .def_prop_ro(
      "slot_size", &Publisher::slot_size,
      "Bytes one slot holds. A generated adapter loans this much and commits the prefix it "
      "actually filled.")
    .def_prop_ro(
      "segment_name", &Publisher::segment_name,
      "The fixed rendezvous name derived from the topic and fingerprint -- the signpost, not "
      "the segment. The segment behind it carries a per-instance suffix and is recreated on "
      "every publisher restart.")
    .def_prop_ro(
      "domain", &Publisher::domain,
      "The domain this publisher resolved for itself. Reported because a domain the caller did "
      "not expect is invisible otherwise: peers elsewhere simply never appear, which reads "
      "exactly like peers that have not started yet.");

  nb::class_<Subscription>(m, "Subscription")
    .def(
      nb::init<
        const std::string &, std::uint64_t, const flux::QoS &, nb::handle,
        const flux::MemoryPolicy &>(),
      nb::arg("topic"), nb::arg("fingerprint") = flux::kNoSchema, nb::arg("qos") = flux::QoS{},
      nb::arg("device") = nb::none(), nb::arg("memory") = flux::MemoryPolicy{})
    .def(
      "peek", [](nb::handle self) { return nb::cast<Subscription &>(self).peek(self); },
      "The newest frame as a read-only zero-copy numpy view, without consuming it. Returns the "
      "same frame again until something new is published; None only if nothing ever was. Use "
      "this to read current state on your own schedule.")
    .def(
      "take", [](nb::handle self) { return nb::cast<Subscription &>(self).take(self); },
      "The next frame in publish order, consumed, as a read-only zero-copy view. None once "
      "caught up. qos.depth bounds how far behind this may fall; frames dropped by that window "
      "or lapped by the ring are counted in .lost.")
    .def(
      "take_blocking",
      [](nb::handle self, std::int64_t timeout_ns) {
        return nb::cast<Subscription &>(self).take_blocking(self, timeout_ns);
      },
      nb::arg("timeout_ns") = -1,
      "take(), parking on the futex until a frame arrives or timeout_ns elapses (negative = "
      "forever). Near-zero CPU. Returns None on timeout. Prefer this over polling take().")
    .def_prop_ro(
      "lost", &Subscription::lost,
      "Frames the last take() never delivered: lapped by the ring, or dropped by qos.depth.")
    .def_prop_ro(
      "can_borrow", &Subscription::can_borrow,
      "False once max_borrow views are held. A take() that returned None with this False is "
      "this subscription holding its own views, not an idle stream, and no publish clears it.")
    .def_prop_ro(
      "refused", &Subscription::refused,
      "Why take()/peek() returned None when a frame was not simply absent. Cumulative.")
    .def_prop_ro("qos", &Subscription::qos)
    .def_prop_ro(
      "stream", &Subscription::stream,
      "Stream handle this subscription's frames are consumed on. None on a host subscription.")
    .def_prop_ro(
      "fence_wait", &Subscription::fence_wait,
      "How long releasing a frame blocked on the declared stream. On the callback path that "
      "wait lands on the executor's spin thread.")
    .def_prop_ro(
      "fence_failed", &Subscription::fence_failed,
      "Borrows never released because the declared stream could not be waited on. Each costs a "
      "slot that is never reused, so a nonzero value is a fault, not a rate. Always 0 on a host "
      "subscription.")
    .def_prop_ro(
      "host_addressable", &Subscription::host_addressable,
      "Whether this channel's payload may be touched by host code. False on a discrete GPU, where "
      "the slot is VRAM: publish() and .array/.bits are refused there and the paths are loan() "
      "plus __cuda_array_interface__ / __dlpack__.")
    .def_prop_ro(
      "segment_name", &Subscription::segment_name,
      "The fixed rendezvous name (the signpost), not the segment it currently points at.")
    .def_prop_ro(
      "attached", &Subscription::attached,
      "True while joined to a publisher's segment. Turns False when the publisher group dies "
      "and the dead mapping is dropped; a later publisher re-attaches lazily.")
    .def_prop_ro(
      "domain", &Subscription::domain,
      "The domain this subscription resolved for itself. `attached` says whether a publisher was "
      "found; this says which domain was searched. Apart they separate 'no publisher yet' from 'a "
      "publisher exists, in a domain this process is not looking in'.");

  nb::class_<Executor>(m, "Executor", nb::type_slots(kExecutorSlots))
    .def(
      nb::init<unsigned, std::int64_t>(), nb::arg("max_channels") = 32u,
      nb::arg("poll_tick_ns") = 2'000'000LL,
      "Wait on many Subscriptions with one blocking syscall and dispatch a callback per frame. "
      "On Linux 6.7+ every channel's wake word is armed in a single io_uring (no extra threads); "
      "on older kernels it falls back to a bounded poll tick (poll_tick_ns). Check "
      ".uses_io_uring to see which path is active. Every flux timeout is nanoseconds, in both "
      "languages.")
    .def(
      "add", &Executor::add, nb::arg("subscription"), nb::arg("callback"), nb::arg("priority") = 0,
      "Register a Subscription and the callback to invoke per frame -- a zero-copy numpy view, "
      "or a flux.Frame to open with `with` on a device subscription. What arrives is the "
      "subscription's QoS: depth=1 delivers the newest frame per wake, "
      "depth=N drains up to N in publish order. The callback runs on the spin thread; do not "
      "retain the view past the call unless max_borrow allows it. priority orders the visit "
      "within one dispatch pass, higher first, ties in registration order; it does not preempt "
      "a callback that is already running.")
    .def(
      "spin_once", &Executor::spin_once, nb::arg("timeout_ns") = -1,
      "Dispatch whatever is ready; if nothing is, block up to timeout_ns (negative = forever) "
      "and dispatch what arrives. Returns the number of callbacks invoked.")
    .def(
      "spin", &Executor::spin, nb::arg("tick_ns") = 100'000'000LL,
      "Loop spin_once until stop(). tick_ns bounds each wait so Ctrl-C is delivered promptly and "
      "late publishers are picked up; it is a wait bound, not a polling interval.")
    .def_prop_ro(
      "is_spinning", &Executor::is_spinning,
      "True while a spin() is in progress. stop() before spin() is not lost: the request is "
      "held until a spin consumes it, and cleared on the way out so the executor can be spun "
      "again.")
    .def(
      "stop", &Executor::stop,
      "Break out of spin(). Safe to call from a callback or another thread.")
    .def(
      "interrupt", &Executor::interrupt,
      "Break the current wait without stopping the loop. Safe to call from another thread. "
      "(Not called `wake`: flux uses that word for the in-segment futex wake generation.)")
    .def(
      "dispatch", &Executor::dispatch,
      "Deliver every ready frame and re-arm, without blocking. Returns the callback count. "
      "Pair with wait_for_work() to embed flux in another event loop; the two must not run "
      "concurrently.")
    .def(
      "wait_for_work", &Executor::wait_for_work, nb::arg("timeout_ns") = -1,
      "Block until some channel is woken, interrupt()/stop() is called, or timeout_ns elapses. "
      "Dispatches nothing; call dispatch() afterwards. Releases the GIL while blocked.")
    .def_prop_ro(
      "has_more", &Executor::has_more,
      "True if the last dispatch() stopped on pass_budget with frames still ready. Driving "
      "wait_for_work()/dispatch() yourself, skip the wait while this is true: nothing pokes the "
      "ring again for a frame that was already there.")
    .def_prop_rw(
      "pass_budget", &Executor::pass_budget, &Executor::set_pass_budget,
      "Callbacks one dispatch() may run over all channels together. Default 64. Set it before "
      "spin(). It bounds a pass, not throughput: what it cuts off the next pass delivers without "
      "blocking. Lower it to tighten the delay a high-priority channel can see behind lower "
      "ones; raise it to spread the per-pass arming over more frames.")
    .def_prop_ro(
      "uses_io_uring", &Executor::uses_io_uring,
      "True if every channel is waited on through one io_uring (Linux 6.7+). False means the "
      "kernel lacks the opcode and the fallback is active: one parker thread per channel, "
      "waking the spin thread through a control eventfd.")
    .def("__len__", &Executor::size);

  nb::class_<flux::EndpointView>(m, "Endpoint", "One live process's endpoint on a channel.")
    .def_prop_ro(
      "pid", [](const flux::EndpointView & e) { return e.owner.pid; },
      "Pid of the process holding this endpoint.")
    .def_prop_ro(
      "starttime", [](const flux::EndpointView & e) { return e.owner.starttime; },
      "Process start time, which is what makes the pid safe to compare across pid reuse.")
    .def_ro(
      "label", &flux::EndpointView::label,
      "What the boundary announced. flux_cpp puts the ROS node's fully qualified name "
      "here; empty when nothing was announced.")
    .def_ro(
      "publisher", &flux::EndpointView::publisher,
      "True for a publisher, False for a "
      "subscriber.");

  nb::class_<flux::TopicView>(m, "Topic", "One flux channel as enumeration sees it.")
    .def_ro(
      "signpost", &flux::TopicView::signpost,
      "The channel's fixed /dev/shm name. The identity everything else joins on.")
    .def_ro("domain", &flux::TopicView::domain, "The domain the name is under.")
    .def_ro("key", &flux::TopicView::key, "The channel key.")
    .def_ro("fingerprint", &flux::TopicView::fingerprint, "The schema fingerprint.")
    .def_ro(
      "key_exact", &flux::TopicView::key_exact,
      "False when `key` was read back out of the name instead of from a live "
      "participant, in which case its punctuation is the name's, not the key's.")
    .def_ro(
      "endpoints", &flux::TopicView::endpoints,
      "Endpoints held by processes that are still alive.");

  nb::class_<flux::ChannelStats>(
    m, "ChannelStats", "What a channel's current segment reports, read without attaching.")
    .def_ro(
      "live", &flux::ChannelStats::live,
      "False when the signpost advertises no readable segment right now.")
    .def_ro("slot_size", &flux::ChannelStats::slot_size, "Bytes per slot.")
    .def_ro("slot_count", &flux::ChannelStats::slot_count, "Slots in the ring.")
    .def_ro(
      "storage_kind", &flux::ChannelStats::storage_kind,
      "Where the payload lives: 0 host-inline, otherwise a device-backed path.")
    .def_ro(
      "epoch", &flux::ChannelStats::epoch,
      "Signpost epoch. A change means the publisher group restarted.")
    .def_ro("fingerprint", &flux::ChannelStats::fingerprint, "Schema fingerprint.")
    .def_ro(
      "publish_seq", &flux::ChannelStats::publish_seq,
      "Global ticket dispenser, monotone. Sample it twice and take the difference for an "
      "exact frame count over the interval.")
    .def_ro("waiters", &flux::ChannelStats::waiters, "Subscribers parked on the wake gate.");

  m.def(
    "resolve_domain",
    [](std::optional<std::string> domain_env) {
      return domain_env.has_value() ? flux::resolve_domain(domain_env->c_str())
                                    : flux::resolve_domain(nullptr);
    },
    nb::arg("domain_env").none() = flux::kDefaultDomainEnv,
    "Resolve a domain from the environment, now: FLUX_DOMAIN if set, else the integer in "
    "`domain_env` if that names a set variable, else \"0\". Pass None to opt out of the "
    "inherited variable entirely. This reads the environment on every call, so it is a query "
    "rather than the answer names are built from -- process_domain() is that.");

  // flux::rt, bound rather than rewritten in ctypes. The rollback on a half-applied request and
  // the read-back that catches an affinity a cpuset narrowed are the parts that are easy to get
  // wrong and easy to leave out; a second implementation is a second place for them to drift.
  //
  // Only the applying half is here. `apply` runs preflight at Soft strictness, so a request the
  // kernel cannot satisfy is refused with the reason, while a host that merely cannot hold a
  // deadline is reported and applied. That split is the whole of what Python needs: it is not an
  // RT target, so the Hard verdicts have nothing to gate here.
  nb::module_ rt = m.def_submodule(
    "rt",
    "OS scheduling for the threads a flux executor runs on. Not a real-time guarantee: GIL and "
    "GC stay unbounded latency sources whatever policy a thread holds. What this "
    "settles is which thread the kernel prefers and which cores it may use (docs/en/api.en.md 4).");

  nb::enum_<flux::rt::Policy>(rt, "Policy")
    .value("Inherit", flux::rt::Policy::Inherit, "Leave the scheduling class alone. The default.")
    .value("Other", flux::rt::Policy::Other, "SCHED_OTHER: explicit return to the default class.")
    .value(
      "Fifo", flux::rt::Policy::Fifo,
      "SCHED_FIFO: static priority, runs until it blocks or is preempted. Needs RLIMIT_RTPRIO or "
      "CAP_SYS_NICE; without either this is refused, never downgraded.")
    .value(
      "RoundRobin", flux::rt::Policy::RoundRobin,
      "SCHED_RR: Fifo plus a timeslice among equal priorities. Same permission.");

  nb::class_<flux::rt::ThreadState>(rt, "ThreadState", "What a thread actually runs at.")
    .def_ro(
      "policy", &flux::rt::ThreadState::policy,
      "The raw SCHED_* number, not Policy: a thread may hold a class this enum does not name.")
    .def_ro("priority", &flux::rt::ThreadState::priority)
    .def_ro("cpus", &flux::rt::ThreadState::cpus, "The cores this thread may run on.")
    .def("__repr__", [](const flux::rt::ThreadState & st) {
      std::string cpus;
      for (auto c : st.cpus) cpus += (cpus.empty() ? "" : ",") + std::to_string(c);
      return "<flux.rt.ThreadState policy=" + std::to_string(st.policy) +
             " priority=" + std::to_string(st.priority) + " cpus=[" + cpus + "]>";
    });

  rt.def(
    "apply",
    [](flux::rt::Policy policy, int priority, std::vector<std::uint32_t> cpus) {
      flux::rt::Options opts{policy, priority, std::move(cpus)};
      return flux::rt::apply_checked(opts, flux::rt::Strictness::Soft).to_string();
    },
    nb::arg("policy") = flux::rt::Policy::Inherit, nb::arg("priority") = 0,
    nb::arg("cpus") = std::vector<std::uint32_t>{},
    "Put the CALLING thread on this policy, priority and cpu set, and confirm by reading back "
    "what the kernel gave. A thread can only do this to itself, which is why an executor's "
    "children apply their own.\n\n"
    "Raises rather than settling for less: OSError when the kernel refuses, RuntimeError when a "
    "check finds the request cannot take effect on this host or when the read-back disagrees "
    "with what was asked. Asking for nothing does nothing. Returns a human-readable report of "
    "what the host was found to be -- diagnostic text, not a format to parse; empty when nothing "
    "was asked.");

  rt.def(
    "current", &flux::rt::current,
    "What the calling thread runs at right now, read from the kernel.");

  rt.def(
    "this_tid", &flux::rt::this_tid,
    "The calling thread's kernel thread id, which is what `top -H` and /proc name it by. "
    "threading.get_ident() is not this number.");

  m.def(
    "process_domain", &flux::process_domain,
    "This process's domain, resolved once on first use and fixed for the life of the process. "
    "Every name a Publisher or Subscription builds carries it, and rcl latches ROS_DOMAIN_ID the "
    "same way, so the flux half and the ROS half of one process cannot end up in different "
    "domains. Tooling calls this to agree with the nodes rather than deriving the answer "
    "twice.");

  m.def(
    "flatten_key", &flux::flatten_key, nb::arg("key"),
    "The key as a shm name spells it: every non-alnum character becomes '.', so '/a/b' and "
    "'.a.b' flatten alike. A channel with no live participant reports this spelling rather than "
    "its real key (Topic.key_exact is False), so a tool resolving a user-typed name compares "
    "through this instead of restating the rule.");

  m.def(
    "read_channel_stats", &flux::read_channel_stats, nb::arg("signpost"),
    "Read a channel's current segment without attaching to it. Takes no lock and no borrow, so "
    "it perturbs nothing and no publisher or subscriber can tell it happened. Every field is "
    "zero and `live` is False when no publisher has a segment up.");

  m.def(
    "enumerate_topics", &flux::enumerate_topics,
    "Every flux channel visible in /dev/shm right now, with the endpoints of every process "
    "still holding its owner lock. A snapshot taken by reading files: there is no daemon and "
    "no registry, and nothing here is a subscription. Reports; never unlinks. Costs one open "
    "per name, so it belongs in tooling rather than a loop.");
}
