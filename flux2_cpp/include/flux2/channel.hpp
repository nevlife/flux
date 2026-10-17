#ifndef FLUX2_CHANNEL_HPP
#define FLUX2_CHANNEL_HPP

#include "flux2.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>

namespace flux2
{

inline constexpr std::uint64_t kNoSchema = FLUX2_NO_SCHEMA;
inline constexpr std::size_t kMaxDims = FLUX2_MAX_DIMS;

using Status = flux2_status;
using FrameMeta = flux2_frame_meta;
using MemoryPolicy = flux2_memory_policy;
using Refused = flux2_refused;

class Error : public std::runtime_error
{
public:
  Error(Status status, const char * what) : std::runtime_error(what), status_(status) {}
  Status status() const noexcept { return status_; }

private:
  Status status_;
};

namespace detail
{
class Channel;

inline void check(Status st)
{
  if (st != FLUX2_OK) throw Error(st, flux2_last_error());
}

// The library loaded at run time must be the one these headers describe: another lays out views,
// loans and sources differently and writes past them. Asked of the library once per process.
inline void require_abi()
{
  static const std::uint32_t lib = flux2_abi_version();
  if (lib == FLUX2_ABI_VERSION) return;
  throw Error(
    FLUX2_MISMATCH, ("flux2: the libflux2_ffi loaded has ABI version " + std::to_string(lib) +
                     ", and the flux2 headers this was built with need " +
                     std::to_string(FLUX2_ABI_VERSION) + "; rebuild against the installed flux2")
                      .c_str());
}

// The C storage holds a Rust value, which moves by copying its bytes; zeroed storage is empty.
// It is never copied by value: this is the only move.
template <class C>
void relocate(C & to, C & from) noexcept
{
  std::memcpy(&to, &from, sizeof(C));
  std::memset(&from, 0, sizeof(C));
}
}  // namespace detail

// Where a channel's frames are produced or consumed. Cuda declares a stream flux2 creates.
enum class Device : std::uint8_t {
  Cpu = 0,
  Cuda,
};

namespace gpu
{

enum class Route : std::int32_t {
  None = FLUX2_ROUTE_NONE,
  ShmDirect = FLUX2_ROUTE_SHM_DIRECT,
  ShmRegistered = FLUX2_ROUTE_SHM_REGISTERED,
  DeviceHandle = FLUX2_ROUTE_DEVICE_HANDLE,
};

struct Platform
{
  Route route = Route::None;
  int device = -1;
  bool integrated = false;
  bool pageable_access = false;
  bool host_register = false;
  bool vmm = false;
  bool posix_fd_handle = false;
  // Non-empty exactly when route is None.
  std::string reason;
};

// Initializes CUDA on first use. Never throws: a host without a GPU is Route::None with a reason.
inline Platform probe(int device = 0)
{
  flux2_gpu_platform p{};
  const flux2_status st = flux2_gpu_probe(device, &p);
  return {
    static_cast<Route>(p.route),
    p.device,
    p.integrated,
    p.pageable_access,
    p.host_register,
    p.vmm,
    p.posix_fd_handle,
    st == FLUX2_OK ? std::string() : std::string(flux2_last_error())};
}

// The sticky CUresult that wedged this process's GPU, 0 while none has. After one, every GPU
// channel refuses at once and only a restart helps.
inline int wedged() noexcept
{
  return flux2_gpu_wedged();
}

// A CUDA stream declared on a channel. Commit and release wait for it, which is per stream: keep
// it for frame work, or the fence waits for the application's other work too. Copies share it.
class Stream
{
public:
  // None declared: a host channel.
  Stream() = default;

  // A stream flux2 owns, on the current context or the current device's primary one.
  static Stream create()
  {
    flux2_stream * s = nullptr;
    detail::check(flux2_stream_create(&s));
    return Stream(s);
  }

  // The caller's cudaStream_t, which must outlive every channel and view declared on it. The
  // legacy default stream (nullptr) is refused: it is a different stream on each thread.
  static Stream from_native(void * native)
  {
    flux2_stream * s = nullptr;
    detail::check(flux2_stream_from_native(native, &s));
    return Stream(s);
  }

  bool declared() const noexcept { return s_ != nullptr; }
  void * native() const noexcept { return s_ ? flux2_stream_native(s_.get()) : nullptr; }
  // The ordinal of the stream's device, -1 when none is declared.
  int device() const noexcept { return s_ ? flux2_stream_device(s_.get()) : -1; }
  bool wait() const noexcept { return !s_ || flux2_stream_wait(s_.get()); }
  const flux2_stream * raw() const noexcept { return s_.get(); }

private:
  explicit Stream(flux2_stream * s) : s_(s, flux2_stream_free) {}

  std::shared_ptr<flux2_stream> s_;
};

// Device::Cpu declares nothing; Device::Cuda creates a stream.
inline Stream stream_for(Device device)
{
  return device == Device::Cpu ? Stream() : Stream::create();
}

using FenceWait = flux2_fence_wait;

// Wait: a view's release waits for the stream. Deferred: it returns at once, and the slot goes back
// once the device is done, at the next take or executor pass; until then it counts in max_borrow.
enum class ReleaseMode : std::int32_t {
  Wait = FLUX2_RELEASE_WAIT,
  Deferred = FLUX2_RELEASE_DEFERRED
};

}  // namespace gpu

enum class DType : std::uint8_t {
  U8 = FLUX2_DTYPE_U8,
  I8 = FLUX2_DTYPE_I8,
  U16 = FLUX2_DTYPE_U16,
  I16 = FLUX2_DTYPE_I16,
  U32 = FLUX2_DTYPE_U32,
  I32 = FLUX2_DTYPE_I32,
  U64 = FLUX2_DTYPE_U64,
  I64 = FLUX2_DTYPE_I64,
  F16 = FLUX2_DTYPE_F16,
  F32 = FLUX2_DTYPE_F32,
  F64 = FLUX2_DTYPE_F64,
  BF16 = FLUX2_DTYPE_BF16,
};

enum class [[nodiscard]] Published : std::int32_t {
  Ok = FLUX2_PUBLISHED_OK,
  Backpressure = FLUX2_PUBLISHED_BACKPRESSURE,
  TooLarge = FLUX2_PUBLISHED_TOO_LARGE,
  NotPublisher = FLUX2_PUBLISHED_NOT_PUBLISHER,
  InvalidArgument = FLUX2_PUBLISHED_INVALID_ARGUMENT,
  // Another thread is in a call on the same publisher.
  InUse = FLUX2_PUBLISHED_IN_USE,
  // The payload is device memory: loan, and write through device_ptr().
  WrongDevice = FLUX2_PUBLISHED_WRONG_DEVICE,
  // The declared stream could not be waited for; the slot stays claimed for good.
  FenceFailed = FLUX2_PUBLISHED_FENCE_FAILED,
  // A sticky CUDA error wedged this process's GPU (gpu::wedged()); restart the process.
  GpuWedged = FLUX2_PUBLISHED_GPU_WEDGED,
};

inline std::size_t dtype_size(DType d) noexcept
{
  switch (d) {
    case DType::U8:
    case DType::I8:
      return 1;
    case DType::U16:
    case DType::I16:
    case DType::F16:
    case DType::BF16:
      return 2;
    case DType::U32:
    case DType::I32:
    case DType::F32:
      return 4;
    case DType::U64:
    case DType::I64:
    case DType::F64:
      return 8;
  }
  return 0;
}

// Backpressure is a full ring, which the next publish may not hit. The rest repeat until fixed.
inline bool faulted(Published p) noexcept
{
  return p != Published::Ok && p != Published::Backpressure;
}

inline const char * to_string(Published p) noexcept
{
  switch (p) {
    case Published::Ok:
      return "ok";
    case Published::Backpressure:
      return "backpressure";
    case Published::TooLarge:
      return "too large";
    case Published::NotPublisher:
      return "not a publisher";
    case Published::InvalidArgument:
      return "invalid argument";
    case Published::InUse:
      return "in use by another thread";
    case Published::WrongDevice:
      return "payload is device memory";
    case Published::FenceFailed:
      return "stream wait failed";
    case Published::GpuWedged:
      return "GPU wedged by a sticky CUDA error";
  }
  return "unknown";
}

// Checked when a channel takes it, not here.
class QoS
{
public:
  QoS() : q_(flux2_qos_default()) {}
  QoS(std::uint32_t depth) : QoS() { q_.depth = depth; }  // NOLINT(runtime/explicit): QoS q = 4;

  QoS & transient_local(std::uint32_t replay)
  {
    q_.replay = replay;
    return *this;
  }
  QoS & max_borrow(std::uint32_t n)
  {
    q_.max_borrow = n;
    return *this;
  }

  std::uint32_t depth() const noexcept { return q_.depth; }
  std::uint32_t replay() const noexcept { return q_.replay; }
  std::uint32_t borrow_limit() const noexcept { return q_.max_borrow; }

private:
  friend class detail::Channel;
  explicit QoS(flux2_qos q) : q_(q) {}
  flux2_qos q_;
};

class FrameView
{
public:
  FrameView() = default;
  ~FrameView() { release(); }
  FrameView(FrameView && o) noexcept { detail::relocate(v_, o.v_); }
  FrameView & operator=(FrameView && o) noexcept
  {
    if (this != &o) {
      release();
      detail::relocate(v_, o.v_);
    }
    return *this;
  }
  FrameView(const FrameView &) = delete;
  FrameView & operator=(const FrameView &) = delete;

  bool valid() const noexcept { return flux2_view_valid(raw()); }
  explicit operator bool() const noexcept { return valid(); }

  // Null when the payload is device memory, while size() is still the frame's: use device_ptr().
  const void * data() const noexcept
  {
    std::size_t n = 0;
    return flux2_view_data(raw(), &n);
  }
  std::size_t size() const noexcept
  {
    const FrameMeta * m = flux2_view_meta(raw());
    return m != nullptr ? static_cast<std::size_t>(m->nbytes) : 0;
  }
  // Null on a host channel; read-only memory on a subscriber's mapping.
  const void * device_ptr() const noexcept
  {
    return reinterpret_cast<const void *>(  // NOLINT(*-reinterpret-cast,performance-no-int-to-ptr)
      static_cast<std::uintptr_t>(flux2_view_device_ptr(raw())));
  }
  // The channel's cudaStream_t: queue the frame's work on it, since the release waits for it.
  void * stream() const noexcept { return flux2_view_stream(raw()); }
  // Orders the channel stream's next work after what s has queued, on the device: a deferred
  // release then covers work that read the frame on s.
  bool after(const gpu::Stream & s) const noexcept { return flux2_view_after(raw(), s.raw()); }
  // Requires valid().
  const FrameMeta & meta() const noexcept { return *flux2_view_meta(raw()); }
  std::uint64_t ticket() const noexcept { return flux2_view_ticket(raw()); }
  // The channel as trace events name it (flux2/trace.h).
  const void * trace_channel() const noexcept { return flux2_view_trace_channel(raw()); }
  // CLOCK_MONOTONIC at commit.
  std::uint64_t publish_ns() const noexcept { return flux2_view_publish_ns(raw()); }

  void release() noexcept { flux2_view_release(&v_); }

private:
  friend class detail::Channel;
  const flux2_view * raw() const noexcept { return &v_; }
  flux2_view v_{};
};

// Commit it while no other thread is in a call on its publisher: unlike those calls, a commit is
// not refused. Dropped without a commit, nothing is published, but the frame the slot held is gone:
// the loan wrote over it.
class WriteSlot
{
public:
  WriteSlot() = default;
  ~WriteSlot() { abort(); }
  WriteSlot(WriteSlot && o) noexcept { detail::relocate(l_, o.l_); }
  WriteSlot & operator=(WriteSlot && o) noexcept
  {
    if (this != &o) {
      abort();
      detail::relocate(l_, o.l_);
    }
    return *this;
  }
  WriteSlot(const WriteSlot &) = delete;
  WriteSlot & operator=(const WriteSlot &) = delete;

  bool valid() const noexcept { return flux2_loan_valid(raw()); }
  explicit operator bool() const noexcept { return valid(); }

  // Null when the payload is device memory: use device_ptr().
  void * data() noexcept
  {
    std::size_t n = 0;
    return flux2_loan_data(&l_, &n);
  }
  // Null on a host channel.
  void * device_ptr() const noexcept
  {
    return reinterpret_cast<void *>(  // NOLINT(*-reinterpret-cast,performance-no-int-to-ptr)
      static_cast<std::uintptr_t>(flux2_loan_device_ptr(raw())));
  }
  // The channel's cudaStream_t: queue the producing work on it, since the commit waits for it.
  void * stream() const noexcept { return flux2_loan_stream(raw()); }
  std::size_t capacity() noexcept
  {
    std::size_t n = 0;
    flux2_loan_data(&l_, &n);
    return n;
  }

  Published commit() noexcept { return static_cast<Published>(flux2_loan_commit(&l_)); }
  // Shrinks a one-dimensional loan to nbytes.
  Published commit(std::size_t nbytes) noexcept
  {
    return static_cast<Published>(flux2_loan_commit_len(&l_, nbytes));
  }
  void abort() noexcept { flux2_loan_abort(&l_); }

private:
  friend class detail::Channel;
  const flux2_loan * raw() const noexcept { return &l_; }
  flux2_loan l_{};
};

namespace detail
{

// A subscription's hold on a signpost while it has no publisher: the publisher that comes up wakes
// it. Creates the signpost if absent.
class Watch
{
public:
  explicit Watch(const std::string & signpost)
  {
    require_abi();
    detail::check(flux2_watch_open(signpost.c_str(), &w_));
  }
  ~Watch() { flux2_watch_free(w_); }
  Watch(const Watch &) = delete;
  Watch & operator=(const Watch &) = delete;
  Watch(Watch &&) = delete;
  Watch & operator=(Watch &&) = delete;

  // 0 until the first publisher; it changes with every restart.
  std::uint32_t epoch() const noexcept { return flux2_watch_epoch(w_); }
  // Notes the epoch now, for wait() and an executor to wait until it moves. Returns it.
  std::uint32_t arm() const noexcept { return flux2_watch_arm(w_); }
  // False only on a timeout. A negative timeout waits forever.
  bool wait(std::int64_t timeout_ns) const noexcept { return flux2_watch_wait(w_, timeout_ns); }
  const flux2_watch * raw() const noexcept { return w_; }

private:
  flux2_watch * w_ = nullptr;
};

// One call at a time: a publish, loan, take or qos() that meets another thread's call is refused
// with Published::InUse, an empty result or Error. A moved-from channel is empty: publish gives
// InvalidArgument, loan and take an empty result, the getters 0.
class Channel
{
public:
  // In-process, for tests and single-process pipelines.
  Channel(std::uint32_t slot_size, std::uint32_t slot_count)
  {
    require_abi();
    detail::check(flux2_channel_new(slot_size, slot_count, &ch_));
  }

  // Error with FLUX2_PUBLISHER_EXISTS while another publisher of `name` is live.
  // A declared `stream` makes a GPU channel for its life.
  static Channel create(
    const std::string & name, std::uint32_t slot_size, std::uint32_t slot_count,
    std::uint64_t fingerprint = kNoSchema, MemoryPolicy mem = {},
    std::uint32_t max_subscribers = FLUX2_DEFAULT_MAX_SUBSCRIBERS, const gpu::Stream & stream = {})
  {
    require_abi();
    flux2_channel * ch = nullptr;
    detail::check(flux2_channel_create(
      name.c_str(), slot_size, slot_count, max_subscribers, fingerprint, mem, stream.raw(),
      flux2_stamp_offset(fingerprint), &ch));
    return Channel(ch);
  }

  // Error with FLUX2_NOT_READY until a publisher is up.
  static Channel open(
    const std::string & name, std::uint64_t fingerprint = kNoSchema, MemoryPolicy mem = {},
    const gpu::Stream & stream = {})
  {
    require_abi();
    flux2_channel * ch = nullptr;
    detail::check(flux2_channel_open(
      name.c_str(), fingerprint, mem, stream.raw(), flux2_stamp_offset(fingerprint), &ch));
    return Channel(ch);
  }

  // Through a watch, which arms it first. Once the watch has attached, a channel starts at its
  // publisher's oldest frame: they all came after this subscriber joined.
  static Channel open(
    const Watch & watch, std::uint64_t fingerprint, const QoS & qos, MemoryPolicy mem = {},
    const gpu::Stream & stream = {})
  {
    flux2_channel * ch = nullptr;
    detail::check(flux2_channel_open_watched(
      watch.raw(), fingerprint, qos.q_, mem, stream.raw(), flux2_stamp_offset(fingerprint), &ch));
    return Channel(ch);
  }

  ~Channel() { flux2_channel_free(ch_); }
  Channel(Channel && o) noexcept : ch_(o.ch_) { o.ch_ = nullptr; }
  Channel & operator=(Channel && o) noexcept
  {
    if (this != &o) {
      flux2_channel_free(ch_);
      ch_ = o.ch_;
      o.ch_ = nullptr;
    }
    return *this;
  }
  Channel(const Channel &) = delete;
  Channel & operator=(const Channel &) = delete;

  Published publish(const void * data, std::size_t n) noexcept
  {
    if (!ch_) return Published::InvalidArgument;
    return static_cast<Published>(
      flux2_channel_publish(ch_, static_cast<const std::uint8_t *>(data), n));
  }
  Published publish(
    const void * data, DType dtype, const std::uint64_t * shape, std::size_t ndim) noexcept
  {
    if (!ch_) return Published::InvalidArgument;
    std::size_t n = dtype_size(dtype);
    for (std::size_t i = 0; i < ndim; ++i) n *= shape[i];
    return static_cast<Published>(flux2_channel_publish_typed(
      ch_, static_cast<const std::uint8_t *>(data), n, static_cast<std::uint8_t>(dtype), shape,
      ndim));
  }
  Published publish(
    const void * data, DType dtype, std::initializer_list<std::uint64_t> shape) noexcept
  {
    return publish(data, dtype, shape.begin(), shape.size());
  }

  // Empty when every slot is borrowed or this is a subscriber.
  WriteSlot loan() noexcept
  {
    WriteSlot w;
    if (ch_) flux2_channel_loan(ch_, &w.l_);
    return w;
  }
  // The shape is checked at commit.
  WriteSlot loan(DType dtype, const std::uint64_t * shape, std::size_t ndim) noexcept
  {
    WriteSlot w;
    if (ch_) flux2_channel_loan_typed(ch_, static_cast<std::uint8_t>(dtype), shape, ndim, &w.l_);
    return w;
  }
  WriteSlot loan(DType dtype, std::initializer_list<std::uint64_t> shape) noexcept
  {
    return loan(dtype, shape.begin(), shape.size());
  }

  // The oldest frame newer than the cursor.
  FrameView take() noexcept
  {
    FrameView v;
    if (ch_) flux2_channel_take(ch_, &v.v_);
    return v;
  }
  // A negative timeout waits forever.
  FrameView take_blocking(std::int64_t timeout_ns) noexcept
  {
    FrameView v;
    if (ch_) flux2_channel_take_blocking(ch_, timeout_ns, &v.v_);
    return v;
  }

  void qos(const QoS & q)
  {
    if (!ch_) throw Error(FLUX2_INVALID_ARGUMENT, "flux2: this channel was moved from");
    detail::check(flux2_channel_set_qos(ch_, q.q_));
  }
  QoS qos() const noexcept { return ch_ ? QoS(flux2_channel_qos(ch_)) : QoS(); }

  Refused refused() const noexcept { return ch_ ? flux2_channel_refused(ch_) : Refused{}; }
  std::uint64_t fence_failed() const noexcept { return ch_ ? flux2_channel_fence_failed(ch_) : 0; }
  gpu::FenceWait fence_wait() const noexcept
  {
    return ch_ ? flux2_channel_fence_wait(ch_) : gpu::FenceWait{};
  }
  // Bounds each commit and release fence; zero, the default, for none. A wait that runs out counts
  // in fence_failed() and keeps its slot until the work is done.
  void set_fence_timeout(std::chrono::nanoseconds t) const noexcept
  {
    if (ch_)
      flux2_channel_set_fence_timeout(
        ch_, t.count() > 0 ? static_cast<std::uint64_t>(t.count()) : 0);
  }
  void set_release_mode(gpu::ReleaseMode m) const noexcept
  {
    if (ch_) flux2_channel_set_release_mode(ch_, static_cast<std::int32_t>(m));
  }
  bool device_backed() const noexcept { return ch_ && flux2_channel_device_backed(ch_); }
  bool ready() const noexcept { return ch_ && flux2_channel_ready(ch_); }
  bool can_borrow() const noexcept { return ch_ && flux2_channel_can_borrow(ch_); }
  std::uint64_t lost() const noexcept { return ch_ ? flux2_channel_lost(ch_) : 0; }
  std::uint64_t dropped() const noexcept { return ch_ ? flux2_channel_dropped(ch_) : 0; }
  std::uint32_t attach_generation() const noexcept
  {
    return ch_ ? flux2_channel_attach_generation(ch_) : 0;
  }
  bool orphaned() const noexcept { return ch_ && flux2_channel_orphaned(ch_); }
  // Throws why a restarted publisher refused this subscriber; takes find nothing until it restarts.
  void check_attach()
  {
    if (ch_) detail::check(flux2_channel_attach_status(ch_));
  }
  bool inherited() const noexcept { return ch_ && flux2_channel_inherited(ch_); }
  std::uint32_t slot_size() const noexcept { return ch_ ? flux2_channel_slot_size(ch_) : 0; }
  std::uint32_t slot_count() const noexcept { return ch_ ? flux2_channel_slot_count(ch_) : 0; }
  bool payload_readonly() const noexcept { return ch_ && flux2_channel_payload_readonly(ch_); }
  bool pages_committed() const noexcept { return ch_ && flux2_channel_pages_committed(ch_); }
  bool pages_locked() const noexcept { return ch_ && flux2_channel_pages_locked(ch_); }

  const flux2_channel * raw() const noexcept { return ch_; }

private:
  explicit Channel(flux2_channel * ch) noexcept : ch_(ch) {}
  flux2_channel * ch_ = nullptr;
};

inline std::string signpost_name(const std::string & key, std::uint64_t fingerprint = kNoSchema)
{
  require_abi();
  std::string out(256, '\0');
  std::size_t len = 0;
  detail::check(flux2_signpost_name(key.c_str(), fingerprint, out.data(), out.size(), &len));
  out.resize(len);
  return out;
}

}  // namespace detail

}  // namespace flux2

#endif  // FLUX2_CHANNEL_HPP
