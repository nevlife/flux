#ifndef FLUX_CHANNEL_HPP
#define FLUX_CHANNEL_HPP

#include "flux.h"
#include "flux/wire.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>

namespace flux
{

inline constexpr std::uint64_t kNoSchema = FLUX_NO_SCHEMA;
inline constexpr std::size_t kMaxDims = FLUX_MAX_DIMS;

using Status = flux_status;
using FrameMeta = flux_frame_meta;
using MemoryPolicy = flux_memory_policy;
using Refused = flux_refused;

class Error : public std::runtime_error
{
public:
  Error(Status status, const char * what) : std::runtime_error(what), status_(status) {}
  [[nodiscard]] auto status() const noexcept -> Status { return status_; }

private:
  Status status_;
};

namespace detail
{
class Channel;

inline void check(Status st)
{
  if (st != FLUX_OK) {
    throw Error(st, flux_last_error());
  }
}

// The library loaded at run time must be the one these headers describe: another lays out views,
// loans and sources differently and writes past them. Asked of the library once per process.
inline void require_abi()
{
  static const std::uint32_t lib = flux_abi_version();
  if (lib == FLUX_ABI_VERSION) {
    return;
  }
  throw Error(
    FLUX_MISMATCH, ("flux: the libflux_ffi loaded has ABI version " + std::to_string(lib) +
                    ", and the flux headers this was built with need " +
                    std::to_string(FLUX_ABI_VERSION) + "; rebuild against the installed flux")
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

// Where a channel's frames are produced or consumed. Cuda declares a stream flux creates.
enum class Device : std::uint8_t {
  Cpu = 0,
  Cuda,
};

namespace gpu
{

enum class Route : std::int32_t {
  None = FLUX_ROUTE_NONE,
  ShmDirect = FLUX_ROUTE_SHM_DIRECT,
  ShmRegistered = FLUX_ROUTE_SHM_REGISTERED,
  DeviceHandle = FLUX_ROUTE_DEVICE_HANDLE,
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
inline auto probe(int device = 0) -> Platform
{
  flux_gpu_platform p{};
  const flux_status st = flux_gpu_probe(device, &p);
  return {
    static_cast<Route>(p.route),
    p.device,
    p.integrated,
    p.pageable_access,
    p.host_register,
    p.vmm,
    p.posix_fd_handle,
    st == FLUX_OK ? std::string() : std::string(flux_last_error())};
}

// The sticky CUresult that wedged this process's GPU, 0 while none has. After one, every GPU
// channel refuses at once and only a restart helps.
inline auto wedged() noexcept -> int
{
  return flux_gpu_wedged();
}

// A CUDA stream declared on a channel. Commit and release wait for it, which is per stream: keep
// it for frame work, or the fence waits for the application's other work too. Copies share it.
class Stream
{
public:
  // None declared: a host channel.
  Stream() = default;

  // A stream flux owns, on the current context or the current device's primary one.
  static auto create() -> Stream
  {
    flux_stream * s = nullptr;
    detail::check(flux_stream_create(&s));
    return Stream(s);
  }

  // The caller's cudaStream_t, which must outlive every channel and view declared on it. The
  // legacy default stream (nullptr) is refused: it is a different stream on each thread.
  static auto from_native(void * native) -> Stream
  {
    flux_stream * s = nullptr;
    detail::check(flux_stream_from_native(native, &s));
    return Stream(s);
  }

  [[nodiscard]] auto declared() const noexcept -> bool { return s_ != nullptr; }
  [[nodiscard]] auto native() const noexcept -> void *
  {
    return s_ ? flux_stream_native(s_.get()) : nullptr;
  }
  // The ordinal of the stream's device, -1 when none is declared.
  [[nodiscard]] auto device() const noexcept -> int
  {
    return s_ ? flux_stream_device(s_.get()) : -1;
  }
  [[nodiscard]] auto wait() const noexcept -> bool { return !s_ || flux_stream_wait(s_.get()); }
  [[nodiscard]] auto raw() const noexcept -> const flux_stream * { return s_.get(); }

private:
  explicit Stream(flux_stream * s) : s_(s, flux_stream_free) {}

  std::shared_ptr<flux_stream> s_;
};

// Device::Cpu declares nothing; Device::Cuda creates a stream.
inline auto stream_for(Device device) -> Stream
{
  return device == Device::Cpu ? Stream() : Stream::create();
}

using FenceWait = flux_fence_wait;

// Wait: a view's release waits for the stream. Deferred: it returns at once, and the slot goes back
// once the device is done, at the next take or executor pass; until then it counts in max_borrow.
enum class ReleaseMode : std::int32_t {
  Wait = FLUX_RELEASE_WAIT,
  Deferred = FLUX_RELEASE_DEFERRED
};

}  // namespace gpu

enum class DType : std::uint8_t {
  U8 = FLUX_DTYPE_U8,
  I8 = FLUX_DTYPE_I8,
  U16 = FLUX_DTYPE_U16,
  I16 = FLUX_DTYPE_I16,
  U32 = FLUX_DTYPE_U32,
  I32 = FLUX_DTYPE_I32,
  U64 = FLUX_DTYPE_U64,
  I64 = FLUX_DTYPE_I64,
  F16 = FLUX_DTYPE_F16,
  F32 = FLUX_DTYPE_F32,
  F64 = FLUX_DTYPE_F64,
  BF16 = FLUX_DTYPE_BF16,
};

enum class [[nodiscard]] Published : std::int32_t {
  Ok = FLUX_PUBLISHED_OK,
  Backpressure = FLUX_PUBLISHED_BACKPRESSURE,
  TooLarge = FLUX_PUBLISHED_TOO_LARGE,
  NotPublisher = FLUX_PUBLISHED_NOT_PUBLISHER,
  InvalidArgument = FLUX_PUBLISHED_INVALID_ARGUMENT,
  // Another thread is in a call on the same publisher.
  InUse = FLUX_PUBLISHED_IN_USE,
  // The payload is device memory: loan, and write through device_ptr().
  WrongDevice = FLUX_PUBLISHED_WRONG_DEVICE,
  // The declared stream could not be waited for; the slot stays claimed for good.
  FenceFailed = FLUX_PUBLISHED_FENCE_FAILED,
  // A sticky CUDA error wedged this process's GPU (gpu::wedged()); restart the process.
  GpuWedged = FLUX_PUBLISHED_GPU_WEDGED,
};

inline auto dtype_size(DType d) noexcept -> std::size_t
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
inline auto faulted(Published p) noexcept -> bool
{
  return p != Published::Ok && p != Published::Backpressure;
}

inline auto to_string(Published p) noexcept -> const char *
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
  QoS() : q_(flux_qos_default()) {}
  QoS(std::uint32_t depth) : QoS() { q_.depth = depth; }  // NOLINT(runtime/explicit): QoS q = 4;

  auto transient_local(std::uint32_t replay) -> QoS &
  {
    q_.replay = replay;
    return *this;
  }
  auto max_borrow(std::uint32_t n) -> QoS &
  {
    q_.max_borrow = n;
    return *this;
  }

  [[nodiscard]] auto depth() const noexcept -> std::uint32_t { return q_.depth; }
  [[nodiscard]] auto replay() const noexcept -> std::uint32_t { return q_.replay; }
  [[nodiscard]] auto borrow_limit() const noexcept -> std::uint32_t { return q_.max_borrow; }

private:
  friend class detail::Channel;
  explicit QoS(flux_qos q) : q_(q) {}
  flux_qos q_;
};

class FrameView
{
public:
  FrameView() = default;
  ~FrameView() { release(); }
  FrameView(FrameView && o) noexcept { detail::relocate(v_, o.v_); }
  auto operator=(FrameView && o) noexcept -> FrameView &
  {
    if (this != &o) {
      release();
      detail::relocate(v_, o.v_);
    }
    return *this;
  }
  FrameView(const FrameView &) = delete;
  auto operator=(const FrameView &) -> FrameView & = delete;

  [[nodiscard]] auto valid() const noexcept -> bool { return flux_view_valid(raw()); }
  explicit operator bool() const noexcept { return valid(); }

  // Null when the payload is device memory, while size() is still the frame's: use device_ptr().
  [[nodiscard]] auto data() const noexcept -> const void *
  {
    std::size_t n = 0;
    return flux_view_data(raw(), &n);
  }
  [[nodiscard]] auto size() const noexcept -> std::size_t
  {
    const FrameMeta * m = flux_view_meta(raw());
    return m != nullptr ? static_cast<std::size_t>(m->nbytes) : 0;
  }
  // Null on a host channel; read-only memory on a subscriber's mapping.
  [[nodiscard]] auto device_ptr() const noexcept -> const void *
  {
    return reinterpret_cast<const void *>(  // NOLINT(*-reinterpret-cast,performance-no-int-to-ptr)
      static_cast<std::uintptr_t>(flux_view_device_ptr(raw())));
  }
  // The channel's cudaStream_t: queue the frame's work on it, since the release waits for it.
  [[nodiscard]] auto stream() const noexcept -> void * { return flux_view_stream(raw()); }
  // Orders the channel stream's next work after what s has queued, on the device: a deferred
  // release then covers work that read the frame on s.
  [[nodiscard]] auto after(const gpu::Stream & s) const noexcept -> bool
  {
    return flux_view_after(raw(), s.raw());
  }
  // The frame's aux bytes, in host memory whatever the payload is.
  [[nodiscard]] auto aux() const noexcept -> wire::Span<const std::uint8_t>
  {
    std::size_t n = 0;
    const std::uint8_t * p = flux_view_aux(raw(), &n);
    return {p, n};
  }
  // Requires valid().
  [[nodiscard]] auto meta() const noexcept -> const FrameMeta & { return *flux_view_meta(raw()); }
  [[nodiscard]] auto ticket() const noexcept -> std::uint64_t { return flux_view_ticket(raw()); }
  // The channel as trace events name it (flux/trace.h).
  [[nodiscard]] auto trace_channel() const noexcept -> const void *
  {
    return flux_view_trace_channel(raw());
  }
  // CLOCK_MONOTONIC at commit.
  [[nodiscard]] auto publish_ns() const noexcept -> std::uint64_t
  {
    return flux_view_publish_ns(raw());
  }

  void release() noexcept { flux_view_release(&v_); }

private:
  friend class detail::Channel;
  [[nodiscard]] auto raw() const noexcept -> const flux_view * { return &v_; }
  flux_view v_{};
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
  auto operator=(WriteSlot && o) noexcept -> WriteSlot &
  {
    if (this != &o) {
      abort();
      detail::relocate(l_, o.l_);
    }
    return *this;
  }
  WriteSlot(const WriteSlot &) = delete;
  auto operator=(const WriteSlot &) -> WriteSlot & = delete;

  [[nodiscard]] auto valid() const noexcept -> bool { return flux_loan_valid(raw()); }
  explicit operator bool() const noexcept { return valid(); }

  // Null when the payload is device memory: use device_ptr().
  auto data() noexcept -> void *
  {
    std::size_t n = 0;
    return flux_loan_data(&l_, &n);
  }
  // Null on a host channel.
  [[nodiscard]] auto device_ptr() const noexcept -> void *
  {
    return reinterpret_cast<void *>(  // NOLINT(*-reinterpret-cast,performance-no-int-to-ptr)
      static_cast<std::uintptr_t>(flux_loan_device_ptr(raw())));
  }
  // The channel's cudaStream_t: queue the producing work on it, since the commit waits for it.
  [[nodiscard]] auto stream() const noexcept -> void * { return flux_loan_stream(raw()); }
  auto capacity() noexcept -> std::size_t
  {
    std::size_t n = 0;
    flux_loan_data(&l_, &n);
    return n;
  }
  // The slot's whole aux region, in host memory whatever the payload is. set_aux_len says how many
  // bytes the frame carries; past the region, the commit is TooLarge.
  auto aux() noexcept -> wire::Span<std::uint8_t>
  {
    std::size_t n = 0;
    std::uint8_t * p = flux_loan_aux(&l_, &n);
    return {p, n};
  }
  void set_aux_len(std::size_t n) noexcept { flux_loan_set_aux_len(&l_, n); }

  auto commit() noexcept -> Published { return static_cast<Published>(flux_loan_commit(&l_)); }
  // Shrinks a one-dimensional loan to nbytes.
  auto commit(std::size_t nbytes) noexcept -> Published
  {
    return static_cast<Published>(flux_loan_commit_len(&l_, nbytes));
  }
  void abort() noexcept { flux_loan_abort(&l_); }

private:
  friend class detail::Channel;
  [[nodiscard]] auto raw() const noexcept -> const flux_loan * { return &l_; }
  flux_loan l_{};
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
    detail::check(flux_watch_open(signpost.c_str(), &w_));
  }
  ~Watch() { flux_watch_free(w_); }
  Watch(const Watch &) = delete;
  auto operator=(const Watch &) -> Watch & = delete;
  Watch(Watch &&) = delete;
  auto operator=(Watch &&) -> Watch & = delete;

  // 0 until the first publisher; it changes with every restart.
  [[nodiscard]] auto epoch() const noexcept -> std::uint32_t { return flux_watch_epoch(w_); }
  // Notes the epoch now, for wait() and an executor to wait until it moves. Returns it.
  [[nodiscard]] auto arm() const noexcept -> std::uint32_t { return flux_watch_arm(w_); }
  // False only on a timeout. A negative timeout waits forever.
  [[nodiscard]] auto wait(std::int64_t timeout_ns) const noexcept -> bool
  {
    return flux_watch_wait(w_, timeout_ns);
  }
  [[nodiscard]] auto raw() const noexcept -> const flux_watch * { return w_; }

private:
  flux_watch * w_ = nullptr;
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
    detail::check(flux_channel_new(slot_size, slot_count, &ch_));
  }

  // Error with FLUX_PUBLISHER_EXISTS while another publisher of `name` is live.
  // A declared `stream` makes a GPU channel for its life. Each slot carries `aux_size` host bytes
  // beside its payload, up to FLUX_MAX_AUX_SIZE.
  static auto create(
    const std::string & name, std::uint32_t slot_size, std::uint32_t slot_count,
    std::uint64_t fingerprint = kNoSchema, MemoryPolicy mem = {},
    std::uint32_t max_subscribers = FLUX_DEFAULT_MAX_SUBSCRIBERS, const gpu::Stream & stream = {},
    std::uint32_t aux_size = 0) -> Channel
  {
    require_abi();
    flux_channel * ch = nullptr;
    detail::check(flux_channel_create(
      name.c_str(), slot_size, slot_count, max_subscribers, aux_size, fingerprint, mem,
      stream.raw(), flux_stamp_offset(fingerprint), &ch));
    return Channel(ch);
  }

  // Error with FLUX_NOT_READY until a publisher is up.
  static auto open(
    const std::string & name, std::uint64_t fingerprint = kNoSchema, MemoryPolicy mem = {},
    const gpu::Stream & stream = {}) -> Channel
  {
    require_abi();
    flux_channel * ch = nullptr;
    detail::check(flux_channel_open(
      name.c_str(), fingerprint, mem, stream.raw(), flux_stamp_offset(fingerprint), &ch));
    return Channel(ch);
  }

  // Through a watch, which arms it first. Once the watch has attached, a channel starts at its
  // publisher's oldest frame: they all came after this subscriber joined.
  static auto open(
    const Watch & watch, std::uint64_t fingerprint, const QoS & qos, MemoryPolicy mem = {},
    const gpu::Stream & stream = {}) -> Channel
  {
    flux_channel * ch = nullptr;
    detail::check(flux_channel_open_watched(
      watch.raw(), fingerprint, qos.q_, mem, stream.raw(), flux_stamp_offset(fingerprint), &ch));
    return Channel(ch);
  }

  ~Channel() { flux_channel_free(ch_); }
  Channel(Channel && o) noexcept : ch_(o.ch_) { o.ch_ = nullptr; }
  auto operator=(Channel && o) noexcept -> Channel &
  {
    if (this != &o) {
      flux_channel_free(ch_);
      ch_ = o.ch_;
      o.ch_ = nullptr;
    }
    return *this;
  }
  Channel(const Channel &) = delete;
  auto operator=(const Channel &) -> Channel & = delete;

  auto publish(const void * data, std::size_t n) noexcept -> Published
  {
    if (!ch_) {
      return Published::InvalidArgument;
    }
    return static_cast<Published>(
      flux_channel_publish(ch_, static_cast<const std::uint8_t *>(data), n));
  }
  auto publish(
    const void * data, DType dtype, const std::uint64_t * shape,
    std::size_t ndim) noexcept -> Published
  {
    if (!ch_) {
      return Published::InvalidArgument;
    }
    std::size_t n = dtype_size(dtype);
    for (std::size_t i = 0; i < ndim; ++i) {
      n *= shape[i];
    }
    return static_cast<Published>(flux_channel_publish_typed(
      ch_, static_cast<const std::uint8_t *>(data), n, static_cast<std::uint8_t>(dtype), shape,
      ndim));
  }
  auto publish(const void * data, DType dtype, std::initializer_list<std::uint64_t> shape) noexcept
    -> Published
  {
    return publish(data, dtype, shape.begin(), shape.size());
  }

  // Empty when every slot is borrowed or this is a subscriber.
  auto loan() noexcept -> WriteSlot
  {
    WriteSlot w;
    if (ch_) {
      flux_channel_loan(ch_, &w.l_);
    }
    return w;
  }
  // The shape is checked at commit.
  auto loan(DType dtype, const std::uint64_t * shape, std::size_t ndim) noexcept -> WriteSlot
  {
    WriteSlot w;
    if (ch_) {
      flux_channel_loan_typed(ch_, static_cast<std::uint8_t>(dtype), shape, ndim, &w.l_);
    }
    return w;
  }
  auto loan(DType dtype, std::initializer_list<std::uint64_t> shape) noexcept -> WriteSlot
  {
    return loan(dtype, shape.begin(), shape.size());
  }

  // The oldest frame newer than the cursor.
  auto take() noexcept -> FrameView
  {
    FrameView v;
    if (ch_) {
      flux_channel_take(ch_, &v.v_);
    }
    return v;
  }
  // A negative timeout waits forever.
  auto take_blocking(std::int64_t timeout_ns) noexcept -> FrameView
  {
    FrameView v;
    if (ch_) {
      flux_channel_take_blocking(ch_, timeout_ns, &v.v_);
    }
    return v;
  }

  void qos(const QoS & q)
  {
    if (!ch_) {
      throw Error(FLUX_INVALID_ARGUMENT, "flux: this channel was moved from");
    }
    detail::check(flux_channel_set_qos(ch_, q.q_));
  }
  [[nodiscard]] auto qos() const noexcept -> QoS
  {
    return ch_ ? QoS(flux_channel_qos(ch_)) : QoS();
  }

  [[nodiscard]] auto refused() const noexcept -> Refused
  {
    return ch_ ? flux_channel_refused(ch_) : Refused{};
  }
  [[nodiscard]] auto fence_failed() const noexcept -> std::uint64_t
  {
    return ch_ ? flux_channel_fence_failed(ch_) : 0;
  }
  [[nodiscard]] auto fence_wait() const noexcept -> gpu::FenceWait
  {
    return ch_ ? flux_channel_fence_wait(ch_) : gpu::FenceWait{};
  }
  // Bounds each commit and release fence; zero, the default, for none. A wait that runs out counts
  // in fence_failed() and keeps its slot until the work is done.
  void set_fence_timeout(std::chrono::nanoseconds t) const noexcept
  {
    if (ch_) {
      flux_channel_set_fence_timeout(
        ch_, t.count() > 0 ? static_cast<std::uint64_t>(t.count()) : 0);
    }
  }
  void set_release_mode(gpu::ReleaseMode m) const noexcept
  {
    if (ch_) {
      flux_channel_set_release_mode(ch_, static_cast<std::int32_t>(m));
    }
  }
  [[nodiscard]] auto device_backed() const noexcept -> bool
  {
    return ch_ && flux_channel_device_backed(ch_);
  }
  [[nodiscard]] auto ready() const noexcept -> bool { return ch_ && flux_channel_ready(ch_); }
  [[nodiscard]] auto can_borrow() const noexcept -> bool
  {
    return ch_ && flux_channel_can_borrow(ch_);
  }
  [[nodiscard]] auto lost() const noexcept -> std::uint64_t
  {
    return ch_ ? flux_channel_lost(ch_) : 0;
  }
  [[nodiscard]] auto dropped() const noexcept -> std::uint64_t
  {
    return ch_ ? flux_channel_dropped(ch_) : 0;
  }
  [[nodiscard]] auto subscriber_count() const noexcept -> std::uint32_t
  {
    return ch_ ? flux_channel_subscriber_count(ch_) : 0;
  }
  [[nodiscard]] auto attach_generation() const noexcept -> std::uint32_t
  {
    return ch_ ? flux_channel_attach_generation(ch_) : 0;
  }
  [[nodiscard]] auto orphaned() const noexcept -> bool { return ch_ && flux_channel_orphaned(ch_); }
  // Throws why a restarted publisher refused this subscriber; takes find nothing until it restarts.
  void check_attach()
  {
    if (ch_) {
      detail::check(flux_channel_attach_status(ch_));
    }
  }
  [[nodiscard]] auto inherited() const noexcept -> bool
  {
    return ch_ && flux_channel_inherited(ch_);
  }
  [[nodiscard]] auto slot_size() const noexcept -> std::uint32_t
  {
    return ch_ ? flux_channel_slot_size(ch_) : 0;
  }
  [[nodiscard]] auto slot_count() const noexcept -> std::uint32_t
  {
    return ch_ ? flux_channel_slot_count(ch_) : 0;
  }
  [[nodiscard]] auto aux_size() const noexcept -> std::uint32_t
  {
    return ch_ ? flux_channel_aux_size(ch_) : 0;
  }
  [[nodiscard]] auto payload_readonly() const noexcept -> bool
  {
    return ch_ && flux_channel_payload_readonly(ch_);
  }
  [[nodiscard]] auto pages_committed() const noexcept -> bool
  {
    return ch_ && flux_channel_pages_committed(ch_);
  }
  [[nodiscard]] auto pages_locked() const noexcept -> bool
  {
    return ch_ && flux_channel_pages_locked(ch_);
  }

  [[nodiscard]] auto raw() const noexcept -> const flux_channel * { return ch_; }

private:
  explicit Channel(flux_channel * ch) noexcept : ch_(ch) {}
  flux_channel * ch_ = nullptr;
};

inline auto signpost_name(const std::string & key, std::uint64_t fingerprint = kNoSchema)
  -> std::string
{
  require_abi();
  std::string out(256, '\0');
  std::size_t len = 0;
  detail::check(flux_signpost_name(key.c_str(), fingerprint, out.data(), out.size(), &len));
  out.resize(len);
  return out;
}

}  // namespace detail

}  // namespace flux

#endif  // FLUX_CHANNEL_HPP
