#ifndef FLUX_WRAPPER_MESSAGES_HPP
#define FLUX_WRAPPER_MESSAGES_HPP

#include <builtin_interfaces/msg/time.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

#ifdef USE_FLUX_ENABLED
#include <sensor_msgs/flux/image.hpp>
#include <sensor_msgs/flux/image_ros.hpp>
#include <sensor_msgs/flux/point_cloud2.hpp>
#include <sensor_msgs/flux/point_cloud2_ros.hpp>
#endif

#ifdef FLUX_WRAPPER_CUDA
#include <cuda_runtime_api.h>
#endif

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace flux_wrapper
{

template <class MsgT>
class Publisher;

// Write side (Loan) and read side (View) of each carried type. Both have the fields of the ROS
// message, so a driver writes one piece of code for the ROS path and the flux path.
template <class MsgT>
class Loan;
template <class MsgT>
class View;

template <class T>
class Span
{
public:
  Span() = default;
  Span(T * data, std::size_t size) noexcept : data_(data), size_(size) {}
  [[nodiscard]] auto data() const noexcept -> T * { return data_; }
  [[nodiscard]] auto size() const noexcept -> std::size_t { return size_; }
  [[nodiscard]] auto empty() const noexcept -> bool { return size_ == 0; }
  [[nodiscard]] auto begin() const noexcept -> T * { return data_; }
  [[nodiscard]] auto end() const noexcept -> T * { return data_ + size_; }
  auto operator[](std::size_t i) const noexcept -> T & { return data_[i]; }

private:
  T * data_ = nullptr;
  std::size_t size_ = 0;
};

// Where a publisher's frames are produced or a subscription's are consumed. Cuda is used only for
// sensor_msgs/Image, and only where this build has CUDA and the host a CUDA device (on the flux
// path, a flux GPU route); otherwise Host, with a warning.
enum class Device { Host, Cuda };

namespace detail
{
class DeviceStaging;
}

// A Loan's data field: host memory, device memory, or both (an integrated GPU's slot). False when
// the requested size did not fit.
class Buffer
{
public:
  Buffer() = default;

  explicit operator bool() const noexcept { return ok_; }
  [[nodiscard]] auto on_device() const noexcept -> bool { return on_device_; }
  [[nodiscard]] auto size() const noexcept -> std::size_t { return size_; }
  // Empty when the buffer has no host mapping.
  [[nodiscard]] auto host() const noexcept -> Span<std::uint8_t>
  {
    return host_ != nullptr ? Span<std::uint8_t>(host_, size_) : Span<std::uint8_t>();
  }
  [[nodiscard]] auto device_ptr() const noexcept -> void * { return device_; }
#ifdef FLUX_WRAPPER_CUDA
  // Queue the producing work here: the publish waits for this stream.
  [[nodiscard]] auto stream() const noexcept -> cudaStream_t
  {
    return static_cast<cudaStream_t>(stream_);
  }
#endif

  // n bytes from host or device memory. A device buffer's copy is queued on stream(), so src must
  // stay unchanged until publish() returns.
  [[nodiscard]] auto copy_from(const void * src, std::size_t n) const noexcept -> bool
  {
    if (!ok_ || n > size_) {
      return false;
    }
    if (n == 0) {
      return true;
    }
#ifdef FLUX_WRAPPER_CUDA
    if (on_device_) {
      return cudaMemcpyAsync(device_, src, n, cudaMemcpyDefault, stream()) == cudaSuccess;
    }
    // A host loan of a publisher that asked for Cuda and did not get it: the caller's data may
    // still be on the device.
    if (query_src_) {
      cudaPointerAttributes a{};
      if (cudaPointerGetAttributes(&a, src) != cudaSuccess) {
        (void)cudaGetLastError();
      } else if (a.type == cudaMemoryTypeDevice) {
        return cudaMemcpy(host_, src, n, cudaMemcpyDeviceToHost) == cudaSuccess;
      }
    }
#endif
    std::memcpy(host_, src, n);
    return true;
  }

private:
  Buffer(
    std::uint8_t * host, void * device, void * stream, std::size_t size, bool on_device,
    bool query_src) noexcept
  : host_(host),
    device_(device),
    stream_(stream),
    size_(size),
    ok_(true),
    on_device_(on_device),
    query_src_(query_src)
  {
  }

  std::uint8_t * host_ = nullptr;
  void * device_ = nullptr;
  void * stream_ = nullptr;
  std::size_t size_ = 0;
  bool ok_ = false;
  bool on_device_ = false;
  bool query_src_ = false;

  friend class Loan<sensor_msgs::msg::Image>;
};

// One PointField as read from a View; `name` aliases the message like the View does.
struct Field
{
  std::string_view name;
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
  std::uint32_t count = 0;
};

namespace detail
{

#ifdef USE_FLUX_ENABLED
template <class MsgT>
struct Adapter;
template <>
struct Adapter<sensor_msgs::msg::Image>
{
  using type = sensor_msgs::flux_msg::Image;
};
template <>
struct Adapter<sensor_msgs::msg::PointCloud2>
{
  using type = sensor_msgs::flux_msg::PointCloud2;
};
template <class MsgT>
using AdapterT = typename Adapter<MsgT>::type;

// The channel of an Image publisher with Device::Cuda, beside the Adapter's typed one: the payload
// is the data field as one u8 run (device memory on a discrete GPU) and the aux bytes are one
// GpuImageAux. A change to that layout bumps GpuImageAux::kVersion.
inline constexpr std::uint64_t kGpuImageFingerprint = 0x780CD7C35A0C29C8ULL;

// In host byte order. The strings are not NUL-terminated.
struct GpuImageAux
{
  static constexpr std::uint32_t kVersion = 1;
  static constexpr std::size_t kEncodingCap = 32;
  static constexpr std::size_t kFrameIdCap = 256;

  std::uint32_t version;
  std::int32_t sec;
  std::uint32_t nanosec;
  std::uint32_t height;
  std::uint32_t width;
  std::uint32_t step;
  std::uint8_t is_bigendian;
  std::uint8_t encoding_len;
  std::uint16_t frame_id_len;
  std::array<char, kEncodingCap> encoding;
  std::array<char, kFrameIdCap> frame_id;
};
static_assert(
  offsetof(GpuImageAux, encoding) == 28 && offsetof(GpuImageAux, frame_id) == 60 &&
  sizeof(GpuImageAux) == 316);

struct GpuImageFields
{
  std::int32_t sec = 0;
  std::uint32_t nanosec = 0;
  std::uint32_t height = 0;
  std::uint32_t width = 0;
  std::uint32_t step = 0;
  std::uint8_t is_bigendian = 0;
  std::string_view encoding;
  std::string_view frame_id;
};

// The aux bytes come from another process: each value is read once, and the strings alias the
// frame only after their lengths were checked against the caps.
inline auto decode_gpu_image(
  flux::wire::Span<const std::uint8_t> aux, GpuImageFields & out) noexcept -> bool
{
  GpuImageAux a{};
  if (aux.size() != sizeof(a)) {
    return false;
  }
  std::memcpy(&a, aux.data(), sizeof(a));
  if (
    a.version != GpuImageAux::kVersion || a.encoding_len > GpuImageAux::kEncodingCap ||
    a.frame_id_len > GpuImageAux::kFrameIdCap) {
    return false;
  }
  const char * p = reinterpret_cast<const char *>(aux.data());  // NOLINT(*-reinterpret-cast)
  out = {
    a.sec,
    a.nanosec,
    a.height,
    a.width,
    a.step,
    a.is_bigendian,
    {p + offsetof(GpuImageAux, encoding), a.encoding_len},
    {p + offsetof(GpuImageAux, frame_id), a.frame_id_len}};
  return true;
}

// A GPU image frame as a View<Image> holds it.
struct GpuImageFrame
{
  GpuImageFields fields;
  const std::uint8_t * host = nullptr;
  std::size_t size = 0;
  const void * device = nullptr;
  void * stream = nullptr;
};
#endif

#ifdef FLUX_WRAPPER_CUDA
// The device buffer of a ROS-path Image publisher with Device::Cuda, shared by its loans.
class DeviceStaging
{
public:
  // Null when no stream could be created.
  static auto create() -> std::unique_ptr<DeviceStaging>
  {
    cudaStream_t s = nullptr;
    if (cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking) != cudaSuccess) {
      return nullptr;
    }
    return std::unique_ptr<DeviceStaging>(new DeviceStaging(s));
  }
  ~DeviceStaging()
  {
    (void)cudaFree(ptr_);
    (void)cudaStreamDestroy(stream_);
  }
  DeviceStaging(const DeviceStaging &) = delete;
  auto operator=(const DeviceStaging &) -> DeviceStaging & = delete;
  DeviceStaging(DeviceStaging &&) = delete;
  auto operator=(DeviceStaging &&) -> DeviceStaging & = delete;

  // At least n bytes. Allocates, waiting for the device, only when n exceeds every earlier n.
  // Null when that allocation failed.
  auto reserve(std::size_t n) noexcept -> void *
  {
    if (n > capacity_) {
      (void)cudaFree(ptr_);
      ptr_ = nullptr;
      capacity_ = 0;
      if (cudaMalloc(&ptr_, n) != cudaSuccess) {
        ptr_ = nullptr;
        return nullptr;
      }
      capacity_ = n;
    }
    return ptr_;
  }
  // Copies the first n bytes to out and waits for the copy.
  [[nodiscard]] auto download(std::uint8_t * out, std::size_t n) const noexcept -> bool
  {
    return n == 0 ||
           (cudaMemcpyAsync(out, ptr_, n, cudaMemcpyDeviceToHost, stream_) == cudaSuccess &&
            cudaStreamSynchronize(stream_) == cudaSuccess);
  }
  [[nodiscard]] auto stream() const noexcept -> cudaStream_t { return stream_; }

private:
  explicit DeviceStaging(cudaStream_t s) noexcept : stream_(s) {}

  cudaStream_t stream_ = nullptr;
  void * ptr_ = nullptr;
  std::size_t capacity_ = 0;
};
#endif

template <class MsgT>
class LoanBase
{
public:
  // False when the flux ring had no free slot or a field did not fit the slot.
  explicit operator bool() const noexcept
  {
#ifdef USE_FLUX_ENABLED
    if (builder_) {
      return builder_->ok__();
    }
#endif
    return msg_ != nullptr;
  }

  void set_header(const std_msgs::msg::Header & h)
  {
#ifdef USE_FLUX_ENABLED
    if (builder_) {
      builder_->set__header__stamp(h.stamp.sec, h.stamp.nanosec);
      builder_->set__header__frame_id(h.frame_id);
      return;
    }
#endif
    msg_->header = h;
  }

  // `n` bytes of the data field to write in place. Empty when they do not fit the flux slot.
  auto alloc_data(std::size_t n) -> Span<std::uint8_t>
  {
#ifdef USE_FLUX_ENABLED
    if (builder_) {
      auto s = builder_->alloc__data(n);
      return s.size() == n ? Span<std::uint8_t>(s.data(), n) : Span<std::uint8_t>();
    }
#endif
    msg_->data.resize(n);
    return {msg_->data.data(), n};
  }

protected:
  LoanBase() = default;
  explicit LoanBase(std::unique_ptr<MsgT> msg) : msg_(std::move(msg)) {}
#ifdef USE_FLUX_ENABLED
  using Builder = typename AdapterT<MsgT>::Builder;
  explicit LoanBase(Builder && b) : builder_(std::move(b)) {}
  std::optional<Builder> builder_;
#endif
  std::unique_ptr<MsgT> msg_;

  friend class flux_wrapper::Publisher<MsgT>;
};

// A flux View aliases the slot: valid only during the callback it was passed to.
template <class MsgT>
class ViewBase
{
public:
  explicit ViewBase(const MsgT & m) noexcept : msg_(&m) {}
#ifdef USE_FLUX_ENABLED
  explicit ViewBase(const typename AdapterT<MsgT>::View & v) noexcept : flux_(v) {}
#endif

  [[nodiscard]] auto stamp() const noexcept -> builtin_interfaces::msg::Time
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      builtin_interfaces::msg::Time t;
      t.sec = flux_->header__sec();
      t.nanosec = flux_->header__nanosec();
      return t;
    }
#endif
    return msg_->header.stamp;
  }
  [[nodiscard]] auto frame_id() const noexcept -> std::string_view
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return flux::wire::until_nul(flux_->header__frame_id());
    }
#endif
    return msg_->header.frame_id;
  }
  [[nodiscard]] auto data() const noexcept -> Span<const std::uint8_t>
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      auto s = flux_->data();
      return {s.data(), s.size()};
    }
#endif
    return {msg_->data.data(), msg_->data.size()};
  }

  // The message itself on the ROS path, for code that keeps it; null on the flux path.
  [[nodiscard]] auto ros_message() const noexcept -> const MsgT * { return msg_; }

  // A copy that outlives the callback.
  [[nodiscard]] auto to_msg() const -> MsgT
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return sensor_msgs::flux_msg::frame_to_msg(*flux_);
    }
#endif
    return *msg_;
  }

protected:
  ViewBase() = default;

  const MsgT * msg_ = nullptr;
#ifdef USE_FLUX_ENABLED
  std::optional<typename AdapterT<MsgT>::View> flux_;
#endif
};

}  // namespace detail

#ifdef USE_FLUX_ENABLED
#define FLUX_WRAPPER_SET(field, value) \
  if (builder_) {                      \
    builder_->set__##field(value);     \
  } else {                             \
    msg_->field = value;               \
  }
#define FLUX_WRAPPER_GET(field) return flux_ ? flux_->field() : msg_->field;
#else
#define FLUX_WRAPPER_SET(field, value) msg_->field = value;
#define FLUX_WRAPPER_GET(field) return msg_->field;
#endif

#ifdef USE_FLUX_ENABLED
#define FLUX_WRAPPER_GPU_SET(field, value)                    \
  if (gpu_put(offsetof(detail::GpuImageAux, field), value)) { \
    return;                                                   \
  }
#define FLUX_WRAPPER_GPU_GET(field) \
  if (gpu_) {                       \
    return gpu_->fields.field;      \
  }
#else
#define FLUX_WRAPPER_GPU_SET(field, value)
#define FLUX_WRAPPER_GPU_GET(field)
#endif

// On a publisher with Device::Cuda the flux path writes the header fields to the frame's aux bytes,
// where encoding and frame_id have caps (detail::GpuImageAux); a longer one makes the Loan false.
template <>
class Loan<sensor_msgs::msg::Image> : public detail::LoanBase<sensor_msgs::msg::Image>
{
public:
  Loan() = default;

  explicit operator bool() const noexcept
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      return slot_.valid() && !bad_;
    }
#endif
    return LoanBase::operator bool();
  }

  void set_header(const std_msgs::msg::Header & h)
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      (void)gpu_put(offsetof(detail::GpuImageAux, sec), h.stamp.sec);
      (void)gpu_put(offsetof(detail::GpuImageAux, nanosec), h.stamp.nanosec);
      gpu_str(
        offsetof(detail::GpuImageAux, frame_id_len), offsetof(detail::GpuImageAux, frame_id),
        detail::GpuImageAux::kFrameIdCap, h.frame_id, std::uint16_t{});
      return;
    }
#endif
    LoanBase::set_header(h);
  }

  void set_height(std::uint32_t v) { FLUX_WRAPPER_GPU_SET(height, v) FLUX_WRAPPER_SET(height, v) }
  void set_width(std::uint32_t v) { FLUX_WRAPPER_GPU_SET(width, v) FLUX_WRAPPER_SET(width, v) }
  void set_encoding(const std::string & v)
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      gpu_str(
        offsetof(detail::GpuImageAux, encoding_len), offsetof(detail::GpuImageAux, encoding),
        detail::GpuImageAux::kEncodingCap, v, std::uint8_t{});
      return;
    }
#endif
    FLUX_WRAPPER_SET(encoding, v)
  }
  void set_is_bigendian(std::uint8_t v)
  {
    FLUX_WRAPPER_GPU_SET(is_bigendian, v) FLUX_WRAPPER_SET(is_bigendian, v)
  }
  void set_step(std::uint32_t v) { FLUX_WRAPPER_GPU_SET(step, v) FLUX_WRAPPER_SET(step, v) }

  // As LoanBase::alloc_data. Empty on a slot in device memory: use alloc_buffer().
  auto alloc_data(std::size_t n) -> Span<std::uint8_t>
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      auto * p = static_cast<std::uint8_t *>(slot_.data());
      if (p == nullptr || n > slot_.capacity()) {
        return {};
      }
      len_ = n;
      return {p, n};
    }
#endif
#ifdef FLUX_WRAPPER_CUDA
    staged_ = false;
#endif
    return LoanBase::alloc_data(n);
  }

  // `n` bytes of the data field: on the device when the publisher uses Cuda, else as alloc_data.
  // On the ROS path with Cuda every loan of the publisher gets the same device buffer.
  auto alloc_buffer(std::size_t n) -> Buffer
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      if (!slot_ || n > slot_.capacity()) {
        return {};
      }
      len_ = n;
      return {static_cast<std::uint8_t *>(slot_.data()),
              slot_.device_ptr(),
              slot_.stream(),
              n,
              true,
              false};
    }
#endif
    bool query_src = false;
#ifdef FLUX_WRAPPER_CUDA
    if (staging_ != nullptr) {
      void * d = staging_->reserve(n);
      if (d == nullptr && n != 0) {
        return {};
      }
      msg_->data.resize(n);
      staged_ = true;
      return {nullptr, d, staging_->stream(), n, true, false};
    }
    query_src = query_src_;
#endif
    const auto s = LoanBase::alloc_data(n);
    if (s.size() != n) {
      return {};
    }
    return {s.data(), nullptr, nullptr, n, false, query_src};
  }

private:
  using LoanBase::LoanBase;

#ifdef USE_FLUX_ENABLED
  explicit Loan(flux::WriteSlot && slot) noexcept : slot_(std::move(slot)), gpu_(true)
  {
    if (slot_) {
      aux_ = slot_.aux().data();
      std::memset(aux_, 0, sizeof(detail::GpuImageAux));
      (void)gpu_put(offsetof(detail::GpuImageAux, version), detail::GpuImageAux::kVersion);
    }
  }

  // False when this is not a GPU image loan.
  template <class T>
  auto gpu_put(std::size_t at, T v) noexcept -> bool
  {
    if (aux_ != nullptr) {
      std::memcpy(aux_ + at, &v, sizeof(v));
    }
    return gpu_;
  }
  template <class Len>
  void gpu_str(std::size_t len_at, std::size_t at, std::size_t cap, std::string_view s, Len)
  {
    if (s.size() > cap) {
      bad_ = true;
      return;
    }
    (void)gpu_put(len_at, static_cast<Len>(s.size()));
    if (aux_ != nullptr) {
      std::memcpy(aux_ + at, s.data(), s.size());
    }
  }

  flux::WriteSlot slot_;
  std::uint8_t * aux_ = nullptr;
  std::size_t len_ = 0;
  bool gpu_ = false;
  bool bad_ = false;
#endif
#ifdef FLUX_WRAPPER_CUDA
  Loan(std::unique_ptr<sensor_msgs::msg::Image> msg, detail::DeviceStaging * staging) noexcept
  : LoanBase(std::move(msg)), staging_(staging)
  {
  }

  detail::DeviceStaging * staging_ = nullptr;
  bool staged_ = false;
  bool query_src_ = false;
#endif

  friend class Publisher<sensor_msgs::msg::Image>;
};

template <>
class Loan<sensor_msgs::msg::PointCloud2> : public detail::LoanBase<sensor_msgs::msg::PointCloud2>
{
public:
  Loan() = default;

  void set_height(std::uint32_t v) { FLUX_WRAPPER_SET(height, v) }
  void set_width(std::uint32_t v) { FLUX_WRAPPER_SET(width, v) }
  void set_is_bigendian(bool v) { FLUX_WRAPPER_SET(is_bigendian, v) }
  void set_point_step(std::uint32_t v) { FLUX_WRAPPER_SET(point_step, v) }
  void set_row_step(std::uint32_t v) { FLUX_WRAPPER_SET(row_step, v) }
  void set_is_dense(bool v) { FLUX_WRAPPER_SET(is_dense, v) }

  void set_fields(const std::vector<sensor_msgs::msg::PointField> & fields)
  {
#ifdef USE_FLUX_ENABLED
    if (builder_) {
      auto out = builder_->alloc__fields(fields.size());
      for (std::size_t i = 0; i < out.size(); ++i) {
        out[i].set__name(fields[i].name);
        out[i].set__offset(fields[i].offset);
        out[i].set__datatype(fields[i].datatype);
        out[i].set__count(fields[i].count);
      }
      return;
    }
#endif
    msg_->fields = fields;
  }

private:
  using LoanBase::LoanBase;
  friend class Publisher<sensor_msgs::msg::PointCloud2>;
};

// A frame of a publisher with Device::Cuda can be on the device (on_device()), when the
// subscription asked for Cuda; data() is then empty.
template <>
class View<sensor_msgs::msg::Image> : public detail::ViewBase<sensor_msgs::msg::Image>
{
public:
  using ViewBase::ViewBase;
#ifdef USE_FLUX_ENABLED
  View(const flux::FrameView & f, const detail::GpuImageFields & fields) noexcept
  : gpu_(detail::GpuImageFrame{
      fields, static_cast<const std::uint8_t *>(f.data()), f.size(), f.device_ptr(), f.stream()})
  {
  }
#endif

  [[nodiscard]] auto stamp() const noexcept -> builtin_interfaces::msg::Time
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      builtin_interfaces::msg::Time t;
      t.sec = gpu_->fields.sec;
      t.nanosec = gpu_->fields.nanosec;
      return t;
    }
#endif
    return ViewBase::stamp();
  }
  [[nodiscard]] auto frame_id() const noexcept -> std::string_view
  {
    FLUX_WRAPPER_GPU_GET(frame_id)
    return ViewBase::frame_id();
  }
  [[nodiscard]] auto height() const noexcept -> std::uint32_t
  {
    FLUX_WRAPPER_GPU_GET(height) FLUX_WRAPPER_GET(height)
  }
  [[nodiscard]] auto width() const noexcept -> std::uint32_t
  {
    FLUX_WRAPPER_GPU_GET(width) FLUX_WRAPPER_GET(width)
  }
  [[nodiscard]] auto encoding() const noexcept -> std::string_view
  {
#ifdef USE_FLUX_ENABLED
    FLUX_WRAPPER_GPU_GET(encoding)
    if (flux_) {
      return flux::wire::until_nul(flux_->encoding());
    }
#endif
    return msg_->encoding;
  }
  [[nodiscard]] auto is_bigendian() const noexcept -> std::uint8_t
  {
    FLUX_WRAPPER_GPU_GET(is_bigendian) FLUX_WRAPPER_GET(is_bigendian)
  }
  [[nodiscard]] auto step() const noexcept -> std::uint32_t
  {
    FLUX_WRAPPER_GPU_GET(step) FLUX_WRAPPER_GET(step)
  }

  // Empty when the frame is on the device.
  [[nodiscard]] auto data() const noexcept -> Span<const std::uint8_t>
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      return gpu_->host != nullptr ? Span<const std::uint8_t>(gpu_->host, gpu_->size)
                                   : Span<const std::uint8_t>();
    }
#endif
    return ViewBase::data();
  }
  [[nodiscard]] auto on_device() const noexcept -> bool { return device_ptr() != nullptr; }
  // Read-only device memory, null when the frame is not on the device.
  [[nodiscard]] auto device_ptr() const noexcept -> const void *
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      return gpu_->device;
    }
#endif
    return nullptr;
  }
  // The data field's size in bytes, wherever it is.
  [[nodiscard]] auto size() const noexcept -> std::size_t
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      return gpu_->size;
    }
#endif
    return ViewBase::data().size();
  }
#ifdef FLUX_WRAPPER_CUDA
  // Queue the frame's work here: the slot goes back only after it. Null when not on_device().
  [[nodiscard]] auto stream() const noexcept -> cudaStream_t
  {
#ifdef USE_FLUX_ENABLED
    if (on_device()) {
      return static_cast<cudaStream_t>(gpu_->stream);
    }
#endif
    return nullptr;
  }
#endif

  // A copy that outlives the callback; a frame on the device is copied to the host and waited for.
  // Its data is empty when that copy failed.
  [[nodiscard]] auto to_msg() const -> sensor_msgs::msg::Image
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      sensor_msgs::msg::Image m;
      m.header.stamp = stamp();
      m.header.frame_id = gpu_->fields.frame_id;
      m.height = gpu_->fields.height;
      m.width = gpu_->fields.width;
      m.encoding = gpu_->fields.encoding;
      m.is_bigendian = gpu_->fields.is_bigendian;
      m.step = gpu_->fields.step;
      if (gpu_->host != nullptr) {
        m.data.assign(gpu_->host, gpu_->host + gpu_->size);
      }
#ifdef FLUX_WRAPPER_CUDA
      else if (gpu_->size != 0) {
        m.data.resize(gpu_->size);
        if (
          cudaMemcpyAsync(
            m.data.data(), gpu_->device, gpu_->size, cudaMemcpyDeviceToHost, stream()) !=
            cudaSuccess ||
          cudaStreamSynchronize(stream()) != cudaSuccess) {
          m.data.clear();
        }
      }
#endif
      return m;
    }
#endif
    return ViewBase::to_msg();
  }

private:
#ifdef USE_FLUX_ENABLED
  std::optional<detail::GpuImageFrame> gpu_;
#endif
};

template <>
class View<sensor_msgs::msg::PointCloud2> : public detail::ViewBase<sensor_msgs::msg::PointCloud2>
{
public:
  using ViewBase::ViewBase;

  [[nodiscard]] auto height() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(height) }
  [[nodiscard]] auto width() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(width) }
  [[nodiscard]] auto fields_size() const noexcept -> std::size_t
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return flux_->fields__size();
    }
#endif
    return msg_->fields.size();
  }
  [[nodiscard]] auto field(std::size_t i) const noexcept -> Field
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      const auto f = flux_->fields(i);
      return {flux::wire::until_nul(f.name()), f.offset(), f.datatype(), f.count()};
    }
#endif
    const auto & f = msg_->fields[i];
    return {f.name, f.offset, f.datatype, f.count};
  }
  [[nodiscard]] auto is_bigendian() const noexcept -> bool { FLUX_WRAPPER_GET(is_bigendian) }
  [[nodiscard]] auto point_step() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(point_step) }
  [[nodiscard]] auto row_step() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(row_step) }
  [[nodiscard]] auto is_dense() const noexcept -> bool { FLUX_WRAPPER_GET(is_dense) }
};

#undef FLUX_WRAPPER_SET
#undef FLUX_WRAPPER_GET
#undef FLUX_WRAPPER_GPU_SET
#undef FLUX_WRAPPER_GPU_GET

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_MESSAGES_HPP
