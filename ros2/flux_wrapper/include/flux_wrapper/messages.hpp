#ifndef FLUX_WRAPPER_MESSAGES_HPP
#define FLUX_WRAPPER_MESSAGES_HPP

#include "flux_wrapper/runtime.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

#ifdef USE_FLUX_ENABLED
#include <flux/channel.hpp>

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

// Where a publisher's frames are produced or a subscription's are consumed. Cuda is used only
// where this build has CUDA and the host a CUDA device (on the flux path, a flux GPU route);
// otherwise Host, with a warning.
enum class Device { Host, Cuda };

namespace detail
{
template <class MsgT>
class LoanBase;
class DeviceStaging;
}  // namespace detail

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

  template <class MsgT>
  friend class detail::LoanBase;
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

// Never throws: a Cuda that cannot be had is Host, with the reason logged for this topic.
inline auto cuda_usable(const rclcpp::Logger & logger, const std::string & topic, Device requested)
  -> bool
{
  if (requested == Device::Host) {
    return false;
  }
#ifndef FLUX_WRAPPER_CUDA
  RCLCPP_WARN(
    logger, "%s: Device::Cuda, but flux_wrapper was built without CUDA; using Host", topic.c_str());
  return false;
#else
#ifdef USE_FLUX_ENABLED
  if (use_flux()) {
    const flux::gpu::Platform p = flux::gpu::probe();
    if (p.route == flux::gpu::Route::None) {
      RCLCPP_WARN(
        logger, "%s: Device::Cuda: no flux GPU route (%s); using Host", topic.c_str(),
        p.reason.c_str());
      return false;
    }
    return true;
  }
#endif
  int n = 0;
  const cudaError_t e = cudaGetDeviceCount(&n);
  if (e != cudaSuccess || n == 0) {
    RCLCPP_WARN(
      logger, "%s: Device::Cuda: no CUDA device (%s); using Host", topic.c_str(),
      e != cudaSuccess ? cudaGetErrorString(e) : "none found");
    return false;
  }
  return true;
#endif
}

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

// The channels of a publisher with Device::Cuda, beside the Adapter's typed ones: the payload is
// the data field as one u8 run (device memory on a discrete GPU) and the aux bytes are a frame of
// the Adapter's type with data empty, which tools read as they read the type (flux::Schemas).
template <class MsgT>
struct Gpu
{
  static constexpr std::uint64_t kFingerprint = flux::gpu_fingerprint(AdapterT<MsgT>::kFingerprint);
  // Room for every field but data: the header, encoding or the point fields and their names.
  static constexpr std::uint32_t kAuxBytes = 4096;
  static constexpr const char * kOverCap = "a field other than data is over the aux's 4096 bytes";
};

// The first string or list of a loan that did not fit where it goes (a GPU loan's aux, a host
// loan's slot), for the publisher's log line.
struct AuxOverflow
{
  const char * field = nullptr;  // null: every field fit
  std::size_t bytes = 0;         // its string bytes, or its entries' name bytes
  std::size_t entries = 0;       // its entries (PointCloud2 fields), 0 for a string
  std::size_t used = 0;          // aux bytes written before it
  std::int32_t sec = 0;          // header.stamp
  std::uint32_t nanosec = 0;
};

// A GPU channel frame's fields but data, read from its aux; empty when the aux is not a frame of
// the type. Each field is read once to check it, as a View built over another process's bytes.
template <class MsgT>
auto gpu_fields(const flux::FrameView & f) -> std::optional<typename AdapterT<MsgT>::View>
{
  const auto aux = f.aux();
  typename AdapterT<MsgT>::View v(aux.data(), aux.size());
  try {
    (void)sensor_msgs::flux_msg::frame_to_msg(v);
  } catch (const std::exception &) {
    return std::nullopt;
  }
  if (!v.ok__()) {
    return std::nullopt;
  }
  return v;
}

#endif

#ifdef FLUX_WRAPPER_CUDA
// The device buffer of a ROS-path publisher or subscription with Device::Cuda, shared by its loans
// or messages.
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
  // Queues a copy of n bytes of pageable host memory on stream(), after the work queued there
  // before; CUDA has read src when the call returns. Null when the buffer or the copy failed.
  auto upload(const std::uint8_t * src, std::size_t n) noexcept -> const void *
  {
    // One byte at least, so that an empty data field has a device address too.
    void * d = reserve(n != 0 ? n : 1);
    if (
      d == nullptr ||
      (n != 0 && cudaMemcpyAsync(d, src, n, cudaMemcpyHostToDevice, stream_) != cudaSuccess)) {
      return nullptr;
    }
    return d;
  }
  [[nodiscard]] auto stream() const noexcept -> cudaStream_t { return stream_; }

private:
  explicit DeviceStaging(cudaStream_t s) noexcept : stream_(s) {}

  cudaStream_t stream_ = nullptr;
  void * ptr_ = nullptr;
  std::size_t capacity_ = 0;
};
#endif

// On a publisher with Device::Cuda the flux path writes the fields other than data to the frame's
// aux bytes, through a Builder there; fields over Gpu<MsgT>::kAuxBytes make the Loan false.
template <class MsgT>
class LoanBase
{
public:
  // False when the flux ring had no free slot or a field did not fit the slot or its cap.
  explicit operator bool() const noexcept
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      return slot_.valid() && !bad_ && builder_ && builder_->ok__();
    }
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
      over_.sec = h.stamp.sec;
      over_.nanosec = h.stamp.nanosec;
      put_variable("header.frame_id", h.frame_id.size(), 0, [&] {
        builder_->set__header__frame_id(h.frame_id);
      });
      return;
    }
#endif
    msg_->header = h;
  }

  // `n` bytes of the data field to write in place. Empty when they do not fit the flux slot, and
  // on a slot in device memory: use alloc_buffer().
  auto alloc_data(std::size_t n) -> Span<std::uint8_t>
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      auto * p = static_cast<std::uint8_t *>(slot_.data());
      if (n > slot_.capacity()) {
        bad_ = true;
        return {};
      }
      if (p == nullptr) {
        return {};
      }
      len_ = n;
      return {p, n};
    }
    if (builder_) {
      auto s = builder_->alloc__data(n);
      return s.size() == n ? Span<std::uint8_t>(s.data(), n) : Span<std::uint8_t>();
    }
#endif
#ifdef FLUX_WRAPPER_CUDA
    staged_ = false;
#endif
    msg_->data.resize(n);
    return {msg_->data.data(), n};
  }

  // `n` bytes of the data field: on the device when the publisher uses Cuda, else as alloc_data.
  // On the ROS path with Cuda every loan of the publisher gets the same device buffer.
  auto alloc_buffer(std::size_t n) -> Buffer
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      if (!slot_) {
        return {};
      }
      if (n > slot_.capacity()) {
        bad_ = true;
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
    const auto s = alloc_data(n);
    if (s.size() != n) {
      return {};
    }
    return {s.data(), nullptr, nullptr, n, false, query_src};
  }

protected:
  LoanBase() = default;
  explicit LoanBase(std::unique_ptr<MsgT> msg) : msg_(std::move(msg)) {}
#ifdef USE_FLUX_ENABLED
  using Builder = typename AdapterT<MsgT>::Builder;
  explicit LoanBase(Builder && b) : builder_(std::move(b)) {}
  // The fields but data go to a Builder over the slot's aux, the data field to its payload.
  explicit LoanBase(flux::WriteSlot && slot) noexcept : slot_(std::move(slot)), gpu_(true)
  {
    if (slot_) {
      builder_.emplace(slot_.aux().data(), slot_.aux().size());
    }
  }

  // Runs put(), which writes `field`; notes it when it is the first that does not fit.
  template <class Put>
  void put_variable(const char * field, std::size_t bytes, std::size_t entries, Put && put)
  {
    const bool fit = builder_->ok__();
    const std::size_t used = builder_->size__();
    put();
    if (fit && !builder_->ok__()) {
      over_.field = field;
      over_.bytes = bytes;
      over_.entries = entries;
      over_.used = used;
    }
  }

  std::optional<Builder> builder_;
  flux::WriteSlot slot_;
  std::size_t len_ = 0;
  bool gpu_ = false;
  bool bad_ = false;
  AuxOverflow over_;
#endif
  std::unique_ptr<MsgT> msg_;
#ifdef FLUX_WRAPPER_CUDA
  LoanBase(std::unique_ptr<MsgT> msg, DeviceStaging * staging) noexcept
  : msg_(std::move(msg)), staging_(staging)
  {
  }

  DeviceStaging * staging_ = nullptr;
  bool staged_ = false;
  bool query_src_ = false;
#endif

  friend class flux_wrapper::Publisher<MsgT>;
};

// A flux View aliases the slot: valid only during the callback it was passed to, or with a
// HeldCallback while its pointer lives. The data field is in host memory (data()), in device
// memory (on_device()), or in both.
template <class MsgT>
class ViewBase
{
public:
  explicit ViewBase(const MsgT & m, const void * device = nullptr, void * stream = nullptr) noexcept
  : msg_(&m), device_(device), stream_(stream)
  {
  }
#ifdef USE_FLUX_ENABLED
  explicit ViewBase(
    const typename AdapterT<MsgT>::View & v, const void * device = nullptr,
    void * stream = nullptr) noexcept
  : flux_(v), device_(device), stream_(stream)
  {
  }
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
  // Empty when the frame is in device memory only.
  [[nodiscard]] auto data() const noexcept -> Span<const std::uint8_t>
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      return host_ != nullptr ? Span<const std::uint8_t>(host_, size_) : Span<const std::uint8_t>();
    }
    if (flux_) {
      auto s = flux_->data();
      return {s.data(), s.size()};
    }
#endif
    return {msg_->data.data(), msg_->data.size()};
  }
  [[nodiscard]] auto on_device() const noexcept -> bool { return device_ != nullptr; }
  // Read-only device memory, null when the frame is not on the device.
  [[nodiscard]] auto device_ptr() const noexcept -> const void * { return device_; }
  // The data field's size in bytes, wherever it is.
  [[nodiscard]] auto size() const noexcept -> std::size_t
  {
#ifdef USE_FLUX_ENABLED
    if (gpu_) {
      return size_;
    }
#endif
    return data().size();
  }
#ifdef FLUX_WRAPPER_CUDA
  // Queue the frame's work here: the slot goes back, or the subscription's device buffer is
  // written again, only after it. Null when not on_device().
  [[nodiscard]] auto stream() const noexcept -> cudaStream_t
  {
    return on_device() ? static_cast<cudaStream_t>(stream_) : nullptr;
  }
  // Orders stream() after the work queued on s so far, on the device: the slot or device buffer
  // then goes back only after that work too. For a frame read on another stream than stream().
  // True when not on_device().
  [[nodiscard]] auto after(cudaStream_t s) const noexcept -> bool
  {
    if (!on_device()) {
      return true;
    }
    cudaEvent_t e = nullptr;
    if (cudaEventCreateWithFlags(&e, cudaEventDisableTiming) != cudaSuccess) {
      return false;
    }
    // The wait takes the event as recorded now, so destroying it at once is safe.
    const bool ok =
      cudaEventRecord(e, s) == cudaSuccess && cudaStreamWaitEvent(stream(), e, 0) == cudaSuccess;
    (void)cudaEventDestroy(e);
    return ok;
  }
#endif

  // The message itself on the ROS path, for code that keeps it; null on the flux path.
  [[nodiscard]] auto ros_message() const noexcept -> const MsgT * { return msg_; }

  // A copy that outlives the callback.
  [[nodiscard]] auto to_msg() const -> MsgT
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      MsgT m = sensor_msgs::flux_msg::frame_to_msg(*flux_);
      if (gpu_) {
        gpu_data_to_msg(m);
      }
      return m;
    }
#endif
    return *msg_;
  }

protected:
  ViewBase() = default;
#ifdef USE_FLUX_ENABLED
  // A GPU channel frame: `fields` read from its aux, the data field its payload.
  ViewBase(const flux::FrameView & f, const typename AdapterT<MsgT>::View & fields) noexcept
  : flux_(fields),
    gpu_(true),
    host_(static_cast<const std::uint8_t *>(f.data())),
    size_(f.size()),
    device_(f.device_ptr()),
    stream_(f.stream())
  {
  }

  // A GPU channel frame's data field into m. A frame only on the device is copied to the host and
  // waited for; m.data is empty when that copy failed.
  void gpu_data_to_msg(MsgT & m) const
  {
    if (host_ != nullptr) {
      m.data.assign(host_, host_ + size_);
    }
#ifdef FLUX_WRAPPER_CUDA
    else if (size_ != 0) {
      m.data.resize(size_);
      if (
        cudaMemcpyAsync(m.data.data(), device_, size_, cudaMemcpyDeviceToHost, stream()) !=
          cudaSuccess ||
        cudaStreamSynchronize(stream()) != cudaSuccess) {
        m.data.clear();
      }
    }
#endif
  }
#endif

  const MsgT * msg_ = nullptr;
#ifdef USE_FLUX_ENABLED
  std::optional<typename AdapterT<MsgT>::View> flux_;
  // Set on a frame of a publisher with Device::Cuda: flux_ reads its aux, and its data field is
  // host_ and device_.
  bool gpu_ = false;
  const std::uint8_t * host_ = nullptr;
  std::size_t size_ = 0;
#endif
  const void * device_ = nullptr;
  void * stream_ = nullptr;
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

template <>
class Loan<sensor_msgs::msg::Image> : public detail::LoanBase<sensor_msgs::msg::Image>
{
public:
  Loan() = default;

  void set_height(std::uint32_t v) { FLUX_WRAPPER_SET(height, v) }
  void set_width(std::uint32_t v) { FLUX_WRAPPER_SET(width, v) }
  void set_encoding(const std::string & v)
  {
#ifdef USE_FLUX_ENABLED
    if (builder_) {
      put_variable("encoding", v.size(), 0, [&] { builder_->set__encoding(v); });
      return;
    }
#endif
    msg_->encoding = v;
  }
  void set_is_bigendian(std::uint8_t v) { FLUX_WRAPPER_SET(is_bigendian, v) }
  void set_step(std::uint32_t v) { FLUX_WRAPPER_SET(step, v) }

private:
  using LoanBase::LoanBase;

  // Every field but data.
  void set_from(const sensor_msgs::msg::Image & m)
  {
    set_header(m.header);
    set_height(m.height);
    set_width(m.width);
    set_encoding(m.encoding);
    set_is_bigendian(m.is_bigendian);
    set_step(m.step);
  }

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
      std::size_t names = 0;
      for (const auto & f : fields) {
        names += f.name.size();
      }
      put_variable("fields", names, fields.size(), [&] {
        auto out = builder_->alloc__fields(fields.size());
        for (std::size_t i = 0; i < out.size(); ++i) {
          out[i].set__name(fields[i].name);
          out[i].set__offset(fields[i].offset);
          out[i].set__datatype(fields[i].datatype);
          out[i].set__count(fields[i].count);
        }
      });
      return;
    }
#endif
    msg_->fields = fields;
  }

private:
  using LoanBase::LoanBase;

  // Every field but data.
  void set_from(const sensor_msgs::msg::PointCloud2 & m)
  {
    set_header(m.header);
    set_height(m.height);
    set_width(m.width);
    set_fields(m.fields);
    set_is_bigendian(m.is_bigendian);
    set_point_step(m.point_step);
    set_row_step(m.row_step);
    set_is_dense(m.is_dense);
  }

  friend class Publisher<sensor_msgs::msg::PointCloud2>;
};

template <>
class View<sensor_msgs::msg::Image> : public detail::ViewBase<sensor_msgs::msg::Image>
{
public:
  using ViewBase::ViewBase;
#ifdef USE_FLUX_ENABLED
  View(const flux::FrameView & f, const sensor_msgs::flux_msg::Image::View & fields) noexcept
  : ViewBase(f, fields)
  {
  }
#endif

  [[nodiscard]] auto height() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(height) }
  [[nodiscard]] auto width() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(width) }
  [[nodiscard]] auto encoding() const noexcept -> std::string_view
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return flux::wire::until_nul(flux_->encoding());
    }
#endif
    return msg_->encoding;
  }
  [[nodiscard]] auto is_bigendian() const noexcept -> std::uint8_t
  {
    FLUX_WRAPPER_GET(is_bigendian)
  }
  [[nodiscard]] auto step() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(step) }

  // A copy that outlives the callback; a frame on the device only is copied to the host and
  // waited for. Its data is empty when that copy failed.
  using ViewBase::to_msg;
};

template <>
class View<sensor_msgs::msg::PointCloud2> : public detail::ViewBase<sensor_msgs::msg::PointCloud2>
{
public:
  using ViewBase::ViewBase;
#ifdef USE_FLUX_ENABLED
  View(const flux::FrameView & f, const sensor_msgs::flux_msg::PointCloud2::View & fields) noexcept
  : ViewBase(f, fields)
  {
  }
#endif

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

  // As View<Image>::to_msg().
  using ViewBase::to_msg;
};

#undef FLUX_WRAPPER_SET
#undef FLUX_WRAPPER_GET

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_MESSAGES_HPP
