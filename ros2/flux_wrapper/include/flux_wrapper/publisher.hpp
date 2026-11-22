#ifndef FLUX_WRAPPER_PUBLISHER_HPP
#define FLUX_WRAPPER_PUBLISHER_HPP

#include "flux_wrapper/messages.hpp"
#include "flux_wrapper/runtime.hpp"

#include <rclcpp/rclcpp.hpp>

#ifdef USE_FLUX_ENABLED
#include <flux/publisher.hpp>
#endif

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

namespace flux_wrapper
{

// Why a message did not go out; Ok when it did. Values may be added.
enum class Published {
  Ok,
  Backpressure,  // no free slot: this message is dropped
  TooLarge,      // a field did not fit the slot, or a Device::Cuda message its aux
  InvalidArgument,
  InUse,
  NotPublisher,
  FenceFailed,
  GpuWedged,
  DeviceError,  // a CUDA copy failed, or an image's step is not its slot's pitch
};

// What publish() gives: true when the message went out, status() why not otherwise.
class [[nodiscard]] PublishResult
{
public:
  explicit PublishResult(Published p) noexcept : p_(p) {}
  explicit operator bool() const noexcept { return p_ == Published::Ok; }
  [[nodiscard]] auto status() const noexcept -> Published { return p_; }

private:
  Published p_;
};

#ifdef USE_FLUX_ENABLED
namespace detail
{
// The flux encoding of a sensor_msgs/Image encoding; an empty one is none.
inline auto image_encoding(const std::string & e) -> flux::ImageEncoding
{
  using E = flux::ImageEncoding;
  static const std::pair<const char *, E> kNames[] = {
    {"", E::None},         {"bgra8", E::Bgra8}, {"rgba8", E::Rgba8}, {"mono8", E::Mono8},
    {"mono16", E::Mono16}, {"nv12", E::Nv12},   {"uyvy", E::Uyvy},   {"yuyv", E::Yuyv}};
  for (const auto & [name, value] : kNames) {
    if (e == name) {
      return value;
    }
  }
  const std::string why = "flux_wrapper: no image slot takes encoding '" + e + "'";
  throw flux::Error(flux::Status::InvalidArgument, why.c_str());
}
}  // namespace detail
#endif

struct PublisherOptions
{
  Alloc alloc = Alloc::Arena;
  // flux only, as flux::PublisherOptions: the slot in bytes, the slots a subscriber may hold at
  // once plus one to write into, and the most an Alloc::Slot or Device::Cuda publisher may be
  // given of each. Zero is flux's default. A Device::Cuda slot holds the data field, 4 MiB by
  // default; an Arena slot is the type's and its data field goes to the publisher's arena.
  std::uint32_t slot_size = 0;
  std::uint32_t slot_count = 0;
  std::uint32_t max_slot_size = 0;
  std::uint32_t max_slot_count = 0;
  // flux only, as flux::PublisherOptions: how a slot_size of 0 grows past a message.
  Growth growth = Growth::PowerOfTwo;
  // flux with Alloc::Slot or Device::Cuda only: the slots get memory once a subscriber attaches,
  // and publish() is false until then. For topics nobody may subscribe to.
  bool alloc_on_subscriber = false;
  // flux only, as flux::PublisherOptions: image slots, and their format as a sensor_msgs/Image
  // encoding ("bgra8", "nv12", ...), which needs Device::Cuda and a step that is image_layout's.
  ImageSlots image_slots = ImageSlots::Auto;
  std::string image_encoding;
  std::uint32_t image_width = 0;
  std::uint32_t image_height = 0;
  // flux only: also publish each message as its ROS message on the same topic while a ROS
  // subscriber (rviz2, ros2 bag) wants it. With Device::Cuda the mirror copies the data field
  // from the device for each of those messages.
  bool mirror_to_ros = false;
  // Cuda: Loan::alloc_buffer() hands out device memory (see Device). On the flux path a slot of a
  // discrete GPU is device memory; on the ROS path publish() copies the buffer to the message. A
  // type without a root data is Cpu, with a warning.
  Device device = Device::Cpu;
};

// One topic over flux or over ROS, chosen once per process by use_flux(). Writing a message is
// the same on both paths: loan(), fill the Loan, publish(). One thread at a time. A build with
// flux needs the adapter of MsgT (detail::Adapter).
template <class MsgT>
class Publisher
{
public:
  template <class NodeT>
  Publisher(
    NodeT & node, const std::string & topic, const rclcpp::QoS & qos,
    const PublisherOptions & options = {})
  : logger_(node.get_logger())
  {
    [[maybe_unused]] const bool cuda = detail::cuda_usable<MsgT>(logger_, topic, options.device);
#ifdef FLUX_WRAPPER_CUDA
    requested_cuda_ = options.device == Device::Cuda;
#endif
#ifdef USE_FLUX_ENABLED
    if (use_flux()) {
      flux::PublisherOptions o;
      o.alloc = options.alloc == Alloc::Slot ? flux::Alloc::Slot : flux::Alloc::Arena;
      o.slot_size = options.slot_size;
      o.slot_count = options.slot_count;
      o.max_slot_size = options.max_slot_size;
      o.max_slot_count = options.max_slot_count;
      o.growth = options.growth == Growth::BestFit  ? flux::Growth::BestFit
                 : options.growth == Growth::Static ? flux::Growth::Static
                                                    : flux::Growth::PowerOfTwo;
      o.alloc_on_subscriber = options.alloc_on_subscriber;
      o.image_slots = options.image_slots == ImageSlots::On    ? flux::ImageSlots::On
                      : options.image_slots == ImageSlots::Off ? flux::ImageSlots::Off
                                                               : flux::ImageSlots::Auto;
      o.image_encoding = detail::image_encoding(options.image_encoding);
      o.image_width = options.image_width;
      o.image_height = options.image_height;
      if constexpr (detail::kRootData<MsgT>) {
        if (cuda) {
          o.device = flux::Device::Cuda;
          o.aux_size = detail::Gpu<MsgT>::kAuxBytes;
          flux_ =
            std::make_unique<flux::Publisher>(node, topic, o, detail::Gpu<MsgT>::kFingerprint);
          device_ = true;
          if (options.mirror_to_ros) {
            flux_->mirror_to_ros(qos);
          }
          return;
        }
      }
      if (o.image_encoding != flux::ImageEncoding::None) {
        // A host message starts with its header, so the image is not where the layout says. Asked
        // of Cpu it is a mistake; of a Cuda that could not be had, the fallback drops it.
        if (options.device != Device::Cuda) {
          throw flux::Error(
            flux::Status::InvalidArgument, "flux_wrapper: an image format needs Device::Cuda");
        }
        RCLCPP_WARN(
          node.get_logger(), "%s: Device::Cuda could not be had, so its image format is dropped",
          topic.c_str());
        o.image_encoding = flux::ImageEncoding::None;
        o.image_width = 0;
        o.image_height = 0;
        o.image_slots = flux::ImageSlots::Auto;
      }
      flux_ =
        std::make_unique<flux::Publisher>(node, topic, o, detail::AdapterT<MsgT>::kFingerprint);
      if (options.mirror_to_ros) {
        flux_->mirror_to_ros(qos);
      }
      return;
    }
#else
    (void)options;
    if (flux_requested()) {
      RCLCPP_WARN_ONCE(logger_, "ENABLE_FLUX=1, but flux_wrapper was built without flux: ROS path");
    }
#endif
#ifdef FLUX_WRAPPER_CUDA
    if (cuda) {
      staging_ = detail::DeviceStaging::create();
      if (staging_) {
        device_ = true;
      } else {
        RCLCPP_WARN(
          logger_, "%s: Device::Cuda: no CUDA stream could be created; using Cpu", topic.c_str());
      }
    }
#endif
    ros_ = rclcpp::create_publisher<MsgT>(node, topic, qos);
  }

  // A Loan that is false had no free flux slot: fill it or not, publish() sends nothing.
  auto loan() -> Loan<MsgT>
  {
#ifdef USE_FLUX_ENABLED
    if (flux_ && device_) {
      return Loan<MsgT>(flux_->loan());
    }
#endif
#ifdef FLUX_WRAPPER_CUDA
    if (staging_) {
      return Loan<MsgT>(std::make_unique<MsgT>(), staging_.get());
    }
#endif
#ifdef USE_FLUX_ENABLED
    Loan<MsgT> l = flux_ ? Loan<MsgT>(typename detail::AdapterT<MsgT>::Builder(flux_->loan()))
                         : Loan<MsgT>(std::make_unique<MsgT>());
#else
    Loan<MsgT> l(std::make_unique<MsgT>());
#endif
#ifdef FLUX_WRAPPER_CUDA
    l.query_src_ = requested_cuda_;
#endif
    return l;
  }

  // False when nothing went out: no free slot, a field that did not fit, or a fault (logged).
  auto publish(Loan<MsgT> && loan) -> PublishResult
  {
#ifdef USE_FLUX_ENABLED
    if constexpr (detail::kRootData<MsgT>) {
      if (loan.gpu_) {
        return commit_gpu(loan);
      }
    }
    if (loan.builder_) {
      if (!loan.builder_->ok__() && loan.over_.field != nullptr) {
        // Refused, and tells a slot left to flux the size to grow to (Growth).
        const std::uint32_t slot = flux_->slot_size();
        (void)loan.builder_->commit__();
        overflow(loan.over_, "", "slot", slot);
        return PublishResult(Published::TooLarge);
      }
      return report(loan.builder_->commit__());
    }
#endif
    if (!loan.msg_) {
      return PublishResult(Published::Backpressure);
    }
#ifdef FLUX_WRAPPER_CUDA
    if constexpr (detail::kRootData<MsgT>) {
      if (loan.staged_ && !staging_->download(loan.msg_->data.data(), loan.msg_->data.size())) {
        device_error("the device to host copy failed");
        return PublishResult(Published::DeviceError);
      }
    }
#endif
    ros_->publish(std::move(loan.msg_));
    return PublishResult(Published::Ok);
  }

  // For a message already built as a ROS message. The flux path copies it into a slot once.
  auto publish(std::unique_ptr<MsgT> msg) -> PublishResult
  {
#ifdef USE_FLUX_ENABLED
    if constexpr (detail::kRootData<MsgT>) {
      if (flux_ && device_) {
        return publish_gpu(*msg);
      }
    }
    if (flux_) {
      return publish_host(*msg);
    }
#endif
    ros_->publish(std::move(msg));
    return PublishResult(Published::Ok);
  }

  // For a message the caller keeps. The flux path copies it into a slot once.
  auto publish(const MsgT & msg) -> PublishResult
  {
#ifdef USE_FLUX_ENABLED
    if constexpr (detail::kRootData<MsgT>) {
      if (flux_ && device_) {
        return publish_gpu(msg);
      }
    }
    if (flux_) {
      return publish_host(msg);
    }
#endif
    ros_->publish(msg);
    return PublishResult(Published::Ok);
  }

  // The flux path counts subscribing processes, and a mirror counts as one while a ROS
  // subscriber wants it.
  [[nodiscard]] auto get_subscription_count() const -> std::size_t
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return flux_->subscriber_count();
    }
#endif
    return ros_->get_subscription_count();
  }

  [[nodiscard]] auto uses_flux() const noexcept -> bool
  {
#ifdef USE_FLUX_ENABLED
    return flux_ != nullptr;
#else
    return false;
#endif
  }

  // Device::Cuda was asked for and is used.
  [[nodiscard]] auto uses_device() const noexcept -> bool { return device_; }

private:
#if defined(USE_FLUX_ENABLED) || defined(FLUX_WRAPPER_CUDA)
  void device_error(const char * what)
  {
    if (!device_logged_) {
      device_logged_ = true;
      RCLCPP_ERROR(logger_, "Device::Cuda publish failed: %s; later failures are not logged", what);
    }
  }
#endif

#ifdef USE_FLUX_ENABLED
  auto commit_gpu(Loan<MsgT> & loan) -> PublishResult
  {
    if (!loan.slot_) {
      return PublishResult(Published::Backpressure);
    }
    // The slot is the subscribers' after the commit, and the slot of nobody after an abort.
    if (loan.bad_ || !loan.builder_ || !loan.builder_->ok__()) {
      loan.slot_.abort();
      if (loan.over_.field != nullptr) {
        overflow(loan.over_, " on Device::Cuda", "aux", detail::Gpu<MsgT>::kAuxBytes);
      } else if (loan.bad_) {
        (void)report(flux::Published::TooLarge);  // data past the slot, as publish(msg) logs it
      } else {
        device_error(detail::Gpu<MsgT>::kOverCap);
      }
      return PublishResult(Published::TooLarge);
    }
    if constexpr (std::is_same_v<MsgT, sensor_msgs::msg::Image>) {
      if (const auto l = loan.slot_.image_layout(); l && loan.step_ != l->pitch[0]) {
        loan.slot_.abort();
        device_error(("step " + std::to_string(loan.step_) + " is not the image slot's pitch " +
                      std::to_string(l->pitch[0]))
                       .c_str());
        return PublishResult(Published::DeviceError);
      }
    }
    loan.slot_.set_aux_len(loan.builder_->size__());
    return report(loan.slot_.commit(loan.len_));
  }

  // Once per publisher: which field of which message did not fit, to size the aux or slot by.
  void overflow(
    const detail::AuxOverflow & o, const char * on, const char * where, std::uint32_t size)
  {
    if (over_logged_) {
      return;
    }
    over_logged_ = true;
    std::string stamp;
    if constexpr (detail::kRootHeader<MsgT>) {
      const std::string ns = std::to_string(o.nanosec);
      stamp = "; header.stamp " + std::to_string(o.sec) + "." +
              std::string(ns.size() < 9 ? 9 - ns.size() : 0, '0') + ns;
    }
    if (*o.field == '\0') {
      // set_from() writes the fields in one call, so which one did not fit is not known.
      RCLCPP_ERROR(
        logger_,
        "%s: a message did not go out%s: the fields set from it did not fit the %s (%u B, %zu B "
        "used before them)%s. Later ones are not logged",
        flux_->topic_name().c_str(), on, where, size, o.used, stamp.c_str());
      return;
    }
    std::string what = std::string(o.field) + " (" + std::to_string(o.bytes) + " B";
    if (o.entries != 0) {
      what = std::string(o.field) + " (" + std::to_string(o.entries) + " entries, names " +
             std::to_string(o.bytes) + " B";
    }
    RCLCPP_ERROR(
      logger_,
      "%s: a message did not go out%s: %s) did not fit the %s (%u B, %zu B used before it)%s. "
      "Later ones are not logged",
      flux_->topic_name().c_str(), on, what.c_str(), where, size, o.used, stamp.c_str());
  }

  // As a Loan filled from msg, so a field that does not fit is logged as one set by hand is.
  auto publish_host(const MsgT & msg) -> PublishResult
  {
    Loan<MsgT> l = loan();
    if (!l) {
      return PublishResult(Published::Backpressure);
    }
    l.set_from(msg);
    if constexpr (detail::kRootData<MsgT>) {
      const auto d = l.alloc_data(msg.data.size());
      if (d.size() == msg.data.size() && !msg.data.empty()) {
        std::memcpy(d.data(), msg.data.data(), msg.data.size());
      }
    }
    return publish(std::move(l));
  }

  auto publish_gpu(const MsgT & msg) -> PublishResult
  {
    Loan<MsgT> l = loan();
    l.set_from(msg);
    // A false Loan, data past the slot among its causes, gives no buffer; commit_gpu() logs why.
    const Buffer b = l.alloc_buffer(msg.data.size());
    if (b && !b.copy_from(msg.data.data(), msg.data.size())) {
      device_error("the host to device copy failed");
      return PublishResult(Published::DeviceError);
    }
    return commit_gpu(l);
  }

  auto report(flux::Published p) -> PublishResult
  {
    if (flux::faulted(p) && !fault_logged_) {
      fault_logged_ = true;
      RCLCPP_ERROR(
        logger_, "flux publish on %s failed: %s (slot %u bytes); later failures are not logged",
        flux_->topic_name().c_str(), flux::to_string(p), flux_->slot_size());
    }
    switch (p) {
      case flux::Published::Ok:
        return PublishResult(Published::Ok);
      case flux::Published::Backpressure:
        return PublishResult(Published::Backpressure);
      case flux::Published::TooLarge:
        return PublishResult(Published::TooLarge);
      case flux::Published::InUse:
        return PublishResult(Published::InUse);
      case flux::Published::NotPublisher:
        return PublishResult(Published::NotPublisher);
      case flux::Published::FenceFailed:
        return PublishResult(Published::FenceFailed);
      case flux::Published::GpuWedged:
        return PublishResult(Published::GpuWedged);
      case flux::Published::InvalidArgument:
        break;
    }
    return PublishResult(Published::InvalidArgument);
  }

  std::unique_ptr<flux::Publisher> flux_;
  bool fault_logged_ = false;
  bool over_logged_ = false;
#endif
#ifdef FLUX_WRAPPER_CUDA
  std::unique_ptr<detail::DeviceStaging> staging_;
  bool requested_cuda_ = false;
#endif
#if defined(USE_FLUX_ENABLED) || defined(FLUX_WRAPPER_CUDA)
  bool device_logged_ = false;
#endif
  bool device_ = false;
  rclcpp::Logger logger_;
  typename rclcpp::Publisher<MsgT>::SharedPtr ros_;
};

// As rclcpp::create_publisher.
template <class MsgT, class NodeT>
auto create_publisher(
  NodeT & node, const std::string & topic, const rclcpp::QoS & qos,
  const PublisherOptions & options = {}) -> std::shared_ptr<Publisher<MsgT>>
{
  return std::make_shared<Publisher<MsgT>>(node, topic, qos, options);
}

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_PUBLISHER_HPP
