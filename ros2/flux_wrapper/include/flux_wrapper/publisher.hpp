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
#include <utility>

namespace flux_wrapper
{

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
  // flux with Alloc::Slot or Device::Cuda only: the slots get memory once a subscriber attaches,
  // and publish() is false until then. For topics nobody may subscribe to.
  bool alloc_on_subscriber = false;
  // flux only: also publish each message as its ROS message on the same topic while a ROS
  // subscriber (rviz2, ros2 bag) wants it. With Device::Cuda the mirror copies the data field
  // from the device for each of those messages.
  bool mirror_to_ros = false;
  // Cuda: Loan::alloc_buffer() hands out device memory (see Device). On the flux path a slot of a
  // discrete GPU is device memory; on the ROS path publish() copies the buffer to the message. A
  // type without a root data is Host, with a warning.
  Device device = Device::Host;
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
      o.alloc_on_subscriber = options.alloc_on_subscriber;
      if constexpr (detail::kRootData<MsgT>) {
        if (cuda) {
          o.device = flux::Device::Cuda;
          o.aux_size = detail::Gpu<MsgT>::kAuxBytes;
          flux_ =
            std::make_unique<flux::Publisher>(node, topic, o, detail::Gpu<MsgT>::kFingerprint);
          device_ = true;
          if (options.mirror_to_ros) {
            flux_->mirror_to_ros();
          }
          return;
        }
      }
      flux_ =
        std::make_unique<flux::Publisher>(node, topic, o, detail::AdapterT<MsgT>::kFingerprint);
      if (options.mirror_to_ros) {
        flux_->mirror_to_ros();
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
          logger_, "%s: Device::Cuda: no CUDA stream could be created; using Host", topic.c_str());
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
  auto publish(Loan<MsgT> && loan) -> bool
  {
#ifdef USE_FLUX_ENABLED
    if constexpr (detail::kRootData<MsgT>) {
      if (loan.gpu_) {
        return commit_gpu(loan);
      }
    }
    if (loan.builder_) {
      if (!loan.builder_->ok__() && loan.over_.field != nullptr) {
        (void)loan.builder_->commit__();  // refused: the slot goes back
        overflow(loan.over_, "", "slot", flux_->slot_size());
        return false;
      }
      return report(loan.builder_->commit__());
    }
#endif
    if (!loan.msg_) {
      return false;
    }
#ifdef FLUX_WRAPPER_CUDA
    if constexpr (detail::kRootData<MsgT>) {
      if (loan.staged_ && !staging_->download(loan.msg_->data.data(), loan.msg_->data.size())) {
        device_error("the device to host copy failed");
        return false;
      }
    }
#endif
    ros_->publish(std::move(loan.msg_));
    return true;
  }

  // For a message already built as a ROS message. The flux path copies it into a slot once.
  auto publish(std::unique_ptr<MsgT> msg) -> bool
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
    return true;
  }

  // For a message the caller keeps. The flux path copies it into a slot once.
  auto publish(const MsgT & msg) -> bool
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
    return true;
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
  auto commit_gpu(Loan<MsgT> & loan) -> bool
  {
    if (!loan.slot_) {
      return false;
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
      return false;
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
  auto publish_host(const MsgT & msg) -> bool
  {
    Loan<MsgT> l = loan();
    if (!l) {
      return false;
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

  auto publish_gpu(const MsgT & msg) -> bool
  {
    Loan<MsgT> l = loan();
    l.set_from(msg);
    // A false Loan, data past the slot among its causes, gives no buffer; commit_gpu() logs why.
    const Buffer b = l.alloc_buffer(msg.data.size());
    if (b && !b.copy_from(msg.data.data(), msg.data.size())) {
      device_error("the host to device copy failed");
      return false;
    }
    return commit_gpu(l);
  }

  auto report(flux::Published p) -> bool
  {
    if (flux::faulted(p) && !fault_logged_) {
      fault_logged_ = true;
      RCLCPP_ERROR(
        logger_, "flux publish on %s failed: %s (slot %u bytes); later failures are not logged",
        flux_->topic_name().c_str(), flux::to_string(p), flux_->slot_size());
    }
    return p == flux::Published::Ok;
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
