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
#include <memory>
#include <string>
#include <utility>

namespace flux_wrapper
{

struct PublisherOptions
{
  // flux only: the largest message in bytes. Zero takes the flux default slot size (8 MiB).
  std::uint32_t max_message_size = 0;
  // flux only: messages a subscriber may hold at once, plus one to write into. Zero takes the
  // flux default (16). The topic reserves max_message_size * slot_count of shared memory.
  std::uint32_t slot_count = 0;
  // flux only: also publish each message as its ROS message on the same topic while a ROS
  // subscriber (rviz2, ros2 bag) wants it. Not with Device::Cuda.
  bool mirror_to_ros = false;
  // Cuda: Loan::alloc_buffer() hands out device memory (see Device). On the flux path a slot of a
  // discrete GPU is device memory; on the ROS path publish() copies the buffer to the message.
  Device device = Device::Host;
};

// One topic over flux or over ROS, chosen once per process by use_flux(). Writing a message is
// the same on both paths: loan(), fill the Loan, publish(). One thread at a time.
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
    [[maybe_unused]] const bool cuda = detail::cuda_usable(logger_, topic, options.device);
#ifdef FLUX_WRAPPER_CUDA
    requested_cuda_ = options.device == Device::Cuda;
#endif
#ifdef USE_FLUX_ENABLED
    if (use_flux()) {
      const std::uint32_t slot = options.max_message_size != 0 ? options.max_message_size
                                                               : flux::Publisher::kDefaultSlotSize;
      const std::uint32_t count =
        options.slot_count != 0 ? options.slot_count : flux::Publisher::kDefaultSlotCount;
      if (cuda) {
        flux_ = std::make_unique<flux::Publisher>(
          node, topic, detail::Gpu<MsgT>::kFingerprint, slot, count, flux::Device::Cuda,
          flux::MemoryPolicy{}, flux::Publisher::kDefaultMaxSubscribers,
          static_cast<std::uint32_t>(sizeof(typename detail::Gpu<MsgT>::Aux)));
        device_ = true;
        if (options.mirror_to_ros) {
          RCLCPP_WARN(
            logger_,
            "%s: mirror_to_ros is ignored with Device::Cuda: flux does not mirror a GPU "
            "channel",
            topic.c_str());
        }
        return;
      }
      flux_ = std::make_unique<flux::Publisher>(
        node, topic, detail::AdapterT<MsgT>::kFingerprint, slot, count);
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
    if (loan.gpu_) {
      return commit_gpu(loan);
    }
    if (loan.builder_) {
      return report(loan.builder_->commit__());
    }
#endif
    if (!loan.msg_) {
      return false;
    }
#ifdef FLUX_WRAPPER_CUDA
    if (loan.staged_ && !staging_->download(loan.msg_->data.data(), loan.msg_->data.size())) {
      device_error("the device to host copy failed");
      return false;
    }
#endif
    ros_->publish(std::move(loan.msg_));
    return true;
  }

  // For a message already built as a ROS message. The flux path copies it into a slot once.
  auto publish(std::unique_ptr<MsgT> msg) -> bool
  {
#ifdef USE_FLUX_ENABLED
    if (flux_ && device_) {
      return publish_gpu(*msg);
    }
    if (flux_) {
      auto b = detail::AdapterT<MsgT>::build__(*flux_);
      if (!b) {
        return false;
      }
      sensor_msgs::flux_msg::msg_to_frame(*msg, b);
      return report(b.commit__());
    }
#endif
    ros_->publish(std::move(msg));
    return true;
  }

  // For a message the caller keeps. The flux path copies it into a slot once.
  auto publish(const MsgT & msg) -> bool
  {
#ifdef USE_FLUX_ENABLED
    if (flux_ && device_) {
      return publish_gpu(msg);
    }
    if (flux_) {
      auto b = detail::AdapterT<MsgT>::build__(*flux_);
      if (!b) {
        return false;
      }
      sensor_msgs::flux_msg::msg_to_frame(msg, b);
      return report(b.commit__());
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
    loan.aux_ = nullptr;
    if (loan.bad_) {
      loan.slot_.abort();
      device_error(detail::Gpu<MsgT>::kOverCap);
      return false;
    }
    loan.slot_.set_aux_len(sizeof(typename detail::Gpu<MsgT>::Aux));
    return report(loan.slot_.commit(loan.len_));
  }

  auto publish_gpu(const MsgT & msg) -> bool
  {
    Loan<MsgT> l = loan();
    l.set_from(msg);
    const Buffer b = l.alloc_buffer(msg.data.size());
    if (l.slot_ && !b) {
      return report(flux::Published::TooLarge);
    }
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

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_PUBLISHER_HPP
