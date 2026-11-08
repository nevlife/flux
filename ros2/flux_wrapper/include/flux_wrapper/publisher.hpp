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
  // subscriber (rviz2, ros2 bag) wants it.
  bool mirror_to_ros = false;
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
#ifdef USE_FLUX_ENABLED
    if (use_flux()) {
      const std::uint32_t slot = options.max_message_size != 0 ? options.max_message_size
                                                               : flux::Publisher::kDefaultSlotSize;
      const std::uint32_t count =
        options.slot_count != 0 ? options.slot_count : flux::Publisher::kDefaultSlotCount;
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
    ros_ = rclcpp::create_publisher<MsgT>(node, topic, qos);
  }

  // A Loan that is false had no free flux slot: fill it or not, publish() sends nothing.
  auto loan() -> Loan<MsgT>
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return Loan<MsgT>(typename detail::AdapterT<MsgT>::Builder(flux_->loan()));
    }
#endif
    return Loan<MsgT>(std::make_unique<MsgT>());
  }

  // False when nothing went out: no free slot, a field that did not fit, or a fault (logged).
  auto publish(Loan<MsgT> && loan) -> bool
  {
#ifdef USE_FLUX_ENABLED
    if (loan.builder_) {
      return report(loan.builder_->commit__());
    }
#endif
    if (!loan.msg_) {
      return false;
    }
    ros_->publish(std::move(loan.msg_));
    return true;
  }

  // For a message already built as a ROS message. The flux path copies it into a slot once.
  auto publish(std::unique_ptr<MsgT> msg) -> bool
  {
#ifdef USE_FLUX_ENABLED
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

  // flux keeps no subscriber count, so the flux path reports 1: a driver that skips work while
  // nobody listens keeps publishing.
  [[nodiscard]] auto get_subscription_count() const -> std::size_t
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return 1;
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

private:
#ifdef USE_FLUX_ENABLED
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
  rclcpp::Logger logger_;
  typename rclcpp::Publisher<MsgT>::SharedPtr ros_;
};

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_PUBLISHER_HPP
