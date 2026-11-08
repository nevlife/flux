#ifndef FLUX_WRAPPER_SUBSCRIPTION_HPP
#define FLUX_WRAPPER_SUBSCRIPTION_HPP

#include "flux_wrapper/messages.hpp"
#include "flux_wrapper/runtime.hpp"

#include <rclcpp/rclcpp.hpp>

#ifdef USE_FLUX_ENABLED
#include <flux/subscription.hpp>
#endif

#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace flux_wrapper
{

struct SubscriptionOptions
{
  // Null: the node's default group. On the flux path the subscription is served by the flux
  // executor that runs this group (flux_wrapper::Executor, a flux component container).
  rclcpp::CallbackGroup::SharedPtr callback_group;
};

// One topic over flux or over ROS, chosen once per process by use_flux(). The callback gets a
// View with the message's fields on both paths. On the flux path the View aliases the slot and
// is valid only during the call; copy out (to_msg()) what is kept longer. Keep the Subscription
// alive as long as the executor that serves it, as a member of the node.
template <class MsgT>
class Subscription
{
public:
  using Callback = std::function<void(const View<MsgT> &)>;

  template <class NodeT>
  Subscription(
    NodeT & node, const std::string & topic, const rclcpp::QoS & qos, Callback callback,
    const SubscriptionOptions & options = {})
  {
#ifdef USE_FLUX_ENABLED
    if (use_flux()) {
      flux::QoS q(static_cast<std::uint32_t>(qos.depth()));
      if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
        q.transient_local(static_cast<std::uint32_t>(qos.depth()));
      }
      using Adapter = detail::AdapterT<MsgT>;
      flux_ = std::make_unique<flux::Subscription>(
        node, topic, Adapter::kFingerprint, q,
        [cb = std::move(callback)](const flux::FrameView & f) {
          cb(View<MsgT>(typename Adapter::View(f)));
        });
      flux_->add_to_callback_group(node, options.callback_group);
      return;
    }
#else
    if (flux_requested()) {
      RCLCPP_WARN_ONCE(
        node.get_logger(), "ENABLE_FLUX=1, but flux_wrapper was built without flux: ROS path");
    }
#endif
    rclcpp::SubscriptionOptions ros_options;
    ros_options.callback_group = options.callback_group;
    ros_ = rclcpp::create_subscription<MsgT>(
      node, topic, qos,
      [cb = std::move(callback)](const typename MsgT::ConstSharedPtr & m) { cb(View<MsgT>(*m)); },
      ros_options);
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
  std::unique_ptr<flux::Subscription> flux_;
#endif
  typename rclcpp::Subscription<MsgT>::SharedPtr ros_;
};

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_SUBSCRIPTION_HPP
