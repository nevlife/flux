#ifndef FLUX_WRAPPER_SUBSCRIPTION_HPP
#define FLUX_WRAPPER_SUBSCRIPTION_HPP

#include "flux_wrapper/messages.hpp"
#include "flux_wrapper/runtime.hpp"

#include <rclcpp/rclcpp.hpp>

#ifdef USE_FLUX_ENABLED
#include <flux/subscription.hpp>
#endif

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

namespace flux_wrapper
{

struct SubscriptionOptions
{
  // Null: the node's default group. On the flux path the subscription is served by the flux
  // executor that runs this group (flux_wrapper::Executor, a flux component container).
  rclcpp::CallbackGroup::SharedPtr callback_group;
  // Image on the flux path: Cuda keeps the frames of a Device::Cuda publisher on the device
  // (View::on_device()). With Host, a discrete GPU's frames are copied to host memory at take.
  Device device = Device::Host;
};

// One topic over flux or over ROS, chosen once per process by use_flux(). The callback gets a
// View with the message's fields on both paths. On the flux path the View aliases the slot and
// is valid only during the call; copy out (to_msg()) what is kept longer. Keep the Subscription
// alive as long as the executor that serves it, as a member of the node. An Image subscription on
// the flux path also takes the frames of a publisher with Device::Cuda.
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
      auto cb = std::make_shared<Callback>(std::move(callback));
      flux_ = std::make_unique<flux::Subscription>(
        node, topic, Adapter::kFingerprint, q,
        [cb](const flux::FrameView & f) { (*cb)(View<MsgT>(typename Adapter::View(f))); });
      flux_->add_to_callback_group(node, options.callback_group);
      if constexpr (std::is_same_v<MsgT, sensor_msgs::msg::Image>) {
        open_gpu(node, topic, q, std::move(cb), options);
      }
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

  // Frames of a Device::Cuda publisher skipped for a bad aux header; the first is logged.
  [[nodiscard]] auto rejected() const noexcept -> std::uint64_t
  {
#ifdef USE_FLUX_ENABLED
    if (rejects_) {
      return rejects_->count.load(std::memory_order_relaxed);
    }
#endif
    return 0;
  }

private:
#ifdef USE_FLUX_ENABLED
  struct Rejects
  {
    Rejects(rclcpp::Logger l, std::string t) : logger(std::move(l)), topic(std::move(t)) {}

    rclcpp::Logger logger;
    std::string topic;
    std::atomic<std::uint64_t> count{0};
  };

  template <class NodeT>
  void open_gpu(
    NodeT & node, const std::string & topic, const flux::QoS & q, std::shared_ptr<Callback> cb,
    const SubscriptionOptions & options)
  {
    rejects_ = std::make_shared<Rejects>(node.get_logger(), topic);
    auto on_frame = [cb = std::move(cb), r = rejects_](const flux::FrameView & f) {
      detail::GpuImageFields fields;
      if (!detail::decode_gpu_image(f.aux(), fields)) {
        if (r->count.fetch_add(1, std::memory_order_relaxed) == 0) {
          RCLCPP_ERROR(
            r->logger,
            "%s: skipped a GPU image frame with a bad aux header (%zu bytes); later "
            "ones are counted in rejected(), not logged",
            r->topic.c_str(), f.aux().size());
        }
        return;
      }
      (*cb)(View<MsgT>(f, fields));
    };
    if (on_device(node, options.device)) {
      gpu_ = std::make_unique<flux::Subscription>(
        node, topic, detail::kGpuImageFingerprint, q, std::move(on_frame), flux::Device::Cuda);
    } else {
      gpu_ = std::make_unique<flux::Subscription>(
        node, topic, detail::kGpuImageFingerprint, q, std::move(on_frame));
    }
    gpu_->add_to_callback_group(node, options.callback_group);
  }

  // Never throws: a Cuda that cannot be had is Host, with the reason logged once.
  template <class NodeT>
  static auto on_device(NodeT & node, Device requested) -> bool
  {
    if (requested == Device::Host) {
      return false;
    }
#ifdef FLUX_WRAPPER_CUDA
    const flux::gpu::Platform p = flux::gpu::probe();
    if (p.route != flux::gpu::Route::None) {
      return true;
    }
    RCLCPP_WARN_ONCE(
      node.get_logger(), "Device::Cuda: no flux GPU route (%s); using Host", p.reason.c_str());
#else
    RCLCPP_WARN_ONCE(
      node.get_logger(), "Device::Cuda, but flux_wrapper was built without CUDA; using Host");
#endif
    return false;
  }

  std::unique_ptr<flux::Subscription> flux_;
  std::shared_ptr<Rejects> rejects_;
  std::unique_ptr<flux::Subscription> gpu_;
#endif
  typename rclcpp::Subscription<MsgT>::SharedPtr ros_;
};

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_SUBSCRIPTION_HPP
