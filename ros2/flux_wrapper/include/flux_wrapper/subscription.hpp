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
#include <utility>

namespace flux_wrapper
{

struct SubscriptionOptions
{
  // Null: the node's default group. On the flux path the subscription is served by the flux
  // executor that runs this group (flux_wrapper::Executor, a flux component container).
  rclcpp::CallbackGroup::SharedPtr callback_group;
  // Cuda: every frame's data field is in device memory (View::on_device()), whatever the
  // publisher's device. Where the GPU cannot read a host publisher's frame in place, flux copies it
  // to the device at take; on the ROS path each message is copied to one device buffer of the
  // subscription's, so its callbacks must not overlap (no Reentrant group). With Host, frames of a
  // Device::Cuda publisher on a discrete GPU are copied to host memory at take.
  Device device = Device::Host;
};

// One topic over flux or over ROS, chosen once per process by use_flux(). The callback gets a
// View with the message's fields on both paths. On the flux path the View aliases the slot and
// is valid only during the call; copy out (to_msg()) what is kept longer. Keep the Subscription
// alive as long as the executor that serves it, as a member of the node. On the flux path it also
// takes the frames of a publisher with Device::Cuda.
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
    [[maybe_unused]] const bool cuda =
      detail::cuda_usable(node.get_logger(), topic, options.device);
#ifdef USE_FLUX_ENABLED
    if (use_flux()) {
      flux::QoS q(static_cast<std::uint32_t>(qos.depth()));
      if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
        q.transient_local(static_cast<std::uint32_t>(qos.depth()));
      }
      using Adapter = detail::AdapterT<MsgT>;
      const flux::Device device = cuda ? flux::Device::Cuda : flux::Device::Cpu;
      auto cb = std::make_shared<Callback>(std::move(callback));
      if (cuda) {
        flux_ = std::make_unique<flux::Subscription>(
          node, topic, Adapter::kFingerprint, q,
          [cb](const flux::FrameView & f) {
            const typename Adapter::View v(f);
            (*cb)(View<MsgT>(v, detail::device_at(f, v.data().data()), f.stream()));
          },
          device);
      } else {
        flux_ = std::make_unique<flux::Subscription>(
          node, topic, Adapter::kFingerprint, q,
          [cb](const flux::FrameView & f) { (*cb)(View<MsgT>(typename Adapter::View(f))); });
      }
      flux_->add_to_callback_group(node, options.callback_group);
      open_gpu(node, topic, q, std::move(cb), options.callback_group, device);
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
#ifdef FLUX_WRAPPER_CUDA
    if (cuda) {
      std::shared_ptr<detail::DeviceStaging> staging = detail::DeviceStaging::create();
      if (staging) {
        ros_ = rclcpp::create_subscription<MsgT>(
          node, topic, qos,
          [cb = std::move(callback), staging, logger = node.get_logger(), topic,
           logged =
             std::make_shared<std::atomic<bool>>(false)](const typename MsgT::ConstSharedPtr & m) {
            const void * d = staging->upload(m->data.data(), m->data.size());
            if (d == nullptr) {
              if (!logged->exchange(true, std::memory_order_relaxed)) {
                RCLCPP_ERROR(
                  logger,
                  "%s: Device::Cuda: a message could not be copied to the device and was "
                  "skipped; later ones are not logged",
                  topic.c_str());
              }
              return;
            }
            cb(View<MsgT>(*m, d, staging->stream()));
          },
          ros_options);
        return;
      }
      RCLCPP_WARN(
        node.get_logger(), "%s: Device::Cuda: no CUDA stream could be created; using Host",
        topic.c_str());
    }
#endif
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
    const rclcpp::CallbackGroup::SharedPtr & group, flux::Device device)
  {
    rejects_ = std::make_shared<Rejects>(node.get_logger(), topic);
    auto on_frame = [cb = std::move(cb), r = rejects_](const flux::FrameView & f) {
      typename detail::Gpu<MsgT>::Fields fields;
      if (!detail::decode_gpu(f.aux(), fields)) {
        if (r->count.fetch_add(1, std::memory_order_relaxed) == 0) {
          RCLCPP_ERROR(
            r->logger,
            "%s: skipped a GPU frame with a bad aux header (%zu bytes); later ones are counted "
            "in rejected(), not logged",
            r->topic.c_str(), f.aux().size());
        }
        return;
      }
      (*cb)(View<MsgT>(f, fields));
    };
    gpu_ = std::make_unique<flux::Subscription>(
      node, topic, detail::Gpu<MsgT>::kFingerprint, q, std::move(on_frame), device);
    gpu_->add_to_callback_group(node, group);
  }

  std::unique_ptr<flux::Subscription> flux_;
  std::shared_ptr<Rejects> rejects_;
  std::unique_ptr<flux::Subscription> gpu_;
#endif
  typename rclcpp::Subscription<MsgT>::SharedPtr ros_;
};

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_SUBSCRIPTION_HPP
