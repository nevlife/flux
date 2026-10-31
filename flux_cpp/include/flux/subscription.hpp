#ifndef FLUX_SUBSCRIPTION_HPP
#define FLUX_SUBSCRIPTION_HPP

#include "flux/channel.hpp"
#include "flux/detail/executor.hpp"
#include "flux/detail/manifest.hpp"
#include "flux/detail/subscriber.hpp"
#include "flux/node_ref.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace rclcpp
{
class CallbackGroup;
class Context;
}  // namespace rclcpp

namespace flux
{
namespace message_filters
{
template <class Adapter>
class Subscriber;
}
namespace detail
{
class GroupMember;
}

// A view passed to the callback holds its slot only for that call: copy out what is kept longer.
// One thread at a time: a second thread's call throws std::logic_error. A topic takes as many
// subscribing processes as its publisher allows (16 unless set); the subscriptions of one process
// share one place. Under an executor, a subscription refused when it attaches is logged once and
// left out until its publisher restarts; the other sources keep running.
class Subscription : public detail::Source
{
public:
  using Callback = std::function<void(const FrameView &)>;

  // Throws on a QoS the channel would refuse, before anything is announced.
  Subscription(
    const NodeRef & node, const std::string & topic, std::uint64_t fingerprint = kNoSchema,
    const QoS & qos = QoS{}, Callback cb = {}, MemoryPolicy mem = {});
  // Frames consumed on the GPU; a device payload takes no other kind of subscription.
  Subscription(
    const NodeRef & node, const std::string & topic, std::uint64_t fingerprint, const QoS & qos,
    Callback cb, Device device, MemoryPolicy mem = {});
  Subscription(
    const NodeRef & node, const std::string & topic, std::uint64_t fingerprint, const QoS & qos,
    Callback cb, const gpu::Stream & stream, MemoryPolicy mem = {});

  auto take() -> FrameView { return sub_.take(); }
  // With no publisher yet it also waits for one. A negative timeout waits forever. Empty once the
  // node's context is shut down, as rclcpp::wait_for_message, and at once while max_borrow views
  // are held.
  auto take_blocking(std::int64_t timeout_ns = -1) -> FrameView;

  // Served by the executor serving `group` (null: default group), which it outlives. Not add()ed.
  void add_to_callback_group(
    const NodeRef & node, const std::shared_ptr<rclcpp::CallbackGroup> & group = nullptr,
    std::int32_t priority = 0);

  [[nodiscard]] auto has_callback() const noexcept -> bool { return static_cast<bool>(cb_); }
  [[nodiscard]] auto attached() const noexcept -> bool { return sub_.attached(); }
  [[nodiscard]] auto topic_name() const noexcept -> const std::string & { return topic_; }
  [[nodiscard]] auto signpost_name() const noexcept -> const std::string &
  {
    return sub_.signpost_name();
  }
  [[nodiscard]] auto qos() const noexcept -> const QoS & { return sub_.qos(); }
  [[nodiscard]] auto lost() const noexcept -> std::uint64_t { return sub_.lost(); }
  // Changes on every re-attach. lost() goes on counting across it.
  [[nodiscard]] auto attach_generation() const noexcept -> std::uint32_t
  {
    return sub_.attach_generation();
  }
  [[nodiscard]] auto refused() const noexcept -> Refused { return sub_.refused(); }
  // Releases that could not wait for the stream, each holding its slot; as of the last take.
  [[nodiscard]] auto fence_failed() const noexcept -> std::uint64_t { return sub_.fence_failed(); }
  [[nodiscard]] auto fence_wait() const noexcept -> gpu::FenceWait { return sub_.fence_wait(); }
  // Bounds each release fence (zero: none); this and the release mode are kept across a re-attach.
  void set_fence_timeout(std::chrono::nanoseconds t) noexcept { sub_.set_fence_timeout(t); }
  void set_release_mode(gpu::ReleaseMode m) noexcept { sub_.set_release_mode(m); }

  auto attach() -> bool override;
  auto deliver_one() -> bool override;
  [[nodiscard]] auto channel() const -> const detail::Channel * override { return sub_.channel(); }
  [[nodiscard]] auto watch() const -> const detail::Watch * override { return sub_.watch(); }
  [[nodiscard]] auto endpoint() const -> const void * override { return this; }

private:
  template <class Adapter>
  friend class message_filters::Subscriber;

  void report_refusal();

  std::string topic_;
  std::string logger_;
  std::shared_ptr<rclcpp::Context> context_;
  detail::Subscriber sub_;
  Callback cb_;
  detail::Announcement announced_;
  // Last, so it leaves the group first.
  std::shared_ptr<detail::GroupMember> member_;
};

}  // namespace flux

#endif  // FLUX_SUBSCRIPTION_HPP
