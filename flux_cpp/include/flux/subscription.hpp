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
class Context;
}  // namespace rclcpp

namespace flux
{
namespace message_filters
{
template <class Adapter>
class Subscriber;
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

  FrameView take() { return sub_.take(); }
  // With no publisher yet it also waits for one. A negative timeout waits forever. Empty once the
  // node's context is shut down, as rclcpp::wait_for_message, and at once while max_borrow views
  // are held.
  FrameView take_blocking(std::int64_t timeout_ns = -1);

  bool has_callback() const noexcept { return static_cast<bool>(cb_); }
  bool attached() const noexcept { return sub_.attached(); }
  const std::string & topic_name() const noexcept { return topic_; }
  const std::string & signpost_name() const noexcept { return sub_.signpost_name(); }
  const QoS & qos() const noexcept { return sub_.qos(); }
  std::uint64_t lost() const noexcept { return sub_.lost(); }
  // Changes on every re-attach. lost() goes on counting across it.
  std::uint32_t attach_generation() const noexcept { return sub_.attach_generation(); }
  Refused refused() const noexcept { return sub_.refused(); }
  // Releases that could not wait for the stream, each holding its slot; as of the last take.
  std::uint64_t fence_failed() const noexcept { return sub_.fence_failed(); }
  gpu::FenceWait fence_wait() const noexcept { return sub_.fence_wait(); }
  // Bounds each release fence (zero: none); this and the release mode are kept across a re-attach.
  void set_fence_timeout(std::chrono::nanoseconds t) noexcept { sub_.set_fence_timeout(t); }
  void set_release_mode(gpu::ReleaseMode m) noexcept { sub_.set_release_mode(m); }

  bool attach() override;
  bool deliver_one() override;
  const detail::Channel * channel() const override { return sub_.channel(); }
  const detail::Watch * watch() const override { return sub_.watch(); }
  const void * endpoint() const override { return this; }

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
};

}  // namespace flux

#endif  // FLUX_SUBSCRIPTION_HPP
