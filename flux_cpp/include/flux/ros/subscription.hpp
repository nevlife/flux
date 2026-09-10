#ifndef FLUX_ROS_SUBSCRIPTION_HPP
#define FLUX_ROS_SUBSCRIPTION_HPP

#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/executor.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace rclcpp
{
class Node;
}

namespace flux::ros
{

// Subscriber side of a flux channel: the one object that holds a flux subscription, the way
// rclcpp::Subscription does for ROS. Attaches lazily (the publisher may start later). The
// FrameView holds the borrow only for the duration of one call; copy out anything kept longer.
//
// Two ways to read it, and the callback decides which. With one, hand this to a
// flux::ros::Executor or a PartitionedExecutor and the callback runs per frame on their
// spin thread. Without one, read it yourself with peek()/take()/take_blocking() on whatever
// schedule you already have.
//
// It never drives itself. A wall-timer path used to exist and was removed: it woke on a period
// rather than on a publish, so every subscription paid a latency bound and an idle wakeup that
// the futex does not.
//
// `qos` decides which frames are delivered (docs/en/qos.en.md): depth == 1 yields the newest only,
// depth == N drains up to N per wake in publish order, durability whether the ring's backlog is
// replayed on attach. Throws std::invalid_argument if the QoS is not honourable.
class Subscription : public flux::Source
{
public:
  using Callback = std::function<void(const FrameView &)>;

  // `mem` is opt-in page residency for this subscriber's own mapping. It is
  // per-process: a publisher that committed its pages says nothing about this mapping, and it is
  // re-applied on every re-attach.
  Subscription(
    rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint = kNoSchema,
    Callback cb = {}, const QoS & qos = QoS{}, Device device = Device::Cpu,
    const MemoryPolicy & mem = {});
  ~Subscription() override;

  Subscription(const Subscription &) = delete;
  Subscription & operator=(const Subscription &) = delete;

  // ---- pull surface: read on your own schedule, no executor ----

  // The newest frame without consuming it. The same frame comes back until something new is
  // published; invalid only if nothing ever was. Attaches on the way in.
  FrameView peek();

  // The next frame in publish order, consumed. Invalid once caught up. qos().depth bounds how
  // far behind this may fall; frames dropped by that window or lapped by the ring count in lost().
  FrameView take();

  // take(), parking on the futex until a frame arrives or `timeout_ns` elapses (negative =
  // forever). Near-zero CPU. Prefer this over polling take().
  FrameView take_blocking(std::int64_t timeout_ns = -1);

  // ---- state ----

  bool has_callback() const noexcept { return static_cast<bool>(cb_); }
  bool attached() const noexcept { return ch_.has_value(); }
  // The fixed rendezvous name (the signpost), not the segment it
  // currently points at.
  const std::string & segment_name() const noexcept { return seg_name_; }

  // This process's domain, the one this subscription searched.
  // attached() says whether a publisher was found; this says where it looked. Apart, they
  // separate "no publisher yet" from "a publisher exists, in a domain this node is not looking
  // in" -- one frameless subscription otherwise.
  const std::string & domain() const noexcept { return flux::process_domain(); }
  const QoS & qos() const noexcept { return qos_; }

  // Frames never delivered to this subscription: lapped by the ring, or dropped by the depth
  // window. Cumulative.
  std::uint64_t lost() const noexcept { return ch_ ? ch_->lost() : 0; }

  // False once max_borrow views are held. An empty frame with can_borrow() false is this
  // subscription holding its own leases, not an idle stream, and no publish can clear it.
  bool can_borrow() const noexcept { return ch_ ? ch_->can_borrow() : true; }

  // Why an empty frame came back when one was not simply absent (docs/en/qos.en.md 1). Cumulative.
  Channel::Refused refused() const noexcept { return ch_ ? ch_->refused() : Channel::Refused{}; }

  // Borrows not released because the declared stream could not be waited on (docs/en/qos.en.md 1).
  // Each one costs a slot that is never reused. Always 0 with Device::Cpu.
  std::uint64_t fence_failed() const noexcept { return ch_ ? ch_->fence_failed() : 0; }

  // False when the frames this subscription hands out live in GPU memory, so a reader must use
  // a kernel through device_ptr() rather than host loads through data().
  bool host_addressable() const noexcept { return ch_ ? ch_->host_addressable() : true; }

  // The stream a consuming kernel must be launched on -- the one a release waits for. Undeclared
  // on a host subscription.
  const gpu::Stream & stream() const noexcept { return stream_; }

  // How long dropping a view blocked waiting on the declared stream. On the callback
  // path that wait lands on the executor's spin thread.
  Channel::FenceWait fence_wait() const noexcept
  {
    return ch_ ? ch_->fence_wait() : Channel::FenceWait{};
  }

  // What the declared MemoryPolicy got for the current mapping. False while unattached.
  bool pages_committed() const noexcept { return ch_ && ch_->pages_committed(); }
  bool pages_locked() const noexcept { return ch_ && ch_->pages_locked(); }

  // ---- flux::Source: what an executor drives it through ----
  bool attach() override;
  int deliver_one() override;
  Channel * channel() noexcept override { return ch_ ? &*ch_ : nullptr; }

private:
  std::string seg_name_;
  std::uint64_t fingerprint_;
  Callback cb_;
  QoS qos_;
  // Declared once and reused across attaches: the declaration belongs to this subscription, not
  // to whichever segment it is currently attached to.
  gpu::Stream stream_;
  MemoryPolicy mem_;  // same reason as stream_: declared here, re-applied per attach
  std::optional<Channel> ch_;
};

}  // namespace flux::ros

#endif  // FLUX_ROS_SUBSCRIPTION_HPP
