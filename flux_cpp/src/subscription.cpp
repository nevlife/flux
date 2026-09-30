#include "flux/ros/subscription.hpp"

#include "flux/discovery.hpp"
#include "flux/owner.hpp"
#include "flux/segment.hpp"
#include "ros_boundary.hpp"

#include <rclcpp/node.hpp>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>

namespace flux::ros
{

Subscription::Subscription(
  rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint, const QoS & qos,
  Callback cb, Device device, const MemoryPolicy & mem)
: topic_(detail::resolve(node, topic)),
  seg_name_(flux::signpost_name(topic_, fingerprint)),
  fingerprint_(fingerprint),
  cb_(std::move(cb)),
  qos_(qos),
  stream_(gpu::stream_for(device)),  // same reason as qos_.validate(): refuse here, not later
  mem_(mem)
{
  qos_.validate();  // fail at construction, not inside a callback, and before announcing
  announced_ = detail::announce(node, seg_name_, topic_, /*publisher=*/false);
  attach();  // volatile counts from where this subscription joined, not from the first wake
}

Subscription::~Subscription() = default;

FrameView Subscription::peek()
{
  if (!attach()) return FrameView{};
  return ch_->peek();
}

FrameView Subscription::take()
{
  if (!attach()) return FrameView{};
  FrameView v = ch_->take();
  // Same orphan check deliver() makes: a dead stream's mapping is dropped so a returning
  // publisher is picked up by the next call instead of never.
  if (!v && ch_->orphaned()) ch_.reset();
  return v;
}

FrameView Subscription::take_blocking(std::int64_t timeout_ns)
{
  // Before the publisher exists there is no wake word to park on, so look for it at the same
  // cadence an executor does, until the deadline.
  constexpr auto kAttachPoll = std::chrono::milliseconds(100);
  const bool infinite = timeout_ns < 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::nanoseconds(timeout_ns);
  while (!attach()) {
    auto park = std::chrono::steady_clock::duration(kAttachPoll);
    if (!infinite) {
      const auto rel = deadline - std::chrono::steady_clock::now();
      if (rel <= std::chrono::steady_clock::duration::zero()) return FrameView{};
      park = std::min(park, rel);
    }
    std::this_thread::sleep_for(park);
  }
  if (!infinite) {
    const auto rel = deadline - std::chrono::steady_clock::now();
    timeout_ns =
      std::max<std::int64_t>(0, std::chrono::duration_cast<std::chrono::nanoseconds>(rel).count());
  }
  FrameView v = ch_->take_blocking(timeout_ns);
  if (!v && ch_->orphaned()) ch_.reset();
  return v;
}

bool Subscription::attach()
{
  if (ch_) return true;
  if (!flux::read_channel_stats(seg_name_).live) return false;  // no publisher yet; retry later
  try {
    // Channel::open, not open_subscriber_segment: only the former records the signpost
    // name + epoch, and without those a publisher restart is never detected.
    ch_.emplace(Channel::open(seg_name_, fingerprint_, stream_, mem_));
  } catch (const SegmentMismatch &) {
    throw;  // wrong fingerprint/version/config: retrying can never fix it, so say so
  } catch (const std::system_error &) {
    throw;  // a refused MemoryPolicy: reported, never downgraded to an unattached retry
  } catch (const std::runtime_error &) {
    // Transient only if the publisher left meanwhile; still live, no retry fixes it.
    if (flux::read_channel_stats(seg_name_).live) throw;
    return false;
  }
  ch_->qos(qos_);
  return true;
}

int Subscription::deliver_one()
{
  if (!ch_) return 0;
  FrameView v = ch_->take();  // v is released before the next call, so max_borrow == 1 delivers
  if (!v) {
    if (ch_->orphaned()) ch_.reset();  // dead stream: drop the mapping, re-attach lazily
    return 0;
  }
  if (cb_) cb_(v);
  return 1;
}

}  // namespace flux::ros
