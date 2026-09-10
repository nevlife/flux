#include "flux/ros/subscription.hpp"

#include "flux/discovery.hpp"
#include "flux/owner.hpp"
#include "flux/segment.hpp"
#include "ros_boundary.hpp"

#include <rclcpp/node.hpp>

#include <utility>

namespace flux::ros
{

namespace
{
std::string resolve(rclcpp::Node & node, const std::string & topic)
{
  return node.get_node_topics_interface()->resolve_topic_name(topic);
}

}  // namespace

Subscription::Subscription(
  rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint, Callback cb,
  const QoS & qos, Device device, const MemoryPolicy & mem)
: seg_name_(flux::signpost_name(resolve(node, topic), fingerprint)),
  fingerprint_(fingerprint),
  cb_(std::move(cb)),
  qos_(qos),
  stream_(gpu::stream_for(device)),  // same reason as qos_.validate(): refuse here, not later
  mem_(mem)
{
  detail::announce(node, seg_name_, resolve(node, topic), /*publisher=*/false);
  qos_.validate();  // fail at construction, not inside a callback
  attach();  // join the stream here if the publisher is already up: volatile is measured from
             // where this subscription joined, not from the first wake
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
  if (!attach()) return FrameView{};
  FrameView v = ch_->take_blocking(timeout_ns);
  if (!v && ch_->orphaned()) ch_.reset();
  return v;
}

bool Subscription::attach()
{
  if (ch_) return true;
  try {
    // Channel::open, not open_subscriber_segment: only the former records the signpost
    // name + epoch, and without those a publisher restart is never detected.
    ch_.emplace(Channel::open(seg_name_, fingerprint_, stream_, mem_));
  } catch (const SegmentMismatch &) {
    throw;  // wrong fingerprint/version/config: retrying can never fix it, so say so
  } catch (...) {
    return false;  // publisher segment not up yet; retry on the next call
  }
  ch_->qos(qos_);  // validated in the constructor, so this cannot throw here
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
