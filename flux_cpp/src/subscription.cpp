#include "flux/subscription.hpp"

#include "flux/detail/group_member.hpp"
#include "ros_boundary.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>

#include <stdexcept>
#include <utility>

namespace flux
{

Subscription::Subscription(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint, const QoS & qos,
  Callback cb, MemoryPolicy mem)
: Subscription(node, topic, fingerprint, qos, std::move(cb), gpu::Stream{}, mem)
{
}

Subscription::Subscription(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint, const QoS & qos,
  Callback cb, Device device, MemoryPolicy mem)
: Subscription(node, topic, fingerprint, qos, std::move(cb), gpu::stream_for(device), mem)
{
}

Subscription::Subscription(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint, const QoS & qos,
  Callback cb, const gpu::Stream & stream, MemoryPolicy mem)
: topic_(detail::resolve(node, topic)),
  logger_(node.logging()->get_logger_name()),
  context_(node.base()->get_context()),
  sub_(detail::signpost_name(topic_, fingerprint), fingerprint, qos, mem, stream),
  cb_(std::move(cb)),
  announced_(detail::announce(node, sub_.signpost_name(), topic_, false))
{
  detail::trace_endpoint(this, node, topic_, sub_.signpost_name(), fingerprint, false);
  // A volatile subscription counts from here, not from the first wake.
  sub_.attach();
}

void Subscription::add_to_callback_group(
  const NodeRef & node, const std::shared_ptr<rclcpp::CallbackGroup> & group, std::int32_t priority)
{
  if (!cb_) {
    throw std::invalid_argument("flux: a Subscription without a callback has nothing to run");
  }
  if (member_) {
    throw std::logic_error("flux: the subscription is already in a callback group");
  }
  member_ = detail::join_group(node, *this, topic_, group, priority);
}

auto Subscription::take_blocking(std::int64_t timeout_ns) -> FrameView
{
  const auto shut_down = [this] { return !context_->is_valid(); };
  if (shut_down()) {
    return {};
  }
  return sub_.take_blocking(timeout_ns, shut_down);
}

auto Subscription::attach() -> bool
{
  const bool attached = sub_.attach_for_executor();
  report_refusal();
  return attached;
}

auto Subscription::deliver_one() -> bool
{
  FrameView v = sub_.take_for_executor();
  report_refusal();
  if (!v) {
    return false;
  }
  cb_(std::move(v));
  return true;
}

void Subscription::report_refusal()
{
  if (const std::string * why = sub_.new_refusal()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(logger_), "flux: %s is left out until its publisher restarts: %s",
      topic_.c_str(), why->c_str());
  }
}

}  // namespace flux
