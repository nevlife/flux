#include "flux/ros/publisher.hpp"

#include "flux/discovery.hpp"
#include "flux/owner.hpp"
#include "flux/segment.hpp"
#include "ros_boundary.hpp"

#include <rclcpp/node.hpp>

namespace flux::ros
{

namespace
{
std::string resolve(rclcpp::Node & node, const std::string & topic)
{
  return node.get_node_topics_interface()->resolve_topic_name(topic);
}
}  // namespace

Publisher::Publisher(
  rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, Device device, const MemoryPolicy & mem)
: seg_name_(flux::signpost_name(resolve(node, topic), fingerprint)),
  ch_(
    open_publisher_segment(seg_name_, slot_size, slot_count, fingerprint, device),
    gpu::stream_for(device), mem)
{
  detail::announce(node, seg_name_, resolve(node, topic), /*publisher=*/true);
}

Published Publisher::publish(const void * data, std::size_t nbytes) noexcept
{
  return ch_.publish(data, nbytes);
}

Published Publisher::publish(
  const void * data, DType dt, std::initializer_list<std::uint64_t> shape) noexcept
{
  return ch_.publish(data, dt, shape);
}

WriteSlot Publisher::loan() noexcept
{
  return ch_.loan();
}

WriteSlot Publisher::loan(DType dt, std::initializer_list<std::uint64_t> shape) noexcept
{
  return ch_.loan(dt, shape);
}

std::uint64_t Publisher::dropped() const noexcept
{
  return ch_.dropped();
}

}  // namespace flux::ros
