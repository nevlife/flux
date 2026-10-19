#include "flux/publisher.hpp"

#include "ros_boundary.hpp"

namespace flux
{

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, MemoryPolicy mem,
  std::uint32_t max_subscribers)
: Publisher(node, topic, fingerprint, slot_size, slot_count, gpu::Stream{}, mem, max_subscribers)
{
}

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, Device device, MemoryPolicy mem,
  std::uint32_t max_subscribers)
: Publisher(
    node, topic, fingerprint, slot_size, slot_count, gpu::stream_for(device), mem, max_subscribers)
{
}

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, const gpu::Stream & stream, MemoryPolicy mem,
  std::uint32_t max_subscribers)
: topic_(detail::resolve(node, topic)),
  signpost_(detail::signpost_name(topic_, fingerprint)),
  ch_(detail::Channel::create(
    signpost_, slot_size, slot_count, fingerprint, mem, max_subscribers, stream)),
  announced_(detail::announce(node, signpost_, topic_, true))
{
  detail::trace_endpoint(this, node, topic_, signpost_, fingerprint, true);
}

}  // namespace flux
