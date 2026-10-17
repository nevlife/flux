#ifndef FLUX2_ROS_BOUNDARY_HPP
#define FLUX2_ROS_BOUNDARY_HPP

#include "flux2/detail/manifest.hpp"
#include "flux2/trace.h"

#include <rclcpp/node.hpp>

#include <cstdint>
#include <string>

namespace flux2::detail
{

// Remapped and fully qualified, so every node names a topic the same way.
inline std::string resolve(rclcpp::Node & node, const std::string & topic)
{
  return node.get_node_topics_interface()->resolve_topic_name(topic);
}

// The roster label is the node's fully qualified name: armo reads a label starting with '/' as
// the node (trace contract 2).
inline Announcement announce(
  rclcpp::Node & node, const std::string & signpost, const std::string & topic, bool publisher)
{
  return Announcement(signpost, topic, node.get_fully_qualified_name(), publisher);
}

inline void trace_endpoint(
  const void * endpoint, rclcpp::Node & node, const std::string & topic,
  const std::string & signpost, std::uint64_t fingerprint, bool publisher)
{
  flux2_trace_endpoint_init(
    endpoint, node.get_fully_qualified_name(), topic.c_str(), signpost.c_str(), fingerprint,
    publisher ? FLUX2_TRACE_ROLE_PUBLISHER : FLUX2_TRACE_ROLE_SUBSCRIPTION, FLUX2_TRACE_LANG_CPP);
}

}  // namespace flux2::detail

#endif  // FLUX2_ROS_BOUNDARY_HPP
