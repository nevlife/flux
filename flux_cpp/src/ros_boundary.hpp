#ifndef FLUX_ROS_BOUNDARY_HPP
#define FLUX_ROS_BOUNDARY_HPP

// What the ROS boundary contributes on top of flux_core, shared by Publisher and Subscription so
// the two ends of one contract cannot drift apart.

#include <rclcpp/node.hpp>

#include <string>

namespace flux::ros::detail
{

// The signpost is keyed by the resolved topic name: remapped and fully qualified.
std::string resolve(rclcpp::Node & node, const std::string & topic);

// Enumeration metadata, not a registration: nothing in the data plane reads it, and a failure to
// write it is swallowed. The label is what a tool shows as the owner, so it is the node's fully
// qualified name; core has no notion of a node and never looks inside.
void announce(
  rclcpp::Node & node, const std::string & signpost, const std::string & key, bool publisher);

}  // namespace flux::ros::detail

#endif  // FLUX_ROS_BOUNDARY_HPP
