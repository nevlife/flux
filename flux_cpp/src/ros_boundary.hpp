#ifndef FLUX_ROS_BOUNDARY_HPP
#define FLUX_ROS_BOUNDARY_HPP

// What the ROS boundary contributes on top of flux_core: the enumeration label it announces.
// Publisher and Subscription both need it and had identical copies; a copy is where the two ends
// of one contract drift apart. Domain resolution used to live here too and now does not -- core
// names the inherited variable itself, so a boundary copy of that rule is exactly the divergence
// core closes.

#include <rclcpp/node.hpp>

#include <string>

namespace flux::ros::detail
{

// Enumeration metadata, not a registration: nothing in the data plane reads it, and a failure to
// write it is swallowed. The label is what a tool shows as the owner, so it
// is the node's fully qualified name -- core has no notion of a node and never looks inside.
void announce(
  rclcpp::Node & node, const std::string & signpost, const std::string & key, bool publisher);

}  // namespace flux::ros::detail

#endif  // FLUX_ROS_BOUNDARY_HPP
