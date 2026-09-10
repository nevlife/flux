#include "ros_boundary.hpp"

#include "flux/owner.hpp"

namespace flux::ros::detail
{

void announce(
  rclcpp::Node & node, const std::string & signpost, const std::string & key, bool publisher)
{
  flux::ManifestEntry e;
  e.signpost = signpost;
  e.key = key;
  e.label = node.get_fully_qualified_name();
  e.publisher = publisher;
  flux::OwnerFile::announce(e);
}

}  // namespace flux::ros::detail
