#include "flux/detail/group_member.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/node_waitables_interface.hpp>

#include <utility>

namespace flux::detail
{

auto GroupMember::in_flux_executor() noexcept -> bool &
{
  thread_local bool inside = false;
  return inside;
}

// A flux executor reaches here through rclcpp's own wait set too, and bridges the member at its
// next rescan.
void GroupMember::refuse_outside_flux()
{
  if (served_ || in_flux_executor() || warned_.exchange(true)) {
    return;
  }
  RCLCPP_ERROR(
    rclcpp::get_logger("flux"),
    "flux: %s is in a callback group that an rclcpp executor serves, so no frame of it is "
    "delivered; serve the node with flux::Executor, flux::PartitionedExecutor or "
    "flux_components' component_container",
    topic_.c_str());
}

auto join_group(
  const NodeRef & node, Source & src, const std::string & topic,
  const rclcpp::CallbackGroup::SharedPtr & group,
  std::int32_t priority) -> std::shared_ptr<GroupMember>
{
  auto member = std::make_shared<GroupMember>(src, priority, topic);
  node.waitables()->add_waitable(member, group);
  return member;
}

}  // namespace flux::detail
