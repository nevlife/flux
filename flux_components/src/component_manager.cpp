#include "flux/component_manager.hpp"

#include <utility>

namespace flux
{

ComponentManager::ComponentManager(std::string node_name, const rclcpp::NodeOptions & options)
: rclcpp_components::ComponentManager({}, std::move(node_name), options)
{
  ex_.on_thread_start(
    [this](const NodeBase::SharedPtr & node, const auto & group) { on_thread_start(node, group); });
  ex_.add_node(get_node_base_interface());
}

void ComponentManager::spin()
{
  ex_.spin();
}

void ComponentManager::cancel() noexcept
{
  ex_.cancel();
}

void ComponentManager::on_node_added(const NodeBase::SharedPtr &)
{
}

void ComponentManager::on_thread_start(
  const NodeBase::SharedPtr &, const rclcpp::CallbackGroup::SharedPtr &)
{
}

// rclcpp_components reports only its own exception type as a failed load, and leaves the node in
// node_wrappers_ when this throws.
void ComponentManager::add_node_to_executor(std::uint64_t node_id)
{
  const NodeBase::SharedPtr node = node_wrappers_[node_id].get_node_base_interface();
  try {
    on_node_added(node);
    ex_.add_node(node);
  } catch (const std::exception & e) {
    node_wrappers_.erase(node_id);
    throw rclcpp_components::ComponentManagerException(
      "Component refused by " + std::string(get_fully_qualified_name()) + ": " + e.what());
  }
}

void ComponentManager::remove_node_from_executor(std::uint64_t node_id)
{
  ex_.remove_node(node_wrappers_[node_id].get_node_base_interface());
}

}  // namespace flux
