#ifndef FLUX_COMPONENT_MANAGER_HPP
#define FLUX_COMPONENT_MANAGER_HPP

#include "flux/partitioned_executor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include <pthread.h>

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace flux
{

// rclcpp_components' manager over a PartitionedExecutor: each callback group of each loaded node,
// and of the manager, runs on a thread of its own. A node with a Reentrant group is refused.
class ComponentManager : public rclcpp_components::ComponentManager
{
public:
  using NodeBase = PartitionedExecutor::NodeBase;

  explicit ComponentManager(
    std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & options =
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));

  // Rethrows the first error of any group thread. The manager outlives it.
  void spin();
  void cancel() noexcept;

protected:
  // Before the node's groups get threads. An exception refuses the load.
  virtual void on_node_added(const NodeBase::SharedPtr & node);
  // On each group's thread before its first callback. An exception stops spin().
  virtual void on_thread_start(
    const NodeBase::SharedPtr & node, const rclcpp::CallbackGroup::SharedPtr & group);

  void add_node_to_executor(std::uint64_t node_id) override;
  void remove_node_from_executor(std::uint64_t node_id) override;

private:
  PartitionedExecutor ex_;
};

// A container's main(). The process takes the manager node's name: containers would otherwise
// share one.
template <class Manager = ComponentManager>
int run_component_container(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    const auto manager = std::make_shared<Manager>();
    pthread_setname_np(pthread_self(), std::string(manager->get_name()).substr(0, 15).c_str());
    manager->spin();
  } catch (const std::exception & e) {
    std::cerr << "component container: " << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

}  // namespace flux

#endif  // FLUX_COMPONENT_MANAGER_HPP
