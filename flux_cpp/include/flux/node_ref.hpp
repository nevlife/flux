#ifndef FLUX_NODE_REF_HPP
#define FLUX_NODE_REF_HPP

#include <memory>
#include <utility>

namespace rclcpp::node_interfaces
{
class NodeBaseInterface;
class NodeTopicsInterface;
class NodeLoggingInterface;
}  // namespace rclcpp::node_interfaces

namespace flux
{

// rclcpp::Node, rclcpp_lifecycle::LifecycleNode, or any node with their interface getters.
class NodeRef
{
public:
  template <class NodeT, class = decltype(std::declval<NodeT &>().get_node_base_interface())>
  NodeRef(NodeT & node)  // NOLINT(runtime/explicit): Publisher(*this, ...) keeps compiling
  : base_(node.get_node_base_interface()),
    topics_(node.get_node_topics_interface()),
    logging_(node.get_node_logging_interface())
  {
  }

  const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> & base() const noexcept
  {
    return base_;
  }
  const std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> & topics() const noexcept
  {
    return topics_;
  }
  const std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> & logging() const noexcept
  {
    return logging_;
  }

private:
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_;
  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_;
  std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_;
};

}  // namespace flux

#endif  // FLUX_NODE_REF_HPP
