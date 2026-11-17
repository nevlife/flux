// test_no_adapter builds this and expects the static_assert: std_msgs/String has no flux adapter.
#include "flux_wrapper/flux_wrapper.hpp"

#include <std_msgs/msg/string.hpp>

auto main() -> int
{
  rclcpp::init(0, nullptr);
  rclcpp::Node node("no_adapter");
  const flux_wrapper::Publisher<std_msgs::msg::String> pub(node, "t", rclcpp::QoS(1));
  rclcpp::shutdown();
}
