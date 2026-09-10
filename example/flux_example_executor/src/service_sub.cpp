#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

// flux frames and a ROS service on one executor. The service is what merged_sub does not show:
// readiness for a service, a client and a waitable (which is what an action server is) is armed
// on the same eventfd as a subscription's, so a request wakes the ring when it arrives instead
// of on the next tick.
//
// The tick here is deliberately long. A request answered promptly under a 3 s tick is the whole
// claim; if the service were not bridged, `ros2 service call` would sit there for seconds.
//
//   ros2 service call /frame_count std_srvs/srv/Trigger
class ServiceSubscriber : public rclcpp::Node
{
public:
  ServiceSubscriber()
  : Node("flux_service_sub"),
    sub_(*this, kTopic, Image::kFingerprint, [this](const flux::FrameView & f) { on_frame(f); })
  {
    service_ = create_service<std_srvs::srv::Trigger>(
      kService, [this](
                  const std_srvs::srv::Trigger::Request::SharedPtr,
                  std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->success = true;
        res->message = std::to_string(seen_) + " frames";
        RCLCPP_INFO(get_logger(), "answered: %s", res->message.c_str());
      });
  }

  flux::ros::Subscription & subscription() { return sub_; }

  static constexpr std::int64_t kTickNs = 3'000'000'000;  // 3 s: long enough to be visible

private:
  using Image = sensor_msgs::flux_msg::Image;

  static constexpr const char * kTopic = "image";
  static constexpr const char * kService = "frame_count";

  void on_frame(const flux::FrameView & f)
  {
    Image::View v{f};
    if (v.ok__()) {
      ++seen_;
    }
  }

  flux::ros::Subscription sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::uint64_t seen_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceSubscriber>();

  flux::ros::Executor ex;
  ex.add(node->subscription());
  ex.add_ros_node(node);

  rclcpp::on_shutdown([&ex]() { ex.stop(); });

  RCLCPP_INFO(node->get_logger(), "tick %ld ms", ServiceSubscriber::kTickNs / 1000000);
  ex.spin(ServiceSubscriber::kTickNs);
  rclcpp::shutdown();
  return 0;
}
