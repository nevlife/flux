#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cstdint>
#include <memory>

// One thread waits on both transports. The flux channel's futex and the ROS subscription's
// readiness eventfd are armed in the same io_uring, so a merged wait costs one syscall.
class MergedSubscriber : public rclcpp::Node
{
public:
  MergedSubscriber()
  : Node("flux_merged_sub"),
    flux_sub_(*this, kTopic, Image::kFingerprint, [this](const flux::FrameView & f) { on_flux(f); })
  {
    ros_sub_ = create_subscription<sensor_msgs::msg::Image>(
      kTopic, rclcpp::QoS(rclcpp::KeepLast(kDepth)).best_effort(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) { on_ros(*msg); });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

  flux::ros::Subscription & flux_subscription() { return flux_sub_; }

private:
  using Image = sensor_msgs::flux_msg::Image;

  static constexpr const char * kTopic = "image";
  static constexpr std::size_t kDepth = 8;

  void on_flux(const flux::FrameView & f)
  {
    Image::View v{f};
    const std::uint32_t width = v.width();
    if (v.ok__()) {
      flux_width_ = width;
      ++flux_seen_;
    }
  }

  void on_ros(const sensor_msgs::msg::Image & msg)
  {
    ros_width_ = msg.width;
    ++ros_seen_;
  }

  void report()
  {
    RCLCPP_INFO(
      get_logger(), "flux %lu (w=%u)  ros %lu (w=%u)", flux_seen_, flux_width_, ros_seen_,
      ros_width_);
  }

  flux::ros::Subscription flux_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t flux_seen_ = 0;
  std::uint64_t ros_seen_ = 0;
  std::uint32_t flux_width_ = 0;
  std::uint32_t ros_width_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MergedSubscriber>();

  flux::ros::Executor ex(4);
  ex.add(node->flux_subscription());
  ex.add_ros_node(node);

  rclcpp::on_shutdown([&ex]() { ex.stop(); });

  RCLCPP_INFO(node->get_logger(), "merged wait: io_uring=%d", ex.uses_io_uring());
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
