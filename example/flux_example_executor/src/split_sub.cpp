#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

// Two executors, two threads: a stock rclcpp one for the ROS side and a flux one for the flux
// side. This is the arrangement merged_sub does NOT use, and it is available because a flux
// subscription is not an rclcpp entity -- it is in no callback group, so nothing here contends
// for a group claim.
//
// What it costs is the mutual exclusion merged_sub gets for free. The two callbacks run on
// different threads at the same time, so anything both touch needs a lock. That lock is the
// whole reason to look at this file before choosing this shape.
class SplitSubscriber : public rclcpp::Node
{
public:
  SplitSubscriber()
  : Node("flux_split_sub"),
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
    if (!v.ok__()) {
      return;
    }
    const std::lock_guard<std::mutex> lock(mutex_);  // the ROS thread reads these too
    flux_width_ = width;
    ++flux_seen_;
  }

  void on_ros(const sensor_msgs::msg::Image & msg)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ros_width_ = msg.width;
    ++ros_seen_;
  }

  void report()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(
      get_logger(), "flux %lu (w=%u)  ros %lu (w=%u)", flux_seen_, flux_width_, ros_seen_,
      ros_width_);
  }

  flux::ros::Subscription flux_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::uint64_t flux_seen_ = 0;
  std::uint64_t ros_seen_ = 0;
  std::uint32_t flux_width_ = 0;
  std::uint32_t ros_width_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SplitSubscriber>();

  // The node goes to the rclcpp executor. The flux executor is given the flux subscription and
  // nothing else -- handing the node to both is what would throw.
  flux::ros::Executor fex;
  fex.add(node->flux_subscription());

  rclcpp::on_shutdown([&fex]() { fex.stop(); });

  std::thread ros_thread([node]() { rclcpp::spin(node); });
  fex.spin();
  ros_thread.join();

  rclcpp::shutdown();
  return 0;
}
