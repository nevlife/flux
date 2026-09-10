#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/point_cloud2.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

class CloudSubscriber : public rclcpp::Node
{
public:
  CloudSubscriber()
  : Node("flux_lidar_cloud_sub"),
    sub_(*this, kTopic, Cloud::kFingerprint, [this](const flux::FrameView & f) { on_frame(f); })
  {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

  flux::ros::Subscription & subscription() { return sub_; }

private:
  using Cloud = sensor_msgs::flux_msg::PointCloud2;

  static constexpr const char * kTopic = "cloud";

  void on_frame(const flux::FrameView & f)
  {
    Cloud::View v{f};
    const std::uint32_t width = v.width();
    const std::uint32_t point_step = v.point_step();
    auto data = v.data();
    if (!v.ok__()) {
      return;
    }
    points_ = width;
    point_step_ = point_step;
    data_bytes_ = data.size();
    ++seen_;
  }

  void report()
  {
    RCLCPP_INFO(
      get_logger(), "seen %lu  points %u  point_step %u  bytes %zu  lost %lu", seen_, points_,
      point_step_, data_bytes_, sub_.lost());
  }

  flux::ros::Subscription sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t seen_ = 0;
  std::uint32_t points_ = 0;
  std::uint32_t point_step_ = 0;
  std::size_t data_bytes_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudSubscriber>();

  flux::ros::Executor ex;
  ex.add(node->subscription());
  ex.add_ros_node(node);

  rclcpp::on_shutdown([&ex]() { ex.stop(); });
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
