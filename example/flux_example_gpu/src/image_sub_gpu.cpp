#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>

// The release seam is a stream synchronization: dropping the view waits for the consuming kernel
// before the publisher may reuse the slot. fence_wait() is what that costs, and a nonzero
// fence_failed() is a fault, not a rate -- each one leaks a slot that is never reused.
class GpuImageSubscriber : public rclcpp::Node
{
public:
  GpuImageSubscriber()
  : Node("flux_gpu_image_sub"),
    sub_(
      *this, kTopic, flux::kNoSchema, [this](const flux::FrameView & f) { on_frame(f); },
      flux::QoS{}, flux::Device::Cuda)
  {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

  flux::ros::Subscription & subscription() { return sub_; }

private:
  static constexpr const char * kTopic = "image_gpu";

  void on_frame(const flux::FrameView & f)
  {
    device_ptr_ = f.device_ptr();
    ++seen_;
  }

  void report()
  {
    const flux::Channel::FenceWait fw = sub_.fence_wait();
    RCLCPP_INFO(
      get_logger(), "seen %lu  device_ptr %p  release max %lu us  fence_failed %lu", seen_,
      device_ptr_, fw.release_max_ns / 1000, sub_.fence_failed());
  }

  flux::ros::Subscription sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t seen_ = 0;
  const void * device_ptr_ = nullptr;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GpuImageSubscriber>();

    flux::ros::Executor ex;
    ex.add(node->subscription());
    ex.add_ros_node(node);

    rclcpp::on_shutdown([&ex]() { ex.stop(); });
    ex.spin();
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(rclcpp::get_logger("flux_gpu_image_sub"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
