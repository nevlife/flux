#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

// ROS callbacks on a MultiThreadedExecutor, flux frames on a flux executor. PartitionedExecutor-
// Executor cannot serve this: it claims every callback group of any node it is given, so the
// node cannot also go to the MultiThreadedExecutor.
//
// The split works for the same reason split_sub does -- a flux subscription is in no callback
// group, so the two executors never contend. What differs from split_sub is only which executor
// the ROS side gets, and that choice is the point: a MultiThreadedExecutor runs callbacks on
// whichever pool thread is free, while PartitionedExecutor pins a group to one thread. Only
// the second can carry an RT priority, because only there is it known which thread runs what.
class MultiSubscriber : public rclcpp::Node
{
public:
  MultiSubscriber()
  : Node("flux_multi_sub"),
    slow_group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    fast_group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    flux_sub_(*this, kTopic, Image::kFingerprint, [this](const flux::FrameView & f) { on_flux(f); })
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(kDepth)).best_effort();
    rclcpp::SubscriptionOptions slow_opts;
    slow_opts.callback_group = slow_group_;
    rclcpp::SubscriptionOptions fast_opts;
    fast_opts.callback_group = fast_group_;

    ros_slow_ = create_subscription<sensor_msgs::msg::Image>(
      kTopic, qos, [this](const sensor_msgs::msg::Image::ConstSharedPtr &) { on_ros_slow(); },
      slow_opts);
    ros_fast_ = create_subscription<sensor_msgs::msg::Image>(
      kTopic, qos, [this](const sensor_msgs::msg::Image::ConstSharedPtr &) { on_ros_fast(); },
      fast_opts);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

  flux::ros::Subscription & flux_subscription() { return flux_sub_; }

private:
  using Image = sensor_msgs::flux_msg::Image;

  static constexpr const char * kTopic = "image";
  static constexpr std::size_t kDepth = 8;
  static constexpr std::chrono::milliseconds kSlowWork{60};

  void on_flux(const flux::FrameView & f)
  {
    Image::View v{f};
    if (!v.ok__()) {
      return;
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    ++flux_seen_;
  }

  void on_ros_slow()
  {
    std::this_thread::sleep_for(kSlowWork);
    const std::lock_guard<std::mutex> lock(mutex_);
    ++ros_slow_seen_;
  }

  void on_ros_fast()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++ros_fast_seen_;
  }

  void report()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(
      get_logger(), "flux %lu  ros_fast %lu  ros_slow %lu", flux_seen_, ros_fast_seen_,
      ros_slow_seen_);
  }

  rclcpp::CallbackGroup::SharedPtr slow_group_;
  rclcpp::CallbackGroup::SharedPtr fast_group_;
  flux::ros::Subscription flux_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_slow_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_fast_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::uint64_t flux_seen_ = 0;
  std::uint64_t ros_slow_seen_ = 0;
  std::uint64_t ros_fast_seen_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiSubscriber>();

  rclcpp::executors::MultiThreadedExecutor mex(rclcpp::ExecutorOptions(), 2);
  mex.add_node(node);

  flux::ros::Executor fex;
  fex.add(node->flux_subscription());

  rclcpp::on_shutdown([&fex, &mex]() {
    fex.stop();
    mex.cancel();
  });

  std::thread ros_thread([&mex]() { mex.spin(); });
  fex.spin();
  ros_thread.join();

  rclcpp::shutdown();
  return 0;
}
