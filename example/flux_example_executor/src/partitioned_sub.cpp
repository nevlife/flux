#include "flux/ros/partitioned_executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

// Two flux subscriptions, two callback groups, two threads. A slow callback in one group cannot
// delay the other -- which is the whole reason to pay for the extra thread.
class PartitionedSubscriber : public rclcpp::Node
{
public:
  PartitionedSubscriber()
  : Node("flux_partitioned_sub"),
    fast_group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    slow_group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    fast_(*this, kTopic, Image::kFingerprint, [this](const flux::FrameView & f) { on_fast(f); }),
    slow_(*this, kTopic, Image::kFingerprint, [this](const flux::FrameView & f) { on_slow(f); })
  {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

  rclcpp::CallbackGroup::SharedPtr fast_group() { return fast_group_; }
  rclcpp::CallbackGroup::SharedPtr slow_group() { return slow_group_; }
  flux::ros::Subscription & fast() { return fast_; }
  flux::ros::Subscription & slow() { return slow_; }

private:
  using Image = sensor_msgs::flux_msg::Image;

  static constexpr const char * kTopic = "image";
  static constexpr std::chrono::milliseconds kSlowWork{80};

  void on_fast(const flux::FrameView & f)
  {
    Image::View v{f};
    if (v.ok__()) {
      ++fast_seen_;
    }
  }

  void on_slow(const flux::FrameView & f)
  {
    Image::View v{f};
    if (v.ok__()) {
      std::this_thread::sleep_for(kSlowWork);
      ++slow_seen_;
    }
  }

  void report() { RCLCPP_INFO(get_logger(), "fast %lu  slow %lu", fast_seen_, slow_seen_); }

  rclcpp::CallbackGroup::SharedPtr fast_group_;
  rclcpp::CallbackGroup::SharedPtr slow_group_;
  flux::ros::Subscription fast_;
  flux::ros::Subscription slow_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t fast_seen_ = 0;
  std::uint64_t slow_seen_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PartitionedSubscriber>();

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.add(node->fast(), node->fast_group());
  ex.add(node->slow(), node->slow_group());

  rclcpp::on_shutdown([&ex]() { ex.stop(); });
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
