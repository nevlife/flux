#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

// depth 8 with transient_local(4): drain up to eight frames per wake in publish order, and on
// attach replay the four the ring still holds. The callback sleeps so lost() actually moves --
// a consumer slower than the publisher is what this QoS is for.
class BacklogSubscriber : public rclcpp::Node
{
public:
  BacklogSubscriber()
  : Node("flux_qos_backlog"),
    sub_(
      *this, kTopic, Image::kFingerprint, [this](const flux::FrameView & f) { on_frame(f); },
      backlog_qos())
  {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

  flux::ros::Subscription & subscription() { return sub_; }

private:
  using Image = sensor_msgs::flux_msg::Image;

  static constexpr const char * kTopic = "image";
  static constexpr std::chrono::milliseconds kWork{50};

  static flux::QoS backlog_qos()
  {
    flux::QoS q;
    q.depth = 8;
    q.durability = flux::Durability::TransientLocal(4);
    q.max_borrow = 4;
    return q;
  }

  void on_frame(const flux::FrameView & f)
  {
    Image::View v{f};
    if (v.ok__()) {
      std::this_thread::sleep_for(kWork);
      ++seen_;
    }
  }

  void report()
  {
    const flux::Channel::Refused r = sub_.refused();
    RCLCPP_INFO(
      get_logger(), "seen %lu  lost %lu  refused %lu (max_borrow %lu)  can_borrow %d", seen_,
      sub_.lost(), r.total(), r.max_borrow, sub_.can_borrow());
  }

  flux::ros::Subscription sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t seen_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BacklogSubscriber>();

  flux::ros::Executor ex;
  ex.add(node->subscription());
  ex.add_ros_node(node);

  rclcpp::on_shutdown([&ex]() { ex.stop(); });
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
