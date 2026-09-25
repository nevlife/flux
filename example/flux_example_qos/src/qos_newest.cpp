#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <chrono>
#include <cstdint>
#include <memory>

// depth 1: the newest frame per wake and nothing else. A consumer that only ever wants current
// state takes this -- everything it falls behind by is counted in lost() and never queued.
class NewestSubscriber : public rclcpp::Node
{
public:
  NewestSubscriber()
  : Node("flux_qos_newest"),
    sub_(*this, kTopic, Image::kFingerprint, newest_qos(), [this](const flux::FrameView & f) {
      on_frame(f);
    })
  {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

  flux::ros::Subscription & subscription() { return sub_; }

private:
  using Image = sensor_msgs::flux_msg::Image;

  static constexpr const char * kTopic = "image";

  static flux::QoS newest_qos() { return flux::QoS(1).max_borrow(1); }

  void on_frame(const flux::FrameView & f)
  {
    Image::View v{f};
    if (v.ok__()) {
      ++seen_;
    }
  }

  void report()
  {
    const flux::Channel::Refused r = sub_.refused();
    RCLCPP_INFO(
      get_logger(), "seen %lu  lost %lu  refused %lu (max_borrow %lu)", seen_, sub_.lost(),
      r.total(), r.max_borrow);
  }

  flux::ros::Subscription sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t seen_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NewestSubscriber>();

  flux::ros::Executor ex;
  ex.add(node->subscription());
  ex.add_ros_node(node);

  ex.spin();
  rclcpp::shutdown();
  return 0;
}
