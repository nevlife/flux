#include "flux/ros/partitioned_executor.hpp"
#include "flux/ros/rt_spec.hpp"
#include "flux/ros/subscription.hpp"
#include "flux/rt.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <chrono>
#include <cstdint>
#include <memory>

// The schedule is not in this file. A chain spans processes, and no node can see the ordering
// its own settings have to respect, so the declaration lives in one file and each node reads its
// own stage out of it. FLUX_RT_SPEC names that file; unset leaves an empty spec and a no-op.
class RtStageNode : public rclcpp::Node
{
public:
  RtStageNode()
  : Node("flux_rt_stage"),
    group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    sub_(*this, kTopic, Image::kFingerprint, [this](const flux::FrameView & f) { on_frame(f); })
  {
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

  rclcpp::CallbackGroup::SharedPtr group() { return group_; }
  flux::ros::Subscription & subscription() { return sub_; }

  static constexpr const char * kGroupLabel = "work";

private:
  using Image = sensor_msgs::flux_msg::Image;

  static constexpr const char * kTopic = "image";

  void on_frame(const flux::FrameView & f)
  {
    Image::View v{f};
    if (v.ok__()) {
      ++seen_;
    }
  }

  void report()
  {
    const flux::rt::ThreadState st = flux::rt::current();
    RCLCPP_INFO(
      get_logger(), "seen %lu  this thread policy %d priority %d", seen_, st.policy, st.priority);
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  flux::ros::Subscription sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t seen_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RtStageNode>();

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.add(node->subscription(), node->group());

  const flux::ros::RtSpec spec = flux::ros::RtSpec::load();
  if (spec.empty()) {
    RCLCPP_WARN(node->get_logger(), "FLUX_RT_SPEC is unset: running with no declared schedule");
  } else {
    const flux::ros::RtStage & stage = spec.stage(*node, RtStageNode::kGroupLabel);
    ex.schedule(node->group(), stage);
    RCLCPP_INFO(
      node->get_logger(), "chain %s stage %s/%s", stage.chain.c_str(), stage.node.c_str(),
      stage.group.c_str());
    // The stages flux does not set are still part of the chain, and checking them is the only
    // thing it can do for them.
    for (const flux::ros::RtStage * e : spec.external()) {
      RCLCPP_INFO(
        node->get_logger(), "external stage %s expects priority %d", e->node.c_str(),
        e->expect_priority);
    }
  }

  rclcpp::on_shutdown([&ex]() { ex.stop(); });
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
