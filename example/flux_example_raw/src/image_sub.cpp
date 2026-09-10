#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>

// The read side with no .msg. There is no View to build, so the frame's dtype and shape are read
// off FrameMeta and the payload is whatever bytes the publisher wrote.
class ImageRawSubscriber : public rclcpp::Node
{
public:
  ImageRawSubscriber()
  : Node("flux_raw_image_sub"),
    sub_(*this, kTopic, flux::kNoSchema, [this](const flux::FrameView & f) { on_frame(f); })
  {
  }

  flux::ros::Subscription & subscription() { return sub_; }

private:
  static constexpr const char * kTopic = "image_raw";

  void on_frame(const flux::FrameView & f)
  {
    const flux::FrameMeta & m = f.meta();
    if (m.dtype != flux::DType::U8 || m.ndim != 3) {
      RCLCPP_ERROR_ONCE(get_logger(), "unexpected frame shape on a schemaless topic");
      return;
    }
    height_ = m.shape[0];
    width_ = m.shape[1];
    channels_ = m.shape[2];
    data_bytes_ = f.size();
    ++seen_;
  }

  flux::ros::Subscription sub_;
  std::uint64_t seen_ = 0;
  std::uint64_t height_ = 0;
  std::uint64_t width_ = 0;
  std::uint64_t channels_ = 0;
  std::size_t data_bytes_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageRawSubscriber>();

  flux::ros::Executor ex;
  ex.add(node->subscription());
  ex.add_ros_node(node);

  rclcpp::on_shutdown([&ex]() { ex.stop(); });
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
