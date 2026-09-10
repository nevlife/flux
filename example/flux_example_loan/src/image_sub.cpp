#include "flux/ros/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("flux_loan_image_sub"),
    sub_(*this, kTopic, Image::kFingerprint, [this](const flux::FrameView & f) { on_frame(f); })
  {
  }

  flux::ros::Subscription & subscription() { return sub_; }

private:
  using Image = sensor_msgs::flux_msg::Image;

  static constexpr const char * kTopic = "image";

  void on_frame(const flux::FrameView & f)
  {
    Image::View v{f};
    const std::uint32_t width = v.width();
    const std::uint32_t height = v.height();
    const std::string encoding{v.encoding()};
    auto data = v.data();
    if (!v.ok__()) {
      return;
    }
    width_ = width;
    height_ = height;
    encoding_ = encoding;
    data_bytes_ = data.size();
    ++seen_;
  }

  flux::ros::Subscription sub_;
  std::uint64_t seen_ = 0;
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  std::string encoding_;
  std::size_t data_bytes_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriber>();

  flux::ros::Executor ex;
  ex.add(node->subscription());
  ex.add_ros_node(node);

  rclcpp::on_shutdown([&ex]() { ex.stop(); });
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
