#include "flux/ros/publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

class ImagePublishPublisher : public rclcpp::Node
{
public:
  ImagePublishPublisher()
  : Node("flux_publish_image_pub"),
    pub_(*this, kTopic, Image::kFingerprint, kSlotSize, kSlotCount),
    buf_(kSlotSize)
  {
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kRateHz)),
      [this]() { tick(); });
  }

private:
  using Image = sensor_msgs::flux_msg::Image;

  static constexpr const char * kTopic = "image";
  static constexpr const char * kFrameId = "camera";
  static constexpr const char * kEncoding = "bgr8";
  static constexpr std::uint32_t kWidth = 1920;
  static constexpr std::uint32_t kHeight = 1200;
  static constexpr std::uint32_t kStep = kWidth * 3;
  static constexpr double kRateHz = 30.0;
  static constexpr std::uint32_t kSlotCount = 8;
  static constexpr std::size_t kDataBytes = static_cast<std::size_t>(kStep) * kHeight;
  static constexpr std::uint32_t kSlotSize = static_cast<std::uint32_t>(kDataBytes + 1024);

  void tick()
  {
    const rclcpp::Time t = now();
    Image::Builder b{buf_.data(), buf_.size()};
    b.set__header__stamp(static_cast<std::int32_t>(t.seconds()), t.nanoseconds() % 1000000000u);
    b.set__header__frame_id(kFrameId);
    b.set__height(kHeight);
    b.set__width(kWidth);
    b.set__encoding(kEncoding);
    b.set__is_bigendian(0);
    b.set__step(kStep);

    auto data = b.alloc__data(kDataBytes);
    for (std::size_t i = 0; i < data.size(); ++i) {
      data[i] = static_cast<std::uint8_t>(i + phase_);
    }
    if (!b.ok__()) {
      return;
    }

    // The schema is in the fingerprint, so an adapter frame carries no descriptor of its own.
    // Backpressure is a dropped frame on a best-effort transport, so it is not an error.
    // Anything else does not clear on its own.
    if (const flux::Published p = pub_.publish(buf_.data(), b.size__()); flux::faulted(p)) {
      RCLCPP_ERROR_ONCE(get_logger(), "publish refused: %s", flux::to_string(p));
    }
    ++phase_;
  }

  flux::ros::Publisher pub_;
  std::vector<std::uint8_t> buf_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint8_t phase_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublishPublisher>());
  rclcpp::shutdown();
  return 0;
}
