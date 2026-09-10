#include "flux/ros/publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

class ImageLoanPublisher : public rclcpp::Node
{
public:
  ImageLoanPublisher()
  : Node("flux_loan_image_pub"), pub_(*this, kTopic, Image::kFingerprint, kSlotSize, kSlotCount)
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
    Image::Builder b = Image::build__(pub_);
    if (!b) {
      return;  // every slot borrowed; delivery is best-effort
    }

    const rclcpp::Time t = now();
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
    // Backpressure is a dropped frame on a best-effort transport, so it is not an error.
    // Anything else does not clear on its own.
    if (const flux::Published p = b.commit__(); flux::faulted(p)) {
      RCLCPP_ERROR_ONCE(get_logger(), "commit refused: %s", flux::to_string(p));
    }
    ++phase_;
  }

  flux::ros::Publisher pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint8_t phase_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageLoanPublisher>());
  rclcpp::shutdown();
  return 0;
}
