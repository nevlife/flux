#include "flux/ros/publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

// The path a node takes with no .msg: no adapter, no schema, no header fields. The frame is a
// bare uint8 volume and the publisher names its dtype and shape itself. Kept next to the three
// adapter nodes so the cost of having no schema is visible -- this one carries no stamp and no
// frame_id, because there is nowhere to put them.
class ImageRawPublisher : public rclcpp::Node
{
public:
  ImageRawPublisher()
  : Node("flux_raw_image_pub"), pub_(*this, kTopic, flux::kNoSchema, kSlotSize, kSlotCount)
  {
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kRateHz)),
      [this]() { tick(); });
  }

private:
  static constexpr const char * kTopic = "image_raw";
  static constexpr std::uint64_t kWidth = 1920;
  static constexpr std::uint64_t kHeight = 1200;
  static constexpr std::uint64_t kChannels = 3;
  static constexpr double kRateHz = 30.0;
  static constexpr std::uint32_t kSlotCount = 8;
  static constexpr std::size_t kDataBytes = static_cast<std::size_t>(kWidth * kHeight * kChannels);
  static constexpr std::uint32_t kSlotSize = static_cast<std::uint32_t>(kDataBytes);

  void tick()
  {
    flux::WriteSlot w = pub_.loan(flux::DType::U8, {kHeight, kWidth, kChannels});
    if (!w.valid()) {
      return;
    }

    auto * px = static_cast<std::uint8_t *>(w.data());
    for (std::size_t i = 0; i < kDataBytes; ++i) {
      px[i] = static_cast<std::uint8_t>(i + phase_);
    }

    // Backpressure is a dropped frame on a best-effort transport, so it is not an error.
    // Anything else does not clear on its own.
    if (const flux::Published p = w.commit(); flux::faulted(p)) {
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
  rclcpp::spin(std::make_shared<ImageRawPublisher>());
  rclcpp::shutdown();
  return 0;
}
