#include "flux/ros/publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

// Device::Cuda makes flux create the stream both seams fence on. Nothing on the wire changes: a
// host subscriber on this same topic keeps working, because the declaration is about who touches
// the bytes, not about how they travel.
class GpuImagePublisher : public rclcpp::Node
{
public:
  GpuImagePublisher()
  : Node("flux_gpu_image_pub"),
    pub_(*this, kTopic, flux::kNoSchema, kSlotSize, kSlotCount, flux::Device::Cuda)
  {
    RCLCPP_INFO(get_logger(), "host_addressable %d", static_cast<int>(pub_.host_addressable()));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kRateHz)),
      [this]() { tick(); });
  }

private:
  static constexpr const char * kTopic = "image_gpu";
  static constexpr std::uint64_t kWidth = 1920;
  static constexpr std::uint64_t kHeight = 1200;
  static constexpr std::uint64_t kChannels = 3;
  static constexpr double kRateHz = 30.0;
  static constexpr std::uint32_t kSlotCount = 4;
  static constexpr std::size_t kDataBytes = static_cast<std::size_t>(kWidth * kHeight * kChannels);
  static constexpr std::uint32_t kSlotSize = static_cast<std::uint32_t>(kDataBytes);

  void tick()
  {
    flux::WriteSlot w = pub_.loan(flux::DType::U8, {kHeight, kWidth, kChannels});
    if (!w.valid()) {
      return;
    }

    // On an integrated GPU the slot is one mapping the host and a kernel both reach, so this
    // fill is a host store. On a discrete GPU data() is null and the fill is a kernel launched
    // on w.stream() writing w.device_ptr().
    if (w.host_addressable()) {
      auto * px = static_cast<std::uint8_t *>(w.data());
      for (std::size_t i = 0; i < kDataBytes; ++i) {
        px[i] = static_cast<std::uint8_t>(i + phase_);
      }
    } else {
      RCLCPP_ERROR_ONCE(get_logger(), "device-backed slot: fill it with a kernel on w.stream()");
      w.abort();
      return;
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
  try {
    rclcpp::spin(std::make_shared<GpuImagePublisher>());
  } catch (const std::invalid_argument & e) {
    // The constructor refuses rather than quietly running on the host path: a publisher that
    // believes it is on the GPU must not go on being wrong.
    RCLCPP_ERROR(rclcpp::get_logger("flux_gpu_image_pub"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
