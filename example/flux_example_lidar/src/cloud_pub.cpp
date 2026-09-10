#include "flux/ros/publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/point_cloud2.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

// A lidar frame is not a fixed size: the point count changes every sweep. The slot is sized for
// the worst case and the Builder commits only the prefix it actually built, so a short sweep
// costs a short frame rather than a padded one.
class CloudPublisher : public rclcpp::Node
{
public:
  CloudPublisher()
  : Node("flux_lidar_cloud_pub"), pub_(*this, kTopic, Cloud::kFingerprint, kSlotSize, kSlotCount)
  {
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kRateHz)),
      [this]() { tick(); });
  }

private:
  using Cloud = sensor_msgs::flux_msg::PointCloud2;

  static constexpr const char * kTopic = "cloud";
  static constexpr const char * kFrameId = "lidar";
  static constexpr std::uint32_t kPointStep = 16;  // x y z intensity, float32 each
  static constexpr std::uint32_t kMinPoints = 40000;
  static constexpr std::uint32_t kMaxPoints = 131072;
  static constexpr double kRateHz = 10.0;
  static constexpr std::uint32_t kSlotCount = 4;
  static constexpr std::uint32_t kSlotSize = kMaxPoints * kPointStep + 4096;

  void tick()
  {
    Cloud::Builder b = Cloud::build__(pub_);
    if (!b) {
      return;  // every slot borrowed; delivery is best-effort
    }

    const std::uint32_t n = kMinPoints + (sweep_ % (kMaxPoints - kMinPoints));
    const rclcpp::Time t = now();

    b.set__header__stamp(static_cast<std::int32_t>(t.seconds()), t.nanoseconds() % 1000000000u);
    b.set__header__frame_id(kFrameId);
    b.set__height(1);
    b.set__width(n);
    b.set__is_bigendian(false);
    b.set__point_step(kPointStep);
    b.set__row_step(n * kPointStep);
    b.set__is_dense(true);

    auto fields = b.alloc__fields(4);
    fields[0].set__name("x");
    fields[0].set__offset(0);
    fields[0].set__datatype(kFloat32);
    fields[0].set__count(1);
    fields[1].set__name("y");
    fields[1].set__offset(4);
    fields[1].set__datatype(kFloat32);
    fields[1].set__count(1);
    fields[2].set__name("z");
    fields[2].set__offset(8);
    fields[2].set__datatype(kFloat32);
    fields[2].set__count(1);
    fields[3].set__name("intensity");
    fields[3].set__offset(12);
    fields[3].set__datatype(kFloat32);
    fields[3].set__count(1);

    auto data = b.alloc__data(static_cast<std::size_t>(n) * kPointStep);
    auto * xyzi = reinterpret_cast<float *>(data.data());
    for (std::uint32_t i = 0; i < n; ++i) {
      const float a = static_cast<float>(i) * 0.001F;
      xyzi[4 * i + 0] = a;
      xyzi[4 * i + 1] = a * 2.0F;
      xyzi[4 * i + 2] = a * 3.0F;
      xyzi[4 * i + 3] = static_cast<float>(sweep_ % 256);
    }

    // Backpressure is a dropped frame on a best-effort transport, so it is not an error.
    // Anything else does not clear on its own.
    if (const flux::Published p = b.commit__(); flux::faulted(p)) {
      RCLCPP_ERROR_ONCE(get_logger(), "commit refused: %s", flux::to_string(p));
    }
    ++sweep_;
  }

  static constexpr std::uint8_t kFloat32 = 7;  // sensor_msgs/PointField::FLOAT32

  flux::ros::Publisher pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint32_t sweep_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
