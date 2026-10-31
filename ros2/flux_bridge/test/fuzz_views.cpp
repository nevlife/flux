// The generated View and frame_to_msg on frames another process wrote, for the standard adapters.
// Built as a libFuzzer target with -DFLUX_LIBFUZZER (clang, -fsanitize=fuzzer); otherwise a test
// replays a fixed corpus: frames msg_to_frame writes, truncated and with bits flipped. The first
// byte picks the type. With FLUX_FUZZ_SEEDS=<dir> the test also writes the sound frames there.

#include "nav_msgs/flux/occupancy_grid_ros.hpp"
#include "nav_msgs/flux/odometry_ros.hpp"
#include "sensor_msgs/flux/camera_info_ros.hpp"
#include "sensor_msgs/flux/compressed_image_ros.hpp"
#include "sensor_msgs/flux/image_ros.hpp"
#include "sensor_msgs/flux/imu_ros.hpp"
#include "sensor_msgs/flux/laser_scan_ros.hpp"
#include "sensor_msgs/flux/point_cloud2_ros.hpp"
#include "stereo_msgs/flux/disparity_image_ros.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace
{

constexpr std::size_t kTypes = 9;

template <class Adapter>
auto read(const void * data, std::size_t size) -> bool
{
  const typename Adapter::View v(data, size);
  try {
    // ADL: the generated namespace is the message package's.
    (void)frame_to_msg(v);
  } catch (const std::runtime_error &) {
    return false;  // how frame_to_msg refuses a frame that does not match
  }
  return v.ok__();
}

// True if the frame matched its schema.
auto replay(const std::uint8_t * data, std::size_t size) -> bool
{
  if (size == 0) {
    return false;
  }
  // A slot starts 8-byte aligned; libFuzzer's buffer need not.
  std::vector<std::uint64_t> words((size - 1 + 7) / 8 + 1);
  std::memcpy(words.data(), data + 1, size - 1);
  const void * f = words.data();
  const std::size_t n = size - 1;
  switch (data[0] % kTypes) {
    case 0:
      return read<sensor_msgs::flux_msg::Image>(f, n);
    case 1:
      return read<sensor_msgs::flux_msg::CompressedImage>(f, n);
    case 2:
      return read<sensor_msgs::flux_msg::CameraInfo>(f, n);
    case 3:
      return read<sensor_msgs::flux_msg::Imu>(f, n);
    case 4:
      return read<sensor_msgs::flux_msg::LaserScan>(f, n);
    case 5:
      return read<sensor_msgs::flux_msg::PointCloud2>(f, n);
    case 6:
      return read<stereo_msgs::flux_msg::DisparityImage>(f, n);
    case 7:
      return read<nav_msgs::flux_msg::OccupancyGrid>(f, n);
    default:
      return read<nav_msgs::flux_msg::Odometry>(f, n);
  }
}

}  // namespace

#ifdef FLUX_LIBFUZZER
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  replay(data, size);
  return 0;
}
#else

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>

namespace
{

template <class Adapter, class Msg>
auto frame(std::uint8_t pick, const Msg & m) -> std::vector<std::uint8_t>
{
  std::vector<std::uint64_t> words(1 << 14);
  typename Adapter::Builder b(words.data(), words.size() * 8);
  msg_to_frame(m, b);
  EXPECT_TRUE(b.ok__());
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(words.data());  // NOLINT
  std::vector<std::uint8_t> out{pick};
  out.insert(out.end(), bytes, bytes + b.size__());
  return out;
}

auto seeds() -> std::vector<std::vector<std::uint8_t> >
{
  sensor_msgs::msg::Image img;
  img.header.frame_id = "cam";
  img.height = 2;
  img.width = 3;
  img.encoding = "rgb8";
  img.step = 9;
  img.data.assign(18, 7);
  sensor_msgs::msg::CompressedImage jpg;
  jpg.format = "rgb8; jpeg compressed bgr8";
  jpg.data.assign(32, 1);
  sensor_msgs::msg::CameraInfo info;
  info.distortion_model = "plumb_bob";
  info.d = {0.1, 0.2, 0.0, 0.0, 0.3};
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "imu";
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = {1.0F, 2.0F, 3.0F};
  scan.intensities = {4.0F};
  sensor_msgs::msg::PointCloud2 cloud;
  for (const char * name : {"x", "y", "z"}) {
    sensor_msgs::msg::PointField pf;
    pf.name = name;
    pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
    pf.count = 1;
    cloud.fields.push_back(pf);
  }
  cloud.data.assign(24, 2);
  stereo_msgs::msg::DisparityImage disp;
  disp.image = img;
  disp.image.encoding = "32FC1";
  nav_msgs::msg::OccupancyGrid grid;
  grid.data.assign(16, 50);
  nav_msgs::msg::Odometry odom;
  odom.child_frame_id = "base";
  return {
    frame<sensor_msgs::flux_msg::Image>(0, img),
    frame<sensor_msgs::flux_msg::CompressedImage>(1, jpg),
    frame<sensor_msgs::flux_msg::CameraInfo>(2, info),
    frame<sensor_msgs::flux_msg::Imu>(3, imu),
    frame<sensor_msgs::flux_msg::LaserScan>(4, scan),
    frame<sensor_msgs::flux_msg::PointCloud2>(5, cloud),
    frame<stereo_msgs::flux_msg::DisparityImage>(6, disp),
    frame<nav_msgs::flux_msg::OccupancyGrid>(7, grid),
    frame<nav_msgs::flux_msg::Odometry>(8, odom)};
}

auto next(std::uint64_t & s) -> std::uint64_t
{
  s ^= s << 13;
  s ^= s >> 7;
  s ^= s << 17;
  return s;
}

TEST(FuzzViews, WrittenFramesReadAndDamagedOnesStayInBounds)
{
  std::uint64_t seed = 0x2545F4914F6CDD1D;
  for (const std::vector<std::uint8_t> & good : seeds()) {
    ASSERT_TRUE(replay(good.data(), good.size())) << int{good[0]};
    if (const char * dir = std::getenv("FLUX_FUZZ_SEEDS")) {
      std::ofstream(std::string(dir) + "/" + std::to_string(good[0]), std::ios::binary)
        << std::string(good.begin(), good.end());
    }
    for (std::size_t n = 1; n < good.size(); ++n) {
      replay(good.data(), n);
    }
    for (int round = 0; round < 2000; ++round) {
      std::vector<std::uint8_t> bad = good;
      for (int flips = 1 + static_cast<int>(next(seed) % 4); flips > 0; --flips) {
        bad[1 + next(seed) % (bad.size() - 1)] ^= static_cast<std::uint8_t>(1U << (next(seed) % 8));
      }
      replay(bad.data(), bad.size());
    }
  }
}

}  // namespace

#endif
