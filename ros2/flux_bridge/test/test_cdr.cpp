// The bridge publishes the CDR flux writes from a frame. It must be the bytes rmw writes for the
// same message. rmw_fastrtps reports an estimated length and leaves padding unwritten, so rmw
// serializes into a zeroed buffer, flux's bytes must be a prefix of it, and they must
// deserialize back to the message.

#include "flux/schemas.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include "nav_msgs/flux/occupancy_grid_ros.hpp"
#include "nav_msgs/flux/odometry_ros.hpp"
#include "sensor_msgs/flux/camera_info_ros.hpp"
#include "sensor_msgs/flux/compressed_image_ros.hpp"
#include "sensor_msgs/flux/image_ros.hpp"
#include "sensor_msgs/flux/imu_ros.hpp"
#include "sensor_msgs/flux/laser_scan_ros.hpp"
#include "sensor_msgs/flux/point_cloud2_ros.hpp"
#include "stereo_msgs/flux/disparity_image_ros.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

const flux::Schemas & schemas()
{
  static const flux::Schemas s;
  return s;
}

// u64 storage: a frame starts 8-byte aligned, as a flux slot does.
struct Frame
{
  std::vector<std::uint64_t> words = std::vector<std::uint64_t>(1 << 17);
  std::size_t size = 0;
  void * data() { return words.data(); }
  std::size_t capacity() const { return words.size() * sizeof(std::uint64_t); }
};

template <class Adapter, class Msg>
Frame frame_of(const Msg & m)
{
  Frame f;
  typename Adapter::Builder b(f.data(), f.capacity());
  // The generated namespace is the message package's, so ADL finds msg_to_frame.
  msg_to_frame(m, b);
  EXPECT_TRUE(b.ok__());
  f.size = b.size__();
  return f;
}

template <class Adapter, class Msg>
void expect_rmw_bytes(const Msg & m)
{
  Frame f = frame_of<Adapter>(m);
  ASSERT_TRUE(schemas().contains(Adapter::kFingerprint)) << Adapter::kTypeName;
  const flux::Schemas::Type type = schemas().at(Adapter::kFingerprint);
  EXPECT_EQ(type.ros_type(), rosidl_generator_traits::name<Msg>());

  std::vector<std::uint8_t> ours(type.cdr_size(f.data(), f.size));
  ASSERT_EQ(type.write_cdr(f.data(), f.size, ours.data(), ours.size()), ours.size());

  rclcpp::Serialization<Msg> ser;
  rclcpp::SerializedMessage rmw(1 << 20);
  auto & raw = rmw.get_rcl_serialized_message();
  std::fill(raw.buffer, raw.buffer + raw.buffer_capacity, 0);
  ser.serialize_message(&m, &rmw);
  ASSERT_LE(ours.size(), raw.buffer_length);
  EXPECT_EQ(ours, std::vector<std::uint8_t>(raw.buffer, raw.buffer + ours.size()));

  rclcpp::SerializedMessage back(ours.size());
  auto & b = back.get_rcl_serialized_message();
  std::copy(ours.begin(), ours.end(), b.buffer);
  b.buffer_length = ours.size();
  Msg out;
  ser.deserialize_message(&back, &out);
  EXPECT_EQ(out, m);
}

std_msgs::msg::Header header(const char * frame_id)
{
  std_msgs::msg::Header h;
  h.stamp.sec = 1700000000;
  h.stamp.nanosec = 123456789;
  h.frame_id = frame_id;
  return h;
}

TEST(Cdr, Imu)
{
  sensor_msgs::msg::Imu m;
  m.header = header("imu");
  m.orientation.w = 1.0;
  m.angular_velocity.x = 0.5;
  m.linear_acceleration.z = 9.81;
  for (std::size_t i = 0; i < 9; ++i) m.orientation_covariance[i] = static_cast<double>(i);
  expect_rmw_bytes<sensor_msgs::flux_msg::Imu>(m);
}

TEST(Cdr, ImageWithAnOddLengthAndAnEmptyBody)
{
  sensor_msgs::msg::Image m;
  m.header = header("camera");
  m.height = 1;
  m.width = 13;
  m.encoding = "mono8";
  m.is_bigendian = 1;
  m.step = 13;
  for (std::uint8_t i = 0; i < 13; ++i) m.data.push_back(i);
  expect_rmw_bytes<sensor_msgs::flux_msg::Image>(m);
  m.data.clear();
  m.encoding.clear();
  expect_rmw_bytes<sensor_msgs::flux_msg::Image>(m);
}

TEST(Cdr, CameraInfoWithAndWithoutDistortion)
{
  sensor_msgs::msg::CameraInfo m;
  m.header = header("camera");
  m.width = 640;
  m.distortion_model = "plumb_bob";
  m.d = {0.1, -0.2, 0.0, 0.0, 0.03};
  m.k[0] = 500.0;
  m.p[11] = 1.0;
  m.binning_x = 2;
  m.roi.do_rectify = true;
  m.roi.width = 10;
  expect_rmw_bytes<sensor_msgs::flux_msg::CameraInfo>(m);
  m.d.clear();
  expect_rmw_bytes<sensor_msgs::flux_msg::CameraInfo>(m);
}

TEST(Cdr, PointCloud2WithFieldsAndWithout)
{
  sensor_msgs::msg::PointCloud2 m;
  m.header = header("lidar");
  m.height = 1;
  m.width = 2;
  for (const char * name : {"x", "y", "intensity"}) {
    sensor_msgs::msg::PointField pf;
    pf.name = name;
    pf.offset = static_cast<std::uint32_t>(4 * m.fields.size());
    pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
    pf.count = 1;
    m.fields.push_back(pf);
  }
  m.point_step = 12;
  m.row_step = 24;
  m.data.assign(24, 7);
  m.is_dense = true;
  expect_rmw_bytes<sensor_msgs::flux_msg::PointCloud2>(m);
  m.fields.clear();
  m.data.clear();
  expect_rmw_bytes<sensor_msgs::flux_msg::PointCloud2>(m);
}

TEST(Cdr, LaserScan)
{
  sensor_msgs::msg::LaserScan m;
  m.header = header("laser");
  m.angle_min = -1.5F;
  m.ranges = {1.0F, 2.0F, 3.5F};
  expect_rmw_bytes<sensor_msgs::flux_msg::LaserScan>(m);
}

TEST(Cdr, CompressedImage)
{
  sensor_msgs::msg::CompressedImage m;
  m.header = header("camera");
  m.format = "jpeg";
  m.data = {0xFF, 0xD8, 0xFF};
  expect_rmw_bytes<sensor_msgs::flux_msg::CompressedImage>(m);
}

TEST(Cdr, Odometry)
{
  nav_msgs::msg::Odometry m;
  m.header = header("odom");
  m.child_frame_id = "base_link";
  m.pose.pose.position.x = 3.0;
  m.pose.covariance[35] = 0.25;
  m.twist.twist.angular.z = -0.5;
  expect_rmw_bytes<nav_msgs::flux_msg::Odometry>(m);
}

TEST(Cdr, OccupancyGridWithANestedTime)
{
  nav_msgs::msg::OccupancyGrid m;
  m.header = header("map");
  m.info.map_load_time.sec = 42;
  m.info.resolution = 0.05F;
  m.info.width = 3;
  m.info.height = 1;
  m.data = {-1, 0, 100};
  expect_rmw_bytes<nav_msgs::flux_msg::OccupancyGrid>(m);
}

TEST(Cdr, DisparityImageWithANestedHeader)
{
  stereo_msgs::msg::DisparityImage m;
  m.header = header("stereo");
  m.image.header = header("inner");
  m.image.encoding = "32FC1";
  m.image.data = {1, 2, 3, 4};
  m.f = 700.0F;
  m.valid_window.height = 5;
  m.min_disparity = 1.0F;
  expect_rmw_bytes<stereo_msgs::flux_msg::DisparityImage>(m);
}

TEST(Cdr, AFrameThatDoesNotMatchItsSchemaIsRefused)
{
  sensor_msgs::msg::Image m;
  m.data = {1, 2, 3};
  Frame f = frame_of<sensor_msgs::flux_msg::Image>(m);
  const flux::Schemas::Type type = schemas().at(sensor_msgs::flux_msg::Image::kFingerprint);
  // Cut the frame inside its data: the descriptor now points past the end.
  try {
    type.cdr_size(f.data(), f.size - 1);
    FAIL() << "a truncated frame converted";
  } catch (const flux::Error & e) {
    EXPECT_EQ(e.status(), FLUX_MISMATCH);
  }
}

// A PointCloud2 frame with every field name pointing at the longest one, as a crafted frame would.
Frame overlapping_frame()
{
  sensor_msgs::msg::PointCloud2 m;
  const std::size_t n = 200;
  for (std::size_t i = 0; i < n; ++i) {
    sensor_msgs::msg::PointField pf;
    pf.name.assign(i + 1, 'a');
    m.fields.push_back(pf);
  }
  Frame f = frame_of<sensor_msgs::flux_msg::PointCloud2>(m);
  auto * bytes = static_cast<std::uint8_t *>(f.data());
  std::vector<std::size_t> names;
  std::uint32_t longest = 0;
  for (std::size_t at = 0; at + 8 <= f.size; at += 4) {
    std::array<std::uint32_t, 2> desc{};
    std::memcpy(desc.data(), bytes + at, sizeof desc);
    const bool is_name =
      desc[1] >= 1 && desc[1] <= n && desc[0] + desc[1] <= f.size &&
      std::all_of(
        bytes + desc[0], bytes + desc[0] + desc[1], [](std::uint8_t b) { return b == 'a'; });
    if (!is_name) continue;
    names.push_back(at);
    if (desc[1] == n) longest = desc[0];
  }
  EXPECT_EQ(names.size(), n);
  const std::array<std::uint32_t, 2> to{longest, n};
  for (const std::size_t at : names) std::memcpy(bytes + at, to.data(), sizeof to);
  return f;
}

TEST(Cdr, AFrameThatReadsTheSameBytesTwiceIsRefused)
{
  Frame f = overlapping_frame();
  const flux::Schemas::Type type = schemas().at(sensor_msgs::flux_msg::PointCloud2::kFingerprint);
  try {
    type.cdr_size(f.data(), f.size);
    FAIL() << "an overlapping frame converted";
  } catch (const flux::Error & e) {
    EXPECT_EQ(e.status(), FLUX_MISMATCH);
  }
}

TEST(Cdr, FrameToMsgRefusesAFrameThatReadsTheSameBytesTwice)
{
  // Unchecked, the copy grows with the square of the frame: 818 times a 64 KiB frame.
  Frame f = overlapping_frame();
  try {
    sensor_msgs::flux_msg::frame_to_msg(sensor_msgs::flux_msg::PointCloud2::View(f.data(), f.size));
    FAIL() << "an overlapping frame was copied";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("same bytes twice"), std::string::npos) << e.what();
  }
}

TEST(Cdr, AShortBufferIsRefusedInsteadOfAborting)
{
  sensor_msgs::msg::Image m;
  m.data = {1, 2, 3};
  Frame f = frame_of<sensor_msgs::flux_msg::Image>(m);
  const flux::Schemas::Type type = schemas().at(sensor_msgs::flux_msg::Image::kFingerprint);
  std::vector<std::uint8_t> out(type.cdr_size(f.data(), f.size) - 1);
  for (std::size_t cap : {std::size_t{0}, out.size()}) {
    try {
      type.write_cdr(f.data(), f.size, out.data(), cap);
      FAIL() << "wrote into " << cap << " bytes";
    } catch (const flux::Error & e) {
      EXPECT_EQ(e.status(), FLUX_INVALID_ARGUMENT);
    }
  }
}

TEST(Cdr, AStringEndsAtItsFirstNulAsRclpyReadsIt)
{
  // rclpy holds a C string, and rclcpp throws on publishing one with a NUL inside: the bridge
  // and frame_to_msg give what an rclpy publisher of the same message would.
  sensor_msgs::msg::Image m;
  m.encoding = std::string("mono8\0junk", 10);
  Frame f = frame_of<sensor_msgs::flux_msg::Image>(m);
  m.encoding = "mono8";
  const auto back =
    sensor_msgs::flux_msg::frame_to_msg(sensor_msgs::flux_msg::Image::View(f.data(), f.size));
  EXPECT_EQ(back, m);
  const flux::Schemas::Type type = schemas().at(sensor_msgs::flux_msg::Image::kFingerprint);
  std::vector<std::uint8_t> ours(type.cdr_size(f.data(), f.size));
  type.write_cdr(f.data(), f.size, ours.data(), ours.size());
  Frame clean = frame_of<sensor_msgs::flux_msg::Image>(m);
  std::vector<std::uint8_t> want(type.cdr_size(clean.data(), clean.size));
  type.write_cdr(clean.data(), clean.size, want.data(), want.size());
  EXPECT_EQ(ours, want);
}

// The byte of a bool object: one holding anything but 0 or 1 is undefined to read as a bool.
template <class T>
unsigned char byte_of(const T & b)
{
  unsigned char raw = 0;
  std::memcpy(&raw, &b, 1);
  return raw;
}

TEST(Cdr, ABoolByteOtherThanZeroOrOneIsTrueEverywhere)
{
  // C++ copied 0xCA into the message, where the Python adapter reads True and the bridge 1.
  using sensor_msgs::flux_msg::PointCloud2;
  sensor_msgs::msg::PointCloud2 m;
  Frame off = frame_of<PointCloud2>(m);
  m.is_dense = true;
  Frame on = frame_of<PointCloud2>(m);
  auto * a = static_cast<std::uint8_t *>(off.data());
  const auto * b = static_cast<const std::uint8_t *>(on.data());
  const auto at = std::mismatch(a, a + off.size, b).first - a;
  ASSERT_LT(static_cast<std::size_t>(at), off.size);
  a[at] = 0xCA;
  const PointCloud2::View v(off.data(), off.size);
  EXPECT_EQ(byte_of(v.is_dense()), 1);
  EXPECT_EQ(byte_of(sensor_msgs::flux_msg::frame_to_msg(v).is_dense), 1);
}

}  // namespace
