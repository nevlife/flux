#include "flux/channel.hpp"
#include "flux_tools/cloud_from_frame.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace
{

using Cloud = sensor_msgs::flux_msg::PointCloud2;

// x, y, z float32 at 0, 4, 8, and rgb at `rgb`; point_step 16.
auto cloud(
  std::uint32_t width, std::uint32_t height, const std::vector<float> & xyz,
  std::uint32_t rgb = 12) -> std::vector<std::uint8_t>
{
  std::vector<std::uint8_t> buf(4096);
  Cloud::Builder b(buf.data(), buf.size());
  b.set__header__frame_id("map");
  b.set__height(height);
  b.set__width(width);
  auto f = b.alloc__fields(4);
  const std::array<const char *, 4> names = {"x", "y", "z", "rgb"};
  for (std::uint32_t i = 0; i < 4; ++i) {
    f[i].set__name(names.at(i));
    f[i].set__offset(i < 3 ? 4 * i : rgb);
    f[i].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    f[i].set__count(1);
  }
  b.set__point_step(16);
  const std::size_t points = xyz.size() / 3;
  auto data = b.alloc__data(points * 16);
  for (std::size_t p = 0; p < points; ++p) {
    std::memcpy(data.data() + p * 16, &xyz[p * 3], 12);
  }
  EXPECT_TRUE(b.ok__());
  buf.resize(b.size__());
  return buf;
}

}  // namespace

TEST(CloudFromFrame, KeepsTheFinitePoints)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto frame = cloud(3, 1, {1, 2, 3, nan, 0, 0, 4, 5, 6});
  std::string error;
  const auto msg = flux_tools::cloud_from_frame(frame.data(), frame.size(), error);
  ASSERT_TRUE(msg) << error;
  EXPECT_EQ(msg->width, 2u);
  EXPECT_EQ(msg->header.frame_id, "map");
  ASSERT_EQ(msg->fields.size(), 4u);
  EXPECT_EQ(msg->fields[2].name, "z");
  float z = 0;
  std::memcpy(&z, msg->data.data() + 16 + 8, sizeof z);
  EXPECT_EQ(z, 6.0f);
}

// 2^31 x 2^31 x 16 is 2^66, which a 64-bit product wraps to 0 and lets past a size check.
TEST(CloudFromFrame, RefusesAGeometryWhoseByteCountWraps)
{
  const auto frame = cloud(1U << 31, 1U << 31, {1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 2, 3});
  std::string error;
  EXPECT_FALSE(flux_tools::cloud_from_frame(frame.data(), frame.size(), error));
  EXPECT_NE(error.find("less than width x height x point_step"), std::string::npos) << error;
}

// Every field name pointing at the whole frame: copying them would take fields x frame bytes.
TEST(CloudFromFrame, RefusesFieldNamesThatShareBytes)
{
  std::vector<std::uint8_t> buf(std::size_t{64} << 10);
  Cloud::Builder b(buf.data(), buf.size());
  b.set__height(1);
  b.set__width(1);
  b.set__point_step(16);
  const std::size_t n = (buf.size() - Cloud::kScalarBytes - 64) / Cloud::FieldsElem::kStride;
  b.alloc__fields(n);
  b.alloc__data(16);
  ASSERT_TRUE(b.ok__());
  const std::size_t size = b.size__();
  flux::wire::Desc fields{};
  std::memcpy(&fields, buf.data() + 24, sizeof fields);
  const flux::wire::Desc whole{0, static_cast<std::uint32_t>(size)};
  for (std::size_t i = 0; i < n; ++i) {
    std::memcpy(buf.data() + fields.off + i * Cloud::FieldsElem::kStride, &whole, sizeof whole);
  }
  std::string error;
  EXPECT_FALSE(flux_tools::cloud_from_frame(buf.data(), size, error));
  EXPECT_NE(error.find("overlap"), std::string::npos) << error;
}

// rviz's RGB8 transformer reads every point at this offset: 4 GiB past the data.
TEST(CloudFromFrame, RefusesAFieldPastThePoint)
{
  const auto frame = cloud(1, 1, {1, 2, 3}, 0xFFFFFF00U);
  std::string error;
  EXPECT_FALSE(flux_tools::cloud_from_frame(frame.data(), frame.size(), error));
  EXPECT_NE(error.find("'rgb' lies past point_step"), std::string::npos) << error;
}

// A cloud larger than its 512-byte slot: its points go to the publisher's arena, and the View made
// from the FrameView reads them there. The frame's bytes alone do not hold them.
TEST(CloudFromFrame, ReadsAPointCloudPastItsSlotFromTheArena)
{
  const std::string name = flux::detail::signpost_name(
    "/flux_tools_cloud" + std::to_string(getpid()), Cloud::kFingerprint);
  auto pub = flux::detail::Channel::create(name, 512, 2, Cloud::kFingerprint);
  auto sub = flux::detail::Channel::open(name, Cloud::kFingerprint);
  flux::WriteSlot slot = pub.loan();
  ASSERT_TRUE(slot);
  constexpr std::size_t kPoints = 100;
  constexpr std::size_t kStep = 12;
  Cloud::Builder b(slot);
  b.set__header__frame_id("map");
  b.set__height(1);
  b.set__width(static_cast<std::uint32_t>(kPoints));
  auto f = b.alloc__fields(3);
  for (std::uint32_t i = 0; i < 3; ++i) {
    f[i].set__name(std::string(1, static_cast<char>('x' + i)));
    f[i].set__offset(4 * i);
    f[i].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    f[i].set__count(1);
  }
  b.set__point_step(kStep);
  auto data = b.alloc__data(kPoints * kStep);
  ASSERT_EQ(data.size(), kPoints * kStep);
  for (std::size_t p = 0; p < kPoints; ++p) {
    const std::array<float, 3> xyz = {static_cast<float>(p), 1, 2};
    std::memcpy(data.data() + p * kStep, xyz.data(), kStep);
  }
  ASSERT_TRUE(b.ok__());
  ASSERT_EQ(slot.commit(b.size__()), flux::Published::Ok);
  const flux::FrameView frame = sub.take_blocking(1'000'000'000);
  ASSERT_TRUE(frame);
  ASSERT_FALSE(frame.heap().empty()) << "the points should be in the arena";
  std::string error;
  EXPECT_FALSE(flux_tools::cloud_from_frame(frame.data(), frame.size(), error));
  const auto msg = flux_tools::cloud_from_frame(Cloud::View(frame), error);
  ASSERT_TRUE(msg) << error;
  EXPECT_EQ(msg->width, kPoints);
  float x = 0;
  std::memcpy(&x, msg->data.data() + (kPoints - 1) * kStep, sizeof x);
  EXPECT_EQ(x, 99.0F);
}
