#include "flux_tools/cloud_from_frame.hpp"

#include <gtest/gtest.h>

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
std::vector<std::uint8_t> cloud(
  std::uint32_t width, std::uint32_t height, const std::vector<float> & xyz, std::uint32_t rgb = 12)
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
