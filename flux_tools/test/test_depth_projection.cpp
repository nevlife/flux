#include "flux_tools/depth_projection.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace
{

constexpr float kFx = 500.0f;
constexpr float kFy = 400.0f;
constexpr float kCx = 320.0f;
constexpr float kCy = 240.0f;

sensor_msgs::msg::CameraInfo make_info(std::uint32_t w = 640, std::uint32_t h = 480)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = w;
  info.height = h;
  info.p[0] = kFx;
  info.p[2] = kCx;
  info.p[5] = kFy;
  info.p[6] = kCy;
  info.p[10] = 1.0;
  return info;
}

template <typename T>
std::vector<std::uint8_t> filled(std::uint32_t w, std::uint32_t h, T value)
{
  std::vector<std::uint8_t> buf(std::size_t{w} * h * sizeof(T));
  for (std::uint32_t i = 0; i < w * h; ++i) {
    std::memcpy(buf.data() + i * sizeof(T), &value, sizeof(T));
  }
  return buf;
}

}  // namespace

TEST(DepthProjection, ReadsFocalLengthAndCentreFromP)
{
  const flux_tools::Projection p = flux_tools::projection_of(make_info());
  EXPECT_FLOAT_EQ(p.cx, kCx);
  EXPECT_FLOAT_EQ(p.cy, kCy);
  EXPECT_FLOAT_EQ(p.inv_fx, 1.0f / kFx);
  EXPECT_FLOAT_EQ(p.inv_fy, 1.0f / kFy);
}

TEST(DepthProjection, BinningHalvesFocalLengthAndCentre)
{
  sensor_msgs::msg::CameraInfo info = make_info();
  info.binning_x = 2;
  info.binning_y = 2;
  const flux_tools::Projection p = flux_tools::projection_of(info);
  EXPECT_FLOAT_EQ(p.cx, kCx / 2.0f);
  EXPECT_FLOAT_EQ(p.cy, kCy / 2.0f);
  EXPECT_FLOAT_EQ(p.inv_fx, 2.0f / kFx);
  EXPECT_FLOAT_EQ(p.inv_fy, 2.0f / kFy);
}

TEST(DepthProjection, RoiOffsetShiftsTheCentre)
{
  sensor_msgs::msg::CameraInfo info = make_info();
  info.roi.x_offset = 40;
  info.roi.y_offset = 30;
  const flux_tools::Projection p = flux_tools::projection_of(info);
  EXPECT_FLOAT_EQ(p.cx, kCx - 40.0f);
  EXPECT_FLOAT_EQ(p.cy, kCy - 30.0f);
}

// The pinhole identity: a pixel at the principal point is straight ahead, and one pixel off
// centre is off by depth / focal length.
TEST(DepthProjection, PrincipalPointProjectsToTheOpticalAxis)
{
  const std::uint32_t w = 3;
  const std::uint32_t h = 3;
  sensor_msgs::msg::CameraInfo info = make_info(w, h);
  info.p[2] = 1.0;  // cx
  info.p[6] = 1.0;  // cy
  const flux_tools::Projection proj = flux_tools::projection_of(info);

  const std::vector<std::uint8_t> src = filled<float>(w, h, 2.0f);
  std::vector<float> out(std::size_t{w} * h * 3);
  const std::size_t n = flux_tools::project_depth<float>(
    src.data(), w, h, w * sizeof(float), proj, 0.0f, 0.0f, out.data());
  ASSERT_EQ(n, std::size_t{9});

  // Pixel (1, 1) is the principal point: x and y are 0, z is the depth.
  const float * centre = out.data() + 4 * 3;
  EXPECT_FLOAT_EQ(centre[0], 0.0f);
  EXPECT_FLOAT_EQ(centre[1], 0.0f);
  EXPECT_FLOAT_EQ(centre[2], 2.0f);

  // Pixel (2, 1) is one to the right: +x by depth / fx, y still 0.
  const float * right = out.data() + 5 * 3;
  EXPECT_FLOAT_EQ(right[0], 2.0f / kFx);
  EXPECT_FLOAT_EQ(right[1], 0.0f);

  // Pixel (1, 2) is one down: +y by depth / fy, which differs from x because fy differs.
  const float * below = out.data() + 7 * 3;
  EXPECT_FLOAT_EQ(below[0], 0.0f);
  EXPECT_FLOAT_EQ(below[1], 2.0f / kFy);
}

TEST(DepthProjection, Uint16IsMillimetres)
{
  const std::uint32_t w = 1;
  const std::uint32_t h = 1;
  sensor_msgs::msg::CameraInfo info = make_info(w, h);
  info.p[2] = 0.0;
  info.p[6] = 0.0;
  const flux_tools::Projection proj = flux_tools::projection_of(info);

  const std::vector<std::uint8_t> src = filled<std::uint16_t>(w, h, 1500);
  std::vector<float> out(3);
  const std::size_t n = flux_tools::project_depth<std::uint16_t>(
    src.data(), w, h, w * sizeof(std::uint16_t), proj, 0.0f, 0.0f, out.data());
  ASSERT_EQ(n, std::size_t{1});
  EXPECT_FLOAT_EQ(out[2], 1.5f);
}

TEST(DepthProjection, DropsNoReturnPixels)
{
  const std::uint32_t w = 4;
  const std::uint32_t h = 1;
  const flux_tools::Projection proj = flux_tools::projection_of(make_info(w, h));

  std::vector<std::uint8_t> src(std::size_t{w} * sizeof(float));
  const float values[] = {
    std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), 0.0f, 3.0f};
  for (std::uint32_t i = 0; i < w; ++i) {
    std::memcpy(src.data() + i * sizeof(float), &values[i], sizeof(float));
  }
  std::vector<float> out(std::size_t{w} * 3);
  const std::size_t n = flux_tools::project_depth<float>(
    src.data(), w, h, w * sizeof(float), proj, 0.0f, 0.0f, out.data());
  EXPECT_EQ(n, std::size_t{1});
  EXPECT_FLOAT_EQ(out[2], 3.0f);
}

TEST(DepthProjection, ZeroIsNoReturnForUint16Too)
{
  const std::uint32_t w = 2;
  const std::uint32_t h = 1;
  const flux_tools::Projection proj = flux_tools::projection_of(make_info(w, h));

  std::vector<std::uint8_t> src(std::size_t{w} * sizeof(std::uint16_t));
  const std::uint16_t values[] = {0, 800};
  std::memcpy(src.data(), values, sizeof(values));
  std::vector<float> out(std::size_t{w} * 3);
  const std::size_t n = flux_tools::project_depth<std::uint16_t>(
    src.data(), w, h, w * sizeof(std::uint16_t), proj, 0.0f, 0.0f, out.data());
  EXPECT_EQ(n, std::size_t{1});
  EXPECT_FLOAT_EQ(out[2], 0.8f);
}

TEST(DepthProjection, RangeLimitsClipNearAndFar)
{
  const std::uint32_t w = 5;
  const std::uint32_t h = 1;
  const flux_tools::Projection proj = flux_tools::projection_of(make_info(w, h));

  std::vector<std::uint8_t> src(std::size_t{w} * sizeof(float));
  const float values[] = {0.5f, 1.0f, 2.0f, 3.0f, 9.0f};
  std::memcpy(src.data(), values, sizeof(values));
  std::vector<float> out(std::size_t{w} * 3);

  std::size_t n = flux_tools::project_depth<float>(
    src.data(), w, h, w * sizeof(float), proj, 1.0f, 3.0f, out.data());
  EXPECT_EQ(n, std::size_t{3});

  // A max of 0 means no far limit, so only the near limit applies.
  n = flux_tools::project_depth<float>(
    src.data(), w, h, w * sizeof(float), proj, 1.0f, 0.0f, out.data());
  EXPECT_EQ(n, std::size_t{4});
}

// A padded row pitch is what a capture buffer hands over; reading it as width would shear the
// cloud, so every row must be found through step.
TEST(DepthProjection, PaddedRowPitchIsHonoured)
{
  const std::uint32_t w = 2;
  const std::uint32_t h = 2;
  const std::uint32_t step = 3 * sizeof(float);  // one float of padding per row
  sensor_msgs::msg::CameraInfo info = make_info(w, h);
  info.p[2] = 0.0;
  info.p[6] = 0.0;
  const flux_tools::Projection proj = flux_tools::projection_of(info);

  std::vector<std::uint8_t> src(std::size_t{step} * h, 0);
  const float rows[2][3] = {{1.0f, 2.0f, -1.0f}, {3.0f, 4.0f, -1.0f}};
  std::memcpy(src.data(), rows[0], step);
  std::memcpy(src.data() + step, rows[1], step);

  std::vector<float> out(std::size_t{w} * h * 3);
  const std::size_t n =
    flux_tools::project_depth<float>(src.data(), w, h, step, proj, 0.0f, 0.0f, out.data());
  ASSERT_EQ(n, std::size_t{4});
  EXPECT_FLOAT_EQ(out[2], 1.0f);
  EXPECT_FLOAT_EQ(out[5], 2.0f);
  EXPECT_FLOAT_EQ(out[8], 3.0f);
  EXPECT_FLOAT_EQ(out[11], 4.0f);
}
