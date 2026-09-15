#include "flux_tools/depth_projection.hpp"

#include <gtest/gtest.h>

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
