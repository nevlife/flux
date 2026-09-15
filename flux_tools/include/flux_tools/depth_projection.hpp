#ifndef FLUX_TOOLS__DEPTH_PROJECTION_HPP_
#define FLUX_TOOLS__DEPTH_PROJECTION_HPP_

#include <sensor_msgs/msg/camera_info.hpp>

#include <cstdint>

// CameraInfo to the pinhole parameters the vertex shader back-projects with. Separate from the
// rviz display so it can be tested without a render context.
namespace flux_tools
{

// Pixel to ray, with the principal point and focal length already divided out. Rebuilt per
// frame because CameraInfo can change resolution, binning or ROI without the channel changing.
struct Projection
{
  float cx = 0.0f;
  float cy = 0.0f;
  float inv_fx = 0.0f;
  float inv_fy = 0.0f;
};

// p, not k: on a rectified stereo pair p carries the calibration that matches the image.
inline Projection projection_of(const sensor_msgs::msg::CameraInfo & info)
{
  const double sx = info.binning_x > 1 ? 1.0 / info.binning_x : 1.0;
  const double sy = info.binning_y > 1 ? 1.0 / info.binning_y : 1.0;
  Projection p;
  p.cx = static_cast<float>((info.p[2] - info.roi.x_offset) * sx);
  p.cy = static_cast<float>((info.p[6] - info.roi.y_offset) * sy);
  p.inv_fx = static_cast<float>(1.0 / (info.p[0] * sx));
  p.inv_fy = static_cast<float>(1.0 / (info.p[5] * sy));
  return p;
}

}  // namespace flux_tools

#endif  // FLUX_TOOLS__DEPTH_PROJECTION_HPP_
