#ifndef FLUX_TOOLS__DEPTH_PROJECTION_HPP_
#define FLUX_TOOLS__DEPTH_PROJECTION_HPP_

#include <sensor_msgs/msg/camera_info.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>

// Depth image to 3D points. Separate from the rviz display so it can be tested without a
// render context; the display owns only the subscription, the properties and the status.
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

inline float depth_to_meters(std::uint16_t raw) { return static_cast<float>(raw) * 0.001f; }
inline float depth_to_meters(float raw) { return raw; }

// One pass over the source straight into xyz float32 triples; there is no intermediate depth
// copy. Zero, negative and non-finite pixels are the no-return values of the cameras that
// publish these encodings and are dropped, as are pixels outside [lo, hi]. hi <= 0 means no far
// limit. `out` must have room for w * h triples. Returns the number written.
template <typename T>
std::size_t project_depth(
  const std::uint8_t * data, std::uint32_t w, std::uint32_t h, std::uint32_t step,
  const Projection & proj, float lo, float hi, float * out)
{
  std::size_t count = 0;
  for (std::uint32_t v = 0; v < h; ++v) {
    const std::uint8_t * row = data + std::size_t{step} * v;
    const float ray_y = (static_cast<float>(v) - proj.cy) * proj.inv_fy;
    for (std::uint32_t u = 0; u < w; ++u) {
      T raw;
      std::memcpy(&raw, row + std::size_t{u} * sizeof(T), sizeof raw);
      const float z = depth_to_meters(raw);
      if (!(z > 0.0f) || !std::isfinite(z) || z < lo || (hi > 0.0f && z > hi)) {
        continue;
      }
      out[0] = (static_cast<float>(u) - proj.cx) * proj.inv_fx * z;
      out[1] = ray_y * z;
      out[2] = z;
      out += 3;
      ++count;
    }
  }
  return count;
}

}  // namespace flux_tools

#endif  // FLUX_TOOLS__DEPTH_PROJECTION_HPP_
