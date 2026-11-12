#ifndef FLUX_TOOLS_CLOUD_FROM_FRAME_HPP
#define FLUX_TOOLS_CLOUD_FROM_FRAME_HPP

#include "sensor_msgs/flux/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

// A PointCloud2 frame to the message the stock PointCloudCommon renders. Apart from the rviz
// display so it can be tested without a render context.
namespace flux_tools
{

namespace detail
{

struct XyzOffsets
{
  std::uint32_t x = 0;
  std::uint32_t y = 0;
  std::uint32_t z = 0;
  bool found = false;
};

inline auto xyz_offsets(const sensor_msgs::msg::PointCloud2 & m) -> XyzOffsets
{
  XyzOffsets o;
  int seen = 0;
  for (const auto & f : m.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      continue;
    }
    if (f.name == "x") {
      o.x = f.offset;
      ++seen;
    } else if (f.name == "y") {
      o.y = f.offset;
      ++seen;
    } else if (f.name == "z") {
      o.z = f.offset;
      ++seen;
    }
  }
  o.found = seen == 3;
  return o;
}

// 0 for a datatype PointField does not define.
inline auto field_bytes(std::uint8_t datatype) -> std::uint32_t
{
  using F = sensor_msgs::msg::PointField;
  switch (datatype) {
    case F::INT8:
    case F::UINT8:
      return 1;
    case F::INT16:
    case F::UINT16:
      return 2;
    case F::INT32:
    case F::UINT32:
    case F::FLOAT32:
      return 4;
    case F::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

inline auto finite_at(const std::uint8_t * p, std::uint32_t at) -> bool
{
  float v = 0;
  std::memcpy(&v, p + at, sizeof v);
  return std::isfinite(v);
}

// The stock display copies the cloud once to drop NaN points before PointCloudCommon sees it.
// Here that copy is the one that moves the points out of the slot, so the cost is the same.
inline void copy_valid_points(
  const std::uint8_t * data, std::size_t n, const XyzOffsets & o, sensor_msgs::msg::PointCloud2 & m)
{
  const std::uint32_t step = m.point_step;
  m.data.reserve(n * step);
  const std::uint8_t * run = nullptr;
  std::size_t run_len = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint8_t * p = data + i * step;
    if (finite_at(p, o.x) && finite_at(p, o.y) && finite_at(p, o.z)) {
      if (run == nullptr) {
        run = p;
      }
      ++run_len;
    } else if (run != nullptr) {
      m.data.insert(m.data.end(), run, run + run_len * step);
      run = nullptr;
      run_len = 0;
    }
  }
  if (run != nullptr) {
    m.data.insert(m.data.end(), run, run + run_len * step);
  }
  m.height = 1;
  m.width = static_cast<std::uint32_t>(m.data.size() / step);
  m.row_step = m.data.size();
}

}  // namespace detail

// The points of the frame whose x, y and z are finite, or nullptr with `error` saying why the
// frame is refused. Every size the frame states is checked against the bytes it has before
// anything is copied, so a frame cannot make this allocate or read more than it holds.
// `view` over a frame and, when its fields are past its slot, its publisher's arena: a View made
// from the FrameView maps both.
inline auto cloud_from_frame(
  const sensor_msgs::flux_msg::PointCloud2::View & view,
  std::string & error) -> sensor_msgs::msg::PointCloud2::SharedPtr
{
  using Cloud = sensor_msgs::flux_msg::PointCloud2;
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec = view.header__sec();
  msg->header.stamp.nanosec = view.header__nanosec();
  msg->header.frame_id = std::string(view.header__frame_id());
  msg->height = view.height();
  msg->width = view.width();
  msg->is_bigendian = view.is_bigendian();
  msg->point_step = view.point_step();
  msg->is_dense = view.is_dense();
  const flux::wire::Span<const std::uint8_t> data = view.data();
  // Disjoint names fit in the bytes the frame names; more than that means descriptors share them.
  std::size_t name_bytes = 0;
  for (std::size_t i = 0; i < view.fields__size(); ++i) {
    name_bytes += view.fields(i).name().size();
  }
  if (!view.ok__()) {
    error = "Frame does not parse as sensor_msgs/PointCloud2";
    return nullptr;
  }
  if (name_bytes > view.budget__()) {
    error = "Field names overlap: they hold more bytes than the frame";
    return nullptr;
  }
  // width x height fits in 64 bits; times point_step may not, so divide instead.
  const std::size_t points = std::size_t{msg->width} * msg->height;
  if (msg->point_step == 0 || points > data.size() / msg->point_step) {
    error = "Data size (" + std::to_string(data.size()) +
            " bytes) is less than width x height x point_step";
    return nullptr;
  }
  msg->fields.resize(view.fields__size());
  for (std::size_t i = 0; i < msg->fields.size(); ++i) {
    const Cloud::FieldsElem f = view.fields(i);
    msg->fields[i].name = std::string(f.name());
    msg->fields[i].offset = f.offset();
    msg->fields[i].datatype = f.datatype();
    msg->fields[i].count = f.count();
    // rviz's color transformers read any field at its offset in every point, unchecked.
    const std::uint32_t bytes = detail::field_bytes(f.datatype());
    const std::uint64_t end =
      std::uint64_t{f.offset()} + std::uint64_t{bytes} * std::max<std::uint32_t>(f.count(), 1);
    if (bytes == 0 || end > msg->point_step) {
      error = "Field '" + msg->fields[i].name + "' lies past point_step or has an unknown type";
      return nullptr;
    }
  }
  const detail::XyzOffsets xyz = detail::xyz_offsets(*msg);
  if (!xyz.found) {
    error = "No float32 x, y, z fields";
    return nullptr;
  }
  detail::copy_valid_points(data.data(), points, xyz, *msg);
  return msg;
}

// A frame whose fields are all in its slot.
inline auto cloud_from_frame(const void * frame, std::size_t frame_size, std::string & error)
  -> sensor_msgs::msg::PointCloud2::SharedPtr
{
  return cloud_from_frame(sensor_msgs::flux_msg::PointCloud2::View(frame, frame_size), error);
}

}  // namespace flux_tools

#endif  // FLUX_TOOLS_CLOUD_FROM_FRAME_HPP
