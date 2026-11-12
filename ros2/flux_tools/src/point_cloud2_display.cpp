#include "flux_tools/rviz/point_cloud2_display.hpp"

#include "flux_tools/cloud_from_frame.hpp"
#include "rviz_common/properties/status_property.hpp"

#include <QString>
#include <pluginlib/class_list_macros.hpp>

#include "sensor_msgs/flux/point_cloud2.hpp"

#include <memory>
#include <string>

namespace flux_tools::rviz
{

using rviz_common::properties::StatusProperty;

PointCloud2Display::PointCloud2Display()
: ChannelDisplay(
    "flux channel carrying sensor_msgs/PointCloud2 frames.",
    sensor_msgs::flux_msg::PointCloud2::kFingerprint)
{
  common_ = std::make_unique<rviz_default_plugins::PointCloudCommon>(this);
}

void PointCloud2Display::onInitialize()
{
  ChannelDisplay::onInitialize();
  common_->initialize(context_, scene_node_);
}

void PointCloud2Display::onDisable()
{
  ChannelDisplay::onDisable();
  common_->onDisable();
}

void PointCloud2Display::reset()
{
  ChannelDisplay::reset();
  common_->reset();
}

auto PointCloud2Display::takeFrame() -> sensor_msgs::msg::PointCloud2::SharedPtr
{
  const flux::FrameView frame = take();
  if (!frame) {
    return nullptr;
  }
  std::string error;
  // From the FrameView, so a cloud past its slot is read from the publisher's arena, and a GPU
  // memory channel's from its aux and payload.
  sensor_msgs::msg::PointCloud2::SharedPtr msg =
    cloud_from_frame(view_of<sensor_msgs::flux_msg::PointCloud2::View>(frame), error);
  if (!msg) {
    setStatus(StatusProperty::Error, "Message", QString::fromStdString(error));
    return nullptr;
  }
  shown();
  return msg;
}

void PointCloud2Display::update(float wall_dt, float ros_dt)
{
  if (sensor_msgs::msg::PointCloud2::SharedPtr msg = takeFrame()) {
    common_->addMessage(msg);
  }
  common_->update(wall_dt, ros_dt);
}

}  // namespace flux_tools::rviz

PLUGINLIB_EXPORT_CLASS(flux_tools::rviz::PointCloud2Display, rviz_common::Display)
