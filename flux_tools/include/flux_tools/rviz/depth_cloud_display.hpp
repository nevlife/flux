#ifndef FLUX_TOOLS__RVIZ__DEPTH_CLOUD_DISPLAY_HPP_
#define FLUX_TOOLS__RVIZ__DEPTH_CLOUD_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include "flux/ros/subscription.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#endif

namespace flux_tools::rviz
{

// rviz2 Display that back-projects a depth sensor_msgs/Image flux channel into 3D points.
// The depth frame comes from shared memory; the CameraInfo that calibrates it comes from DDS,
// because it is a few hundred static bytes and every driver already publishes it there.
// Rendering, transformers, decay and selection are the stock PointCloudCommon.
class DepthCloudDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  DepthCloudDisplay();
  ~DepthCloudDisplay() override;

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateTopic();
  void updateCameraInfoTopic();
  void fillTopicList(rviz_common::properties::EditableEnumProperty * property);

private:
  void subscribe();
  void subscribeCameraInfo();
  void unsubscribe();
  sensor_msgs::msg::PointCloud2::SharedPtr takeFrame();

  rviz_common::properties::EditableEnumProperty * topic_property_;
  rviz_common::properties::StringProperty * info_topic_property_;
  rviz_common::properties::FloatProperty * min_range_property_;
  rviz_common::properties::FloatProperty * max_range_property_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<flux::ros::Subscription> sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  // Written by the executor thread, read by update() on the GUI thread.
  std::mutex info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;

  // The last name derived from the channel. A name still equal to it is ours to re-derive when
  // the channel changes; anything else was typed by the user and is left alone.
  std::string derived_info_topic_;

  std::unique_ptr<rviz_default_plugins::PointCloudCommon> common_;
  std::uint64_t frames_ = 0;
};

}  // namespace flux_tools::rviz

#endif  // FLUX_TOOLS__RVIZ__DEPTH_CLOUD_DISPLAY_HPP_
