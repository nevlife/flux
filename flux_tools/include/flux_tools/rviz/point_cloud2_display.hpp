#ifndef FLUX_TOOLS__RVIZ__POINT_CLOUD2_DISPLAY_HPP_
#define FLUX_TOOLS__RVIZ__POINT_CLOUD2_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include "flux/ros/subscription.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#endif

namespace flux_tools::rviz
{

// rviz2 Display for a sensor_msgs/PointCloud2 flux channel. The frame is taken once per render
// tick and copied into a ROS message object with the NaN points dropped on the way; rendering,
// color transformers, selection and decay are the stock PointCloudCommon. The one copy is the
// same one the stock display makes when it filters, and it is what lets the slot go back to the
// publisher before the transformers run.
class PointCloud2Display : public rviz_common::Display
{
  Q_OBJECT

public:
  PointCloud2Display();
  ~PointCloud2Display() override;

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateTopic();
  void fillTopicList(rviz_common::properties::EditableEnumProperty * property);

private:
  void subscribe();
  void unsubscribe();
  sensor_msgs::msg::PointCloud2::SharedPtr takeFrame();

  rviz_common::properties::EditableEnumProperty * topic_property_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<flux::ros::Subscription> sub_;
  std::unique_ptr<rviz_default_plugins::PointCloudCommon> common_;
  std::uint64_t frames_ = 0;
};

}  // namespace flux_tools::rviz

#endif  // FLUX_TOOLS__RVIZ__POINT_CLOUD2_DISPLAY_HPP_
