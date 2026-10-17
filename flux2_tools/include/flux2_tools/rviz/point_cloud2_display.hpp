#ifndef FLUX2_TOOLS_RVIZ_POINT_CLOUD2_DISPLAY_HPP
#define FLUX2_TOOLS_RVIZ_POINT_CLOUD2_DISPLAY_HPP

#ifndef Q_MOC_RUN
#include "flux2_tools/rviz/channel_display.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#endif

namespace flux2_tools::rviz
{

// rviz2 Display for a sensor_msgs/PointCloud2 flux2 channel. The frame is taken once per render
// tick and copied into a ROS message object with the NaN points dropped on the way; rendering,
// color transformers, selection and decay are the stock PointCloudCommon. The one copy is the
// same one the stock display makes when it filters, and it is what lets the slot go back to the
// publisher before the transformers run.
class PointCloud2Display : public ChannelDisplay
{
  Q_OBJECT

public:
  PointCloud2Display();
  PointCloud2Display(const PointCloud2Display &) = delete;
  PointCloud2Display & operator=(const PointCloud2Display &) = delete;
  PointCloud2Display(PointCloud2Display &&) = delete;
  PointCloud2Display & operator=(PointCloud2Display &&) = delete;
  ~PointCloud2Display() override = default;

  void onInitialize() override;
  void update(UpdateDt wall_dt, UpdateDt ros_dt) override;
  void reset() override;

protected:
  void onDisable() override;

private:
  sensor_msgs::msg::PointCloud2::SharedPtr takeFrame();

  std::unique_ptr<rviz_default_plugins::PointCloudCommon> common_;
};

}  // namespace flux2_tools::rviz

#endif  // FLUX2_TOOLS_RVIZ_POINT_CLOUD2_DISPLAY_HPP
