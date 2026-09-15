#ifndef FLUX_TOOLS__RVIZ__DEPTH_CLOUD_DISPLAY_HPP_
#define FLUX_TOOLS__RVIZ__DEPTH_CLOUD_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include "flux/ros/subscription.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/string_property.hpp"

#include <OgreMaterial.h>
#include <OgrePixelFormat.h>
#include <OgreTexture.h>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#endif

namespace Ogre
{
class ManualObject;
}

namespace flux_tools::rviz
{

// rviz2 Display that back-projects a depth sensor_msgs/Image flux channel into 3D points on the
// GPU. The depth frame goes from shared memory into a texture as-is; a vertex shader turns each
// pixel into a point with the CameraInfo intrinsics, so the CPU never touches a point and the
// frame rate does not depend on the point count. The CameraInfo comes from DDS, because it is a
// few hundred static bytes and every driver already publishes it there.
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
  void setupMaterial();
  bool ensureTexture(std::uint32_t width, std::uint32_t height, Ogre::PixelFormat format);
  bool ensureGrid(std::uint32_t width, std::uint32_t height);
  void destroyGrid();

  rviz_common::properties::EditableEnumProperty * topic_property_;
  rviz_common::properties::StringProperty * info_topic_property_;
  rviz_common::properties::FloatProperty * min_range_property_;
  rviz_common::properties::FloatProperty * max_range_property_;
  rviz_common::properties::IntProperty * point_size_property_;
  rviz_common::properties::FloatProperty * alpha_property_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<flux::ros::Subscription> sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  // Written by the executor thread, read by update() on the GUI thread.
  std::mutex info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;

  // The last name derived from the channel. A name still equal to it is ours to re-derive when
  // the channel changes; anything else was typed by the user and is left alone.
  std::string derived_info_topic_;

  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
  // One vertex per pixel, holding the pixel coordinate. The shader reads the depth of that
  // pixel from the texture, so the grid is rebuilt only when the resolution changes.
  Ogre::ManualObject * grid_ = nullptr;
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  Ogre::PixelFormat format_ = Ogre::PF_UNKNOWN;
  std::uint64_t frames_ = 0;
};

}  // namespace flux_tools::rviz

#endif  // FLUX_TOOLS__RVIZ__DEPTH_CLOUD_DISPLAY_HPP_
