#ifndef FLUX_TOOLS_RVIZ_DEPTH_CLOUD_DISPLAY_HPP
#define FLUX_TOOLS_RVIZ_DEPTH_CLOUD_DISPLAY_HPP

#ifndef Q_MOC_RUN
#include "flux_tools/rviz/channel_display.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/string_property.hpp"

#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <OgreMaterial.h>
#include <OgrePixelFormat.h>
#include <OgreTexture.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
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
class DepthCloudDisplay : public ChannelDisplay
{
  Q_OBJECT

public:
  DepthCloudDisplay();
  DepthCloudDisplay(const DepthCloudDisplay &) = delete;
  DepthCloudDisplay & operator=(const DepthCloudDisplay &) = delete;
  DepthCloudDisplay(DepthCloudDisplay &&) = delete;
  DepthCloudDisplay & operator=(DepthCloudDisplay &&) = delete;
  ~DepthCloudDisplay() override;

  void onInitialize() override;
  void update(UpdateDt wall_dt, UpdateDt ros_dt) override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;
  void subscribe() override;
  void unsubscribe() override;

protected Q_SLOTS:
  void updateTopic() override;

private Q_SLOTS:
  void updateCameraInfoTopic();

private:
  void subscribeCameraInfo();
  void setupMaterial();
  bool ensureTexture(std::uint32_t width, std::uint32_t height, Ogre::PixelFormat format);
  bool ensureGrid(std::uint32_t width, std::uint32_t height);
  void destroyGrid();

  rviz_common::properties::StringProperty * info_topic_property_;
  rviz_common::properties::FloatProperty * min_range_property_;
  rviz_common::properties::FloatProperty * max_range_property_;
  rviz_common::properties::IntProperty * point_size_property_;
  rviz_common::properties::FloatProperty * alpha_property_;

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
  std::vector<std::uint8_t> packed_;
};

}  // namespace flux_tools::rviz

#endif  // FLUX_TOOLS_RVIZ_DEPTH_CLOUD_DISPLAY_HPP
