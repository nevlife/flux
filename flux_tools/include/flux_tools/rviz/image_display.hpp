#ifndef FLUX_TOOLS__RVIZ__IMAGE_DISPLAY_HPP_
#define FLUX_TOOLS__RVIZ__IMAGE_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include "flux/ros/subscription.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/render_panel.hpp"

#include <rclcpp/node.hpp>

#include <OgreMaterial.h>
#include <OgrePixelFormat.h>
#include <OgreTexture.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#endif

namespace Ogre
{
class Rectangle2D;
}

namespace flux_tools::rviz
{

// rviz2 Display that reads a sensor_msgs/Image flux channel from inside the rviz2 process.
// Derives from Display rather than RosTopicDisplay: the latter owns an rclcpp subscription, and
// a flux channel is not a DDS topic. One frame is taken per render tick on the GUI thread and
// its bytes go straight into the Ogre texture; the view is dropped before the tick ends, so the
// slot is byte-locked for the upload only.
class ImageDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  ImageDisplay();
  ~ImageDisplay() override;

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
  void setupScreenRectangle();
  void setupRenderPanel();
  void subscribe();
  void unsubscribe();
  bool ensureTexture(std::uint32_t width, std::uint32_t height, Ogre::PixelFormat format);
  bool ensureNv12Textures(std::uint32_t width, std::uint32_t height);
  void setupNv12Material(const std::string & id);
  void fitRectangleToImage();

  rviz_common::properties::EditableEnumProperty * topic_property_;

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<flux::ros::Subscription> sub_;

  std::unique_ptr<Ogre::Rectangle2D> screen_rect_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
  // NV12 goes up as two textures (Y, interleaved UV) and a fragment shader does YUV to RGB, so
  // the GUI thread only uploads planes.
  Ogre::MaterialPtr nv12_material_;
  Ogre::TexturePtr y_texture_;
  Ogre::TexturePtr uv_texture_;
  bool nv12_active_ = false;
  std::unique_ptr<rviz_common::RenderPanel> render_panel_;

  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  Ogre::PixelFormat format_ = Ogre::PF_UNKNOWN;
  std::uint64_t frames_ = 0;
};

}  // namespace flux_tools::rviz

#endif  // FLUX_TOOLS__RVIZ__IMAGE_DISPLAY_HPP_
