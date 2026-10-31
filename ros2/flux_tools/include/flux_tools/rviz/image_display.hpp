#ifndef FLUX_TOOLS_RVIZ_IMAGE_DISPLAY_HPP
#define FLUX_TOOLS_RVIZ_IMAGE_DISPLAY_HPP

#ifndef Q_MOC_RUN
#include "flux_tools/rviz/channel_display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/render_panel.hpp"

#include <OgreMaterial.h>
#include <OgrePixelFormat.h>
#include <OgreTexture.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#endif

namespace Ogre
{
class Rectangle2D;
}

namespace flux_tools::rviz
{

// rviz2 Display that reads a sensor_msgs/Image flux channel from inside the rviz2 process.
// One frame is taken per render tick on the GUI thread and its bytes go straight into the Ogre
// texture; the view is dropped before the tick ends, so the slot is byte-locked for the upload
// only.
class ImageDisplay : public ChannelDisplay
{
  Q_OBJECT

public:
  ImageDisplay();
  ImageDisplay(const ImageDisplay &) = delete;
  auto operator=(const ImageDisplay &) -> ImageDisplay & = delete;
  ImageDisplay(ImageDisplay &&) = delete;
  auto operator=(ImageDisplay &&) -> ImageDisplay & = delete;
  ~ImageDisplay() override;

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  void onDisable() override;

private:
  void setupScreenRectangle();
  void setupRenderPanel();
  void ensureTexture(std::uint32_t width, std::uint32_t height, Ogre::PixelFormat format);
  void ensureNv12Textures(std::uint32_t width, std::uint32_t height);
  void setupNv12Material(const std::string & id);
  void setupRangeMaterial(const std::string & id);
  void setRange(float lo, float hi);
  void fitRectangleToImage();

  rviz_common::properties::BoolProperty * normalize_property_;
  rviz_common::properties::FloatProperty * min_property_;
  rviz_common::properties::FloatProperty * max_property_;

  std::unique_ptr<Ogre::Rectangle2D> screen_rect_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
  // NV12 goes up as two textures (Y, interleaved UV) and a fragment shader does YUV to RGB, so
  // the GUI thread only uploads planes.
  Ogre::MaterialPtr nv12_material_;
  Ogre::TexturePtr y_texture_;
  Ogre::TexturePtr uv_texture_;
  bool nv12_active_ = false;
  // 16-bit and float frames (depth) are uploaded as-is and mapped to a range in the shader.
  Ogre::MaterialPtr range_material_;
  std::unique_ptr<rviz_common::RenderPanel> render_panel_;

  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  Ogre::PixelFormat format_ = Ogre::PF_UNKNOWN;
  std::vector<std::uint8_t> packed_;
};

}  // namespace flux_tools::rviz

#endif  // FLUX_TOOLS_RVIZ_IMAGE_DISPLAY_HPP
