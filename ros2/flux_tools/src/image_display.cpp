#include "flux_tools/rviz/image_display.hpp"

#include "flux/channel.hpp"
#include "flux_tools/whole_pixel_rows.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/render_window.hpp"

#include <QString>
#include <pluginlib/class_list_macros.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <OgreGpuProgramParams.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreHighLevelGpuProgram.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgrePass.h>
#include <OgreRectangle2D.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

namespace flux_tools::rviz
{

namespace
{

using Image = sensor_msgs::flux_msg::Image;
using rviz_common::properties::StatusProperty;

// Only the encodings Ogre uploads as-is. 16-bit and float go up as-is too and are scaled to a
// range in the fragment shader; YUV is NV12 only.
auto pixel_format(std::string_view encoding) -> Ogre::PixelFormat
{
  if (encoding == "rgb8") {
    return Ogre::PF_BYTE_RGB;
  }
  if (encoding == "bgr8") {
    return Ogre::PF_BYTE_BGR;
  }
  if (encoding == "rgba8") {
    return Ogre::PF_BYTE_RGBA;
  }
  if (encoding == "bgra8") {
    return Ogre::PF_BYTE_BGRA;
  }
  if (encoding == "mono8" || encoding == "8UC1") {
    return Ogre::PF_BYTE_L;
  }
  if (encoding == "mono16" || encoding == "16UC1") {
    return Ogre::PF_L16;
  }
  if (encoding == "32FC1") {
    return Ogre::PF_FLOAT32_R;
  }
  return Ogre::PF_UNKNOWN;
}

auto is_range_format(Ogre::PixelFormat format) -> bool
{
  return format == Ogre::PF_L16 || format == Ogre::PF_FLOAT32_R;
}

// One read pass over the slot, no copy. NaN and inf (32FC1 no-return pixels) are skipped, as
// are zeros in 16UC1 (the no-return value of the depth cameras that publish it).
template <typename T>
auto scan_range(const std::uint8_t * data, std::uint32_t w, std::uint32_t h, std::uint32_t step)
  -> std::pair<float, float>
{
  float lo = std::numeric_limits<float>::infinity();
  float hi = -std::numeric_limits<float>::infinity();
  for (std::uint32_t y = 0; y < h; ++y) {
    const std::uint8_t * row = data + std::size_t{step} * y;
    for (std::uint32_t x = 0; x < w; ++x) {
      T v;
      std::memcpy(&v, row + std::size_t{x} * sizeof(T), sizeof v);
      const float f = static_cast<float>(v);
      if (!std::isfinite(f) || f == 0.0f) {
        continue;
      }
      lo = std::min(lo, f);
      hi = std::max(hi, f);
    }
  }
  if (!(lo < hi)) {
    return {0.0f, 1.0f};
  }
  return {lo, hi};
}

auto next_id() -> int
{
  static int count = 0;
  return count++;
}

}  // namespace

ImageDisplay::ImageDisplay()
: ChannelDisplay("flux channel carrying sensor_msgs/Image frames.", Image::kFingerprint)
{
  normalize_property_ = new rviz_common::properties::BoolProperty(
    "Normalize Range", true,
    "16-bit and 32FC1 frames: map the smallest and largest value of each frame to black and "
    "white. Off: use Min Value and Max Value.",
    this);
  min_property_ = new rviz_common::properties::FloatProperty(
    "Min Value", 0.0f, "Value shown as black when Normalize Range is off, in the frame's units.",
    this);
  max_property_ = new rviz_common::properties::FloatProperty(
    "Max Value", 1.0f, "Value shown as white when Normalize Range is off, in the frame's units.",
    this);
}

ImageDisplay::~ImageDisplay()
{
  auto & tm = Ogre::TextureManager::getSingleton();
  for (Ogre::TexturePtr * t : {&texture_, &y_texture_, &uv_texture_}) {
    if (*t) {
      tm.remove(*t);
    }
  }
}

void ImageDisplay::onInitialize()
{
  ChannelDisplay::onInitialize();
  setupScreenRectangle();
  setupRenderPanel();
  render_panel_->getRenderWindow()->setupSceneAfterInit(
    [this](Ogre::SceneNode * scene_node) { scene_node->attachObject(screen_rect_.get()); });
}

void ImageDisplay::setupScreenRectangle()
{
  const std::string id = "FluxImageDisplay" + std::to_string(next_id());

  screen_rect_ = std::make_unique<Ogre::Rectangle2D>(true);
  screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(id + "Material");
  material_->setSceneBlending(Ogre::SBT_REPLACE);
  material_->setDepthWriteEnabled(false);
  material_->setDepthCheckEnabled(false);
  material_->setCullingMode(Ogre::CULL_NONE);

  Ogre::TextureUnitState * tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureFiltering(Ogre::TFO_NONE);
  tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

  Ogre::AxisAlignedBox infinite;
  infinite.setInfinite();
  screen_rect_->setBoundingBox(infinite);
  screen_rect_->setMaterial(material_);
  setupNv12Material(id);
  setupRangeMaterial(id);
}

void ImageDisplay::setupRangeMaterial(const std::string & id)
{
  auto & mgr = Ogre::HighLevelGpuProgramManager::getSingleton();
  const std::string group = material_->getGroup();

  Ogre::HighLevelGpuProgramPtr vp =
    mgr.createProgram(id + "RangeVP", group, "glsl", Ogre::GPT_VERTEX_PROGRAM);
  vp->setSource(
    "void main() {\n"
    "  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
    "  gl_TexCoord[0] = gl_MultiTexCoord0;\n"
    "}\n");

  // lo and hi are in sampled units: 16-bit textures sample as value / 65535, float as-is.
  Ogre::HighLevelGpuProgramPtr fp =
    mgr.createProgram(id + "RangeFP", group, "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
  fp->setSource(
    "uniform sampler2D tex;\n"
    "uniform float lo;\n"
    "uniform float hi;\n"
    "void main() {\n"
    "  float v = texture2D(tex, gl_TexCoord[0].xy).r;\n"
    "  float g = clamp((v - lo) / (hi - lo), 0.0, 1.0);\n"
    "  gl_FragColor = vec4(g, g, g, 1.0);\n"
    "}\n");

  range_material_ =
    rviz_rendering::MaterialManager::createMaterialWithNoLighting(id + "RangeMaterial");
  range_material_->setSceneBlending(Ogre::SBT_REPLACE);
  range_material_->setDepthWriteEnabled(false);
  range_material_->setDepthCheckEnabled(false);
  range_material_->setCullingMode(Ogre::CULL_NONE);
  Ogre::Pass * pass = range_material_->getTechnique(0)->getPass(0);
  pass->setVertexProgram(vp->getName());
  pass->setFragmentProgram(fp->getName());
  Ogre::TextureUnitState * tu = pass->createTextureUnitState();
  tu->setName("tex");
  tu->setTextureFiltering(Ogre::TFO_NONE);
  tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  Ogre::GpuProgramParametersSharedPtr params = pass->getFragmentProgramParameters();
  params->setNamedConstant("tex", 0);
  params->setNamedConstant("lo", 0.0f);
  params->setNamedConstant("hi", 1.0f);
}

void ImageDisplay::setRange(float lo, float hi)
{
  Ogre::GpuProgramParametersSharedPtr params =
    range_material_->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  params->setNamedConstant("lo", lo);
  params->setNamedConstant("hi", hi > lo ? hi : lo + 1.0f);
}

void ImageDisplay::setupNv12Material(const std::string & id)
{
  auto & mgr = Ogre::HighLevelGpuProgramManager::getSingleton();
  const std::string group = material_->getGroup();

  Ogre::HighLevelGpuProgramPtr vp =
    mgr.createProgram(id + "Nv12VP", group, "glsl", Ogre::GPT_VERTEX_PROGRAM);
  vp->setSource(
    "void main() {\n"
    "  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
    "  gl_TexCoord[0] = gl_MultiTexCoord0;\n"
    "}\n");

  // BT.601 limited range. The UV texture is luminance-alpha: U in .r, V in .a.
  Ogre::HighLevelGpuProgramPtr fp =
    mgr.createProgram(id + "Nv12FP", group, "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
  fp->setSource(
    "uniform sampler2D texY;\n"
    "uniform sampler2D texUV;\n"
    "void main() {\n"
    "  float y = 1.164 * (texture2D(texY, gl_TexCoord[0].xy).r - 0.0625);\n"
    "  vec2 uv = texture2D(texUV, gl_TexCoord[0].xy).ra - 0.5;\n"
    "  gl_FragColor = vec4(y + 1.596 * uv.y, y - 0.391 * uv.x - 0.813 * uv.y, y + 2.018 * uv.x, "
    "1.0);\n"
    "}\n");

  nv12_material_ =
    rviz_rendering::MaterialManager::createMaterialWithNoLighting(id + "Nv12Material");
  nv12_material_->setSceneBlending(Ogre::SBT_REPLACE);
  nv12_material_->setDepthWriteEnabled(false);
  nv12_material_->setDepthCheckEnabled(false);
  nv12_material_->setCullingMode(Ogre::CULL_NONE);
  Ogre::Pass * pass = nv12_material_->getTechnique(0)->getPass(0);
  pass->setVertexProgram(vp->getName());
  pass->setFragmentProgram(fp->getName());
  for (const char * sampler : {"texY", "texUV"}) {
    Ogre::TextureUnitState * tu = pass->createTextureUnitState();
    tu->setName(sampler);
    tu->setTextureFiltering(Ogre::TFO_BILINEAR);
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  }
  Ogre::GpuProgramParametersSharedPtr params = pass->getFragmentProgramParameters();
  params->setNamedConstant("texY", 0);
  params->setNamedConstant("texUV", 1);
}

void ImageDisplay::ensureNv12Textures(std::uint32_t width, std::uint32_t height)
{
  if (y_texture_ && width == width_ && height == height_ && nv12_active_) {
    return;
  }
  auto & tm = Ogre::TextureManager::getSingleton();
  if (y_texture_) {
    tm.remove(y_texture_);
    y_texture_.reset();
  }
  if (uv_texture_) {
    tm.remove(uv_texture_);
    uv_texture_.reset();
  }
  const std::string base = "FluxImageNv12" + std::to_string(next_id());
  const std::string group = nv12_material_->getGroup();
  y_texture_ = tm.createManual(
    base + "Y", group, Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_BYTE_L,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  uv_texture_ = tm.createManual(
    base + "UV", group, Ogre::TEX_TYPE_2D, width / 2, height / 2, 0, Ogre::PF_BYTE_LA,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  Ogre::Pass * pass = nv12_material_->getTechnique(0)->getPass(0);
  pass->getTextureUnitState(0)->setTexture(y_texture_);
  pass->getTextureUnitState(1)->setTexture(uv_texture_);
  screen_rect_->setMaterial(nv12_material_);
  width_ = width;
  height_ = height;
  format_ = Ogre::PF_UNKNOWN;
  nv12_active_ = true;
}

void ImageDisplay::setupRenderPanel()
{
  render_panel_ = std::make_unique<rviz_common::RenderPanel>();
  render_panel_->resize(640, 480);
  render_panel_->initialize(context_);
  setAssociatedWidget(render_panel_.get());
  render_panel_->getRenderWindow()->setObjectName(
    "FluxImageDisplayRenderWindow" + QString::number(next_id()));
}

void ImageDisplay::onDisable()
{
  ChannelDisplay::onDisable();
  reset();
}

void ImageDisplay::ensureTexture(
  std::uint32_t width, std::uint32_t height, Ogre::PixelFormat format)
{
  if (texture_ && width == width_ && height == height_ && format == format_ && !nv12_active_) {
    return;
  }
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
    texture_.reset();
  }
  const std::string name = "FluxImageTexture" + std::to_string(next_id());
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    name, material_->getGroup(), Ogre::TEX_TYPE_2D, width, height, 0, format,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  if (is_range_format(format)) {
    range_material_->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(texture_);
    screen_rect_->setMaterial(range_material_);
  } else {
    material_->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(texture_);
    screen_rect_->setMaterial(material_);
  }
  width_ = width;
  height_ = height;
  format_ = format;
  nv12_active_ = false;
}

void ImageDisplay::fitRectangleToImage()
{
  if (width_ == 0 || height_ == 0) {
    return;
  }
  const float win_w = static_cast<float>(render_panel_->width());
  const float win_h = static_cast<float>(render_panel_->height());
  if (win_w <= 0.0f || win_h <= 0.0f) {
    return;
  }
  const float img_aspect = static_cast<float>(width_) / static_cast<float>(height_);
  const float win_aspect = win_w / win_h;
  if (img_aspect > win_aspect) {
    const float y = win_aspect / img_aspect;
    screen_rect_->setCorners(-1.0f, y, 1.0f, -y, false);
  } else {
    const float x = img_aspect / win_aspect;
    screen_rect_->setCorners(-x, 1.0f, x, -1.0f, false);
  }
}

void ImageDisplay::update(float, float)
{
  const flux::FrameView frame = take();
  if (!frame) {
    fitRectangleToImage();
    return;
  }

  Image::View view(frame);
  const std::uint32_t width = view.width();
  const std::uint32_t height = view.height();
  const std::uint32_t step = view.step();
  const std::string_view encoding = view.encoding();
  flux::wire::Span<const std::uint8_t> data = view.data();
  if (!view.ok__()) {
    setStatus(StatusProperty::Error, "Image", "Frame does not parse as sensor_msgs/Image");
    return;
  }

  const bool nv12 = encoding == "nv12";
  if (nv12) {
    // step is the Y row pitch; the UV plane follows the Y plane with the same pitch.
    const std::size_t y_bytes = std::size_t{step} * height;
    if (width < 2 || height < 2 || step < width || data.size() < y_bytes + y_bytes / 2) {
      setStatus(StatusProperty::Error, "Image", "NV12 frame shorter than width x height x 1.5");
      return;
    }
    ensureNv12Textures(width, height);
    // PixelBox takes a non-const pointer; blitFromMemory only reads through it.
    // NOLINTNEXTLINE(*-const-cast)
    Ogre::PixelBox ybox(width, height, 1, Ogre::PF_BYTE_L, const_cast<std::uint8_t *>(data.data()));
    ybox.rowPitch = step;
    ybox.slicePitch = ybox.rowPitch * height;
    y_texture_->getBuffer()->blitFromMemory(ybox);
    std::size_t uv_pitch = 0;
    const std::uint8_t * uv = whole_pixel_rows(
      data.data() + y_bytes, std::size_t{width / 2} * 2, height / 2, step, 2, packed_, uv_pitch);
    Ogre::PixelBox uvbox(
      width / 2, height / 2, 1, Ogre::PF_BYTE_LA,
      const_cast<std::uint8_t *>(uv));  // NOLINT(*-const-cast)
    uvbox.rowPitch = uv_pitch;
    uvbox.slicePitch = uvbox.rowPitch * (height / 2);
    uv_texture_->getBuffer()->blitFromMemory(uvbox);
  } else {
    const Ogre::PixelFormat format = pixel_format(encoding);
    if (format == Ogre::PF_UNKNOWN) {
      setStatus(
        StatusProperty::Error, "Image",
        QString("Unsupported encoding [%1]")
          .arg(QString::fromUtf8(encoding.data(), static_cast<int>(encoding.size()))));
      return;
    }
    const std::size_t bpp = Ogre::PixelUtil::getNumElemBytes(format);
    if (
      width == 0 || height == 0 || step < width * bpp || data.size() < std::size_t{step} * height) {
      setStatus(StatusProperty::Error, "Image", "Frame data shorter than width x height x step");
      return;
    }
    ensureTexture(width, height, format);
    // The upload reads the slot directly; `frame` is released when this function returns.
    std::size_t pitch = 0;
    const std::uint8_t * rows =
      whole_pixel_rows(data.data(), width * bpp, height, step, bpp, packed_, pitch);
    Ogre::PixelBox box(
      width, height, 1, format, const_cast<std::uint8_t *>(rows));  // NOLINT(*-const-cast)
    box.rowPitch = pitch;
    box.slicePitch = box.rowPitch * height;
    texture_->getBuffer()->blitFromMemory(box);
    if (is_range_format(format)) {
      float lo = min_property_->getFloat();
      float hi = max_property_->getFloat();
      if (normalize_property_->getBool()) {
        std::tie(lo, hi) = format == Ogre::PF_L16
                             ? scan_range<std::uint16_t>(data.data(), width, height, step)
                             : scan_range<float>(data.data(), width, height, step);
      }
      const float unit = format == Ogre::PF_L16 ? 1.0f / 65535.0f : 1.0f;
      setRange(lo * unit, hi * unit);
    }
  }

  shown();
  fitRectangleToImage();
  setStatus(
    StatusProperty::Ok, "Image",
    QString("%1x%2 %3")
      .arg(width)
      .arg(height)
      .arg(QString::fromUtf8(encoding.data(), static_cast<int>(encoding.size()))));
  context_->queueRender();
}

}  // namespace flux_tools::rviz

PLUGINLIB_EXPORT_CLASS(flux_tools::rviz::ImageDisplay, rviz_common::Display)
