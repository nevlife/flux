#include "flux_tools/rviz/image_display.hpp"

#include "flux/discovery.hpp"
#include "flux/qos.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
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

#include <exception>
#include <string>
#include <string_view>
#include <utility>

namespace flux_tools::rviz
{

namespace
{

using Image = sensor_msgs::flux_msg::Image;
using rviz_common::properties::StatusProperty;

// Only the encodings Ogre uploads as-is. Anything else (16-bit, float, YUV) needs a conversion
// pass and would reintroduce the copy this display exists to avoid.
Ogre::PixelFormat pixel_format(std::string_view encoding)
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
  return Ogre::PF_UNKNOWN;
}

int next_id()
{
  static int count = 0;
  return count++;
}

}  // namespace

ImageDisplay::ImageDisplay()
{
  topic_property_ = new rviz_common::properties::EditableEnumProperty(
    "Topic", "", "flux channel carrying sensor_msgs/Image frames.", this, SLOT(updateTopic()));
  connect(
    topic_property_, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
    &ImageDisplay::fillTopicList);
}

ImageDisplay::~ImageDisplay()
{
  unsubscribe();
  auto & tm = Ogre::TextureManager::getSingleton();
  for (Ogre::TexturePtr * t : {&texture_, &y_texture_, &uv_texture_}) {
    if (*t) {
      tm.remove(*t);
    }
  }
}

void ImageDisplay::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
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

bool ImageDisplay::ensureNv12Textures(std::uint32_t width, std::uint32_t height)
{
  if (y_texture_ && width == width_ && height == height_ && nv12_active_) {
    return true;
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
  if (!y_texture_ || !uv_texture_) {
    return false;
  }
  Ogre::Pass * pass = nv12_material_->getTechnique(0)->getPass(0);
  pass->getTextureUnitState(0)->setTexture(y_texture_);
  pass->getTextureUnitState(1)->setTexture(uv_texture_);
  screen_rect_->setMaterial(nv12_material_);
  width_ = width;
  height_ = height;
  format_ = Ogre::PF_UNKNOWN;
  nv12_active_ = true;
  return true;
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

void ImageDisplay::onEnable()
{
  subscribe();
}

void ImageDisplay::onDisable()
{
  unsubscribe();
  reset();
}

void ImageDisplay::reset()
{
  Display::reset();
  frames_ = 0;
}

void ImageDisplay::updateTopic()
{
  unsubscribe();
  reset();
  if (isEnabled()) {
    subscribe();
  }
}

void ImageDisplay::fillTopicList(rviz_common::properties::EditableEnumProperty * property)
{
  property->clearOptions();
  for (const flux::TopicView & t : flux::enumerate_topics()) {
    if (t.fingerprint == Image::kFingerprint && t.domain == flux::process_domain()) {
      property->addOptionStd(t.key);
    }
  }
}

void ImageDisplay::subscribe()
{
  const std::string topic = topic_property_->getStdString();
  if (topic.empty()) {
    setStatus(StatusProperty::Error, "Topic", "No topic set");
    return;
  }
  // depth 1: the newest frame only, the same policy a viewer wants. max_borrow 1: update() holds
  // one view and drops it before returning, so a second lease is never needed.
  flux::QoS qos;
  qos.depth = 1;
  qos.max_borrow = 1;
  try {
    sub_ = std::make_unique<flux::ros::Subscription>(
      *node_, topic, Image::kFingerprint, flux::ros::Subscription::Callback{}, qos);
  } catch (const std::exception & e) {
    setStatus(StatusProperty::Error, "Topic", QString("Subscribe failed: ") + e.what());
    return;
  }
  setStatus(StatusProperty::Warn, "Topic", "Waiting for a publisher");
}

void ImageDisplay::unsubscribe()
{
  sub_.reset();
}

bool ImageDisplay::ensureTexture(
  std::uint32_t width, std::uint32_t height, Ogre::PixelFormat format)
{
  if (texture_ && width == width_ && height == height_ && format == format_ && !nv12_active_) {
    return true;
  }
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
    texture_.reset();
  }
  const std::string name = "FluxImageTexture" + std::to_string(next_id());
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    name, material_->getGroup(), Ogre::TEX_TYPE_2D, width, height, 0, format,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  if (!texture_) {
    return false;
  }
  material_->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(texture_);
  screen_rect_->setMaterial(material_);
  width_ = width;
  height_ = height;
  format_ = format;
  nv12_active_ = false;
  return true;
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
  if (!sub_) {
    return;
  }

  flux::FrameView frame = sub_->take();
  if (!frame) {
    if (sub_->attached() && frames_ == 0) {
      setStatus(StatusProperty::Warn, "Topic", "Attached, no frame yet");
    }
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
    if (!ensureNv12Textures(width, height)) {
      setStatus(StatusProperty::Error, "Image", "Texture allocation failed");
      return;
    }
    Ogre::PixelBox ybox(width, height, 1, Ogre::PF_BYTE_L, const_cast<std::uint8_t *>(data.data()));
    ybox.rowPitch = step;
    ybox.slicePitch = ybox.rowPitch * height;
    y_texture_->getBuffer()->blitFromMemory(ybox);
    Ogre::PixelBox uvbox(
      width / 2, height / 2, 1, Ogre::PF_BYTE_LA,
      const_cast<std::uint8_t *>(data.data() + y_bytes));
    uvbox.rowPitch = step / 2;
    uvbox.slicePitch = uvbox.rowPitch * (height / 2);
    uv_texture_->getBuffer()->blitFromMemory(uvbox);
  } else {
    const Ogre::PixelFormat format = pixel_format(encoding);
    if (format == Ogre::PF_UNKNOWN) {
      setStatus(
        StatusProperty::Error, "Image",
        QString("Unsupported encoding [%1]")
          .arg(QString::fromUtf8(encoding.data(), encoding.size())));
      return;
    }
    const std::size_t bpp = Ogre::PixelUtil::getNumElemBytes(format);
    if (
      width == 0 || height == 0 || step < width * bpp || data.size() < std::size_t{step} * height) {
      setStatus(StatusProperty::Error, "Image", "Frame data shorter than width x height x step");
      return;
    }
    if (!ensureTexture(width, height, format)) {
      setStatus(StatusProperty::Error, "Image", "Texture allocation failed");
      return;
    }
    // The upload reads the slot directly; `frame` is released when this function returns.
    Ogre::PixelBox box(width, height, 1, format, const_cast<std::uint8_t *>(data.data()));
    box.rowPitch = step / bpp;
    box.slicePitch = box.rowPitch * height;
    texture_->getBuffer()->blitFromMemory(box);
  }

  ++frames_;
  fitRectangleToImage();
  setStatus(
    StatusProperty::Ok, "Topic",
    QString("%1 frames, %2 lost").arg(frames_).arg(static_cast<qulonglong>(sub_->lost())));
  setStatus(
    StatusProperty::Ok, "Image",
    QString("%1x%2 %3")
      .arg(width)
      .arg(height)
      .arg(QString::fromUtf8(encoding.data(), encoding.size())));
  context_->queueRender();
}

}  // namespace flux_tools::rviz

PLUGINLIB_EXPORT_CLASS(flux_tools::rviz::ImageDisplay, rviz_common::Display)
