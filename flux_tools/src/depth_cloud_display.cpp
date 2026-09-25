#include "flux_tools/rviz/depth_cloud_display.hpp"

#include "flux/qos.hpp"
#include "flux_tools/depth_projection.hpp"
#include "flux_tools/rviz/topic_list.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_rendering/material_manager.hpp"

#include <QString>
#include <image_transport/camera_common.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <OgreGpuProgramParams.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreHighLevelGpuProgram.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreManualObject.h>
#include <OgrePass.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>

#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>
#include <string_view>

namespace flux_tools::rviz
{

namespace
{

using Image = sensor_msgs::flux_msg::Image;
using rviz_common::properties::StatusProperty;

// Far limit of the depth-to-color ramp when Max Range is 0.
constexpr float kColorSpanMeters = 10.0f;

int next_id()
{
  static int count = 0;
  return count++;
}

// The texture samples 16-bit as value / 65535 and float as-is; the scale brings both to meters.
struct DepthFormat
{
  Ogre::PixelFormat format = Ogre::PF_UNKNOWN;
  std::size_t pixel_bytes = 0;
  float to_meters = 0.0f;
};

DepthFormat depth_format(std::string_view encoding)
{
  if (encoding == "16UC1" || encoding == "mono16") {
    return {Ogre::PF_L16, sizeof(std::uint16_t), 65.535f};
  }
  if (encoding == "32FC1") {
    return {Ogre::PF_FLOAT32_R, sizeof(float), 1.0f};
  }
  return {};
}

// gl_Vertex.xy is the pixel coordinate. NaN fails every comparison, so `z > 0.0` drops it with
// the zeros; inf is caught by the upper bound. A dropped point goes behind the far plane.
const char * kVertexProgram = R"(#version 120
uniform mat4 worldviewproj_matrix;
uniform sampler2D depth_tex;
uniform vec4 proj;
uniform vec4 range;
uniform vec4 extent;
void main() {
  vec2 px = gl_Vertex.xy;
  float z = texture2DLod(depth_tex, (px + 0.5) * extent.xy, 0.0).r * range.z;
  bool ok = z > 0.0 && z < 1.0e30 && z >= range.x && (range.y <= 0.0 || z <= range.y);
  vec4 p = vec4((px.x - proj.x) * proj.z * z, (px.y - proj.y) * proj.w * z, z, 1.0);
  gl_Position = ok ? worldviewproj_matrix * p : vec4(0.0, 0.0, 2.0, 1.0);
  gl_PointSize = range.w;
  float t = clamp((z - range.x) / (extent.w - range.x), 0.0, 1.0);
  vec3 c = clamp(abs(mod(t * 4.0 + vec3(0.0, 4.0, 2.0), 6.0) - 3.0) - 1.0, 0.0, 1.0);
  gl_FrontColor = vec4(c, extent.z);
}
)";

const char * kFragmentProgram = R"(#version 120
void main() {
  gl_FragColor = gl_Color;
}
)";

}  // namespace

DepthCloudDisplay::DepthCloudDisplay()
{
  topic_property_ = new rviz_common::properties::EditableEnumProperty(
    "Topic", "", "flux channel carrying depth sensor_msgs/Image frames.", this,
    SLOT(updateTopic()));
  connect(
    topic_property_, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
    &DepthCloudDisplay::fillTopicList);
  info_topic_property_ = new rviz_common::properties::StringProperty(
    "Camera Info Topic", "",
    "DDS sensor_msgs/CameraInfo topic that calibrates the channel. Derived from the channel "
    "name until it is edited.",
    this, SLOT(updateCameraInfoTopic()));
  min_range_property_ = new rviz_common::properties::FloatProperty(
    "Min Range", 0.0f, "Points nearer than this, in meters, are dropped.", this);
  max_range_property_ = new rviz_common::properties::FloatProperty(
    "Max Range", 0.0f, "Points farther than this, in meters, are dropped. 0 means no limit.", this);
  point_size_property_ = new rviz_common::properties::IntProperty(
    "Point Size (Pixels)", 3, "Size of each point on screen.", this);
  alpha_property_ =
    new rviz_common::properties::FloatProperty("Alpha", 1.0f, "Opacity of the points.", this);
  min_range_property_->setMin(0.0f);
  max_range_property_->setMin(0.0f);
  point_size_property_->setMin(1);
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);
}

DepthCloudDisplay::~DepthCloudDisplay()
{
  unsubscribe();
  destroyGrid();
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
  }
}

void DepthCloudDisplay::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  setupMaterial();
}

void DepthCloudDisplay::setupMaterial()
{
  const std::string id = "FluxDepthCloud" + std::to_string(next_id());
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(id + "Material");
  const std::string group = material_->getGroup();

  auto & mgr = Ogre::HighLevelGpuProgramManager::getSingleton();
  Ogre::HighLevelGpuProgramPtr vp =
    mgr.createProgram(id + "VP", group, "glsl", Ogre::GPT_VERTEX_PROGRAM);
  vp->setSource(kVertexProgram);
  Ogre::HighLevelGpuProgramPtr fp =
    mgr.createProgram(id + "FP", group, "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
  fp->setSource(kFragmentProgram);

  material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material_->setCullingMode(Ogre::CULL_NONE);
  Ogre::Pass * pass = material_->getTechnique(0)->getPass(0);
  pass->setVertexProgram(vp->getName());
  pass->setFragmentProgram(fp->getName());
  // Ogre enables the vertex program point size only with attenuation on.
  pass->setPointAttenuation(true);
  Ogre::TextureUnitState * tu = pass->createTextureUnitState();
  tu->setName("depth_tex");
  tu->setBindingType(Ogre::TextureUnitState::BT_VERTEX);
  tu->setTextureFiltering(Ogre::TFO_NONE);
  tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  Ogre::GpuProgramParametersSharedPtr params = pass->getVertexProgramParameters();
  params->setNamedAutoConstant(
    "worldviewproj_matrix", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
  params->setNamedConstant("depth_tex", 0);
  params->setNamedConstant("proj", Ogre::Vector4::ZERO);
  params->setNamedConstant("range", Ogre::Vector4::ZERO);
  params->setNamedConstant("extent", Ogre::Vector4::ZERO);
}

bool DepthCloudDisplay::ensureTexture(
  std::uint32_t width, std::uint32_t height, Ogre::PixelFormat format)
{
  if (texture_ && width == width_ && height == height_ && format == format_) {
    return true;
  }
  auto & tm = Ogre::TextureManager::getSingleton();
  if (texture_) {
    tm.remove(texture_);
    texture_.reset();
  }
  texture_ = tm.createManual(
    "FluxDepthCloudTexture" + std::to_string(next_id()), material_->getGroup(), Ogre::TEX_TYPE_2D,
    width, height, 0, format, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  if (!texture_) {
    return false;
  }
  material_->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(texture_);
  format_ = format;
  return ensureGrid(width, height);
}

bool DepthCloudDisplay::ensureGrid(std::uint32_t width, std::uint32_t height)
{
  if (grid_ && width == width_ && height == height_) {
    return true;
  }
  destroyGrid();
  grid_ = scene_manager_->createManualObject("FluxDepthCloudGrid" + std::to_string(next_id()));
  grid_->setDynamic(false);
  grid_->estimateVertexCount(static_cast<std::size_t>(width) * height);
  grid_->begin(material_->getName(), Ogre::RenderOperation::OT_POINT_LIST, material_->getGroup());
  for (std::uint32_t v = 0; v < height; ++v) {
    for (std::uint32_t u = 0; u < width; ++u) {
      grid_->position(static_cast<float>(u), static_cast<float>(v), 0.0f);
    }
  }
  grid_->end();
  // The vertices are pixel coordinates; the points the shader makes of them can be anywhere.
  Ogre::AxisAlignedBox infinite;
  infinite.setInfinite();
  grid_->setBoundingBox(infinite);
  scene_node_->attachObject(grid_);
  width_ = width;
  height_ = height;
  return true;
}

void DepthCloudDisplay::destroyGrid()
{
  if (grid_) {
    scene_node_->detachObject(grid_);
    scene_manager_->destroyManualObject(grid_);
    grid_ = nullptr;
  }
  width_ = 0;
  height_ = 0;
}

void DepthCloudDisplay::onEnable()
{
  subscribe();
  subscribeCameraInfo();
  scene_node_->setVisible(true);
}

void DepthCloudDisplay::onDisable()
{
  unsubscribe();
  scene_node_->setVisible(false);
}

void DepthCloudDisplay::reset()
{
  Display::reset();
  destroyGrid();
  frames_ = 0;
}

void DepthCloudDisplay::updateTopic()
{
  unsubscribe();
  const std::string topic = topic_property_->getStdString();
  const std::string current = info_topic_property_->getStdString();
  if (!topic.empty() && (current.empty() || current == derived_info_topic_)) {
    derived_info_topic_ = image_transport::getCameraInfoTopic(topic);
    info_topic_property_->setStdString(derived_info_topic_);
  }
  reset();
  if (isEnabled()) {
    subscribe();
    subscribeCameraInfo();
  }
}

void DepthCloudDisplay::updateCameraInfoTopic()
{
  if (isEnabled()) {
    subscribeCameraInfo();
  }
}

void DepthCloudDisplay::fillTopicList(rviz_common::properties::EditableEnumProperty * property)
{
  fill_topic_list(property, Image::kFingerprint);
}

void DepthCloudDisplay::subscribe()
{
  const std::string topic = topic_property_->getStdString();
  if (topic.empty()) {
    setStatus(StatusProperty::Error, "Topic", "No topic set");
    return;
  }
  const auto qos = flux::QoS(1).max_borrow(1);
  try {
    sub_ = std::make_unique<flux::ros::Subscription>(*node_, topic, Image::kFingerprint, qos);
  } catch (const std::exception & e) {
    setStatus(StatusProperty::Error, "Topic", QString("Subscribe failed: ") + e.what());
    return;
  }
  setStatus(StatusProperty::Warn, "Topic", "Waiting for a publisher");
}

void DepthCloudDisplay::subscribeCameraInfo()
{
  info_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    info_.reset();
  }
  const std::string topic = info_topic_property_->getStdString();
  if (topic.empty()) {
    setStatus(StatusProperty::Error, "Camera Info", "No CameraInfo topic set");
    return;
  }
  // Best effort matches both a best effort and a reliable publisher; reliable matches only one.
  try {
    info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(info_mutex_);
        info_ = std::move(msg);
      });
  } catch (const std::exception & e) {
    setStatus(StatusProperty::Error, "Camera Info", QString("Subscribe failed: ") + e.what());
    return;
  }
  setStatus(StatusProperty::Warn, "Camera Info", "Waiting for CameraInfo");
}

void DepthCloudDisplay::unsubscribe()
{
  sub_.reset();
  info_sub_.reset();
  std::lock_guard<std::mutex> lock(info_mutex_);
  info_.reset();
}

// The view lives inside this function: the slot goes back to the publisher right after the
// texture upload, before rviz renders.
void DepthCloudDisplay::update(float, float)
{
  if (!sub_) {
    return;
  }
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    info = info_;
  }
  if (!info) {
    return;
  }

  flux::FrameView frame = sub_->take();
  if (!frame) {
    if (sub_->attached() && frames_ == 0) {
      setStatus(StatusProperty::Warn, "Topic", "Attached, no frame yet");
    }
    return;
  }

  Image::View view(frame);
  const std::uint32_t w = view.width();
  const std::uint32_t h = view.height();
  const std::uint32_t step = view.step();
  const std::string_view encoding = view.encoding();
  const flux::wire::Span<const std::uint8_t> data = view.data();
  if (!view.ok__()) {
    setStatus(StatusProperty::Error, "Message", "Frame does not parse as sensor_msgs/Image");
    return;
  }
  const DepthFormat fmt = depth_format(encoding);
  if (fmt.format == Ogre::PF_UNKNOWN) {
    setStatus(
      StatusProperty::Error, "Message",
      QString("Encoding '%1' is not a depth encoding; expected 16UC1, mono16 or 32FC1")
        .arg(QString::fromUtf8(encoding.data(), static_cast<int>(encoding.size()))));
    return;
  }
  if (
    w == 0 || h == 0 || step < std::size_t{w} * fmt.pixel_bytes ||
    data.size() < std::size_t{step} * h) {
    setStatus(StatusProperty::Error, "Message", "Frame shorter than height x step");
    return;
  }

  // Same consistency check the stock display makes: the calibration must describe this frame.
  const std::uint32_t bx = info->binning_x > 1 ? info->binning_x : 1;
  const std::uint32_t by = info->binning_y > 1 ? info->binning_y : 1;
  const std::uint32_t roi_w = info->roi.width > 0 ? info->roi.width : info->width;
  const std::uint32_t roi_h = info->roi.height > 0 ? info->roi.height : info->height;
  if (roi_w / bx != w || roi_h / by != h) {
    setStatus(
      StatusProperty::Error, "Camera Info",
      QString("Frame is %1 x %2, CameraInfo describes %3 x %4")
        .arg(w)
        .arg(h)
        .arg(roi_w / bx)
        .arg(roi_h / by));
    return;
  }
  if (info->p[0] == 0.0 || info->p[5] == 0.0) {
    setStatus(
      StatusProperty::Error, "Camera Info", "CameraInfo has no focal length: p[0] or p[5] is 0");
    return;
  }
  setStatus(StatusProperty::Ok, "Camera Info", "OK");

  if (!ensureTexture(w, h, fmt.format)) {
    setStatus(StatusProperty::Error, "Message", "Texture allocation failed");
    return;
  }
  Ogre::PixelBox box(w, h, 1, fmt.format, const_cast<std::uint8_t *>(data.data()));
  box.rowPitch = step / fmt.pixel_bytes;
  box.slicePitch = box.rowPitch * h;
  texture_->getBuffer()->blitFromMemory(box);

  const Projection proj = projection_of(*info);
  const float lo = min_range_property_->getFloat();
  const float hi = max_range_property_->getFloat();
  Ogre::GpuProgramParametersSharedPtr params =
    material_->getTechnique(0)->getPass(0)->getVertexProgramParameters();
  params->setNamedConstant("proj", Ogre::Vector4(proj.cx, proj.cy, proj.inv_fx, proj.inv_fy));
  params->setNamedConstant(
    "range",
    Ogre::Vector4(lo, hi, fmt.to_meters, static_cast<float>(point_size_property_->getInt())));
  params->setNamedConstant(
    "extent", Ogre::Vector4(
                1.0f / static_cast<float>(w), 1.0f / static_cast<float>(h),
                alpha_property_->getFloat(), hi > lo ? hi : lo + kColorSpanMeters));

  const std::string frame_id(view.header__frame_id());
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(
        frame_id, rclcpp::Time(view.header__sec(), view.header__nanosec(), RCL_ROS_TIME), position,
        orientation)) {
    setMissingTransformToFixedFrame(frame_id);
    return;
  }
  setTransformOk();
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  ++frames_;
  setStatus(
    StatusProperty::Ok, "Topic",
    QString("%1 frames, %2 lost").arg(frames_).arg(static_cast<qulonglong>(sub_->lost())));
  context_->queueRender();
}

}  // namespace flux_tools::rviz

PLUGINLIB_EXPORT_CLASS(flux_tools::rviz::DepthCloudDisplay, rviz_common::Display)
