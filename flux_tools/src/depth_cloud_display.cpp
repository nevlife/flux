#include "flux_tools/rviz/depth_cloud_display.hpp"

#include "flux/qos.hpp"
#include "flux_tools/depth_projection.hpp"
#include "flux_tools/rviz/topic_list.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include <QString>
#include <image_transport/camera_common.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "sensor_msgs/flux/image.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace flux_tools::rviz
{

namespace
{

using Image = sensor_msgs::flux_msg::Image;
using rviz_common::properties::StatusProperty;

constexpr std::uint32_t kPointStep = 3 * sizeof(float);

void set_xyz_fields(sensor_msgs::msg::PointCloud2 & m)
{
  m.fields.resize(3);
  const char * names[] = {"x", "y", "z"};
  for (std::size_t i = 0; i < 3; ++i) {
    m.fields[i].name = names[i];
    m.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    m.fields[i].count = 1;
    m.fields[i].offset = static_cast<std::uint32_t>(i * sizeof(float));
  }
  m.point_step = kPointStep;
  m.is_bigendian = false;
}

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
    "Max Range", 0.0f, "Points farther than this, in meters, are dropped. 0 means no limit.",
    this);
  min_range_property_->setMin(0.0f);
  max_range_property_->setMin(0.0f);
  common_ = std::make_unique<rviz_default_plugins::PointCloudCommon>(this);
}

DepthCloudDisplay::~DepthCloudDisplay()
{
  unsubscribe();
}

void DepthCloudDisplay::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  common_->initialize(context_, scene_node_);
}

void DepthCloudDisplay::onEnable()
{
  subscribe();
  subscribeCameraInfo();
}

void DepthCloudDisplay::onDisable()
{
  unsubscribe();
  common_->onDisable();
}

void DepthCloudDisplay::reset()
{
  Display::reset();
  common_->reset();
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
      topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
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

sensor_msgs::msg::PointCloud2::SharedPtr DepthCloudDisplay::takeFrame()
{
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    info = info_;
  }
  if (!info) {
    return nullptr;
  }

  flux::FrameView frame = sub_->take();
  if (!frame) {
    if (sub_->attached() && frames_ == 0) {
      setStatus(StatusProperty::Warn, "Topic", "Attached, no frame yet");
    }
    return nullptr;
  }

  Image::View view(frame);
  const std::uint32_t w = view.width();
  const std::uint32_t h = view.height();
  const std::uint32_t step = view.step();
  const std::string_view encoding = view.encoding();
  const flux::wire::Span<const std::uint8_t> data = view.data();
  if (!view.ok__()) {
    setStatus(StatusProperty::Error, "Message", "Frame does not parse as sensor_msgs/Image");
    return nullptr;
  }

  std::size_t pixel_bytes = 0;
  if (encoding == "16UC1" || encoding == "mono16") {
    pixel_bytes = sizeof(std::uint16_t);
  } else if (encoding == "32FC1") {
    pixel_bytes = sizeof(float);
  } else {
    setStatus(
      StatusProperty::Error, "Message",
      QString("Encoding '%1' is not a depth encoding; expected 16UC1, mono16 or 32FC1")
        .arg(QString::fromUtf8(encoding.data(), static_cast<int>(encoding.size()))));
    return nullptr;
  }
  if (w == 0 || h == 0 || step < std::size_t{w} * pixel_bytes ||
    data.size() < std::size_t{step} * h)
  {
    setStatus(StatusProperty::Error, "Message", "Frame shorter than height x step");
    return nullptr;
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
        .arg(w).arg(h).arg(roi_w / bx).arg(roi_h / by));
    return nullptr;
  }
  if (info->p[0] == 0.0 || info->p[5] == 0.0) {
    setStatus(
      StatusProperty::Error, "Camera Info", "CameraInfo has no focal length: p[0] or p[5] is 0");
    return nullptr;
  }
  setStatus(StatusProperty::Ok, "Camera Info", "OK");

  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec = view.header__sec();
  msg->header.stamp.nanosec = view.header__nanosec();
  msg->header.frame_id = std::string(view.header__frame_id());
  set_xyz_fields(*msg);
  msg->data.resize(std::size_t{w} * h * kPointStep);

  const Projection proj = projection_of(*info);
  const float lo = min_range_property_->getFloat();
  const float hi = max_range_property_->getFloat();
  auto * out = reinterpret_cast<float *>(msg->data.data());
  const std::size_t count = pixel_bytes == sizeof(std::uint16_t)
    ? project_depth<std::uint16_t>(data.data(), w, h, step, proj, lo, hi, out)
    : project_depth<float>(data.data(), w, h, step, proj, lo, hi, out);

  msg->height = 1;
  msg->width = static_cast<std::uint32_t>(count);
  msg->row_step = static_cast<std::uint32_t>(count * kPointStep);
  msg->is_dense = true;
  msg->data.resize(count * kPointStep);

  ++frames_;
  setStatus(
    StatusProperty::Ok, "Topic",
    QString("%1 frames, %2 lost, %3 points")
      .arg(frames_)
      .arg(static_cast<qulonglong>(sub_->lost()))
      .arg(static_cast<qulonglong>(count)));
  return msg;
}

// The view lives inside takeFrame(), so the slot is back with the publisher before the
// transformers and the TF lookup run on the cloud.
void DepthCloudDisplay::update(float wall_dt, float ros_dt)
{
  if (sub_) {
    if (sensor_msgs::msg::PointCloud2::SharedPtr msg = takeFrame()) {
      common_->addMessage(msg);
    }
  }
  common_->update(wall_dt, ros_dt);
}

}  // namespace flux_tools::rviz

PLUGINLIB_EXPORT_CLASS(flux_tools::rviz::DepthCloudDisplay, rviz_common::Display)
