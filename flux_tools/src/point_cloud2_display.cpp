#include "flux_tools/rviz/point_cloud2_display.hpp"

#include "flux/qos.hpp"
#include "flux_tools/rviz/topic_list.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/validate_floats.hpp"

#include <QString>
#include <pluginlib/class_list_macros.hpp>

#include "sensor_msgs/flux/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <cstring>
#include <exception>
#include <memory>
#include <string>

namespace flux_tools::rviz
{

namespace
{

using Cloud = sensor_msgs::flux_msg::PointCloud2;
using rviz_common::properties::StatusProperty;

struct XyzOffsets
{
  std::uint32_t x = 0;
  std::uint32_t y = 0;
  std::uint32_t z = 0;
  bool found = false;
};

XyzOffsets xyz_offsets(const sensor_msgs::msg::PointCloud2 & m)
{
  XyzOffsets o;
  int seen = 0;
  for (const auto & f : m.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      continue;
    }
    if (f.name == "x") {
      o.x = f.offset;
      ++seen;
    } else if (f.name == "y") {
      o.y = f.offset;
      ++seen;
    } else if (f.name == "z") {
      o.z = f.offset;
      ++seen;
    }
  }
  o.found = seen == 3;
  return o;
}

bool finite_at(const std::uint8_t * p, std::uint32_t at)
{
  float v;
  std::memcpy(&v, p + at, sizeof v);
  return rviz_common::validateFloats(v);
}

// The stock display copies the cloud once to drop NaN points before PointCloudCommon sees it.
// Here that copy is the one that moves the points out of the slot, so the cost is the same.
void copy_valid_points(
  const Cloud::View & v, const XyzOffsets & o, sensor_msgs::msg::PointCloud2 & m)
{
  const flux::wire::Span<const std::uint8_t> data = v.data();
  const std::uint32_t step = m.point_step;
  const std::size_t n = std::size_t{m.width} * m.height;
  m.data.reserve(n * step);
  const std::uint8_t * run = nullptr;
  std::size_t run_len = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint8_t * p = data.data() + i * step;
    if (finite_at(p, o.x) && finite_at(p, o.y) && finite_at(p, o.z)) {
      if (run == nullptr) {
        run = p;
      }
      ++run_len;
    } else if (run != nullptr) {
      m.data.insert(m.data.end(), run, run + run_len * step);
      run = nullptr;
      run_len = 0;
    }
  }
  if (run != nullptr) {
    m.data.insert(m.data.end(), run, run + run_len * step);
  }
  m.height = 1;
  m.width = static_cast<std::uint32_t>(m.data.size() / step);
  m.row_step = m.data.size();
}

}  // namespace

PointCloud2Display::PointCloud2Display()
{
  topic_property_ = new rviz_common::properties::EditableEnumProperty(
    "Topic", "", "flux channel carrying sensor_msgs/PointCloud2 frames.", this,
    SLOT(updateTopic()));
  connect(
    topic_property_, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
    &PointCloud2Display::fillTopicList);
  common_ = std::make_unique<rviz_default_plugins::PointCloudCommon>(this);
}

PointCloud2Display::~PointCloud2Display()
{
  unsubscribe();
}

void PointCloud2Display::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  common_->initialize(context_, scene_node_);
}

void PointCloud2Display::onEnable()
{
  subscribe();
}

void PointCloud2Display::onDisable()
{
  unsubscribe();
  common_->onDisable();
}

void PointCloud2Display::reset()
{
  Display::reset();
  common_->reset();
  frames_ = 0;
}

void PointCloud2Display::updateTopic()
{
  unsubscribe();
  reset();
  if (isEnabled()) {
    subscribe();
  }
}

void PointCloud2Display::fillTopicList(rviz_common::properties::EditableEnumProperty * property)
{
  fill_topic_list(property, Cloud::kFingerprint);
}

void PointCloud2Display::subscribe()
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
      *node_, topic, Cloud::kFingerprint, flux::ros::Subscription::Callback{}, qos);
  } catch (const std::exception & e) {
    setStatus(StatusProperty::Error, "Topic", QString("Subscribe failed: ") + e.what());
    return;
  }
  setStatus(StatusProperty::Warn, "Topic", "Waiting for a publisher");
}

void PointCloud2Display::unsubscribe()
{
  sub_.reset();
}

sensor_msgs::msg::PointCloud2::SharedPtr PointCloud2Display::takeFrame()
{
  flux::FrameView frame = sub_->take();
  if (!frame) {
    if (sub_->attached() && frames_ == 0) {
      setStatus(StatusProperty::Warn, "Topic", "Attached, no frame yet");
    }
    return nullptr;
  }

  Cloud::View view(frame);
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec = view.header__sec();
  msg->header.stamp.nanosec = view.header__nanosec();
  msg->header.frame_id = std::string(view.header__frame_id());
  msg->height = view.height();
  msg->width = view.width();
  msg->fields.resize(view.fields__size());
  for (std::size_t i = 0; i < msg->fields.size(); ++i) {
    const Cloud::FieldsElem f = view.fields(i);
    msg->fields[i].name = std::string(f.name());
    msg->fields[i].offset = f.offset();
    msg->fields[i].datatype = f.datatype();
    msg->fields[i].count = f.count();
  }
  msg->is_bigendian = view.is_bigendian();
  msg->point_step = view.point_step();
  msg->is_dense = view.is_dense();
  const std::size_t bytes = std::size_t{msg->width} * msg->height * msg->point_step;
  if (!view.ok__()) {
    setStatus(StatusProperty::Error, "Message", "Frame does not parse as sensor_msgs/PointCloud2");
    return nullptr;
  }
  if (msg->point_step == 0 || view.data().size() < bytes) {
    setStatus(
      StatusProperty::Error, "Message",
      QString("Data size (%1 bytes) is less than width x height x point_step (%2)")
        .arg(static_cast<qulonglong>(view.data().size()))
        .arg(static_cast<qulonglong>(bytes)));
    return nullptr;
  }
  const XyzOffsets xyz = xyz_offsets(*msg);
  if (!xyz.found) {
    setStatus(StatusProperty::Error, "Message", "No float32 x, y, z fields");
    return nullptr;
  }
  copy_valid_points(view, xyz, *msg);
  ++frames_;
  setStatus(
    StatusProperty::Ok, "Topic",
    QString("%1 frames, %2 lost").arg(frames_).arg(static_cast<qulonglong>(sub_->lost())));
  return msg;
}

// The view lives inside takeFrame(), so the slot is back with the publisher before the
// transformers and the TF lookup run on the copy.
void PointCloud2Display::update(float wall_dt, float ros_dt)
{
  if (sub_) {
    if (sensor_msgs::msg::PointCloud2::SharedPtr msg = takeFrame()) {
      common_->addMessage(msg);
    }
  }
  common_->update(wall_dt, ros_dt);
}

}  // namespace flux_tools::rviz

PLUGINLIB_EXPORT_CLASS(flux_tools::rviz::PointCloud2Display, rviz_common::Display)
