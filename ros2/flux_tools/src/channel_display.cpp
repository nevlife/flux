#include "flux_tools/rviz/channel_display.hpp"

#include "flux/topics.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include <QString>

#include <exception>
#include <string>

namespace flux_tools::rviz
{

using rviz_common::properties::StatusProperty;

ChannelDisplay::ChannelDisplay(const char * description, std::uint64_t fingerprint)
: fingerprint_(fingerprint)
{
  topic_property_ = new rviz_common::properties::EditableEnumProperty(
    "Topic", "", description, this, SLOT(updateTopic()));
  connect(
    topic_property_, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
    &ChannelDisplay::fillTopicList);
}

void ChannelDisplay::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
}

void ChannelDisplay::reset()
{
  Display::reset();
  frames_ = 0;
}

void ChannelDisplay::onEnable()
{
  subscribe();
}

void ChannelDisplay::onDisable()
{
  unsubscribe();
}

void ChannelDisplay::updateTopic()
{
  unsubscribe();
  reset();
  if (isEnabled()) {
    subscribe();
  }
}

void ChannelDisplay::fillTopicList(rviz_common::properties::EditableEnumProperty * property)
{
  property->clearOptions();
  for (const flux::Topic & t : flux::enumerate_topics()) {
    if (t.fingerprint == fingerprint_ && t.key_exact && flux::read_channel_stats(t.signpost).live) {
      property->addOptionStd(t.key);
    }
  }
}

void ChannelDisplay::subscribe()
{
  const std::string topic = topic_property_->getStdString();
  if (topic.empty()) {
    setStatus(StatusProperty::Error, "Topic", "No topic set");
    return;
  }
  // depth 1: the newest frame only, the same policy a viewer wants. max_borrow 1: update() holds
  // one view and drops it before returning, so a second lease is never needed.
  const auto qos = flux::QoS(1).max_borrow(1);
  try {
    sub_ = std::make_unique<flux::Subscription>(*node_, topic, fingerprint_, qos);
  } catch (const std::exception & e) {
    setStatus(StatusProperty::Error, "Topic", QString("Subscribe failed: ") + e.what());
    return;
  }
  setStatus(StatusProperty::Warn, "Topic", "Waiting for a publisher");
}

void ChannelDisplay::unsubscribe()
{
  sub_.reset();
}

flux::FrameView ChannelDisplay::take()
{
  if (!sub_) {
    return {};
  }
  flux::FrameView frame;
  try {
    frame = sub_->take();
  } catch (const std::exception & e) {
    setStatus(StatusProperty::Error, "Topic", QString("Attach failed: ") + e.what());
    return {};
  }
  if (!frame && sub_->attached() && frames_ == 0) {
    setStatus(StatusProperty::Warn, "Topic", "Attached, no frame yet");
  }
  return frame;
}

void ChannelDisplay::shown()
{
  ++frames_;
  setStatus(
    StatusProperty::Ok, "Topic",
    QString("%1 frames, %2 lost").arg(frames_).arg(static_cast<qulonglong>(sub_->lost())));
}

}  // namespace flux_tools::rviz
