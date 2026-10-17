#ifndef FLUX2_TOOLS_RVIZ_CHANNEL_DISPLAY_HPP
#define FLUX2_TOOLS_RVIZ_CHANNEL_DISPLAY_HPP

#ifndef Q_MOC_RUN
#include "flux2/subscription.hpp"
#include "flux2_tools/rviz/update_dt.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"

#include <rclcpp/node.hpp>

#include <cstdint>
#include <memory>
#endif

namespace flux2_tools::rviz
{

// What the flux2 displays share: a Topic property offering the live channels of one type, the
// subscription it names, and the Topic status of the frames taken from it. Derives from Display
// rather than RosTopicDisplay: the latter owns an rclcpp subscription, and a flux2 channel is not
// a DDS topic.
class ChannelDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  ChannelDisplay(const char * description, std::uint64_t fingerprint);
  ChannelDisplay(const ChannelDisplay &) = delete;
  ChannelDisplay & operator=(const ChannelDisplay &) = delete;
  ChannelDisplay(ChannelDisplay &&) = delete;
  ChannelDisplay & operator=(ChannelDisplay &&) = delete;
  ~ChannelDisplay() override = default;

  void onInitialize() override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;
  virtual void subscribe();
  virtual void unsubscribe();
  // The next frame, or none. A channel that refuses this subscription is shown in the status
  // rather than thrown out of update(), which would end rviz.
  flux2::FrameView take();
  // Counts a frame shown, in the Topic status.
  void shown();

  rviz_common::properties::EditableEnumProperty * topic_property_;
  rclcpp::Node::SharedPtr node_;

protected Q_SLOTS:
  virtual void updateTopic();

private Q_SLOTS:
  void fillTopicList(rviz_common::properties::EditableEnumProperty * property);

private:
  std::uint64_t fingerprint_;
  std::unique_ptr<flux2::Subscription> sub_;
  std::uint64_t frames_ = 0;
};

}  // namespace flux2_tools::rviz

#endif  // FLUX2_TOOLS_RVIZ_CHANNEL_DISPLAY_HPP
