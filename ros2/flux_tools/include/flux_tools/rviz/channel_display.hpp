#ifndef FLUX_TOOLS_RVIZ_CHANNEL_DISPLAY_HPP
#define FLUX_TOOLS_RVIZ_CHANNEL_DISPLAY_HPP

#ifndef Q_MOC_RUN
#include "flux/subscription.hpp"
#include "flux_tools/typed_frames.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"

#include <rclcpp/node.hpp>

#include <cstdint>
#include <memory>
#endif

namespace flux_tools::rviz
{

// What the flux displays share: a Topic property offering the live channels of one type (and the
// GPU memory channels of it, flux::gpu_fingerprint), the subscription it names, and the Topic
// status of the frames taken from it. Derives from Display
// rather than RosTopicDisplay: the latter owns an rclcpp subscription, and a flux channel is not
// a DDS topic.
class ChannelDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  ChannelDisplay(const char * description, std::uint64_t fingerprint);
  ChannelDisplay(const ChannelDisplay &) = delete;
  auto operator=(const ChannelDisplay &) -> ChannelDisplay & = delete;
  ChannelDisplay(ChannelDisplay &&) = delete;
  auto operator=(ChannelDisplay &&) -> ChannelDisplay & = delete;
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
  auto take() -> flux::FrameView;
  // Counts a frame shown, in the Topic status.
  void shown();

  // `frame` read as its type (TypedFrames::view).
  template <class View>
  auto view_of(const flux::FrameView & frame) -> View
  {
    return frames_of_type_.view<View>(frame);
  }

  rviz_common::properties::EditableEnumProperty * topic_property_;
  rclcpp::Node::SharedPtr node_;

protected Q_SLOTS:
  virtual void updateTopic();

private Q_SLOTS:
  void fillTopicList(rviz_common::properties::EditableEnumProperty * property);

private:
  flux_tools::TypedFrames frames_of_type_;
  std::unique_ptr<flux::Subscription> sub_;
  std::uint64_t frames_ = 0;
};

}  // namespace flux_tools::rviz

#endif  // FLUX_TOOLS_RVIZ_CHANNEL_DISPLAY_HPP
