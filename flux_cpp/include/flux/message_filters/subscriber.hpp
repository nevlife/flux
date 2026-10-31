#ifndef FLUX_MESSAGE_FILTERS_SUBSCRIBER_HPP
#define FLUX_MESSAGE_FILTERS_SUBSCRIBER_HPP

#include "flux/channel.hpp"
#include "flux/detail/executor.hpp"
#include "flux/detail/group_member.hpp"
#include "flux/subscription.hpp"

#include <message_filters/simple_filter.hpp>

#include <std_msgs/msg/header.hpp>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace flux::message_filters
{

// What a synchronizer queues: the frame, still borrowed, and a copy of its header.
template <class Adapter>
struct StampedFrame
{
  std::shared_ptr<const FrameView> frame;
  std_msgs::msg::Header header;

  [[nodiscard]] auto view() const -> typename Adapter::View
  {
    return typename Adapter::View(*frame);
  }
};

// A flux topic as a message_filters input. `Adapter` names the schema: `kFingerprint`, and a
// `View(const FrameView &)` with `header__sec()`, `header__nanosec()`, `header__frame_id()` and
// `ok__()`.
//
// `queue` is the synchronizer's or Cache's size. Every queued frame holds a slot and a filter
// adds before it drops, so max_borrow is raised to queue + 1, and a publisher with no more slots
// than the queue is refused with flux::Error.
template <class Adapter>
class Subscriber : public ::message_filters::SimpleFilter<StampedFrame<Adapter>>,
                   public detail::Source
{
public:
  using Message = StampedFrame<Adapter>;

  Subscriber(
    const NodeRef & node, const std::string & topic, std::uint32_t queue, const QoS & qos = QoS{})
  : sub_(node, topic, Adapter::kFingerprint, covering(qos, queue))
  {
    sub_.sub_.require_slots(queue + 1);
  }

  // As Subscription::add_to_callback_group. The inputs of one synchronizer go in one group.
  void add_to_callback_group(
    const NodeRef & node, const rclcpp::CallbackGroup::SharedPtr & group = nullptr,
    std::int32_t priority = 0)
  {
    if (member_) {
      throw std::logic_error("flux: the subscriber is already in a callback group");
    }
    member_ = detail::join_group(node, *this, sub_.topic_name(), group, priority);
  }

  [[nodiscard]] auto subscription() const noexcept -> const Subscription & { return sub_; }
  [[nodiscard]] auto getTopic() const -> std::string { return sub_.topic_name(); }
  [[nodiscard]] auto forwarded() const noexcept -> std::uint64_t { return forwarded_; }
  // Frames whose header did not read or has a negative stamp, dropped rather than forwarded.
  [[nodiscard]] auto unreadable() const noexcept -> std::uint64_t { return unreadable_; }

  auto attach() -> bool override { return sub_.attach(); }
  auto deliver_one() -> bool override
  {
    FrameView f = sub_.sub_.take_for_executor();
    sub_.report_refusal();
    if (!f) {
      return false;
    }
    forward(std::move(f));
    return true;
  }
  [[nodiscard]] auto channel() const -> const detail::Channel * override { return sub_.channel(); }
  [[nodiscard]] auto watch() const -> const detail::Watch * override { return sub_.sub_.watch(); }
  [[nodiscard]] auto endpoint() const -> const void * override { return sub_.endpoint(); }

private:
  static auto covering(QoS qos, std::uint32_t queue) -> QoS
  {
    if (qos.borrow_limit() <= queue) {
      qos.max_borrow(queue + 1);
    }
    return qos;
  }

  void forward(FrameView && f)
  {
    auto m = std::make_shared<Message>();
    {
      const typename Adapter::View v(f);
      m->header.stamp.sec = v.header__sec();
      m->header.stamp.nanosec = v.header__nanosec();
      m->header.frame_id = v.header__frame_id();
      // Latched by the reads above, so it is asked after them. A negative stamp is no ROS time:
      // rclcpp::Time would throw on it inside the filters.
      if (!v.ok__() || m->header.stamp.sec < 0) {
        ++unreadable_;
        return;
      }
    }
    m->frame = std::make_shared<const FrameView>(std::move(f));
    ++forwarded_;
    this->signalMessage(m);
  }

  Subscription sub_;
  std::uint64_t forwarded_ = 0;
  std::uint64_t unreadable_ = 0;
  std::shared_ptr<detail::GroupMember> member_;
};

}  // namespace flux::message_filters

#endif  // FLUX_MESSAGE_FILTERS_SUBSCRIBER_HPP
