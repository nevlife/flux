#ifndef FLUX_MESSAGE_FILTERS_SUBSCRIBER_HPP
#define FLUX_MESSAGE_FILTERS_SUBSCRIBER_HPP

#include "flux/channel.hpp"
#include "flux/detail/executor.hpp"
#include "flux/subscription.hpp"

#include <message_filters/simple_filter.hpp>

#include <std_msgs/msg/header.hpp>

#include <cstdint>
#include <memory>
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

  typename Adapter::View view() const { return typename Adapter::View(*frame); }
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

  const Subscription & subscription() const noexcept { return sub_; }
  std::string getTopic() const { return sub_.topic_name(); }
  std::uint64_t forwarded() const noexcept { return forwarded_; }
  // Frames whose header did not read or has a negative stamp, dropped rather than forwarded.
  std::uint64_t unreadable() const noexcept { return unreadable_; }

  bool attach() override { return sub_.attach(); }
  bool deliver_one() override
  {
    FrameView f = sub_.sub_.take_for_executor();
    sub_.report_refusal();
    if (!f) return false;
    forward(std::move(f));
    return true;
  }
  const detail::Channel * channel() const override { return sub_.channel(); }
  const detail::Watch * watch() const override { return sub_.sub_.watch(); }
  const void * endpoint() const override { return sub_.endpoint(); }

private:
  static QoS covering(QoS qos, std::uint32_t queue)
  {
    if (qos.borrow_limit() <= queue) qos.max_borrow(queue + 1);
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
};

}  // namespace flux::message_filters

#endif  // FLUX_MESSAGE_FILTERS_SUBSCRIBER_HPP
