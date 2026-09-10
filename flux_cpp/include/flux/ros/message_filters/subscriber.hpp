#ifndef FLUX_ROS_MESSAGE_FILTERS_SUBSCRIBER_HPP
#define FLUX_ROS_MESSAGE_FILTERS_SUBSCRIBER_HPP

#include "flux/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <message_filters/message_traits.h>
#include <message_filters/simple_filter.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace flux::ros::message_filters
{

// What a filter queue holds: the borrow, not the bytes. The payload stays in the segment for as
// long as this lives, which is what keeps the filter path zero-copy -- and also what makes
// max_borrow the filter's business. A synchronizer holds up to queue_size frames per input while
// it waits for partners, so max_borrow must cover inputs x queue_size or the consumer runs out of
// leases and stops taking (docs/en/borrow_lifetime.en.md).
//
// The borrow already lives inside FrameView, so an ordinary shared_ptr to this struct is enough
// and upstream message_filters runs unmodified.
template <typename Adapter>
struct StampedFrame
{
  // Held through a shared_ptr because upstream requires the queued message to be copy-assignable
  // (message_event.h:220 assigns one to another on a branch that a const message never takes, but
  // that still has to compile) and FrameView is move-only by design: it IS the validated holder.
  // The cost is a second allocation per frame, on a path that already allocates.
  std::shared_ptr<flux::FrameView> frame;
  rclcpp::Time stamp;

  typename Adapter::View view() const noexcept { return typename Adapter::View(*frame); }
};

namespace detail
{
template <typename View, typename = void>
struct HasHeaderStamp : std::false_type
{
};
template <typename View>
struct HasHeaderStamp<View, std::void_t<decltype(std::declval<const View &>().header__sec())>>
: std::true_type
{
};
}  // namespace detail

// A flux subscription that is also a message_filters source, so a flux topic and a ROS topic can
// be synchronized in one Synchronizer. Named after the upstream class it stands in for: porting a
// graph to flux changes the namespace, not the shape.
//
// One requirement, and it is not a style rule. The sync policies run the matched callback while
// holding their own std::mutex (approximate_time.h:212 -> :509), and std::mutex has no priority
// inheritance. Every input of one synchronizer must therefore be serviced by the same thread:
// flux::ros::Executor satisfies it by construction, and under PartitionedExecutor every
// input must be assigned to the same callback group. Two groups feeding one synchronizer couple
// their priorities through that lock, which is the isolation that executor exists to provide.
//
//
// The filter path allocates. The policy queues are std::deque and std::map, and each frame needs
// two shared_ptr control blocks (see StampedFrame). Plain flux delivery has no allocation; this
// path is not that path and does not belong in a hard-RT chain.
template <typename Adapter>
class Subscriber : public ::message_filters::SimpleFilter<StampedFrame<Adapter>>,
                   public flux::Source
{
public:
  using Message = StampedFrame<Adapter>;

  // Subscribe later, for a filter declared as a node member before its node exists.
  Subscriber() = default;

  Subscriber(rclcpp::Node & node, const std::string & topic, const QoS & qos = QoS{})
  {
    subscribe(node, topic, qos);
  }

  ~Subscriber() override = default;

  // Replaces whatever this was subscribed to. The address of this object is what an executor
  // registered, and that does not move, so re-subscribing under a running executor is safe.
  void subscribe(rclcpp::Node & node, const std::string & topic, const QoS & qos = QoS{})
  {
    sub_.reset();
    sub_.emplace(node, topic, Adapter::kFingerprint, Subscription::Callback{}, qos);
  }

  void unsubscribe() { sub_.reset(); }
  bool subscribed() const noexcept { return sub_.has_value(); }

  // Frames handed to the filter graph. A synchronizer drops a message whose partners never
  // arrive without reporting it, so this diffed against the synchronizer's own callback count is
  // how a caller sees a stream failing to pair up.
  std::uint64_t forwarded() const noexcept { return forwarded_; }

  // Frames the adapter could not read against its schema. Nothing was forwarded for these: a
  // frame that disagrees with the descriptor is a wrong message, not a late one.
  std::uint64_t unreadable() const noexcept { return unreadable_; }

  // ---- flux::Source: what an executor drives it through ----
  bool attach() override { return sub_ ? sub_->attach() : false; }

  int deliver_one() override
  {
    if (!sub_) return 0;
    // take() rather than a Subscription callback because a callback is handed a const FrameView &
    // and the borrow ends when it returns; the queue needs to own the frame. take() carries the
    // orphan check with it.
    flux::FrameView v = sub_->take();
    if (!v) return 0;
    forward(std::move(v));
    return 1;
  }

  flux::Channel * channel() noexcept override { return sub_ ? sub_->channel() : nullptr; }

private:
  void forward(flux::FrameView && f)
  {
    static_assert(
      detail::HasHeaderStamp<typename Adapter::View>::value,
      "flux: this schema has no std_msgs/Header, so there is no stamp to synchronize on. "
      "message_filters keys on the header stamp and FrameMeta carries no time of its own.");

    typename Adapter::View v(f);
    const std::int32_t sec = v.header__sec();
    const std::uint32_t nanosec = v.header__nanosec();
    // Asked after the read: ok__() latches on the first field that disagrees with the descriptor,
    // so the answer only exists once the fields have been walked.
    if (!v.ok__()) {
      ++unreadable_;
      return;
    }
    auto m = std::make_shared<Message>();
    m->frame = std::make_shared<flux::FrameView>(std::move(f));
    m->stamp = rclcpp::Time(sec, nanosec, RCL_ROS_TIME);
    ++forwarded_;
    this->signalMessage(std::shared_ptr<const Message>(std::move(m)));
  }

  std::optional<Subscription> sub_;
  std::uint64_t forwarded_ = 0;
  std::uint64_t unreadable_ = 0;
};

}  // namespace flux::ros::message_filters

// Upstream reads the synchronization key through this trait; its default answers time 0 for
// anything with no `header` member (message_traits.h:84), which would silently make every frame
// synchronize with every other.
namespace message_filters::message_traits
{
template <typename Adapter>
struct TimeStamp<flux::ros::message_filters::StampedFrame<Adapter>, void>
{
  static rclcpp::Time value(const flux::ros::message_filters::StampedFrame<Adapter> & m)
  {
    return m.stamp;
  }
};
}  // namespace message_filters::message_traits

#endif  // FLUX_ROS_MESSAGE_FILTERS_SUBSCRIBER_HPP
