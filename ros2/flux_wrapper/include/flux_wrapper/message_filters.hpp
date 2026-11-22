#ifndef FLUX_WRAPPER__MESSAGE_FILTERS_HPP_
#define FLUX_WRAPPER__MESSAGE_FILTERS_HPP_

#include "flux_wrapper/subscription.hpp"

#include <rclcpp/qos.hpp>

#include <std_msgs/msg/header.hpp>

#include <message_filters/simple_filter.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace flux_wrapper::message_filters
{

// What a synchronizer queues: a copy of the message's header, and the View, which on the flux
// path holds its slot while the synchronizer or the callback keeps it.
template <class MsgT>
struct Stamped
{
  std_msgs::msg::Header header;
  std::shared_ptr<const View<MsgT>> view;
};

// A topic as a message_filters input over flux or ROS, as flux_wrapper::Subscription picks: the
// same code feeds a Synchronizer or a Cache in a build with flux and one without. MsgT needs a
// root std_msgs/Header `header`.
//
// `queue` is the synchronizer's or Cache's size. Every queued message holds a View and a filter
// adds before it drops, so max_borrow is raised to queue + 1; over flux a publisher with no more
// slots than that is refused with flux::Error. A message with a negative stamp is no ROS time,
// which the filters would throw on: it is counted in unreadable() and not forwarded.
template <class MsgT>
class Subscriber : public ::message_filters::SimpleFilter<Stamped<MsgT>>
{
  static_assert(detail::kRootHeader<MsgT>, "a message_filters input needs a root header");

public:
  using Message = Stamped<MsgT>;

  template <class NodeT>
  Subscriber(
    NodeT & node, const std::string & topic, const rclcpp::QoS & qos, std::uint32_t queue,
    SubscriptionOptions options = {})
  : topic_(topic),
    sub_(
      node, topic, qos,
      typename Subscription<MsgT>::HeldCallback(
        [this](std::shared_ptr<const View<MsgT>> v) { forward(std::move(v)); }),
      covering(std::move(options), queue))
  {
  }
  // The callback holds this filter.
  Subscriber(const Subscriber &) = delete;
  auto operator=(const Subscriber &) -> Subscriber & = delete;
  Subscriber(Subscriber &&) = delete;
  auto operator=(Subscriber &&) -> Subscriber & = delete;
  ~Subscriber() = default;

  [[nodiscard]] auto subscription() const noexcept -> const Subscription<MsgT> & { return sub_; }
  [[nodiscard]] auto getTopic() const -> std::string { return topic_; }
  [[nodiscard]] auto forwarded() const noexcept -> std::uint64_t { return forwarded_; }
  [[nodiscard]] auto unreadable() const noexcept -> std::uint64_t { return unreadable_; }

private:
  static auto covering(SubscriptionOptions options, std::uint32_t queue) -> SubscriptionOptions
  {
    options.max_borrow = std::max(options.max_borrow, queue + 1);
    return options;
  }

  void forward(std::shared_ptr<const View<MsgT>> v)
  {
    auto m = std::make_shared<Message>();
    m->header.stamp = v->stamp();
    m->header.frame_id = std::string(v->frame_id());
    if (m->header.stamp.sec < 0) {
      ++unreadable_;
      return;
    }
    m->view = std::move(v);
    ++forwarded_;
    this->signalMessage(m);
  }

  std::string topic_;
  std::uint64_t forwarded_ = 0;
  std::uint64_t unreadable_ = 0;
  // Last: its callback reaches the members above.
  Subscription<MsgT> sub_;
};

}  // namespace flux_wrapper::message_filters

#endif  // FLUX_WRAPPER__MESSAGE_FILTERS_HPP_
