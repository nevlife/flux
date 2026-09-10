#ifndef FLUX_ROS_EXECUTOR_HPP
#define FLUX_ROS_EXECUTOR_HPP

#include "flux/executor.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/waitable.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <unordered_map>
#include <vector>

namespace flux::ros
{

// Single-threaded executor that blocks on ONE io_uring for both flux subscriptions and ROS
// entities. The merged wait itself is flux::Executor, which knows nothing about ROS; this class
// adds the bridge that makes ROS readiness poke that wait (a POLLIN on the wake fd, fed by the
// readiness callback of every subscription, service, client and waitable).
//
// It waits itself but does NOT take ROS messages itself: once the wake fd fires it hands the node
// over to rclcpp's own execution path. Taking by hand means re-implementing every branch rclcpp
// already has -- serialized subscriptions need a different take, intra-process ones are notified
// through a different object entirely -- and getting one wrong fails silently. Inheriting buys
// those branches, plus timers and services, at no cost to the merged wait.
//
// Like rclcpp's executors it drives entities it is handed; it does not create them.
class Executor : public rclcpp::Executor
{
public:
  // `max_channels` bounds how many flux subscriptions may be registered; the ring is sized for
  // them plus the wake fd so one pass arms every wait in a single syscall. add() rejects past it
  // rather than letting the ring silently drop arms.
  explicit Executor(unsigned max_channels = 32);
  ~Executor();
  Executor(const Executor &) = delete;
  Executor & operator=(const Executor &) = delete;

  // Drive a flux subscription: its callback runs on the spin thread, once per frame its QoS
  // admits. The subscription must outlive this executor and must carry a callback. Registration
  // is pre-spin only: this and the other add calls throw std::logic_error while spin() runs (the
  // spin thread iterates these lists).
  //
  // `priority` orders the visit within a pass, higher first, ties in registration order. It is
  // an ordering priority, not a preemptive one.
  void add(Subscription & sub, int priority = 0);

  // Drive any flux::Source: what a Subscription is one of, and what a message_filters Subscriber
  // (flux/ros/message_filters/subscriber.hpp) is another. The narrower overload above exists to
  // reject a Subscription with no callback; a Source is asked to deliver and answers for itself,
  // so there is nothing to check here.
  void add(flux::Source & src, int priority = 0);

  // Drive a ROS node: rclcpp services its subscriptions, timers, services and actions exactly as
  // its own executor would, while readiness reaches the merged wait through the wake fd. Every
  // entity that has a readiness callback is hooked -- subscriptions (both the rmw queue and, with
  // intra-process comms on, the intra-process waitable), services, clients, and waitables, which
  // is what an action server and an action client each are. Timers have no such event and are
  // answered by shortening the wait to the nearest deadline instead. Entities created after this
  // call are bridged at the top of the next spin pass. Do not also hand the node to an rclcpp
  // executor.
  void add_ros_node(const rclcpp::Node::SharedPtr & node);

  // Drive one callback group instead of a whole node: the group is claimed
  // in rclcpp's registry, so no other executor can also run it. PartitionedExecutor uses
  // this to dedicate a child executor per group.
  void add_ros_callback_group(
    const rclcpp::CallbackGroup::SharedPtr & group,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base);

  // The inherited registration points, so `ex.add_node(node)` bridges rather than registering a
  // node this executor never signals for. add_ros_node() and add_ros_callback_group() are these
  // under their own names. remove_* clears the hooks it installed: a hook left on a removed
  // entity keeps poking the wake fd for work this executor will not run.
  void add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;
  void add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;
  void remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;
  void remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true) override;
  void add_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true) override;
  void remove_callback_group(
    rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify = true) override;

  // Three of the inherited entry points have an exact flux meaning, so they are implemented
  // rather than refused: spin() is spin(default tick), cancel() is stop(), and spin_once(timeout)
  // is spin_once(timeout.count()). Overriding them is what keeps `ex.spin()` from reaching
  // rclcpp's own loop, which would service ROS entities and silently skip flux channels.
  //
  // The rest still throw. Their contracts are about a wait set and a
  // duration budget this executor does not have, and a plausible-looking approximation that is
  // subtly wrong is worse than a refusal the caller can see.
  void cancel() override;
  void spin() override;
  void spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1)) override;
  void spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;
  void spin_all(std::chrono::nanoseconds max_duration) override;
  void spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) override;
  void spin_node_some(std::shared_ptr<rclcpp::Node> node) override;
  void spin_node_all(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::nanoseconds max_duration) override;
  void spin_node_all(
    std::shared_ptr<rclcpp::Node> node, std::chrono::nanoseconds max_duration) override;
  rclcpp::FutureReturnCode spin_until_future_complete_impl(
    std::chrono::nanoseconds timeout,
    const std::function<std::future_status(std::chrono::nanoseconds wait_time)> & wait_for_future)
    override;

  // Spin until stop(). `tick_ns` bounds each blocking wait so the stop is noticed and late flux
  // publishers get attached (negative = block until an event). A ROS timer due sooner shortens
  // the wait further. `ex.spin()` reaches this through the inherited override, with the default
  // tick.
  void spin(std::int64_t tick_ns);

  // Same, also ending when `run` is cleared. For a caller that already owns a shutdown flag.
  void spin(std::atomic<bool> & run, std::int64_t tick_ns = 100'000'000);

  // End spin(). Safe from a callback or another thread. A stop() before spin() is held rather
  // than lost, and cleared on the way out so this executor can be spun again.
  void stop() noexcept;

  // End the current wait without ending the loop. Safe from another thread.
  void interrupt() noexcept { core_.interrupt(); }

  // One pass without looping: deliver ready flux frames, wait up to `timeout_ns` if none were,
  // then service whatever ROS reports ready. Returns flux callbacks run.
  int spin_once(std::int64_t timeout_ns);

  // Deliver every ready flux frame and re-arm, without blocking. Returns the callback count.
  // Pair with wait_for_work() to embed flux in another event loop; the two must not overlap.
  int dispatch() { return core_.dispatch(); }

  // Block until a flux channel or a bridged ROS entity is ready, or `timeout_ns` elapses.
  // Delivers nothing.
  void wait_for_work(std::int64_t timeout_ns) { core_.wait_for_work(timeout_ns); }

  // Hand ready ROS entities to rclcpp, without blocking, at most `budget` of them. Returns how
  // many ran. Unbounded, a ROS stream that outruns its own callbacks holds this thread for as
  // long as it lasts.
  //
  // Raises rclcpp's `spinning` for the pass and restores it. rclcpp reads that flag twice per
  // entity -- to hand work over, and to run it -- because clearing it is how cancel() discards
  // work found before the cancel. A pass that never raises it is indistinguishable from a
  // cancelled executor and services nothing. is_spinning() is true for the pass.
  int pump_ros(int budget);
  int pump_ros() { return pump_ros(ros_budget_); }

  void set_ros_budget(int n);  // bounds a pass, not throughput: spin() carries the remainder

  // True when the last dispatch() stopped on the budget with frames still ready.
  bool has_more() const noexcept { return core_.has_more(); }

  // The same for the flux side: callbacks one dispatch() may run over all channels together.
  void set_pass_budget(int n) { core_.set_pass_budget(n); }
  int pass_budget() const noexcept { return core_.pass_budget(); }

  // Cleared by the read. True when a bridged ROS entity signalled since the last read, so a
  // caller driving this loop itself skips pump_ros() and the rclcpp pass it costs. Set by the
  // hook, not read off the ring: an arrival during a callback lands after that pass's wait.
  bool take_ros_ready() noexcept { return ros_ready_->exchange(false, std::memory_order_acq_rel); }
  int ros_budget() const noexcept { return ros_budget_; }

  // True when this executor merges every wait into one io_uring. False means the per-channel
  // thread fallback is active because the kernel lacks io_uring FUTEX_WAIT.
  bool uses_io_uring() const noexcept { return core_.uses_io_uring(); }

  // Registered flux subscriptions.
  std::size_t size() const noexcept { return core_.size(); }

private:
  static constexpr std::int64_t kDefaultTickNs = 100'000'000;
  static constexpr int kDefaultRosBudget = 64;  // flux::kMaxDrain, for the other transport

  [[noreturn]] static void reject_inherited_spin(const char * name);
  void spin_once_impl(std::chrono::nanoseconds timeout) override;

  void bridge_group(const rclcpp::CallbackGroup::SharedPtr & group);
  void unbridge_group(const rclcpp::CallbackGroup::SharedPtr & group);
  void rebridge_ros();

  // How long the wait may block: `tick_ns`, or the nearest ROS timer deadline when that is
  // sooner. rclcpp times its own waits this way; the merged wait cannot, because it blocks on
  // fds and futexes and knows nothing about deadlines.
  std::int64_t next_ros_timeout(std::int64_t tick_ns);

  flux::Executor core_;
  // Shared: a hook runs on an rmw listener thread and can outlive this executor.
  std::shared_ptr<std::atomic<bool>> ros_ready_ = std::make_shared<std::atomic<bool>>(false);
  int ros_budget_ = kDefaultRosBudget;
  // Nothing pokes the ring again for a message rclcpp already holds, so a pump that stopped on
  // the budget must keep the next pass from blocking.
  bool ros_backlog_ = false;
  std::atomic<bool> stop_requested_{false};
  // A spin loop owns the entry lists. rclcpp's `spinning` cannot say this: it is that library's
  // cancel mechanism and every ROS pass has to raise it, spin_once included.
  std::atomic<bool> reentry_{false};
  // Re-scanned each pass for new subscriptions. The base interface rather than the node, so the
  // NodeBaseInterface overload of add_node() registers the same way the Node one does.
  std::vector<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> ros_nodes_;
  std::vector<rclcpp::CallbackGroup::WeakPtr> ros_groups_;  // group mode: same re-scan, one group
  // Keyed by address so a re-scan can skip what is already bridged; weak so this executor does
  // not keep an entity the user dropped alive just to signal for it. Services, clients and
  // waitables are bridged for the same reason subscriptions are: without a hook they are ready
  // only as far as the tick can see, and an action -- which is one waitable -- would take up to
  // a full tick to accept a goal.
  std::unordered_map<const rclcpp::SubscriptionBase *, rclcpp::SubscriptionBase::WeakPtr> ros_subs_;
  std::unordered_map<const rclcpp::ServiceBase *, rclcpp::ServiceBase::WeakPtr> ros_services_;
  std::unordered_map<const rclcpp::ClientBase *, rclcpp::ClientBase::WeakPtr> ros_clients_;
  std::unordered_map<const rclcpp::Waitable *, rclcpp::Waitable::WeakPtr> ros_waitables_;
};

}  // namespace flux::ros

#endif  // FLUX_ROS_EXECUTOR_HPP
