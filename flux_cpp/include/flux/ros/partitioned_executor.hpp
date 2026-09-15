#ifndef FLUX_ROS_PARTITIONED_EXECUTOR_HPP
#define FLUX_ROS_PARTITIONED_EXECUTOR_HPP

#include "flux/ros/executor.hpp"
#include "flux/ros/rt_spec.hpp"
#include "flux/ros/subscription.hpp"
#include "flux/rt.hpp"
#include "flux/spin_control.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace flux::ros
{

namespace detail
{

// One input of a synchronizer, reduced to whatever this executor can place on a thread. A flux
// input is placed by add(); a plain ROS input by the callback group its subscription sits in.
// Both null is an input this executor cannot place: a chained filter, or a subscription whose
// node was never handed to add_ros_node().
struct SyncInput
{
  flux::Source * src = nullptr;
  rclcpp::SubscriptionBase::SharedPtr ros;
};

template <typename T, typename = void>
struct HasGetSubscriber : std::false_type
{
};
template <typename T>
struct HasGetSubscriber<T, std::void_t<decltype(std::declval<T &>().getSubscriber())>>
: std::true_type
{
};

template <typename T>
SyncInput as_sync_input(T & in)
{
  if constexpr (std::is_base_of_v<flux::Source, T>) {
    return SyncInput{static_cast<flux::Source *>(&in), nullptr};
  } else if constexpr (HasGetSubscriber<T>::value) {
    return SyncInput{nullptr, in.getSubscriber()};
  } else {
    return SyncInput{};
  }
}
}  // namespace detail

// One child executor and one thread per callback group, so no group's callback can delay
// another group's (docs/en/api.en.md, PartitionedExecutor). The thread calling spin() only
// spawns children and scans for groups created after spin started; it runs no callbacks.
class PartitionedExecutor
{
public:
  PartitionedExecutor();
  ~PartitionedExecutor();
  PartitionedExecutor(const PartitionedExecutor &) = delete;
  PartitionedExecutor & operator=(const PartitionedExecutor &) = delete;

  // Assign a flux subscription to a callback group: same group, same thread. The group must
  // belong to a node handed to add_ros_node(); flux-only assignments are rejected at spin, since
  // the group's ROS callbacks running on another executor would break its mutual exclusion.
  void add(Subscription & sub, const rclcpp::CallbackGroup::SharedPtr & group, int priority = 0);

  // Same, for any flux::Source, such as a message_filters Subscriber
  // (flux/ros/message_filters/subscriber.hpp). Every input of one synchronizer must sit in the
  // same group: the sync policy runs the matched callback under its own std::mutex, which has no
  // priority inheritance, so inputs on two group threads would couple those groups' priorities.
  // `priority` orders the visit within the group's own pass and never crosses groups.
  void add(flux::Source & src, const rclcpp::CallbackGroup::SharedPtr & group, int priority = 0);

  // Declare that these inputs feed one synchronizer, so spin() can refuse inputs on two threads.
  // The set has to be declared: upstream message_filters has no seam that reveals it. Takes flux
  // inputs and plain ROS message_filters::Subscriber alike, since a graph mixing a flux topic
  // with a DDS topic is the case this check exists for. An input this executor cannot place is
  // reported by unplaced_sync_inputs() rather than judged.
  template <typename... Inputs>
  void add_sync_group(Inputs &... inputs)
  {
    static_assert(
      sizeof...(Inputs) >= 2, "flux: a sync group needs at least two inputs to constrain");
    declare_sync_group({detail::as_sync_input(inputs)...});
  }

  // Inputs of declared sync groups that spin() could not place on a thread, so the same-thread
  // rule was not judged for them. Zero means every declared input was placed. Valid after spin()
  // has started.
  std::size_t unplaced_sync_inputs() const noexcept;

  // Register a node: every callback group it has (or later creates) is served by its own child.
  // Do not also hand the node or any of its groups to another executor.
  void add_ros_node(const rclcpp::Node::SharedPtr & node);

  // Declare the thread scheduling for the child that will serve `group`. The child applies it to
  // itself through rt::apply_checked before running any callback; a refusal is the spin() error,
  // not a downgrade. One per group, and the group must be served here. `strict` decides what a
  // Warn does. `control_priority` is the consumer's control loop priority, so transport can be
  // checked to stay below it; left at 0 that check is reported Unknown.
  void schedule(
    const rclcpp::CallbackGroup::SharedPtr & group, const rt::Options & sched,
    rt::Strictness strict = rt::Strictness::Soft, int control_priority = 0);

  // Same, taking a stage the declaration file resolved: strictness and control priority come
  // from the chain, which no node can derive from its own settings. Rejects an external stage.
  void schedule(const rclcpp::CallbackGroup::SharedPtr & group, const RtStage & stage);

  // Spawn one child per callback group and block until stop(). Re-scans the registered nodes
  // every `tick_ns` so a group created after spin started gets a child within one tick (negative
  // = only on interrupt()). An exception on any child thread stops every child and rethrows here.
  void spin(std::int64_t tick_ns = 100'000'000);

  // Same, also ending when `run` is cleared. For a caller that already owns a shutdown flag.
  void spin(std::atomic<bool> & run, std::int64_t tick_ns = 100'000'000);

  // End spin() and every child. Safe from a callback or another thread. A stop() before spin()
  // is held rather than lost, and cleared on the way out so this executor can be spun again.
  void stop() noexcept;

  // Wake spin() out of its current wait without ending it.
  void interrupt() noexcept;

private:
  struct Child
  {
    rclcpp::CallbackGroup::WeakPtr group;
    std::unique_ptr<Executor> flux_ex;  // the group has flux subscriptions
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_ex;  // pure ROS group
    std::thread thread;
  };

  struct GroupNode
  {
    rclcpp::CallbackGroup::SharedPtr group;
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base;
  };
  using GroupMap = std::unordered_map<const rclcpp::CallbackGroup *, GroupNode>;

  void declare_sync_group(std::vector<detail::SyncInput> inputs);
  void spawn_children(std::int64_t tick_ns);
  void record_child_error() noexcept;  // from inside a child thread's catch handler
  void check_sync_groups(const GroupMap & known);
  void check_schedule_labels(const GroupMap & known) const;

  SpinControl ctl_;
  std::atomic<bool> child_run_{false};  // parent-owned: children never see the caller's flag

  struct Assigned
  {
    flux::Source * src = nullptr;
    int priority = 0;
  };
  std::vector<std::pair<rclcpp::CallbackGroup::SharedPtr, std::vector<Assigned>>> assigned_;
  // One declared schedule. `node` and `label` carry the stage's identity when a declaration file
  // named it, so spin() can check the file's name against the group handed over: an rclcpp
  // callback group has no name of its own, so that pairing is the only link between the two.
  struct Sched
  {
    rt::Options opts;
    rt::Strictness strict = rt::Strictness::Soft;
    int control_priority = 0;
    std::string node;
    std::string label;
    bool from_stage = false;
  };

  void schedule_impl(const rclcpp::CallbackGroup::SharedPtr & group, Sched sched);

  std::vector<std::pair<rclcpp::CallbackGroup::SharedPtr, Sched>> scheduled_;
  std::vector<std::vector<detail::SyncInput>> sync_groups_;
  std::atomic<std::size_t> unplaced_sync_{0};
  std::vector<rclcpp::Node::WeakPtr> nodes_;
  // Keyed by address so the tick re-scan can skip groups that already have a child; weak so a
  // recycled address is not mistaken for its dead predecessor.
  std::unordered_map<const rclcpp::CallbackGroup *, rclcpp::CallbackGroup::WeakPtr> claimed_;
  std::vector<Child> children_;

  std::mutex error_mutex_;
  std::exception_ptr child_error_;
  std::atomic<bool> failed_{false};
};

}  // namespace flux::ros

#endif  // FLUX_ROS_PARTITIONED_EXECUTOR_HPP
