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
// Both null is an input this executor cannot place -- a chained filter, or a subscription whose
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

// Callback-group partitioning: one child executor and one thread per callback
// group, so no group's callback can delay another group's. A group holding flux subscriptions
// gets a flux::ros::Executor sized to exactly those subscriptions (its own io_uring); a pure ROS
// group gets a stock rclcpp SingleThreadedExecutor. The thread calling spin() only spawns
// children and scans for callback groups created after spin started -- it runs no callbacks.
class PartitionedExecutor
{
public:
  PartitionedExecutor();
  ~PartitionedExecutor();
  PartitionedExecutor(const PartitionedExecutor &) = delete;
  PartitionedExecutor & operator=(const PartitionedExecutor &) = delete;

  // Assign a flux subscription to a callback group: same group, same thread. The group must
  // belong to a node handed to add_ros_node() -- if its ROS callbacks ran on another executor
  // the group's mutual exclusion would break, so flux-only assignments are rejected at spin.
  void add(Subscription & sub, const rclcpp::CallbackGroup::SharedPtr & group, int priority = 0);

  // Same, for any flux::Source -- a message_filters Subscriber
  // (flux/ros/message_filters/subscriber.hpp) is one. Assigning every input of one synchronizer
  // to the same group is not a convention here but a requirement: the sync policy runs the
  // matched callback while holding its own std::mutex, so inputs on two group threads couple
  // those groups' priorities through a lock with no priority inheritance.
  //
  // `priority` orders the visit within the group's own pass, higher first, ties in registration
  // order. It never crosses groups: those are separate threads and the OS orders them by the
  // thread priority schedule() declares.
  void add(flux::Source & src, const rclcpp::CallbackGroup::SharedPtr & group, int priority = 0);

  // Declare that these inputs feed one synchronizer, so spin() can check they land on one thread.
  // The requirement is the one add() above states; this is what turns it from prose into a
  // refusal. A synchronizer does not tell anyone what its inputs are, so the set has to be
  // declared -- upstream message_filters has no seam that would reveal it.
  //
  // Takes flux inputs and plain ROS message_filters::Subscriber alike: a graph mixing a flux
  // topic with a DDS topic is the case this check exists for, because there the two inputs are
  // serviced by different machinery (dispatch() and pump_ros()) that only a shared group puts on
  // one thread. An input this executor cannot place is reported by unplaced_sync_inputs() rather
  // than judged -- the same line rt::verify draws with Unknown findings.
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
  // itself before running any callback, so the thread never runs at the wrong priority. A refusal
  // is the spin() error, not a downgrade. One schedule per group, and the group must actually be
  // served here: a schedule that would silently never apply is rejected at spin.
  //
  // The child applies through rt::apply_checked, so preflight runs on the way in rather than only
  // when a caller remembers to ask. `strict` decides what a Warn does -- Soft tolerates a host
  // that is not configured for bounded latency, Hard refuses it.
  // `control_priority` declares the consumer's control loop priority so transport can be checked
  // to stay below it; left at 0 the check is reported Unknown rather than skipped silently.
  void schedule(
    const rclcpp::CallbackGroup::SharedPtr & group, const rt::Options & sched,
    rt::Strictness strict = rt::Strictness::Soft, int control_priority = 0);

  // Same, taking a stage the declaration file already resolved. Strictness and
  // control priority come from the chain rather than from this call site, which is the point of
  // declaring the chain in one place: no node can derive them from its own settings alone.
  // Rejects an external stage -- flux does not set those, and pretending to would put the one
  // thread the file marked as somebody else's under this executor's schedule.
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
  void check_sync_groups(const GroupMap & known);
  void check_schedule_labels(const GroupMap & known) const;

  SpinControl ctl_;                     // wake fd, the spin/stop flag pair, and the reentry guard
  std::atomic<bool> child_run_{false};  // parent-owned: children never see the caller's flag

  struct Assigned
  {
    flux::Source * src = nullptr;
    int priority = 0;
  };
  std::vector<std::pair<rclcpp::CallbackGroup::SharedPtr, std::vector<Assigned>>> assigned_;
  // One declared schedule: the thread options plus how strictly preflight's verdicts are read.
  // `node` and `label` carry the stage's identity when a declaration file named this schedule, so
  // spin() can check the name the file used against the group the call site handed over. An
  // rclcpp callback group has no name of its own, so that pairing is the only thing linking the
  // two, and nothing checked it.
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
