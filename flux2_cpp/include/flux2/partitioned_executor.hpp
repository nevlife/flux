#ifndef FLUX2_PARTITIONED_EXECUTOR_HPP
#define FLUX2_PARTITIONED_EXECUTOR_HPP

#include "flux2/executor.hpp"
#include "flux2/subscription.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/node.hpp>

#include <atomic>
#include <cstdint>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

namespace flux2
{

// One thread per callback group: a group's flux2 subscriptions and its ROS entities run on that
// group's own flux2::Executor, so a slow callback stalls only its group. Every group of an added
// node gets a thread, groups created while spinning included; a group that is not added to
// executors automatically gets one only if a subscription is assigned to it. A group named in
// add() or on_thread_start() is kept alive by this executor; the thread of any other group holds
// it until this executor is its last owner (Humble's rclcpp waits on a raw pointer into it).
class PartitionedExecutor
{
public:
  // Throws Error with FLUX2_UNSUPPORTED where flux2::Executor would.
  PartitionedExecutor();
  ~PartitionedExecutor();
  PartitionedExecutor(const PartitionedExecutor &) = delete;
  PartitionedExecutor & operator=(const PartitionedExecutor &) = delete;
  PartitionedExecutor(PartitionedExecutor &&) = delete;
  PartitionedExecutor & operator=(PartitionedExecutor &&) = delete;

  // `sub` outlives this executor, and `group` belongs to a node given to add_node(). Not while
  // spinning.
  void add(
    Subscription & sub, const rclcpp::CallbackGroup::SharedPtr & group, std::int32_t priority = 0);
  // Any other source, such as a flux2::message_filters::Subscriber.
  void add(
    detail::Source & src, const rclcpp::CallbackGroup::SharedPtr & group,
    std::int32_t priority = 0);
  void add_node(const rclcpp::Node::SharedPtr & node);
  // The inputs of one message_filters synchronizer: flux2 or rclcpp message_filters
  // subscribers. spin() refuses them in different groups, since the synchronizer would then call
  // back on whichever thread delivered last.
  template <class... Inputs>
  void add_sync_group(Inputs &... inputs)
  {
    static_assert(sizeof...(Inputs) >= 2, "a synchronizer has at least two inputs");
    const std::lock_guard<std::mutex> lk(m_);
    if (spinning_) throw std::logic_error("flux2: add_sync_group() while spinning");
    sync_groups_.push_back({sync_input(inputs)...});
  }
  // Runs on the group's thread before it spins.
  void on_thread_start(const rclcpp::CallbackGroup::SharedPtr & group, std::function<void()> init);

  // The first error on any thread stops every thread and is rethrown here. A Reentrant group is
  // refused: one thread cannot run its callbacks in parallel.
  void spin();
  // From any thread, also from a signal handler. Held if spin() has not started.
  void cancel() noexcept;

private:
  struct Assigned
  {
    detail::Source * src;
    std::int32_t priority;
  };
  struct SyncInput
  {
    const detail::Source * flux;
    rclcpp::SubscriptionBase::SharedPtr ros;
  };
  template <class T>
  static SyncInput sync_input(T & input)
  {
    if constexpr (std::is_base_of_v<detail::Source, T>) {
      return {&input, nullptr};
    } else {
      return {nullptr, input.getSubscriber()};
    }
  }
  struct Child
  {
    std::unique_ptr<Executor> ex;
    std::thread thread;
    rclcpp::CallbackGroup::SharedPtr group;
  };

  void spawn_new_groups();
  void check_sync_groups();
  void spawn(
    const rclcpp::CallbackGroup::SharedPtr & group,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node);
  void fail(std::exception_ptr e);
  void stop_children();
  void join_gone_groups();

  std::mutex m_;
  // Not under m_: cancel() may run in a signal handler that interrupted a holder of m_.
  std::atomic<bool> stop_{false};
  detail::Executor waiter_;
  detail::ExecutorHandle wake_ = waiter_.handle();
  bool spinning_ = false;
  std::exception_ptr failure_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::OnShutdownCallbackHandle on_shutdown_;

  std::vector<rclcpp::Node::WeakPtr> nodes_;
  std::vector<std::vector<SyncInput>> sync_groups_;
  std::map<const rclcpp::CallbackGroup *, std::vector<Assigned>> assigned_;
  std::map<const rclcpp::CallbackGroup *, std::function<void()>> init_;
  std::map<const rclcpp::CallbackGroup *, rclcpp::CallbackGroup::SharedPtr> kept_;
  // Weak, so a recycled address is not taken for a group already served.
  std::map<const rclcpp::CallbackGroup *, rclcpp::CallbackGroup::WeakPtr> claimed_;
  std::vector<Child> children_;
};

}  // namespace flux2

#endif  // FLUX2_PARTITIONED_EXECUTOR_HPP
