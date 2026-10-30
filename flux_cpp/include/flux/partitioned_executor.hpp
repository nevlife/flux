#ifndef FLUX_PARTITIONED_EXECUTOR_HPP
#define FLUX_PARTITIONED_EXECUTOR_HPP

#include "flux/executor.hpp"
#include "flux/subscription.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>

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

namespace flux
{

// One thread and flux::Executor per callback group of the added nodes, groups created while
// spinning included, but not one left out of automatic adding unless a subscription is assigned to
// it, nor one another executor serves. A group named in add() or on_thread_start() is kept alive;
// any other is held by its thread until this is its last owner.
class PartitionedExecutor
{
public:
  using NodeBase = rclcpp::node_interfaces::NodeBaseInterface;
  using ThreadStart =
    std::function<void(const NodeBase::SharedPtr &, const rclcpp::CallbackGroup::SharedPtr &)>;

  // Throws Error with FLUX_UNSUPPORTED where flux::Executor would.
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
  // Any other source, such as a flux::message_filters::Subscriber.
  void add(
    detail::Source & src, const rclcpp::CallbackGroup::SharedPtr & group,
    std::int32_t priority = 0);
  // While spinning too: returns with the node's threads started, or throws with none started.
  void add_node(const NodeBase::SharedPtr & node);
  // rclcpp::Node, rclcpp_lifecycle::LifecycleNode, or any node with get_node_base_interface().
  template <class NodeT>
  void add_node(const std::shared_ptr<NodeT> & node)
  {
    add_node(node->get_node_base_interface());
  }
  // Returns once the node's group threads have ended. Not from one of them.
  void remove_node(const NodeBase::SharedPtr & node);
  template <class NodeT>
  void remove_node(const std::shared_ptr<NodeT> & node)
  {
    remove_node(node->get_node_base_interface());
  }
  // The inputs of one message_filters synchronizer: flux or rclcpp message_filters
  // subscribers. spin() refuses them in different groups, since the synchronizer would then call
  // back on whichever thread delivered last.
  template <class... Inputs>
  void add_sync_group(Inputs &... inputs)
  {
    static_assert(sizeof...(Inputs) >= 2, "a synchronizer has at least two inputs");
    const std::lock_guard<std::mutex> lk(m_);
    if (spinning_) throw std::logic_error("flux: add_sync_group() while spinning");
    sync_groups_.push_back({sync_input(inputs)...});
  }
  // Runs on the group's thread before it spins.
  void on_thread_start(const rclcpp::CallbackGroup::SharedPtr & group, std::function<void()> init);
  // On every group's thread, ahead of that group's hook. A hook's exception stops spin().
  void on_thread_start(ThreadStart init);

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
    NodeBase::WeakPtr node;
  };

  void rescan();
  // Under groups_m_.
  std::vector<rclcpp::CallbackGroup::SharedPtr> unserved_groups(const NodeBase::SharedPtr & node);
  void spawn(const rclcpp::CallbackGroup::SharedPtr & group, const NodeBase::SharedPtr & node);
  void check_sync_groups();
  void fail(std::exception_ptr e);
  void stop_children();
  static void join(std::vector<Child> & children);

  // Before m_ when both are held. Never held while joining: the joined may wait for it.
  std::mutex groups_m_;
  bool serving_ = false;
  std::vector<NodeBase::WeakPtr> nodes_;
  std::vector<Child> children_;
  std::size_t spawned_ = 0;
  // Weak, so a recycled address is not taken for a group already served.
  std::map<const rclcpp::CallbackGroup *, rclcpp::CallbackGroup::WeakPtr> claimed_;

  std::mutex m_;
  // Not under m_: cancel() may run in a signal handler that interrupted a holder of m_.
  std::atomic<bool> stop_{false};
  detail::Executor waiter_;
  detail::ExecutorHandle wake_ = waiter_.handle();
  bool spinning_ = false;
  std::exception_ptr failure_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::OnShutdownCallbackHandle on_shutdown_;

  std::vector<std::vector<SyncInput>> sync_groups_;
  std::map<const rclcpp::CallbackGroup *, std::vector<Assigned>> assigned_;
  std::map<const rclcpp::CallbackGroup *, std::function<void()>> init_;
  ThreadStart any_init_;
  std::map<const rclcpp::CallbackGroup *, rclcpp::CallbackGroup::SharedPtr> kept_;
};

}  // namespace flux

#endif  // FLUX_PARTITIONED_EXECUTOR_HPP
