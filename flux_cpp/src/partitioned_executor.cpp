#include "flux/partitioned_executor.hpp"

#include "on_exit.hpp"

#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/utilities.hpp>

#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace flux
{

PartitionedExecutor::PartitionedExecutor()
: context_(rclcpp::contexts::get_global_default_context()),
  on_shutdown_(context_->add_on_shutdown_callback([this] { cancel(); }))
{
}

PartitionedExecutor::~PartitionedExecutor()
{
  context_->remove_on_shutdown_callback(on_shutdown_);
}

void PartitionedExecutor::add(
  Subscription & sub, const rclcpp::CallbackGroup::SharedPtr & group, std::int32_t priority)
{
  if (!sub.has_callback()) {
    throw std::invalid_argument("flux: a Subscription without a callback has nothing to run");
  }
  add(static_cast<detail::Source &>(sub), group, priority);
}

void PartitionedExecutor::add(
  detail::Source & src, const rclcpp::CallbackGroup::SharedPtr & group, std::int32_t priority)
{
  const std::lock_guard<std::mutex> lk(m_);
  if (spinning_) throw std::logic_error("flux: add() while spinning");
  if (!group) throw std::invalid_argument("flux: add() needs a callback group");
  for (const auto & entry : assigned_) {
    for (const Assigned & a : entry.second) {
      if (a.src == &src) throw std::invalid_argument("flux: source assigned twice");
    }
  }
  assigned_[group.get()].push_back({&src, priority});
  kept_.emplace(group.get(), group);
}

void PartitionedExecutor::add_node(const NodeBase::SharedPtr & node)
{
  const std::lock_guard<std::mutex> lk(groups_m_);
  if (serving_) {
    for (const auto & group : unserved_groups(node)) spawn(group, node);
  }
  nodes_.push_back(node);
}

void PartitionedExecutor::remove_node(const NodeBase::SharedPtr & node)
{
  std::vector<Child> gone;
  std::vector<const rclcpp::CallbackGroup *> groups;
  {
    const std::lock_guard<std::mutex> lk(groups_m_);
    nodes_.erase(
      std::remove_if(
        nodes_.begin(), nodes_.end(),
        [&](const NodeBase::WeakPtr & n) { return n.lock() == node; }),
      nodes_.end());
    node->for_each_callback_group(
      [&](const rclcpp::CallbackGroup::SharedPtr & group) { groups.push_back(group.get()); });
    for (auto it = children_.begin(); it != children_.end();) {
      if (it->node.lock() != node) {
        ++it;
        continue;
      }
      groups.push_back(it->group.get());
      gone.push_back(std::move(*it));
      it = children_.erase(it);
    }
    for (const rclcpp::CallbackGroup * group : groups) claimed_.erase(group);
  }
  {
    const std::lock_guard<std::mutex> lk(m_);
    for (const rclcpp::CallbackGroup * group : groups) {
      assigned_.erase(group);
      init_.erase(group);
      kept_.erase(group);
    }
  }
  join(gone);
}

void PartitionedExecutor::on_thread_start(
  const rclcpp::CallbackGroup::SharedPtr & group, std::function<void()> init)
{
  const std::lock_guard<std::mutex> lk(m_);
  if (spinning_) throw std::logic_error("flux: on_thread_start() while spinning");
  if (!group || !init) throw std::invalid_argument("flux: on_thread_start() needs both");
  if (!init_.emplace(group.get(), std::move(init)).second) {
    throw std::invalid_argument("flux: the group already has a thread-start hook");
  }
  kept_.emplace(group.get(), group);
}

void PartitionedExecutor::on_thread_start(ThreadStart init)
{
  const std::lock_guard<std::mutex> lk(m_);
  if (spinning_) throw std::logic_error("flux: on_thread_start() while spinning");
  if (!init) throw std::invalid_argument("flux: on_thread_start() needs a hook");
  if (any_init_) throw std::invalid_argument("flux: every thread already has a thread-start hook");
  any_init_ = std::move(init);
}

void PartitionedExecutor::cancel() noexcept
{
  stop_ = true;
  wake_.wake();
}

void PartitionedExecutor::fail(std::exception_ptr e)
{
  {
    const std::lock_guard<std::mutex> lk(m_);
    if (!failure_) failure_ = std::move(e);
  }
  wake_.wake();
}

void PartitionedExecutor::spin()
{
  {
    const std::lock_guard<std::mutex> lk(m_);
    if (spinning_) throw std::logic_error("flux: spin() while already spinning");
    spinning_ = true;
  }
  const detail::OnExit reset([this] {
    const std::lock_guard<std::mutex> lk(m_);
    stop_ = false;
    failure_ = nullptr;
    spinning_ = false;
  });

  constexpr std::int64_t kRescanNs = 100'000'000;
  const auto failed = [this] {
    const std::lock_guard<std::mutex> lk(m_);
    return failure_ != nullptr;
  };
  try {
    {
      const std::lock_guard<std::mutex> lk(groups_m_);
      serving_ = true;
    }
    rescan();
    {
      const std::lock_guard<std::mutex> groups_lk(groups_m_);
      const std::lock_guard<std::mutex> lk(m_);
      for (const auto & entry : assigned_) {
        if (claimed_.count(entry.first) == 0) {
          throw std::invalid_argument(
            "flux: a group with subscriptions belongs to no node given to add_node()");
        }
      }
      for (const auto & entry : init_) {
        if (claimed_.count(entry.first) == 0) {
          throw std::invalid_argument("flux: a thread-start hook names a group no thread serves");
        }
      }
      check_sync_groups();
    }
    while (!stop_ && !failed() && rclcpp::ok(context_)) {
      waiter_.wait_for_work(kRescanNs);
      rescan();
    }
  } catch (...) {
    fail(std::current_exception());
  }
  stop_children();
  std::exception_ptr failure;
  {
    const std::lock_guard<std::mutex> lk(m_);
    failure = failure_;
  }
  if (failure) std::rethrow_exception(failure);
}

void PartitionedExecutor::stop_children()
{
  std::vector<Child> all;
  {
    const std::lock_guard<std::mutex> lk(groups_m_);
    serving_ = false;
    all.swap(children_);
    claimed_.clear();
  }
  join(all);
}

void PartitionedExecutor::join(std::vector<Child> & children)
{
  for (Child & c : children) c.ex->cancel();
  for (Child & c : children) c.thread.join();
  children.clear();
}

void PartitionedExecutor::check_sync_groups()
{
  const auto group_of = [this](const SyncInput & in) -> const void * {
    if (in.flux != nullptr) {
      for (const auto & entry : assigned_) {
        for (const Assigned & a : entry.second) {
          if (a.src == in.flux) return entry.first;
        }
      }
      throw std::invalid_argument("flux: a synchronizer input was never assigned to a group");
    }
    for (const auto & entry : claimed_) {
      const auto group = entry.second.lock();
      if (group && group->find_subscription_ptrs_if([&](const auto & s) { return s == in.ros; })) {
        return entry.first;
      }
    }
    throw std::invalid_argument(
      "flux: a synchronizer input belongs to no group this executor serves");
  };
  for (const auto & inputs : sync_groups_) {
    const void * first = group_of(inputs.front());
    for (const SyncInput & in : inputs) {
      if (group_of(in) != first) {
        throw std::invalid_argument(
          "flux: a synchronizer's inputs are in different callback groups; put them in one");
      }
    }
  }
}

// Groups appear and go while spinning, so every pass looks again.
void PartitionedExecutor::rescan()
{
  std::vector<Child> gone;
  {
    const std::lock_guard<std::mutex> lk(groups_m_);
    for (auto it = children_.begin(); it != children_.end();) {
      // Gone once this executor is its last owner.
      if (it->group.use_count() > 1) {
        ++it;
        continue;
      }
      gone.push_back(std::move(*it));
      it = children_.erase(it);
    }
  }
  join(gone);

  const std::lock_guard<std::mutex> lk(groups_m_);
  for (auto it = claimed_.begin(); it != claimed_.end();) {
    it = it->second.expired() ? claimed_.erase(it) : std::next(it);
  }
  for (auto it = nodes_.begin(); it != nodes_.end();) {
    const auto base = it->lock();
    if (!base) {
      it = nodes_.erase(it);
      continue;
    }
    for (const auto & group : unserved_groups(base)) spawn(group, base);
    ++it;
  }
}

// Collected before any is spawned: the walk holds the node's group lock, which is not recursive,
// and a thread-start hook may walk the node's groups too.
std::vector<rclcpp::CallbackGroup::SharedPtr> PartitionedExecutor::unserved_groups(
  const NodeBase::SharedPtr & node)
{
  std::set<const rclcpp::CallbackGroup *> assigned;
  {
    const std::lock_guard<std::mutex> lk(m_);
    for (const auto & entry : assigned_) assigned.insert(entry.first);
  }
  std::vector<rclcpp::CallbackGroup::SharedPtr> found;
  node->for_each_callback_group([&](const rclcpp::CallbackGroup::SharedPtr & group) {
    if (claimed_.count(group.get()) != 0) return;
    if (
      assigned.count(group.get()) == 0 && (!group->automatically_add_to_executor_with_node() ||
                                           group->get_associated_with_executor_atomic().load())) {
      return;
    }
    found.push_back(group);
  });
  for (const auto & group : found) {
    if (group->type() == rclcpp::CallbackGroupType::Reentrant) {
      throw std::invalid_argument(
        "flux: a Reentrant group cannot run on one thread; make it MutuallyExclusive (node " +
        std::string(node->get_fully_qualified_name()) + ")");
    }
  }
  return found;
}

void PartitionedExecutor::spawn(
  const rclcpp::CallbackGroup::SharedPtr & group, const NodeBase::SharedPtr & node)
{
  auto ex = std::make_unique<Executor>();
  std::function<void()> init;
  ThreadStart any;
  {
    const std::lock_guard<std::mutex> lk(m_);
    if (const auto it = assigned_.find(group.get()); it != assigned_.end()) {
      for (const Assigned & a : it->second) ex->add(*a.src, a.priority);
    }
    if (const auto it = init_.find(group.get()); it != init_.end()) init = it->second;
    any = any_init_;
  }
  ex->add_callback_group(group, node);
  claimed_.emplace(group.get(), group);

  Executor * raw = ex.get();
  std::thread t([this, raw, init = std::move(init), any = std::move(any),
                 weak_node = NodeBase::WeakPtr(node),
                 weak_group = rclcpp::CallbackGroup::WeakPtr(group)] {
    try {
      if (any) {
        const auto n = weak_node.lock();
        const auto g = weak_group.lock();
        if (n && g) any(n, g);
      }
      if (init) init();
      raw->spin();
    } catch (...) {
      fail(std::current_exception());
    }
  });
  const std::string name = "flux-g" + std::to_string(spawned_++);
  pthread_setname_np(t.native_handle(), name.c_str());
  children_.push_back({std::move(ex), std::move(t), group, node});
}

}  // namespace flux
