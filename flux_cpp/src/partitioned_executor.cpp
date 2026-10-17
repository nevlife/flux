#include "flux/partitioned_executor.hpp"

#include "on_exit.hpp"

#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/utilities.hpp>

#include <pthread.h>

#include <chrono>
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

void PartitionedExecutor::add_node(const rclcpp::Node::SharedPtr & node)
{
  const std::lock_guard<std::mutex> lk(m_);
  if (spinning_) throw std::logic_error("flux: add_node() while spinning");
  nodes_.push_back(node);
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
    spawn_new_groups();
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
    while (!stop_ && !failed() && rclcpp::ok(context_)) {
      waiter_.wait_for_work(kRescanNs);
      spawn_new_groups();
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
  for (Child & c : children_) c.ex->cancel();
  for (Child & c : children_) c.thread.join();
  children_.clear();
  claimed_.clear();
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

void PartitionedExecutor::join_gone_groups()
{
  std::vector<Child> gone;
  for (auto it = children_.begin(); it != children_.end();) {
    // Gone once this executor is its last owner.
    if (it->group.use_count() > 1) {
      ++it;
      continue;
    }
    it->ex->cancel();
    gone.push_back(std::move(*it));
    it = children_.erase(it);
  }
  for (Child & c : gone) c.thread.join();
}

// Groups appear and go while spinning, so every pass looks again.
void PartitionedExecutor::spawn_new_groups()
{
  join_gone_groups();
  for (auto it = claimed_.begin(); it != claimed_.end();) {
    it = it->second.expired() ? claimed_.erase(it) : std::next(it);
  }
  for (auto it = nodes_.begin(); it != nodes_.end();) {
    const auto node = it->lock();
    if (!node) {
      it = nodes_.erase(it);
      continue;
    }
    const auto base = node->get_node_base_interface();
    base->for_each_callback_group([&](const rclcpp::CallbackGroup::SharedPtr & group) {
      if (claimed_.count(group.get()) != 0) return;
      const bool assigned = assigned_.count(group.get()) != 0;
      if (!assigned && !group->automatically_add_to_executor_with_node()) return;
      if (group->type() == rclcpp::CallbackGroupType::Reentrant) {
        throw std::invalid_argument(
          "flux: a Reentrant group cannot run on one thread; make it MutuallyExclusive");
      }
      spawn(group, base);
    });
    ++it;
  }
}

void PartitionedExecutor::spawn(
  const rclcpp::CallbackGroup::SharedPtr & group,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node)
{
  auto ex = std::make_unique<Executor>();
  if (const auto it = assigned_.find(group.get()); it != assigned_.end()) {
    for (const Assigned & a : it->second) ex->add(*a.src, a.priority);
  }
  ex->add_callback_group(group, node);
  claimed_.emplace(group.get(), group);

  std::function<void()> init;
  if (const auto it = init_.find(group.get()); it != init_.end()) init = it->second;
  Executor * raw = ex.get();
  std::thread t([this, raw, init = std::move(init)] {
    try {
      if (init) init();
      raw->spin();
    } catch (...) {
      fail(std::current_exception());
    }
  });
  const std::string name = "flux-g" + std::to_string(children_.size());
  pthread_setname_np(t.native_handle(), name.c_str());
  children_.push_back({std::move(ex), std::move(t), group});
}

}  // namespace flux
