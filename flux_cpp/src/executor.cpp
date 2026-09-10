#include "flux/ros/executor.hpp"

#include <rclcpp/message_info.hpp>

#include <chrono>
#include <stdexcept>
#include <string>

namespace flux::ros
{

Executor::Executor(unsigned max_channels)
: rclcpp::Executor(rclcpp::ExecutorOptions()), core_(max_channels)
{
}

Executor::~Executor()
{
  // Best effort throughout: stop new pokes before the wake fd goes.
  for (auto & [ptr, weak] : ros_subs_) {
    if (auto s = weak.lock()) {
      try {
        s->clear_on_new_message_callback();
        if (s->get_intra_process_waitable() != nullptr) {
          s->clear_on_new_intra_process_message_callback();
        }
      } catch (...) {
      }
    }
  }
  for (auto & [ptr, weak] : ros_services_) {
    if (auto s = weak.lock()) {
      try {
        s->clear_on_new_request_callback();
      } catch (...) {
      }
    }
  }
  for (auto & [ptr, weak] : ros_clients_) {
    if (auto c = weak.lock()) {
      try {
        c->clear_on_new_response_callback();
      } catch (...) {
      }
    }
  }
  for (auto & [ptr, weak] : ros_waitables_) {
    if (auto wt = weak.lock()) {
      try {
        wt->clear_on_ready_callback();
      } catch (...) {
      }
    }
  }
}

void Executor::add(Subscription & sub, int priority)
{
  if (reentry_.load()) {
    throw std::logic_error("flux: add() must be called before spin()");
  }
  if (!sub.has_callback()) {
    throw std::invalid_argument(
      "flux: this Subscription has no callback, so an executor has nothing to run -- construct it "
      "with one, or read it yourself with take()/take_blocking()");
  }
  core_.add(sub, priority);
}

void Executor::add(flux::Source & src, int priority)
{
  if (reentry_.load()) {
    throw std::logic_error("flux: add() must be called before spin()");
  }
  core_.add(src, priority);
}

void Executor::stop() noexcept
{
  stop_requested_.store(true, std::memory_order_relaxed);
  core_.interrupt();
}

void Executor::bridge_group(const rclcpp::CallbackGroup::SharedPtr & group)
{
  // Hooks run on rmw listener threads and can outlive this executor by a call; the waker reaches
  // the wake fd through a weak reference, never through `this`.
  const auto poke = [w = core_.waker(), r = ros_ready_](std::size_t) {
    r->store(true, std::memory_order_release);  // set before the poke that reveals it
    w();
  };
  // A waitable's hook carries an extra int naming which entity inside it fired; the wait layer
  // does not care which, so it is dropped here.
  const auto poke_waitable = [poke](std::size_t n, int) { poke(n); };
  group->collect_all_ptrs(
    [this, &poke](const rclcpp::SubscriptionBase::SharedPtr & sub) {
      if (!ros_subs_.emplace(sub.get(), sub).second) return;  // already bridged
      try {
        sub->set_on_new_message_callback(poke);
        // Intra-process readiness comes through a separate waitable, not the rmw queue;
        // without its own hook those messages would surface only on the tick.
        if (sub->get_intra_process_waitable() != nullptr) {
          sub->set_on_new_intra_process_message_callback(poke);
        }
      } catch (...) {
        // Racing a context shutdown: the intra-process manager can already be gone.
        // Unmark so a pass on a live context retries instead of keeping a dead bridge.
        ros_subs_.erase(sub.get());
      }
    },
    [this, &poke](const rclcpp::ServiceBase::SharedPtr & srv) {
      if (!ros_services_.emplace(srv.get(), srv).second) return;
      try {
        srv->set_on_new_request_callback(poke);
      } catch (...) {
        ros_services_.erase(srv.get());
      }
    },
    [this, &poke](const rclcpp::ClientBase::SharedPtr & cli) {
      if (!ros_clients_.emplace(cli.get(), cli).second) return;
      try {
        cli->set_on_new_response_callback(poke);
      } catch (...) {
        ros_clients_.erase(cli.get());
      }
    },
    // Timers are not hooked: they have no readiness event to fire. next_ros_timeout() shortens
    // the wait to the nearest deadline instead, which is the only thing that answers a timer.
    [](const rclcpp::TimerBase::SharedPtr &) {},
    // An action server and an action client are each one Waitable, so this is the hook that
    // makes goal, cancel and result reach the ring on arrival rather than on the next tick.
    [this, &poke_waitable](const rclcpp::Waitable::SharedPtr & wt) {
      if (!ros_waitables_.emplace(wt.get(), wt).second) return;
      try {
        wt->set_on_ready_callback(poke_waitable);
      } catch (...) {
        ros_waitables_.erase(wt.get());
      }
    });
}

// Drop the hooks this executor installed. A hook outlives removal otherwise, and keeps waking
// the ring for an entity no pass will service. Clearing is best-effort for the same reason
// setting is: a context already shutting down can have taken the intra-process manager with it.
void Executor::unbridge_group(const rclcpp::CallbackGroup::SharedPtr & group)
{
  if (!group) return;
  group->collect_all_ptrs(
    [this](const rclcpp::SubscriptionBase::SharedPtr & sub) {
      if (ros_subs_.erase(sub.get()) == 0) return;
      try {
        sub->clear_on_new_message_callback();
        if (sub->get_intra_process_waitable() != nullptr) {
          sub->clear_on_new_intra_process_message_callback();
        }
      } catch (...) {
      }
    },
    [this](const rclcpp::ServiceBase::SharedPtr & srv) {
      if (ros_services_.erase(srv.get()) == 0) return;
      try {
        srv->clear_on_new_request_callback();
      } catch (...) {
      }
    },
    [this](const rclcpp::ClientBase::SharedPtr & cli) {
      if (ros_clients_.erase(cli.get()) == 0) return;
      try {
        cli->clear_on_new_response_callback();
      } catch (...) {
      }
    },
    [](const rclcpp::TimerBase::SharedPtr &) {},
    [this](const rclcpp::Waitable::SharedPtr & wt) {
      if (ros_waitables_.erase(wt.get()) == 0) return;
      try {
        wt->clear_on_ready_callback();
      } catch (...) {
      }
    });
}

// Entities created after add_ros_node()/add_ros_callback_group() would otherwise never poke the
// wake fd. Pruning before re-scanning keeps a recycled address from being mistaken for its
// bridged predecessor. Every hook replays the pending count when set, so a request or message
// that arrived before this pass still wakes the ring.
void Executor::rebridge_ros()
{
  const auto prune = [](auto & map) {
    for (auto it = map.begin(); it != map.end();) {
      it = it->second.expired() ? map.erase(it) : std::next(it);
    }
  };
  prune(ros_subs_);
  prune(ros_services_);
  prune(ros_clients_);
  prune(ros_waitables_);
  for (auto it = ros_nodes_.begin(); it != ros_nodes_.end();) {
    if (auto node = it->lock()) {
      node->for_each_callback_group(
        [this](const rclcpp::CallbackGroup::SharedPtr & group) { bridge_group(group); });
      ++it;
    } else {
      it = ros_nodes_.erase(it);
    }
  }
  for (auto it = ros_groups_.begin(); it != ros_groups_.end();) {
    if (auto group = it->lock()) {
      bridge_group(group);
      ++it;
    } else {
      it = ros_groups_.erase(it);
    }
  }
}

void Executor::add_ros_node(const rclcpp::Node::SharedPtr & node)
{
  add_node(node, /*notify=*/true);
}

void Executor::add_ros_callback_group(
  const rclcpp::CallbackGroup::SharedPtr & group,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base)
{
  add_callback_group(group, node_base, /*notify=*/true);
}

void Executor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  if (reentry_.load()) {
    throw std::logic_error("flux: add_ros_node() must be called before spin()");
  }
  rclcpp::Executor::add_node(node_ptr, notify);
  ros_nodes_.push_back(node_ptr);
  node_ptr->for_each_callback_group(
    [this](const rclcpp::CallbackGroup::SharedPtr & group) { bridge_group(group); });
}

void Executor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  add_node(node_ptr->get_node_base_interface(), notify);
}

void Executor::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  if (reentry_.load()) {
    throw std::logic_error("flux: remove_ros_node() must be called before spin()");
  }
  for (auto it = ros_nodes_.begin(); it != ros_nodes_.end();) {
    auto held = it->lock();
    it = (!held || held == node_ptr) ? ros_nodes_.erase(it) : std::next(it);
  }
  node_ptr->for_each_callback_group(
    [this](const rclcpp::CallbackGroup::SharedPtr & group) { unbridge_group(group); });
  rclcpp::Executor::remove_node(node_ptr, notify);
}

void Executor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  remove_node(node_ptr->get_node_base_interface(), notify);
}

void Executor::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  if (reentry_.load()) {
    throw std::logic_error("flux: add_ros_callback_group() must be called before spin()");
  }
  rclcpp::Executor::add_callback_group(group_ptr, node_ptr, notify);
  ros_groups_.push_back(group_ptr);
  bridge_group(group_ptr);
}

void Executor::remove_callback_group(rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify)
{
  if (reentry_.load()) {
    throw std::logic_error("flux: remove_ros_callback_group() must be called before spin()");
  }
  for (auto it = ros_groups_.begin(); it != ros_groups_.end();) {
    auto held = it->lock();
    it = (!held || held == group_ptr) ? ros_groups_.erase(it) : std::next(it);
  }
  unbridge_group(group_ptr);
  rclcpp::Executor::remove_callback_group(group_ptr, notify);
}

void Executor::reject_inherited_spin(const char * name)
{
  throw std::runtime_error(
    std::string("flux: ") + name +
    " is not supported -- its contract is a duration budget over a wait set this executor does "
    "not have, and an approximation would silently skip flux channels; use spin(tick_ns) or "
    "dispatch() + wait_for_work()");
}

// Not rclcpp::Executor::cancel(), which clears `spinning`. This loop ends on stop_requested_,
// so clearing that flag would not end it -- it would only make the next ROS pass look cancelled.
void Executor::cancel()
{
  stop();
}

void Executor::spin()
{
  spin(kDefaultTickNs);
}

void Executor::spin_once(std::chrono::nanoseconds timeout)
{
  spin_once(static_cast<std::int64_t>(timeout.count()));
}

void Executor::spin_some(std::chrono::nanoseconds)
{
  reject_inherited_spin("spin_some()");
}

void Executor::spin_all(std::chrono::nanoseconds)
{
  reject_inherited_spin("spin_all()");
}

// The inherited spin_once() above never routes here; rclcpp's own spin_until_future_complete
// would, and that one is refused.
void Executor::spin_once_impl(std::chrono::nanoseconds)
{
  reject_inherited_spin("spin_once_impl()");
}

void Executor::spin_node_some(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr)
{
  reject_inherited_spin("spin_node_some()");
}

void Executor::spin_node_some(std::shared_ptr<rclcpp::Node>)
{
  reject_inherited_spin("spin_node_some()");
}

void Executor::spin_node_all(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr, std::chrono::nanoseconds)
{
  reject_inherited_spin("spin_node_all()");
}

void Executor::spin_node_all(std::shared_ptr<rclcpp::Node>, std::chrono::nanoseconds)
{
  reject_inherited_spin("spin_node_all()");
}

rclcpp::FutureReturnCode Executor::spin_until_future_complete_impl(
  std::chrono::nanoseconds, const std::function<std::future_status(std::chrono::nanoseconds)> &)
{
  reject_inherited_spin("spin_until_future_complete()");
}

// Without this a tick longer than a timer period stretches that timer to the tick: nothing else
// wakes the ring, so a 10 ms control timer on the default 100 ms tick fires at 100 ms. The wait
// is what has to know the deadline -- pump_ros() can only run what is already due.
std::int64_t Executor::next_ros_timeout(std::int64_t tick_ns)
{
  std::int64_t best = tick_ns;  // negative means block indefinitely
  auto consider = [&best](const rclcpp::CallbackGroup::SharedPtr & group) {
    if (!group) return;
    group->collect_all_ptrs(
      [](const rclcpp::SubscriptionBase::SharedPtr &) {},
      [](const rclcpp::ServiceBase::SharedPtr &) {}, [](const rclcpp::ClientBase::SharedPtr &) {},
      [&best](const rclcpp::TimerBase::SharedPtr & timer) {
        if (!timer) return;
        const std::int64_t due = timer->time_until_trigger().count();
        const std::int64_t wait = due > 0 ? due : 0;
        if (best < 0 || wait < best) best = wait;
      },
      [](const rclcpp::Waitable::SharedPtr &) {});
  };
  for (auto & weak : ros_nodes_) {
    if (auto node = weak.lock()) node->for_each_callback_group(consider);
  }
  for (auto & weak : ros_groups_) {
    if (auto group = weak.lock()) consider(group);
  }
  return best;
}

// Measured on rclcpp 28.1.21 with rmw_fastrtps_cpp: with `spinning` down, get_next_executable()
// reports nothing and execute_any_executable() returns without running the callback it was
// handed. Both are that library's cancel path, not a spin-loop requirement. Saved and restored
// rather than cleared, because spin() is holding the same flag up for its own loop.
int Executor::pump_ros(int budget)
{
  struct Gate
  {
    Executor & ex;
    bool held;
    ~Gate() { ex.spinning.store(held); }
  } gate{*this, spinning.exchange(true)};

  int n = 0;
  rclcpp::AnyExecutable any;
  while (n < budget && rclcpp::ok(context_) &&
         get_next_executable(any, std::chrono::nanoseconds(0))) {
    execute_any_executable(any);
    any = rclcpp::AnyExecutable();
    ++n;
  }
  return n;
}

void Executor::set_ros_budget(int n)
{
  if (reentry_.load()) {
    throw std::logic_error("flux: set_ros_budget() must be called before spin()");
  }
  if (n < 1) {
    throw std::invalid_argument("flux: ros_budget must be at least 1");
  }
  ros_budget_ = n;
}

int Executor::spin_once(std::int64_t timeout_ns)
{
  rebridge_ros();
  const int n = core_.spin_once(next_ros_timeout(timeout_ns));
  ros_backlog_ = pump_ros(ros_budget_) == ros_budget_;
  return n;
}

void Executor::spin(std::atomic<bool> & run, std::int64_t tick_ns)
{
  // Two spins would race the entry lists. The guard is reentry_ rather than rclcpp's `spinning`
  // because that flag means "not cancelled" there and every ROS pass has to raise it, so a
  // spin_once() during a spin() would read as a second spin.
  if (reentry_.exchange(true)) {
    throw std::logic_error("flux: spin() called while already spinning");
  }
  // Held up for the whole loop so is_spinning() answers what a caller means by it. pump_ros()
  // would raise it per pass anyway.
  spinning.store(true);
  struct SpinGuard
  {
    Executor & ex;
    ~SpinGuard()
    {
      ex.stop_requested_.store(false, std::memory_order_relaxed);
      ex.spinning.store(false);
      ex.reentry_.store(false);
    }
  } guard{*this};

  bool flux_pending = false;
  while (run.load(std::memory_order_relaxed) && !stop_requested_.load(std::memory_order_relaxed)) {
    rebridge_ros();
    const std::int64_t due = next_ros_timeout(tick_ns);  // 0 means a timer is due now
    const bool ready = flux_pending || ros_backlog_;
    const auto t0 = std::chrono::steady_clock::now();
    core_.wait_for_work(ready ? 0 : due);
    const std::int64_t waited =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0)
        .count();
    // Same order as spin_once(). Dispatching first put a pump_ros() inside one frame's latency.
    // From t0, not from the end of the wait: that is what catches a timer coming due inside the
    // dispatch, which reading the deadline only beforehand charged a second pass.
    bool ros_due = false;
    flux_pending = core_.dispatch([&] {
      if (ros_due) return true;
      // Peek, never take: the gate below consumes the flag.
      if (ros_ready_->load(std::memory_order_acquire)) return ros_due = true;
      if (due < 0) return false;
      const std::int64_t elapsed =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0)
          .count();
      return ros_due = elapsed >= due;
    }) > 0;
    // A pass of rclcpp costs tens of microseconds ready or not.
    const bool timer_or_tick = ros_due || (due >= 0 && waited >= due);
    if (take_ros_ready() || ros_backlog_ || due <= 0 || timer_or_tick) {
      ros_backlog_ = pump_ros(ros_budget_) == ros_budget_;
    }
  }
}

void Executor::spin(std::int64_t tick_ns)
{
  std::atomic<bool> always{true};
  spin(always, tick_ns);
}

}  // namespace flux::ros
