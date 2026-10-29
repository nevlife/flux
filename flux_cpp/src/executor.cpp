#include "flux/executor.hpp"

#include "flux/detail/group_member.hpp"
#include "on_exit.hpp"

#include <rclcpp/any_executable.hpp>
#include <rclcpp/utilities.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace flux
{

namespace
{

std::int64_t since_ns(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0)
    .count();
}

// Hooked again if the address now holds a different entity than the one hooked.
template <class Map, class Ptr>
bool hooked(const Map & map, const Ptr & p)
{
  const auto it = map.find(p.get());
  return it != map.end() && it->second.lock() == p;
}

template <class Map, class Clear>
void unhook_if(Map & map, const std::unordered_set<const void *> & keep, Clear clear)
{
  for (auto it = map.begin(); it != map.end();) {
    if (keep.count(it->first) != 0) {
      ++it;
      continue;
    }
    if (auto p = it->second.lock()) clear(*p);
    it = map.erase(it);
  }
}

#if RCLCPP_VERSION_MAJOR >= 28
rclcpp::Clock::SharedPtr clock_of(const rclcpp::TimerBase & timer)
{
  return timer.get_clock();
}
#else
// rclcpp before Jazzy keeps the timer's clock protected; a member pointer formed in a derived
// class reads it from any TimerBase.
struct TimerClock : rclcpp::TimerBase
{
  static rclcpp::Clock::SharedPtr of(const rclcpp::TimerBase & timer)
  {
    return timer.*(&TimerClock::clock_);
  }
};

rclcpp::Clock::SharedPtr clock_of(const rclcpp::TimerBase & timer)
{
  return TimerClock::of(timer);
}
#endif

// As rcl_wait does, a timer on active ROS time is left to its clock: the time it reports is ROS
// time, which can run at any rate, and the jump hook pokes once it is due.
template <class Poke>
rclcpp::JumpHandler::SharedPtr on_ros_time(
  const rclcpp::TimerBase::SharedPtr & timer, const Poke & poke)
{
  const rclcpp::Clock::SharedPtr clock = clock_of(*timer);
  if (clock->get_clock_type() != RCL_ROS_TIME) return nullptr;
  rcl_jump_threshold_t any{};
  any.on_clock_change = true;
  any.min_forward.nanoseconds = 1;
  any.min_backward.nanoseconds = -1;
  return clock->create_jump_callback(
    nullptr,
    [weak = rclcpp::TimerBase::WeakPtr(timer), poke](const rcl_time_jump_t & jump) {
      const auto t = weak.lock();
      if (
        jump.clock_change != RCL_ROS_TIME_NO_CHANGE ||
        (t && t->time_until_trigger() <= std::chrono::nanoseconds(0))) {
        poke(1);
      }
    },
    any);
}

}  // namespace

Executor::Executor()
: rclcpp::Executor(rclcpp::ExecutorOptions()),
  handle_(core_.handle()),
  on_shutdown_(context_->add_on_shutdown_callback([this] { cancel(); }))
{
}

Executor::~Executor()
{
  context_->remove_on_shutdown_callback(on_shutdown_);
  // A hook left behind would wake an executor that no longer exists.
  seen_.clear();
  forget_moved(get_all_callback_groups());
  unhook_unseen();
}

void Executor::add(Subscription & sub, std::int32_t priority)
{
  if (!sub.has_callback()) {
    throw std::invalid_argument(
      "flux: a Subscription without a callback gives an executor nothing to run; read it with "
      "take() instead");
  }
  add(static_cast<detail::Source &>(sub), priority);
}

// Holds the spin flag while it adds, so a spin() starting meanwhile is refused instead of walking
// the sources being changed.
void Executor::add(detail::Source & src, std::int32_t priority)
{
  if (spinning.exchange(true)) throw std::logic_error("flux: add() while spinning");
  const detail::OnExit release([this] { spinning.store(false); });
  core_.add(src, priority);
}

void Executor::spin()
{
  run(std::chrono::nanoseconds(-1), [] { return false; });
}

rclcpp::FutureReturnCode Executor::spin_until_future_complete_impl(
  std::chrono::nanoseconds timeout,
  const std::function<std::future_status(std::chrono::nanoseconds)> & wait_for_future)
{
  const auto ready = [&] {
    return wait_for_future(std::chrono::nanoseconds(0)) == std::future_status::ready;
  };
  if (ready()) return rclcpp::FutureReturnCode::SUCCESS;
  return run(timeout, ready);
}

rclcpp::FutureReturnCode Executor::run(
  std::chrono::nanoseconds timeout, const std::function<bool()> & done)
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("flux: spin() while already spinning or in add()");
  }
  detail::GroupMember::in_flux_executor() = true;
  const detail::OnExit reset([this] {
    detail::GroupMember::in_flux_executor() = false;
    stop_ = false;
    core_.clear_interrupt();
    spinning.store(false);
  });

  const auto start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point rescanned{};
  while (!stop_.load() && rclcpp::ok(context_)) {
    std::int64_t wait = detail::kRescanNs - since_ns(rescanned);
    if (wait <= 0) {
      rebridge_ros();
      rescanned = std::chrono::steady_clock::now();
      wait = detail::kRescanNs;
    }
    if (timeout.count() >= 0) {
      wait = std::min(wait, std::max<std::int64_t>(0, timeout.count() - since_ns(start)));
    }
    pass(wait);
    if (done()) return rclcpp::FutureReturnCode::SUCCESS;
    // After the pass, as in rclcpp: a zero timeout still runs the work that is ready.
    if (timeout.count() >= 0 && since_ns(start) >= timeout.count()) {
      return rclcpp::FutureReturnCode::TIMEOUT;
    }
  }
  return rclcpp::FutureReturnCode::INTERRUPTED;
}

void Executor::spin_once_impl(std::chrono::nanoseconds)
{
  throw std::runtime_error(
    "flux: spin_once() is not supported; use spin() or spin_until_future_complete()");
}

void Executor::spin_some(std::chrono::nanoseconds)
{
  throw std::runtime_error(
    "flux: spin_some() is not supported; use spin() or spin_until_future_complete()");
}

void Executor::spin_all(std::chrono::nanoseconds)
{
  throw std::runtime_error(
    "flux: spin_all() is not supported; use spin() or spin_until_future_complete()");
}

void Executor::cancel()
{
  stop_ = true;
  handle_.interrupt();
}

void Executor::pass(std::int64_t wait_ns)
{
  // The merged wait knows no deadlines, so the nearest ROS timer shortens it.
  const std::int64_t due = next_ros_timeout(wait_ns);
  const bool ready = core_.has_more() || ros_backlog_;
  const auto t0 = std::chrono::steady_clock::now();
  core_.wait_for_work(ready ? 0 : due);

  // ROS work that comes due during the flux callbacks cuts them short, so neither side waits
  // out the other's backlog.
  bool ros_due = false;
  auto yield = [&]() noexcept {
    if (!ros_due) {
      ros_due = ros_ready_->load(std::memory_order_acquire) || (due >= 0 && since_ns(t0) >= due);
    }
    return ros_due;
  };
  core_.dispatch(yield);

  const bool woke = ros_ready_->exchange(false, std::memory_order_acq_rel);
  if (woke || ros_backlog_ || ros_due || (due >= 0 && since_ns(t0) >= due)) {
    ros_backlog_ = pump_ros();
  }
}

// Once flux has work the run ends after kTurnNs, so cheap ROS callbacks still drain together but
// a slow one does not hold flux behind it. True when ROS work may be left.
bool Executor::pump_ros()
{
  int n = 0;
  const auto start = std::chrono::steady_clock::now();
  rclcpp::AnyExecutable any;
  while (n < kRosBudget && rclcpp::ok(context_) &&
         get_next_executable(any, std::chrono::nanoseconds(0))) {
    execute_any_executable(any);
    any = rclcpp::AnyExecutable();
    ++n;
    if (since_ns(start) >= kTurnNs && core_.ready()) return true;
  }
  return n == kRosBudget;
}

// Every hook replays what arrived before it was set, so nothing waiting is missed.
void Executor::rebridge_ros()
{
  seen_.clear();
  const auto groups = get_all_callback_groups();
  for (const auto & weak : groups) {
    if (auto group = weak.lock()) bridge_group(group);
  }
  forget_moved(groups);
  unhook_unseen();
}

// A group that left for another executor carries that executor's hooks now: clearing them would
// leave it deaf, so its entities are dropped from the maps untouched.
void Executor::forget_moved(const std::vector<rclcpp::CallbackGroup::WeakPtr> & now)
{
  for (const auto & weak : groups_) {
    const auto group = weak.lock();
    if (!group || !group->get_associated_with_executor_atomic().load()) continue;
    const bool ours =
      std::any_of(now.begin(), now.end(), [&](const auto & w) { return w.lock() == group; });
    if (ours) continue;
    group->collect_all_ptrs(
      [&](const rclcpp::SubscriptionBase::SharedPtr & e) { subs_.erase(e.get()); },
      [&](const rclcpp::ServiceBase::SharedPtr & e) { services_.erase(e.get()); },
      [&](const rclcpp::ClientBase::SharedPtr & e) { clients_.erase(e.get()); },
      [&](const rclcpp::TimerBase::SharedPtr & e) { timers_.erase(e.get()); },
      [&](const rclcpp::Waitable::SharedPtr & e) { waitables_.erase(e.get()); });
  }
  groups_ = now;
}

void Executor::bridge_group(const rclcpp::CallbackGroup::SharedPtr & group)
{
  const auto poke = [h = handle_, r = ros_ready_](std::size_t) {
    r->store(true, std::memory_order_release);
    h.wake();
  };
  group->collect_all_ptrs(
    [&](const rclcpp::SubscriptionBase::SharedPtr & sub) {
      seen_.insert(sub.get());
      if (hooked(subs_, sub)) return;
      sub->set_on_new_message_callback(poke);
      // Intra-process messages arrive through their own waitable, not the rmw queue.
      if (sub->get_intra_process_waitable() != nullptr) {
        sub->set_on_new_intra_process_message_callback(poke);
      }
      subs_[sub.get()] = sub;
    },
    [&](const rclcpp::ServiceBase::SharedPtr & srv) {
      seen_.insert(srv.get());
      if (hooked(services_, srv)) return;
      srv->set_on_new_request_callback(poke);
      services_[srv.get()] = srv;
    },
    [&](const rclcpp::ClientBase::SharedPtr & cli) {
      seen_.insert(cli.get());
      if (hooked(clients_, cli)) return;
      cli->set_on_new_response_callback(poke);
      clients_[cli.get()] = cli;
    },
    // A steady or system timer has no readiness event: next_ros_timeout() serves it.
    [&](const rclcpp::TimerBase::SharedPtr & timer) {
      seen_.insert(timer.get());
      if (hooked(timers_, timer)) return;
      timers_[timer.get()] = {timer, on_ros_time(timer, poke)};
    },
    // Action servers and clients are waitables, and so are flux sources put in the group.
    [&](const rclcpp::Waitable::SharedPtr & wt) {
      if (const auto member = std::dynamic_pointer_cast<detail::GroupMember>(wt)) {
        if (hooked(members_, member)) return;
        core_.add(member->source(), member->priority());
        member->served();
        members_[member.get()] = member;
        return;
      }
      seen_.insert(wt.get());
      if (hooked(waitables_, wt)) return;
      wt->set_on_ready_callback([poke](std::size_t n, int) { poke(n); });
      waitables_[wt.get()] = wt;
    });
}

// Entities whose node or group left this executor: their hooks would wake it for nothing.
void Executor::unhook_unseen()
{
  unhook_if(subs_, seen_, [](rclcpp::SubscriptionBase & s) {
    s.clear_on_new_message_callback();
    if (s.get_intra_process_waitable() != nullptr) s.clear_on_new_intra_process_message_callback();
  });
  unhook_if(services_, seen_, [](rclcpp::ServiceBase & s) { s.clear_on_new_request_callback(); });
  unhook_if(clients_, seen_, [](rclcpp::ClientBase & c) { c.clear_on_new_response_callback(); });
  unhook_if(waitables_, seen_, [](rclcpp::Waitable & w) { w.clear_on_ready_callback(); });
  unhook_if(timers_, seen_, [](rclcpp::TimerBase &) {});
}

std::int64_t Executor::next_ros_timeout(std::int64_t limit_ns)
{
  std::int64_t best = limit_ns;
  for (const auto & entry : timers_) {
    const auto timer = entry.second.lock();
    if (!timer || (entry.second.jump && clock_of(*timer)->ros_time_is_active())) continue;
    const std::int64_t due = timer->time_until_trigger().count();
    const std::int64_t wait = due > 0 ? due : 0;
    if (best < 0 || wait < best) best = wait;
  }
  return best;
}

}  // namespace flux
