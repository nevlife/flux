#ifndef FLUX_EXECUTOR_HPP
#define FLUX_EXECUTOR_HPP

#include "flux/detail/executor.hpp"
#include "flux/subscription.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/waitable.hpp>

#include <rclcpp/version.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace flux
{
namespace detail
{
class GroupMember;
}

// One wait for flux subscriptions and the ROS entities of the nodes and groups added the rclcpp
// way, including the flux subscriptions put in those groups (add_to_callback_group). Ready ROS
// entities run on rclcpp's own path. spin() and spin_until_future_complete() run
// it; spin_once(), spin_some() and spin_all() throw.
class Executor : public rclcpp::Executor
{
public:
  Executor();
  ~Executor() override;
  Executor(const Executor &) = delete;
  auto operator=(const Executor &) -> Executor & = delete;
  Executor(Executor &&) = delete;
  auto operator=(Executor &&) -> Executor & = delete;

  // `sub` outlives this executor and is served by no other. Not while spinning.
  void add(Subscription & sub, std::int32_t priority = 0);
  // Any other source, such as a flux::message_filters::Subscriber.
  void add(detail::Source & src, std::int32_t priority = 0);

  void spin() override;
  void spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) override;
  void spin_all(std::chrono::nanoseconds max_duration) override;
  // Ends spin() from any thread; one sent before spin() starts ends that spin() at once.
  void cancel() override;

protected:
  void spin_once_impl(std::chrono::nanoseconds timeout) override;
  auto spin_until_future_complete_impl(
    std::chrono::nanoseconds timeout,
    const std::function<std::future_status(std::chrono::nanoseconds)> & wait_for_future)
    -> rclcpp::FutureReturnCode override;

private:
  static constexpr int kRosBudget = FLUX_MAX_DRAIN;
  // How long ROS work runs before waiting flux work gets its turn.
  static constexpr std::int64_t kTurnNs = FLUX_DEFAULT_TURN_NS;

  // Spins until cancel(), context shutdown, `done()`, or `timeout` (negative: none).
  auto run(std::chrono::nanoseconds timeout, const std::function<bool()> & done)
    -> rclcpp::FutureReturnCode;
  // Waits up to `wait_ns` (negative: no limit) and runs what is ready.
  void pass(std::int64_t wait_ns);
  auto pump_ros() -> bool;
  void rebridge_ros();
  void bridge_group(const rclcpp::CallbackGroup::SharedPtr & group);
  void forget_moved(const std::vector<rclcpp::CallbackGroup::WeakPtr> & now);
  void unhook_unseen();
  auto next_ros_timeout(std::int64_t limit_ns) -> std::int64_t;

  detail::Executor core_;
  detail::ExecutorHandle handle_;
  // Shared with hooks, which run on rmw threads and can outlive a call into this executor.
  std::shared_ptr<std::atomic<bool>> ros_ready_ = std::make_shared<std::atomic<bool>>(false);
  // A pump cut short keeps the next wait open: nothing pokes again for what rclcpp holds.
  bool ros_backlog_ = false;
  std::atomic<bool> stop_{false};
  rclcpp::OnShutdownCallbackHandle on_shutdown_;

  // Weak, so a hook never keeps an entity the user dropped alive.
  std::unordered_map<const void *, rclcpp::SubscriptionBase::WeakPtr> subs_;
  std::unordered_map<const void *, rclcpp::ServiceBase::WeakPtr> services_;
  std::unordered_map<const void *, rclcpp::ClientBase::WeakPtr> clients_;
  std::unordered_map<const void *, rclcpp::Waitable::WeakPtr> waitables_;
  struct Timer
  {
    rclcpp::TimerBase::WeakPtr timer;
    // Null unless the timer runs on ROS time.
    rclcpp::JumpHandler::SharedPtr jump;
    [[nodiscard]] auto lock() const -> rclcpp::TimerBase::SharedPtr { return timer.lock(); }
  };
  std::unordered_map<const void *, Timer> timers_;
  // Never taken out again, as core_ has no remove: such a source outlives this executor.
  std::unordered_map<const void *, std::weak_ptr<detail::GroupMember>> members_;
  std::unordered_set<const void *> seen_;
  // What the last rescan bridged.
  std::vector<rclcpp::CallbackGroup::WeakPtr> groups_;
};

}  // namespace flux

#endif  // FLUX_EXECUTOR_HPP
