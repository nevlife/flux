#include "flux/ros/partitioned_executor.hpp"

#include <poll.h>
#include <pthread.h>

#include <chrono>
#include <ctime>
#include <stdexcept>
#include <string>

namespace flux::ros
{

namespace
{
constexpr const char * kReentrantRefusal =
  "flux: a Reentrant callback group cannot be served here -- PartitionedExecutor gives each "
  "group exactly one thread, so its callbacks would serialize; split them into separate "
  "MutuallyExclusive groups";
}  // namespace

PartitionedExecutor::PartitionedExecutor()
{
  on_shutdown_ =
    rclcpp::contexts::get_global_default_context()->add_on_shutdown_callback([this] { stop(); });
}

PartitionedExecutor::~PartitionedExecutor()
{
  rclcpp::contexts::get_global_default_context()->remove_on_shutdown_callback(on_shutdown_);
}

void PartitionedExecutor::interrupt() noexcept
{
  ctl_.interrupt();
}

void PartitionedExecutor::add(
  Subscription & sub, const rclcpp::CallbackGroup::SharedPtr & group, int priority)
{
  // Checked here rather than where the child executor is built, so a subscription an executor
  // cannot run fails at the call that registered it.
  if (!sub.has_callback()) {
    throw std::invalid_argument(
      "flux: this Subscription has no callback, so an executor has nothing to run -- construct it "
      "with one, or read it yourself with take()/take_blocking()");
  }
  add(static_cast<flux::Source &>(sub), group, priority);
}

void PartitionedExecutor::add(
  flux::Source & src, const rclcpp::CallbackGroup::SharedPtr & group, int priority)
{
  if (ctl_.is_spinning()) {
    throw std::logic_error("flux: add() must be called before spin()");
  }
  if (!group) {
    throw std::invalid_argument("flux: add(src, group): group is null");
  }
  if (group->type() == rclcpp::CallbackGroupType::Reentrant) {
    throw std::invalid_argument(kReentrantRefusal);
  }
  for (auto & [g, sources] : assigned_) {
    for (const Assigned & a : sources) {
      if (a.src == &src) {
        throw std::invalid_argument("flux: this source is already assigned to a group");
      }
    }
  }
  for (auto & [g, sources] : assigned_) {
    if (g == group) {
      sources.push_back(Assigned{&src, priority});
      return;
    }
  }
  assigned_.push_back({group, {Assigned{&src, priority}}});
}

void PartitionedExecutor::declare_sync_group(std::vector<detail::SyncInput> inputs)
{
  if (ctl_.is_spinning()) {
    throw std::logic_error("flux: add_sync_group() must be called before spin()");
  }
  sync_groups_.push_back(std::move(inputs));
}

std::size_t PartitionedExecutor::unplaced_sync_inputs() const noexcept
{
  return unplaced_sync_.load(std::memory_order_relaxed);
}

void PartitionedExecutor::add_ros_node(const rclcpp::Node::SharedPtr & node)
{
  if (ctl_.is_spinning()) {
    throw std::logic_error("flux: add_ros_node() must be called before spin()");
  }
  // Not rclcpp::Executor::add_node(): the children claim the node's groups one by one, so the
  // node itself must stay unassociated.
  nodes_.push_back(node);
}

void PartitionedExecutor::on_thread_start(
  const rclcpp::CallbackGroup::SharedPtr & group, std::function<void()> init)
{
  if (ctl_.is_spinning()) {
    throw std::logic_error("flux: on_thread_start() must be called before spin()");
  }
  if (!group) {
    throw std::invalid_argument("flux: on_thread_start(group): group is null");
  }
  for (auto & [g, f] : thread_init_) {
    if (g == group) {
      throw std::invalid_argument("flux: this group already has an on_thread_start() hook");
    }
  }
  thread_init_.push_back({group, std::move(init)});
}

namespace
{

// rclcpp offers no subscription -> group lookup, so this is the walk bridge_group() does, read
// the other way round.
bool group_holds(
  const rclcpp::CallbackGroup & group, const rclcpp::SubscriptionBase::SharedPtr & sub)
{
  return group.find_subscription_ptrs_if(
           [&sub](const rclcpp::SubscriptionBase::SharedPtr & s) { return s == sub; }) != nullptr;
}

}  // namespace

// Every input of one synchronizer must be serviced by one thread. The sync policy runs the
// matched callback while holding its own std::mutex (approximate_time.h:212 -> :509), so inputs
// on two group threads make one group wait on the other -- undoing the isolation this executor
// exists to give.
void PartitionedExecutor::check_sync_groups(const GroupMap & known)
{
  std::size_t unplaced = 0;
  for (const auto & inputs : sync_groups_) {
    const rclcpp::CallbackGroup * first = nullptr;
    for (const detail::SyncInput & in : inputs) {
      const rclcpp::CallbackGroup * g = nullptr;
      if (in.src != nullptr) {
        for (const auto & [grp, sources] : assigned_) {
          for (const Assigned & a : sources) {
            if (a.src == in.src) g = grp.get();
          }
        }
        // A flux input nothing was told to service is not an unknown, it is a hole: no child
        // would ever drive it and the synchronizer would wait for a partner that never arrives.
        if (g == nullptr) {
          throw std::invalid_argument(
            "flux: add_sync_group(): a flux input of this synchronizer was never assigned to a "
            "callback group with add(), so no child would service it");
        }
      } else if (in.ros) {
        for (const auto & [ptr, gn] : known) {
          if (group_holds(*gn.group, in.ros)) g = ptr;
        }
      }
      if (g == nullptr) {
        ++unplaced;
        continue;
      }
      if (first == nullptr) {
        first = g;
      } else if (first != g) {
        throw std::invalid_argument(
          "flux: add_sync_group(): the inputs of one synchronizer are in different callback "
          "groups, so two threads would enter the sync policy's mutex and one group would wait "
          "on the other. Put every input of one synchronizer in the same group");
      }
    }
  }
  unplaced_sync_.store(unplaced, std::memory_order_relaxed);
}

void PartitionedExecutor::record_child_error() noexcept
{
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    if (!child_error_) child_error_ = std::current_exception();
  }
  failed_.store(true);
  interrupt();
}

void PartitionedExecutor::spawn_children(std::int64_t tick_ns)
{
  for (auto it = claimed_.begin(); it != claimed_.end();) {
    it = it->second.expired() ? claimed_.erase(it) : std::next(it);
  }

  GroupMap known;
  for (auto it = nodes_.begin(); it != nodes_.end();) {
    if (auto node = it->lock()) {
      auto base = node->get_node_base_interface();
      base->for_each_callback_group([&known, &base](const rclcpp::CallbackGroup::SharedPtr & g) {
        known.emplace(g.get(), GroupNode{g, base});
      });
      ++it;
    } else {
      it = nodes_.erase(it);
    }
  }

  // Before any thread exists: a refusal that arrives after children are running has already let
  // the configuration it refuses run callbacks.
  check_sync_groups(known);

  auto spawn = [this, tick_ns](
                 const rclcpp::CallbackGroup::SharedPtr & group,
                 const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
                 const std::vector<Assigned> & subs) {
    if (claimed_.count(group.get()) != 0) return;
    // Serializing a Reentrant group on its one thread is a silent downgrade, and a service
    // callback waiting on a sibling in the same group would deadlock rather than run slower.
    if (group->type() == rclcpp::CallbackGroupType::Reentrant) {
      throw std::invalid_argument(kReentrantRefusal);
    }
    std::function<void()> init;
    for (auto & [g, f] : thread_init_) {
      if (g == group) {
        init = f;
        break;
      }
    }
    Child c;
    c.group = group;
    // On the OS thread, where top -H, perf and gdb read it; the same scheme flux_py uses.
    const std::string name = "flux-part-g" + std::to_string(children_.size());
    if (subs.empty()) {
      c.ros_ex = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      c.ros_ex->add_callback_group(group, node_base);
      auto ex = c.ros_ex;
      // Not ex->spin(): cancel() is not sticky, so a cancel racing the thread's startup would be
      // lost and the join would hang. A tick-bounded spin_once loop re-checks the run flag the
      // same way the flux children do, which makes shutdown finite without the race.
      const std::int64_t ros_tick = tick_ns < 0 ? 100'000'000 : tick_ns;
      c.thread = std::thread([this, ex, ros_tick, init, name] {
        ::pthread_setname_np(::pthread_self(), name.c_str());
        try {
          if (init) init();
          while (child_run_.load(std::memory_order_relaxed)) {
            ex->spin_once(std::chrono::nanoseconds(ros_tick));
          }
        } catch (...) {
          record_child_error();
        }
      });
    } else {
      c.flux_ex = std::make_unique<Executor>();
      for (const Assigned & a : subs) c.flux_ex->add(*a.src, a.priority);
      c.flux_ex->add_ros_callback_group(group, node_base);
      Executor * ex = c.flux_ex.get();
      c.thread = std::thread([this, ex, tick_ns, init, name] {
        ::pthread_setname_np(::pthread_self(), name.c_str());
        try {
          if (init) init();
          ex->spin(child_run_, tick_ns);
        } catch (...) {
          record_child_error();
        }
      });
    }
    claimed_.emplace(group.get(), group);
    children_.push_back(std::move(c));
  };

  for (auto & [group, subs] : assigned_) {
    auto it = known.find(group.get());
    if (it == known.end()) {
      throw std::invalid_argument(
        "flux: add(sub, group): the group does not belong to any node handed to add_ros_node()");
    }
    spawn(group, it->second.node_base, subs);
  }

  // A group created without automatically_add_to_executor_with_node is rclcpp's "manual" kind:
  // it is serviced only where it is explicitly placed, and here that placement is add(sub, group).
  for (auto & [ptr, gn] : known) {
    if (!gn.group->automatically_add_to_executor_with_node()) continue;
    spawn(gn.group, gn.node_base, {});
  }

  for (auto & [group, init] : thread_init_) {
    if (claimed_.count(group.get()) == 0) {
      throw std::invalid_argument(
        "flux: on_thread_start(group): the group is not served by this executor; hand its node "
        "to add_ros_node() (a manual group also needs a flux assignment via add())");
    }
  }
}

void PartitionedExecutor::spin(std::atomic<bool> & run, std::int64_t tick_ns)
{
  SpinControl::Session session(
    ctl_);  // reentry guard; clears stop and drains the fd on the way out
  child_run_.store(true);
  failed_.store(false);
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    child_error_ = nullptr;
  }

  // ppoll, not poll: whole milliseconds would turn a sub-millisecond tick into a busy loop.
  struct timespec tick;
  tick.tv_sec = static_cast<time_t>(tick_ns / 1'000'000'000);
  tick.tv_nsec = static_cast<long>(tick_ns % 1'000'000'000);
  std::exception_ptr scan_error;
  try {
    while (run.load(std::memory_order_relaxed) && !failed_.load(std::memory_order_relaxed) &&
           !ctl_.stop_requested() && rclcpp::ok()) {
      spawn_children(tick_ns);
      struct pollfd p;
      p.fd = ctl_.fd();
      p.events = POLLIN;
      p.revents = 0;
      ::ppoll(&p, 1, tick_ns < 0 ? nullptr : &tick, nullptr);
      if (p.revents & POLLIN) ctl_.drain();
    }
  } catch (...) {
    scan_error = std::current_exception();
  }

  child_run_.store(false);
  for (auto & c : children_) {
    if (c.flux_ex) c.flux_ex->stop();
    if (c.ros_ex) c.ros_ex->cancel();
  }
  for (auto & c : children_) {
    if (c.thread.joinable()) c.thread.join();
  }
  children_.clear();  // child executor destructors release the group associations
  claimed_.clear();

  if (scan_error) std::rethrow_exception(scan_error);
  std::lock_guard<std::mutex> lock(error_mutex_);
  if (child_error_) {
    std::exception_ptr e = child_error_;
    child_error_ = nullptr;
    std::rethrow_exception(e);
  }
}

void PartitionedExecutor::spin(std::int64_t tick_ns)
{
  std::atomic<bool> always{true};
  spin(always, tick_ns);
}

void PartitionedExecutor::stop() noexcept
{
  ctl_.stop();
}

}  // namespace flux::ros
