#include "flux/ros/partitioned_executor.hpp"

#include <poll.h>

#include <chrono>
#include <stdexcept>
#include <string>

namespace flux::ros
{

PartitionedExecutor::PartitionedExecutor() = default;
PartitionedExecutor::~PartitionedExecutor() = default;

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

void PartitionedExecutor::schedule_impl(const rclcpp::CallbackGroup::SharedPtr & group, Sched sched)
{
  if (ctl_.is_spinning()) {
    throw std::logic_error("flux: schedule() must be called before spin()");
  }
  if (!group) {
    throw std::invalid_argument("flux: schedule(group): group is null");
  }
  sched.opts.validate();
  for (auto & [g, s] : scheduled_) {
    if (g == group) {
      throw std::invalid_argument("flux: this group already has a schedule");
    }
    // One stage names one thread. Handing the same stage to two groups would put two threads
    // under one declaration and leave the file describing neither of them.
    if (s.from_stage && sched.from_stage && s.node == sched.node && s.label == sched.label) {
      throw std::invalid_argument(
        "flux: stage " + sched.node + (sched.label.empty() ? "" : "/" + sched.label) +
        " is already scheduled on another callback group; one stage declares one thread");
    }
  }
  scheduled_.push_back({group, std::move(sched)});
}

void PartitionedExecutor::schedule(
  const rclcpp::CallbackGroup::SharedPtr & group, const rt::Options & sched, rt::Strictness strict,
  int control_priority)
{
  schedule_impl(group, Sched{sched, strict, control_priority, {}, {}, false});
}

void PartitionedExecutor::schedule(
  const rclcpp::CallbackGroup::SharedPtr & group, const RtStage & stage)
{
  if (stage.external) {
    throw std::invalid_argument(
      "flux: stage " + stage.node + (stage.group.empty() ? "" : "/" + stage.group) +
      " is declared external, so flux does not set it; do not hand it to schedule()");
  }
  schedule_impl(
    group, Sched{stage.opts, stage.strict, stage.control_priority, stage.node, stage.group, true});
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
// matched callback while holding its own std::mutex (approximate_time.h:212 -> :509), and that
// mutex has no priority inheritance, so inputs on two group threads couple those groups'
// priorities -- undoing the isolation this executor exists to give.
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
          "groups, so two threads would enter the sync policy's mutex; that mutex has no "
          "priority inheritance and would couple the two groups' priorities. Put every input of "
          "one synchronizer in the same group");
      }
    }
  }
  unplaced_sync_.store(unplaced, std::memory_order_relaxed);
}

// A stage names its thread by (node, label) and an rclcpp callback group has no name, so the only
// thing tying the file's name to the running thread is the pairing at the schedule() call. Check
// what can be checked: the group belongs to the node the stage names, and an empty label means
// that node's default group.
void PartitionedExecutor::check_schedule_labels(const GroupMap & known) const
{
  for (const auto & [group, sched] : scheduled_) {
    if (!sched.from_stage) continue;
    auto it = known.find(group.get());
    if (it == known.end()) continue;  // reported by the served-here check below, with its reason
    const auto & base = it->second.node_base;
    const std::string actual = base->get_fully_qualified_name();
    if (actual != sched.node) {
      throw std::invalid_argument(
        "flux: schedule(group, stage): stage " + sched.node +
        (sched.label.empty() ? "" : "/" + sched.label) + " names node " + sched.node +
        " but the group belongs to " + actual);
    }
    const bool is_default = base->get_default_callback_group() == group;
    if (sched.label.empty() && !is_default) {
      throw std::invalid_argument(
        "flux: schedule(group, stage): stage " + sched.node +
        " names no group, which the file means as the node's default callback group, but the "
        "group handed over is not that one");
    }
    if (!sched.label.empty() && is_default) {
      throw std::invalid_argument(
        "flux: schedule(group, stage): stage " + sched.node + "/" + sched.label +
        " names a group, but the group handed over is the node's default one, which the file "
        "spells as an omitted label");
    }
  }
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
        if (g) known.emplace(g.get(), GroupNode{g, base});
      });
      ++it;
    } else {
      it = nodes_.erase(it);
    }
  }

  // Before any thread exists: a refusal that arrives after children are running has already let
  // the configuration it refuses run callbacks.
  check_sync_groups(known);
  check_schedule_labels(known);

  auto spawn = [this, tick_ns](
                 const rclcpp::CallbackGroup::SharedPtr & group,
                 const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
                 const std::vector<Assigned> & subs) {
    if (claimed_.count(group.get()) != 0) return;
    if (group->get_associated_with_executor_atomic().load()) {
      throw std::runtime_error(
        "flux: a callback group is already associated with another executor; every group of a "
        "node handed to add_ros_node() must be served here");
    }
    // One thread per group is what makes the group a schedulable unit, and it is also what a
    // Reentrant group asks not to have: its callbacks may run concurrently, and here they cannot.
    // Serializing them silently is the downgrade this executor exists to refuse -- and a service
    // callback that waits on a sibling in the same group deadlocks on one thread rather than
    // running slower. Split the work into groups instead; each gets its own thread and priority.
    if (group->type() == rclcpp::CallbackGroupType::Reentrant) {
      throw std::invalid_argument(
        "flux: a Reentrant callback group cannot be served here -- PartitionedExecutor gives "
        "each group exactly one thread, so its callbacks would serialize; split them into "
        "separate MutuallyExclusive groups");
    }
    Sched sched;
    for (auto & [g, s] : scheduled_) {
      if (g == group) {
        sched = s;
        break;
      }
    }
    Child c;
    c.group = group;
    if (subs.empty()) {
      c.ros_ex = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      c.ros_ex->add_callback_group(group, node_base);
      auto ex = c.ros_ex;
      // Not ex->spin(): cancel() is not sticky, so a cancel racing the thread's startup would be
      // lost and the join would hang. A tick-bounded spin_once loop re-checks the run flag the
      // same way the flux children do, which makes shutdown finite without the race.
      const std::int64_t ros_tick = tick_ns < 0 ? 100'000'000 : tick_ns;
      c.thread = std::thread([this, ex, ros_tick, sched] {
        try {
          rt::apply_checked(sched.opts, sched.strict, sched.control_priority);
          while (child_run_.load(std::memory_order_relaxed)) {
            ex->spin_once(std::chrono::nanoseconds(ros_tick));
          }
        } catch (...) {
          {
            std::lock_guard<std::mutex> lock(error_mutex_);
            if (!child_error_) child_error_ = std::current_exception();
          }
          failed_.store(true);
          interrupt();
        }
      });
    } else {
      c.flux_ex = std::make_unique<Executor>(static_cast<unsigned>(subs.size()));
      for (const Assigned & a : subs) c.flux_ex->add(*a.src, a.priority);
      c.flux_ex->add_ros_callback_group(group, node_base);
      Executor * ex = c.flux_ex.get();
      c.thread = std::thread([this, ex, tick_ns, sched] {
        try {
          rt::apply_checked(sched.opts, sched.strict, sched.control_priority);
          ex->spin(child_run_, tick_ns);
        } catch (...) {
          {
            std::lock_guard<std::mutex> lock(error_mutex_);
            if (!child_error_) child_error_ = std::current_exception();
          }
          failed_.store(true);
          interrupt();
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

  // A schedule on a group no child serves would never apply -- the silent-downgrade this
  // interface bans. Reject it instead.
  for (auto & [group, sched] : scheduled_) {
    if (claimed_.count(group.get()) == 0) {
      throw std::invalid_argument(
        "flux: schedule(group): the group is not served by this executor; hand its node to "
        "add_ros_node() (a manual group also needs a flux assignment via add())");
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

  const int tick_ms = tick_ns < 0 ? -1 : static_cast<int>(tick_ns / 1'000'000);
  std::exception_ptr scan_error;
  try {
    while (run.load(std::memory_order_relaxed) && !failed_.load(std::memory_order_relaxed) &&
           !ctl_.stop_requested()) {
      spawn_children(tick_ns);
      struct pollfd p;
      p.fd = ctl_.fd();
      p.events = POLLIN;
      p.revents = 0;
      ::poll(&p, 1, tick_ms);
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
