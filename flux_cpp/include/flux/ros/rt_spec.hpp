#ifndef FLUX_ROS_RT_SPEC_HPP
#define FLUX_ROS_RT_SPEC_HPP

#include "flux/rt.hpp"

#include <rclcpp/node.hpp>

#include <string>
#include <vector>

// The chain declaration file: one place that says how every thread on a
// dataflow path must be scheduled, read by each node as it starts.
//
// Reading the file lives here rather than in flux_core, which builds standalone without ROS.
// Parsing a file is the ROS layer's work; putting a thread on a policy is the engine's.

namespace flux::ros
{

// One thread on the chain.
//
// `external` marks a thread flux cannot set -- an rmw listener, a CUDA worker, a node handed to
// some other executor. Those are declared anyway: a stage flux merely omits and a stage flux does
// not own must not look alike, and the second is a normal part of every chain that mixes flux
// with ROS transport.
struct RtStage
{
  std::string node;   // fully-qualified ROS node name
  std::string group;  // group label; empty means the node's default callback group
  bool external = false;
  int expect_priority = 0;  // external only: what the thread is expected to already run at

  rt::Options opts;                              // what flux applies (empty for external)
  rt::Strictness strict = rt::Strictness::Soft;  // from the chain's `target`
  int control_priority = 0;  // derived: the chain's last RT priority, 0 for the stage holding it

  std::string chain;  // which chain declared this, for error messages
};

// A parsed declaration file. Every rule it enforces is checked once, here, against the whole file
// -- which is the reason the declarations live in one file at all. Per-node settings cannot check
// an ordering no single node can see.
class RtSpec
{
public:
  RtSpec() = default;

  // Parse `path`. Throws std::runtime_error on any violation: an unknown key, a missing one, a
  // chain whose RT priorities do not strictly increase along the flow, or one stage declared
  // twice with different settings.
  static RtSpec load(const std::string & path);

  // Parse the path in FLUX_RT_SPEC. Unset leaves an empty spec, which is the no-op every node
  // that declares nothing already gets.
  static RtSpec load();

  bool empty() const noexcept { return stages_.empty(); }
  const std::vector<RtStage> & stages() const noexcept { return stages_; }

  // The stage declared for this node and group label. Throws if the file does not name it: a
  // label the file never heard of is a typo, and applying nothing would hide it.
  const RtStage & stage(const rclcpp::Node & node, const std::string & group = "") const;

  // Same lookup without the throw; nullptr when absent.
  const RtStage * find(const std::string & node, const std::string & group = "") const noexcept;

  // Stages flux does not set. A caller that can resolve them to threads (a launch tool, an
  // operator) checks them; flux only records what was declared.
  std::vector<const RtStage *> external() const;

private:
  std::vector<RtStage> stages_;
};

// Put a declared stage on the CALLING thread: the bare-thread counterpart of
// PartitionedExecutor::schedule. A node's own worker (an inference loop, a driver poll) is a
// chain stage like any other -- the file names it by a label the caller picks, exactly as it names
// a callback group -- but no executor owns that thread, so only the thread itself can apply.
//
// Takes the stage whole for the reason schedule does. `strict` and `control_priority` are the two
// values no single node can derive, and passing them field by field is where they get lost:
// control_priority defaults to 0, which turns the priority-order check into Unknown without
// saying so. Rejects an external stage, like schedule.
rt::Report apply_checked(const RtStage & stage);

// Check a thread against what the file declared for it, without setting anything. This is the
// only thing flux ever does for an `external` stage, and for the stages it does set it is the
// check that outlives the apply: `chrt`, systemd or a later apply by user code can move a thread
// afterwards, and apply-time readback cannot see that.
//
// `tid` is the thread to read (rt::this_tid on the thread itself, or a tid the caller resolved),
// not the stage's owner -- the file names stages by node and label, and nothing in it maps a
// label to a kernel thread. Findings: `observed-policy`, `observed-priority`, `observed-cpus`.
// A thread that cannot be read is a Fail, not an absent finding.
rt::Report verify(const RtStage & stage, int tid);

}  // namespace flux::ros

#endif  // FLUX_ROS_RT_SPEC_HPP
