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
// `external` marks a thread flux cannot set: an rmw listener, a CUDA worker, a node handed to
// some other executor. Declared anyway, so a stage flux merely omits and a stage flux does not
// own do not look alike.
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

namespace detail
{
// How the file spells a stage: `node`, or `node/group` for a named callback group.
std::string stage_name(const std::string & node, const std::string & group);
// An external stage is a record of a thread flux does not set. `caller` names the refusing call.
void reject_external(const RtStage & stage, const char * caller);
}  // namespace detail

// A parsed declaration file. Every rule it enforces is checked once, here, against the whole
// file. That is why the declarations live in one file: per-node settings cannot check an
// ordering no single node can see.
class RtSpec
{
public:
  RtSpec() = default;

  // Parse `path`. Throws std::runtime_error on any violation (docs/en/api.en.md, RtSpec).
  static RtSpec load(const std::string & path);

  // Parse the path in FLUX_RT_SPEC. Unset leaves an empty spec, which is the no-op every node
  // that declares nothing already gets.
  static RtSpec load();

  bool empty() const noexcept { return stages_.empty(); }
  const std::vector<RtStage> & stages() const noexcept { return stages_; }

  // The stage declared for this node and group label. Throws if the file does not name it:
  // applying nothing would hide a typo.
  const RtStage & stage(const rclcpp::Node & node, const std::string & group = "") const;

  // Same lookup without the throw; nullptr when absent.
  const RtStage * find(const std::string & node, const std::string & group = "") const noexcept;

  // Stages flux does not set. A caller that can resolve them to threads (a launch tool, an
  // operator) checks them; flux only records what was declared.
  std::vector<const RtStage *> external() const;

private:
  std::vector<RtStage> stages_;
};

// Put a declared stage on the calling thread: the bare-thread counterpart of
// PartitionedExecutor::schedule, for a worker (an inference loop, a driver poll) no executor
// owns. Takes the stage whole for the reason schedule does, and rejects an external stage.
rt::Report apply_checked(const RtStage & stage);

// Check a thread against what the file declared for it, without setting anything: the only
// thing flux does for an `external` stage, and for the stages it sets the check that outlives
// the apply (`chrt`, systemd or a later apply can move a thread afterwards).
//
// `tid` is the thread to read (rt::this_tid on the thread itself, or a tid the caller resolved).
// The file names stages by node and label, and nothing in it maps a label to a kernel thread.
rt::Report verify(const RtStage & stage, int tid);

}  // namespace flux::ros

#endif  // FLUX_ROS_RT_SPEC_HPP
