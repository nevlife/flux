#include "flux/ros/rt_spec.hpp"

#include <sched.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdlib>
#include <set>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace flux::ros
{

namespace
{

[[noreturn]] void fail(const std::string & why)
{
  throw std::runtime_error("flux: rt spec: " + why);
}

// Strict by construction: every key a node reads is removed from this set, and whatever is left
// was not understood. This is why the file carries no version field -- a version number announces
// a schema change but says nothing about `priorty`, and refusing what we did not read catches
// both.
void reject_unknown_keys(
  const YAML::Node & map, const std::set<std::string> & known, const std::string & where)
{
  for (const auto & kv : map) {
    const auto key = kv.first.as<std::string>();
    if (known.count(key) == 0) fail("unknown key '" + key + "' in " + where);
  }
}

rt::Policy parse_policy(const std::string & s, const std::string & where)
{
  if (s == "inherit") return rt::Policy::Inherit;
  if (s == "other") return rt::Policy::Other;
  if (s == "fifo") return rt::Policy::Fifo;
  if (s == "rr") return rt::Policy::RoundRobin;
  fail("unknown policy '" + s + "' in " + where + " (inherit / other / fifo / rr)");
}

rt::Strictness parse_target(const YAML::Node & chain, const std::string & where)
{
  if (!chain["target"]) fail("chain " + where + " has no target (hard / soft)");
  const auto t = chain["target"].as<std::string>();
  if (t == "hard") return rt::Strictness::Hard;
  if (t == "soft") return rt::Strictness::Soft;
  fail("unknown target '" + t + "' in chain " + where + " (hard / soft)");
}

// What a stage runs at, whichever side owns it. Non-RT stages report 0 and sit outside the
// ordering rule entirely -- a SCHED_OTHER thread does not take part in RT priority at all, so
// requiring it to keep climbing would reject a soft consumer at the end of a chain.
int effective_priority(const RtStage & s)
{
  return s.external ? s.expect_priority : s.opts.priority;
}

RtStage parse_stage(const YAML::Node & n, const std::string & chain)
{
  if (!n.IsMap()) fail("a stage in chain " + chain + " is not a mapping");
  reject_unknown_keys(
    n, {"node", "group", "external", "expect_priority", "policy", "priority", "cpus"},
    "a stage of chain " + chain);

  RtStage st;
  st.chain = chain;
  if (!n["node"]) fail("a stage in chain " + chain + " has no node");
  st.node = n["node"].as<std::string>();
  if (st.node.empty() || st.node.front() != '/') {
    fail("stage node '" + st.node + "' in chain " + chain + " must be a fully-qualified name");
  }
  if (n["group"]) st.group = n["group"].as<std::string>();
  const std::string where = st.node + (st.group.empty() ? "" : "/" + st.group);

  st.external = n["external"] && n["external"].as<bool>();
  if (st.external) {
    // An external stage is a record, not a request. Carrying thread options here would read as
    // something flux applies, and it never does.
    if (n["policy"] || n["priority"] || n["cpus"]) {
      fail(
        "external stage " + where +
        " must not carry policy/priority/cpus; flux does not set "
        "it. Use expect_priority to record what it should already be running at");
    }
    if (n["expect_priority"]) st.expect_priority = n["expect_priority"].as<int>();
    return st;
  }

  if (n["expect_priority"]) {
    fail(
      "stage " + where +
      " is not external, so expect_priority does not apply; flux sets this "
      "thread and reads back what it got");
  }
  if (n["policy"]) st.opts.policy = parse_policy(n["policy"].as<std::string>(), where);
  if (n["priority"]) st.opts.priority = n["priority"].as<int>();
  if (n["cpus"]) {
    for (const auto & c : n["cpus"]) st.opts.cpus.push_back(c.as<std::uint32_t>());
  }
  try {
    st.opts.validate();
  } catch (const std::exception & e) {
    fail("stage " + where + ": " + e.what());
  }
  return st;
}

// The ordering rule, and the reason the whole chain lives in one file. Upstream must never be
// able to preempt downstream, so RT priorities strictly increase along the flow. This is
// rt::preflight's `priority-order` check (transport below control) generalized past two stages.
void check_order(const std::vector<RtStage> & chain_stages, const std::string & chain)
{
  const RtStage * prev = nullptr;
  for (const auto & st : chain_stages) {
    if (effective_priority(st) <= 0) continue;  // not on the RT ladder
    if (prev != nullptr && effective_priority(st) <= effective_priority(*prev)) {
      fail(
        "chain " + chain + " does not climb: " + prev->node + " runs at " +
        std::to_string(effective_priority(*prev)) + " and the stage after it, " + st.node +
        ", runs at " + std::to_string(effective_priority(st)) +
        ". An upstream stage that can preempt a downstream one starves it");
    }
    prev = &st;
  }
}

bool is_rt(const RtStage & s)
{
  return !s.external &&
         (s.opts.policy == rt::Policy::Fifo || s.opts.policy == rt::Policy::RoundRobin);
}

bool shares_a_cpu(const RtStage & a, const RtStage & b)
{
  for (auto c : a.opts.cpus) {
    if (std::find(b.opts.cpus.begin(), b.opts.cpus.end(), c) != b.opts.cpus.end()) return true;
  }
  return false;
}

// Two RT threads on one core is not isolation, it is the opposite. SCHED_FIFO has no timeslice,
// so the one that gets there first holds the core until it blocks -- the groups were split to
// stop exactly that, and pinning them together undoes the split. It also breaks nohz_full, whose
// condition is at most one runnable task on the cpu; two threads that wake together bring the
// tick back.
//
// Refused only when a hard chain is involved. Under soft RT this is a legitimate arrangement --
// two RT threads sharing a core with different priorities is ordinary Linux -- and the file
// cannot tell whether the core is isolated at all. Strictness is what separates the readings,
// the same split rt::Strictness draws over preflight's Warn findings.
void check_cpu_exclusivity(const std::vector<RtStage> & stages)
{
  for (std::size_t i = 0; i < stages.size(); ++i) {
    for (std::size_t j = i + 1; j < stages.size(); ++j) {
      const RtStage & a = stages[i];
      const RtStage & b = stages[j];
      if (!is_rt(a) || !is_rt(b) || !shares_a_cpu(a, b)) continue;
      if (a.strict != rt::Strictness::Hard && b.strict != rt::Strictness::Hard) continue;
      fail(
        "stages " + a.node + (a.group.empty() ? "" : "/" + a.group) + " and " + b.node +
        (b.group.empty() ? "" : "/" + b.group) +
        " are both pinned to the same cpu with an RT policy. SCHED_FIFO has no timeslice, so one "
        "holds the core until it blocks and the other cannot run; a hard chain gets one RT thread "
        "per core. Give them separate cpus, or declare the chain soft");
    }
  }
}

}  // namespace

RtSpec RtSpec::load(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    fail("cannot read '" + path + "': " + e.what());
  }
  if (!root.IsMap()) fail("'" + path + "' is not a mapping");
  reject_unknown_keys(root, {"chains"}, "the file root");
  if (!root["chains"]) fail("'" + path + "' has no chains");

  RtSpec spec;
  for (const auto & kv : root["chains"]) {
    const auto chain = kv.first.as<std::string>();
    const YAML::Node & body = kv.second;
    if (!body.IsMap()) fail("chain " + chain + " is not a mapping");
    reject_unknown_keys(body, {"target", "stages"}, "chain " + chain);
    const rt::Strictness strict = parse_target(body, chain);
    if (!body["stages"] || !body["stages"].IsSequence()) {
      fail("chain " + chain + " has no stages sequence");
    }

    std::vector<RtStage> chain_stages;
    for (const auto & n : body["stages"]) chain_stages.push_back(parse_stage(n, chain));
    if (chain_stages.empty()) fail("chain " + chain + " has no stages");
    check_order(chain_stages, chain);

    // The control loop is the chain's last RT stage by definition of the flow order, so every
    // earlier stage is checked against it and it is checked against nothing. Deriving it here is
    // what the file buys: a node declaring itself in isolation cannot know this number.
    int control = 0;
    for (const auto & st : chain_stages) {
      if (effective_priority(st) > 0) control = effective_priority(st);
    }
    for (auto & st : chain_stages) {
      st.strict = strict;
      st.control_priority = (effective_priority(st) == control) ? 0 : control;
    }

    for (auto & st : chain_stages) {
      const RtStage * dup = spec.find(st.node, st.group);
      if (dup != nullptr) {
        // A stage legitimately sits on more than one chain (a camera feeding both control and
        // logging). Two chains asking it to run differently is the conflict, and only a file
        // holding both can see it.
        if (
          dup->external != st.external || dup->expect_priority != st.expect_priority ||
          dup->opts.policy != st.opts.policy || dup->opts.priority != st.opts.priority ||
          dup->opts.cpus != st.opts.cpus) {
          fail(
            "stage " + st.node + (st.group.empty() ? "" : "/" + st.group) +
            " is declared differently by chains " + dup->chain + " and " + st.chain);
        }
        continue;  // same declaration on both chains: keep the first
      }
      spec.stages_.push_back(std::move(st));
    }
  }
  // Across every chain, not per chain: two stages can land on one core from two declarations, and
  // that is precisely the collision a split file cannot see.
  check_cpu_exclusivity(spec.stages_);
  return spec;
}

RtSpec RtSpec::load()
{
  const char * path = std::getenv("FLUX_RT_SPEC");
  if (path == nullptr || *path == '\0') return RtSpec{};
  return load(path);
}

const RtStage * RtSpec::find(const std::string & node, const std::string & group) const noexcept
{
  for (const auto & st : stages_) {
    if (st.node == node && st.group == group) return &st;
  }
  return nullptr;
}

const RtStage & RtSpec::stage(const rclcpp::Node & node, const std::string & group) const
{
  const std::string name = node.get_fully_qualified_name();
  const RtStage * st = find(name, group);
  if (st == nullptr) {
    fail(
      "no stage '" + name + (group.empty() ? "" : "/" + group) +
      "' in the spec; a label the file never names is a typo, and applying nothing would hide it");
  }
  return *st;
}

std::vector<const RtStage *> RtSpec::external() const
{
  std::vector<const RtStage *> out;
  for (const auto & st : stages_) {
    if (st.external) out.push_back(&st);
  }
  return out;
}

rt::Report apply_checked(const RtStage & stage)
{
  if (stage.external) {
    throw std::invalid_argument(
      "flux: stage " + stage.node + (stage.group.empty() ? "" : "/" + stage.group) +
      " is declared external, so flux does not set it; do not hand it to apply_checked()");
  }
  return rt::apply_checked(stage.opts, stage.strict, stage.control_priority);
}

namespace
{

const char * sched_name(int policy)
{
  switch (policy) {
    case SCHED_OTHER:
      return "SCHED_OTHER";
    case SCHED_FIFO:
      return "SCHED_FIFO";
    case SCHED_RR:
      return "SCHED_RR";
    case SCHED_BATCH:
      return "SCHED_BATCH";
    case SCHED_IDLE:
      return "SCHED_IDLE";
    default:
      return "an unnamed policy";
  }
}

std::string cpu_list(const std::vector<std::uint32_t> & cpus)
{
  if (cpus.empty()) return "every cpu";
  std::string out;
  for (auto c : cpus) out += (out.empty() ? "" : ",") + std::to_string(c);
  return out;
}

void add(rt::Report & rep, const char * id, rt::Verdict v, std::string detail)
{
  rep.findings.push_back(rt::Finding{id, v, std::move(detail)});
}

}  // namespace

rt::Report verify(const RtStage & stage, int tid)
{
  const std::string where = stage.node + (stage.group.empty() ? "" : "/" + stage.group);
  rt::Report rep;

  rt::ThreadState st;
  try {
    st = rt::observe(tid);
  } catch (const std::exception & e) {
    // Not an omitted finding: a stage whose thread cannot be read is the case where a declaration
    // silently covers nothing, which is what declaring it was for.
    add(
      rep, "observed-thread", rt::Verdict::Fail, "stage " + where + " cannot be read: " + e.what());
    return rep;
  }

  // What the file claims depends on which side owns the thread. An external stage says only what
  // priority it should already be at, so a claim about policy has to be derived from that: a
  // nonzero RT priority is only reachable under an RT policy.
  const bool rt_expected = stage.external ? stage.expect_priority > 0
                                          : (stage.opts.policy == rt::Policy::Fifo ||
                                             stage.opts.policy == rt::Policy::RoundRobin);
  const int want_priority = stage.external ? stage.expect_priority : stage.opts.priority;
  const bool observed_rt = st.policy == SCHED_FIFO || st.policy == SCHED_RR;

  if (stage.external) {
    if (!rt_expected) {
      add(
        rep, "observed-policy", rt::Verdict::Unknown,
        "external stage " + where +
          " declares no expect_priority, so the file claims nothing "
          "about its policy; it runs " +
          sched_name(st.policy));
    } else if (!observed_rt) {
      add(
        rep, "observed-policy", rt::Verdict::Fail,
        "external stage " + where + " is expected at RT priority " + std::to_string(want_priority) +
          ", but its thread runs " + sched_name(st.policy));
    } else {
      add(rep, "observed-policy", rt::Verdict::Ok, where + " runs " + sched_name(st.policy));
    }
  } else if (stage.opts.policy == rt::Policy::Inherit) {
    add(
      rep, "observed-policy", rt::Verdict::Unknown,
      "stage " + where + " asks for no policy, so there is nothing to check; it runs " +
        sched_name(st.policy));
  } else {
    const int want = stage.opts.policy == rt::Policy::Fifo         ? SCHED_FIFO
                     : stage.opts.policy == rt::Policy::RoundRobin ? SCHED_RR
                                                                   : SCHED_OTHER;
    add(
      rep, "observed-policy", want == st.policy ? rt::Verdict::Ok : rt::Verdict::Fail,
      "stage " + where + " declares " + sched_name(want) + " and its thread runs " +
        sched_name(st.policy));
  }

  if (!rt_expected) {
    add(
      rep, "observed-priority", rt::Verdict::Unknown,
      "stage " + where + " declares no RT priority; its thread reports " +
        std::to_string(st.priority));
  } else {
    add(
      rep, "observed-priority", want_priority == st.priority ? rt::Verdict::Ok : rt::Verdict::Fail,
      "stage " + where + " declares priority " + std::to_string(want_priority) +
        " and its thread runs at " + std::to_string(st.priority));
  }

  // Affinity is checked only where flux set it. An external stage's cpus are not in the file at
  // all, and a stage that asked for no pin is free to run anywhere by its own declaration.
  if (stage.external || stage.opts.cpus.empty()) {
    add(
      rep, "observed-cpus", rt::Verdict::Unknown,
      "stage " + where + " pins no cpu; its thread may run on " + cpu_list(st.cpus));
  } else {
    add(
      rep, "observed-cpus", stage.opts.cpus == st.cpus ? rt::Verdict::Ok : rt::Verdict::Fail,
      "stage " + where + " declares cpus " + cpu_list(stage.opts.cpus) +
        " and its thread may run on " + cpu_list(st.cpus));
  }
  return rep;
}

}  // namespace flux::ros
