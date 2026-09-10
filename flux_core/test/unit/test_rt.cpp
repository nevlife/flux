#include "flux/rt.hpp"

#include <gtest/gtest.h>
#include <sched.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <system_error>
#include <thread>

// Covers the RT opt-in surface: validate rejects nonsense, apply is
// all-or-nothing on the calling thread, and preflight's permission verdict agrees with
// what apply then does -- so the tests hold both with and without RT privileges.

namespace
{

// Tests mutate the calling thread's scheduling; put it back so later tests inherit a
// clean thread whichever assertions fired.
struct RestoreThread
{
  flux::rt::ThreadState saved = flux::rt::current();

  ~RestoreThread()
  {
    sched_param param{};
    param.sched_priority = saved.priority;
    ::sched_setscheduler(0, saved.policy, &param);
    cpu_set_t set;
    CPU_ZERO(&set);
    for (auto c : saved.cpus) CPU_SET(static_cast<int>(c), &set);
    ::sched_setaffinity(0, sizeof set, &set);
  }
};

}  // namespace

TEST(Rt, ValidateRejectsReinterpretableCombos)
{
  EXPECT_NO_THROW(flux::rt::Options{}.validate());

  flux::rt::Options fifo_no_prio;
  fifo_no_prio.policy = flux::rt::Policy::Fifo;
  EXPECT_THROW(fifo_no_prio.validate(), std::invalid_argument);

  flux::rt::Options prio_no_policy;
  prio_no_policy.priority = 10;
  EXPECT_THROW(prio_no_policy.validate(), std::invalid_argument);

  flux::rt::Options over;
  over.policy = flux::rt::Policy::RoundRobin;
  over.priority = 100;
  EXPECT_THROW(over.validate(), std::invalid_argument);

  flux::rt::Options bad_cpu;
  bad_cpu.cpus = {100000};
  EXPECT_THROW(bad_cpu.validate(), std::invalid_argument);
}

TEST(Rt, ApplyOfDefaultOptionsChangesNothing)
{
  const auto before = flux::rt::current();
  flux::rt::apply(flux::rt::Options{});
  const auto after = flux::rt::current();
  EXPECT_EQ(before.policy, after.policy);
  EXPECT_EQ(before.priority, after.priority);
  EXPECT_EQ(before.cpus, after.cpus);
}

TEST(Rt, ApplyPinsTheCallingThread)
{
  RestoreThread restore;
  const auto before = flux::rt::current();
  ASSERT_FALSE(before.cpus.empty());
  flux::rt::Options opts;
  opts.cpus = {before.cpus.front()};
  flux::rt::apply(opts);
  EXPECT_EQ(flux::rt::current().cpus, opts.cpus);
}

TEST(Rt, ApplyMatchesPreflightPermissionVerdict)
{
  RestoreThread restore;
  flux::rt::Options opts;
  opts.policy = flux::rt::Policy::Fifo;
  opts.priority = 10;
  const auto rep = flux::rt::preflight(opts);
  const auto * perm = rep.find("policy-permission");
  ASSERT_NE(perm, nullptr);
  if (perm->verdict == flux::rt::Verdict::Ok) {
    flux::rt::apply(opts);
    const auto state = flux::rt::current();
    EXPECT_EQ(state.policy, SCHED_FIFO);
    EXPECT_EQ(state.priority, 10);
  } else {
    EXPECT_THROW(flux::rt::apply(opts), std::system_error);
    EXPECT_NE(flux::rt::current().policy, SCHED_FIFO);
  }
}

TEST(Rt, RefusedApplyRollsBackAffinity)
{
  RestoreThread restore;
  const auto before = flux::rt::current();
  ASSERT_FALSE(before.cpus.empty());
  flux::rt::Options opts;
  opts.cpus = {before.cpus.front()};
  opts.policy = flux::rt::Policy::Fifo;
  opts.priority = 10;
  const auto rep = flux::rt::preflight(opts);
  const auto * perm = rep.find("policy-permission");
  ASSERT_NE(perm, nullptr);
  if (perm->verdict == flux::rt::Verdict::Ok) {
    flux::rt::apply(opts);
    EXPECT_EQ(flux::rt::current().policy, SCHED_FIFO);
    EXPECT_EQ(flux::rt::current().cpus, opts.cpus);
  } else {
    EXPECT_THROW(flux::rt::apply(opts), std::system_error);
    EXPECT_EQ(flux::rt::current().cpus, before.cpus);
  }
}

TEST(Rt, PreflightAlwaysReportsHostFindings)
{
  const auto rep = flux::rt::preflight(flux::rt::Options{});
  EXPECT_NE(rep.find("kernel-preemption"), nullptr);
  EXPECT_NE(rep.find("memory-lock"), nullptr);
  EXPECT_EQ(rep.find("policy-permission"), nullptr);
  EXPECT_EQ(rep.find("cpu-online"), nullptr);
  EXPECT_FALSE(rep.to_string().empty());
}

TEST(Rt, PreflightChecksRequestSpecificItems)
{
  flux::rt::Options opts;
  opts.policy = flux::rt::Policy::Fifo;
  opts.priority = 20;
  opts.cpus = {0};
  const auto rep = flux::rt::preflight(opts);
  for (const char * id :
       {"policy-permission", "rt-throttle", "cpu-online", "cpu-isolation", "cpu-governor"}) {
    EXPECT_NE(rep.find(id), nullptr) << id;
  }
  ASSERT_NE(rep.find("cpu-online"), nullptr);
  EXPECT_EQ(rep.find("cpu-online")->verdict, flux::rt::Verdict::Ok);
}

TEST(Rt, PreflightRejectsTransportAtOrAboveControl)
{
  flux::rt::Options opts;
  opts.policy = flux::rt::Policy::Fifo;
  opts.priority = 80;
  const auto above = flux::rt::preflight(opts, /*control_priority=*/50);
  const auto * order = above.find("priority-order");
  ASSERT_NE(order, nullptr);
  EXPECT_EQ(order->verdict, flux::rt::Verdict::Fail);
  EXPECT_FALSE(above.ok());

  opts.priority = 30;
  const auto below = flux::rt::preflight(opts, /*control_priority=*/50);
  ASSERT_NE(below.find("priority-order"), nullptr);
  EXPECT_EQ(below.find("priority-order")->verdict, flux::rt::Verdict::Ok);
}

TEST(Rt, PreflightFailsOfflineCpu)
{
  if (std::thread::hardware_concurrency() >= 1000) {
    GTEST_SKIP() << "flux-cap:offline-cpu no cpu id that is safely below the set size yet offline";
  }
  flux::rt::Options opts;
  opts.cpus = {1023};
  const auto rep = flux::rt::preflight(opts);
  const auto * online = rep.find("cpu-online");
  ASSERT_NE(online, nullptr);
  EXPECT_EQ(online->verdict, flux::rt::Verdict::Fail);
  EXPECT_FALSE(rep.ok());
}

TEST(Rt, UndeclaredControlPriorityIsReportedNotSkipped)
{
  // A check that did not run and a check that passed must not look alike. Omitting the finding
  // made an undeclared control loop indistinguishable from a verified ordering.
  flux::rt::Options opts;
  opts.policy = flux::rt::Policy::Fifo;
  opts.priority = 80;

  const auto rep = flux::rt::preflight(opts);  // control_priority left undeclared
  const auto * order = rep.find("priority-order");
  ASSERT_NE(order, nullptr) << "the ordering check vanished instead of reporting itself";
  EXPECT_EQ(order->verdict, flux::rt::Verdict::Unknown)
    << "Unknown is not a failure; it is an unanswered question. rep.ok() is not asserted here "
       "because policy-permission decides it on hosts without RT privileges";
}

TEST(Rt, ApplyCheckedIsANoOpForOptionsThatAskNothing)
{
  RestoreThread restore;
  const auto before = flux::rt::current();
  const auto rep = flux::rt::apply_checked(flux::rt::Options{}, flux::rt::Strictness::Hard);
  EXPECT_TRUE(rep.findings.empty()) << "an empty request has no host to judge";
  const auto after = flux::rt::current();
  EXPECT_EQ(after.policy, before.policy);
  EXPECT_EQ(after.priority, before.priority);
}

TEST(Rt, ApplyCheckedRefusesWhatPreflightFails)
{
  RestoreThread restore;
  if (std::thread::hardware_concurrency() >= 1000) {
    GTEST_SKIP() << "flux-cap:offline-cpu no cpu id that is safely below the set size yet offline";
  }
  flux::rt::Options opts;
  opts.cpus = {1023};  // offline: preflight Fails on cpu-online
  const auto before = flux::rt::current();

  EXPECT_THROW(flux::rt::apply_checked(opts, flux::rt::Strictness::Soft), std::runtime_error);

  const auto after = flux::rt::current();
  EXPECT_EQ(after.cpus, before.cpus) << "a refused apply_checked must not have touched the thread";
}

TEST(Rt, HardRefusesTheWarnsSoftTolerates)
{
  RestoreThread restore;
  // The host findings (kernel preemption, memory lock) are always evaluated, so a stock kernel
  // yields Warns. Strictness is the only thing that separates the two readings of them.
  flux::rt::Options opts;
  opts.cpus = {0};
  const auto rep = flux::rt::preflight(opts);
  if (!rep.ok())
    GTEST_SKIP()
      << "flux-cap:rt-warn-verdict this host Fails before Warn strictness can be observed";

  bool warned = false;
  for (const auto & f : rep.findings) {
    if (f.verdict == flux::rt::Verdict::Warn) warned = true;
  }
  if (!warned)
    GTEST_SKIP()
      << "flux-cap:rt-warn-verdict this host is fully configured for RT; no Warn to strengthen";

  EXPECT_NO_THROW(flux::rt::apply_checked(opts, flux::rt::Strictness::Soft));
  EXPECT_THROW(flux::rt::apply_checked(opts, flux::rt::Strictness::Hard), std::runtime_error);
}

TEST(Rt, PreflightJudgesAllFourIsolationAxes)
{
  // Isolation is four boot-time settings, and preflight used to look at two. A core that is in
  // isolcpus but still receives interrupts is not isolated for latency purposes, and reporting Ok
  // on the two it read let a half-configured host pass -- which is what `target: hard` is supposed
  // to refuse.
  flux::rt::Options opts;
  opts.cpus = {0};
  const auto rep = flux::rt::preflight(opts);

  for (const char * id : {"cpu-isolation", "rcu-offload", "irq-affinity"}) {
    const auto * f = rep.find(id);
    ASSERT_NE(f, nullptr) << id << " is not judged";
    EXPECT_NE(f->verdict, flux::rt::Verdict::Fail) << id << " must not be fatal on a stock host";
    EXPECT_FALSE(f->detail.empty()) << id << " says nothing about why";
  }
}

TEST(Rt, IsolationAxesAreNotJudgedWithoutARequestedCpu)
{
  // They answer "is the cpu you asked for prepared", so with no cpu requested there is nothing to
  // answer. Reporting Ok would claim a check that never had a subject.
  flux::rt::Options opts;
  opts.policy = flux::rt::Policy::Fifo;
  opts.priority = 10;
  const auto rep = flux::rt::preflight(opts);

  EXPECT_EQ(rep.find("rcu-offload"), nullptr);
  EXPECT_EQ(rep.find("irq-affinity"), nullptr);
  EXPECT_EQ(rep.find("cpu-isolation"), nullptr);
}

// Reading another thread is the half of the setting/reading asymmetry that makes a chain
// checkable end to end. The claim is that observe() answers about the thread it
// was handed and not about the caller, so the child reports itself through current() and the
// parent -- running at a different policy on purpose -- must arrive at the same answer.
TEST(Rt, ObserveAnswersAboutTheThreadItWasHanded)
{
  flux::rt::Options opts;
  opts.policy = flux::rt::Policy::Fifo;
  opts.priority = 12;
  const auto rep = flux::rt::preflight(opts);
  const auto * perm = rep.find("policy-permission");
  ASSERT_NE(perm, nullptr);

  std::atomic<int> child_tid{0};
  std::atomic<bool> settled{false};
  std::atomic<bool> stop{false};
  flux::rt::ThreadState self_view;
  std::thread child([&] {
    if (perm->verdict == flux::rt::Verdict::Ok) flux::rt::apply(opts);
    self_view = flux::rt::current();
    child_tid.store(flux::rt::this_tid());
    settled.store(true);
    while (!stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  while (!settled.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));

  const int tid = child_tid.load();
  EXPECT_NE(tid, flux::rt::this_tid()) << "this_tid returned the same id on two threads";
  const flux::rt::ThreadState outside_view = flux::rt::observe(tid);
  stop.store(true);
  child.join();

  EXPECT_EQ(outside_view.policy, self_view.policy);
  EXPECT_EQ(outside_view.priority, self_view.priority);
  EXPECT_EQ(outside_view.cpus, self_view.cpus);
  // The caller is deliberately not the observed thread; a read that ignored its argument would
  // have matched the parent instead, and on the privileged branch those differ.
  if (perm->verdict == flux::rt::Verdict::Ok) {
    EXPECT_EQ(outside_view.policy, SCHED_FIFO);
    EXPECT_EQ(outside_view.priority, 12);
    EXPECT_NE(outside_view.policy, flux::rt::current().policy);
  }
}

// A thread that is not there must not read as a thread at rest. SCHED_OTHER with priority 0 is
// what an unset thread looks like, so answering it for a tid the kernel does not know would turn
// "this stage never started" into "this stage is fine".
TEST(Rt, ObserveRefusesWhatItCannotRead)
{
  EXPECT_THROW(flux::rt::observe(0), std::invalid_argument);
  EXPECT_THROW(flux::rt::observe(-1), std::invalid_argument);

  std::ifstream f("/proc/sys/kernel/pid_max");
  int pid_max = 0;
  ASSERT_TRUE(f >> pid_max) << "cannot read pid_max";
  // Above the wrap point, so it names no thread now and cannot be allocated later either --
  // unlike the tid of a thread this test just joined, which the kernel is free to hand out again.
  EXPECT_THROW(flux::rt::observe(pid_max + 1), std::system_error);
}

// Refuses a vacuous pass. Every test above holds on both sides of the permission branch, so this
// suite is green on a host that never once applied SCHED_FIFO -- the default state of any machine
// whose rtprio ulimit nobody raised. A run that never reached the state under test proves
// nothing about it. Nothing readable from the host says whether it
// was MEANT to be RT-capable, so the environment declares it -- the same shape as the chain file's
// `target`, where declaring is what turns a report nobody reads into a refusal.
TEST(Rt, PrivilegedPathIsExercisedWhereDeclared)
{
  flux::rt::Options opts;
  opts.policy = flux::rt::Policy::Fifo;
  opts.priority = 10;
  const auto rep = flux::rt::preflight(opts);
  const auto * perm = rep.find("policy-permission");
  ASSERT_NE(perm, nullptr);
  const bool privileged = perm->verdict == flux::rt::Verdict::Ok;

  if (std::getenv("FLUX_RT_REQUIRE_PRIVILEGE") != nullptr) {
    EXPECT_TRUE(privileged)
      << "FLUX_RT_REQUIRE_PRIVILEGE declares this host RT-capable, but every RT test here could "
         "only watch apply() be refused:\n  "
      << perm->detail;
    return;
  }
  if (!privileged) {
    std::cout << "[  NOTE    ] RT tests took the refusal branch only: " << perm->detail << "\n"
              << "[  NOTE    ] set FLUX_RT_REQUIRE_PRIVILEGE=1 where SCHED_FIFO must work\n";
  }
}
