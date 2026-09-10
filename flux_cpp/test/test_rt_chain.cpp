#include "flux/ros/partitioned_executor.hpp"
#include "flux/ros/rt_spec.hpp"
#include "flux/rt.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>

#include <gtest/gtest.h>
#include <sched.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// The chain declaration file reaching real threads. test_rt_spec.cpp stops at
// the parse: it proves the file says 10/20/30, not that any thread ends up running at those
// values. That gap is the whole point of the file -- a declaration nothing enforces is a comment
// -- and it is where `chrt`, systemd or a later apply by user code would silently win. So the
// assertions here read the policy back from inside the callback, on the thread that runs it.

namespace
{

// A file per test, named by pid so concurrent runs do not share one.
class SpecFile
{
public:
  explicit SpecFile(const std::string & body)
  : path_(
      "/tmp/flux_rt_chain_" + std::to_string(::getpid()) + "_" + std::to_string(++counter_) +
      ".yaml")
  {
    std::ofstream out(path_);
    out << body;
  }
  ~SpecFile() { std::remove(path_.c_str()); }
  const std::string & path() const { return path_; }

private:
  static int counter_;
  std::string path_;
};
int SpecFile::counter_ = 0;

// soft, so a development host's warnings do not block the apply -- what is under test here is
// that the declared values arrive, not whether this host is prepared for hard RT. No cpus: a
// fixed core id names a different core on every machine, and pinning is already covered by
// PartitionedExecutor.ScheduleReachesTheChildThread.
constexpr const char * kChain = R"(
chains:
  perception_to_control:
    target: soft
    stages:
      - node: /rtchain_camera
        group: capture
        policy: fifo
        priority: 10
      - node: /rtchain_perception
        group: infer
        policy: fifo
        priority: 20
      - node: /rtchain_control
        group: loop
        policy: fifo
        priority: 30
)";

struct Stage
{
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub;
  std::string label;
  std::atomic<bool> seen{false};
  flux::rt::ThreadState observed;
};

}  // namespace

// One publish wakes all three stages; each records what the kernel says about the thread its
// callback is running on. Holds on both sides of the permission branch, like every other RT test
// -- Rt.PrivilegedPathIsExercisedWhereDeclared is what refuses a run that only ever took the
// refusal side.
TEST(RtChain, DeclaredStagesReachTheirThreads)
{
  SpecFile file(kChain);
  const auto spec = flux::ros::RtSpec::load(file.path());
  ASSERT_EQ(spec.stages().size(), 3u);

  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("rtchain_pub");
  auto pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/rtchain/tick", 10);

  Stage stages[3];
  stages[0].node = std::make_shared<rclcpp::Node>("rtchain_camera");
  stages[0].label = "capture";
  stages[1].node = std::make_shared<rclcpp::Node>("rtchain_perception");
  stages[1].label = "infer";
  stages[2].node = std::make_shared<rclcpp::Node>("rtchain_control");
  stages[2].label = "loop";

  flux::ros::PartitionedExecutor ex;
  for (Stage & s : stages) {
    s.group = s.node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = s.group;
    s.sub = s.node->create_subscription<std_msgs::msg::UInt64>(
      "/rtchain/tick", 10,
      [&s](const std_msgs::msg::UInt64 &) {
        if (!s.seen.exchange(true)) s.observed = flux::rt::current();
      },
      opts);
    ex.add_ros_node(s.node);
    // The call under test: strictness and control priority come from the chain, not from here.
    ex.schedule(s.group, spec.stage(*s.node, s.label));
  }

  const flux::rt::Report rep = flux::rt::preflight(spec.stages().front().opts);
  const flux::rt::Finding * perm = rep.find("policy-permission");
  ASSERT_NE(perm, nullptr);

  std::atomic<bool> run{true};
  if (perm->verdict != flux::rt::Verdict::Ok) {
    // The declaration must fail loudly rather than leave the chain running at default priority.
    EXPECT_THROW(ex.spin(run, 20'000'000), std::runtime_error);
    rclcpp::shutdown();
    return;
  }

  std::thread spinner([&] { ex.spin(run, 20'000'000); });
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  std_msgs::msg::UInt64 m;
  while (std::chrono::steady_clock::now() < deadline) {
    if (stages[0].seen.load() && stages[1].seen.load() && stages[2].seen.load()) break;
    pub->publish(m);
    std::this_thread::sleep_for(2ms);
  }
  run.store(false);
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  const int declared[3] = {10, 20, 30};
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(stages[i].seen.load()) << "stage " << i << " never received a message";
    EXPECT_EQ(stages[i].observed.policy, SCHED_FIFO)
      << "stage " << i << " declared fifo but its thread runs policy " << stages[i].observed.policy;
    EXPECT_EQ(stages[i].observed.priority, declared[i])
      << "stage " << i << " declared priority " << declared[i] << " but its thread runs at "
      << stages[i].observed.priority;
  }
  // The rule the file exists to enforce, checked on the running threads rather than on the text
  // that asked for it: priority climbs along the flow, so no stage can preempt the one it feeds.
  EXPECT_LT(stages[0].observed.priority, stages[1].observed.priority);
  EXPECT_LT(stages[1].observed.priority, stages[2].observed.priority);
}

// A node's own worker thread is a chain stage too: no executor owns it, so it applies its own
// stage. The label is whatever the file calls it -- nothing requires a stage to name an rclcpp
// callback group, and a thread the node spawned is exactly the case that has none.
TEST(RtChain, AWorkerThreadAppliesItsOwnStage)
{
  SpecFile file(R"(
chains:
  perception_to_control:
    target: soft
    stages:
      - node: /rtchain_worker
        group: infer_worker
        policy: fifo
        priority: 20
      - node: /rtchain_ctl
        group: loop
        policy: fifo
        priority: 30
)");
  const auto spec = flux::ros::RtSpec::load(file.path());
  const flux::ros::RtStage * st = spec.find("/rtchain_worker", "infer_worker");
  ASSERT_NE(st, nullptr);
  // The two values a node cannot derive from its own line, carried by the stage rather than by
  // the call site -- which is why apply_checked takes the stage whole.
  EXPECT_EQ(st->strict, flux::rt::Strictness::Soft);
  EXPECT_EQ(st->control_priority, 30);

  const flux::rt::Report rep = flux::rt::preflight(st->opts);
  const flux::rt::Finding * perm = rep.find("policy-permission");
  ASSERT_NE(perm, nullptr);

  flux::rt::ThreadState observed;
  bool threw = false;
  std::thread worker([&] {
    try {
      flux::ros::apply_checked(*st);
      observed = flux::rt::current();
    } catch (const std::runtime_error &) {
      threw = true;
    }
  });
  worker.join();

  if (perm->verdict != flux::rt::Verdict::Ok) {
    EXPECT_TRUE(threw) << "an unprivileged apply must refuse, not leave the thread at default";
    return;
  }
  EXPECT_FALSE(threw);
  EXPECT_EQ(observed.policy, SCHED_FIFO);
  EXPECT_EQ(observed.priority, 20) << "the worker does not run at the priority the file declared";
}

// The check that outlives the apply. apply_checked reads the thread back at the moment it sets
// it, which says nothing about the next second: `chrt`, systemd or a later apply by user code can
// move the thread, and the declaration would go on being a comment. verify() reads the thread
// again on demand, so the move is visible.
//
// Deliberately not an RT stage. Affinity needs no privilege, so this holds the same on a host
// where nothing may run SCHED_FIFO -- the drift and its detection are the same mechanism either
// way, and a test that only ran where RT is permitted would be one more vacuous green.
TEST(RtChain, VerifySeesADriftAwayFromTheDeclaration)
{
  const auto allowed = flux::rt::current().cpus;
  ASSERT_FALSE(allowed.empty());
  const std::uint32_t pinned = allowed.front();

  SpecFile file(
    "chains:\n  drift:\n    target: soft\n    stages:\n      - node: /rtchain_drift\n"
    "        group: worker\n        policy: other\n        cpus: [" +
    std::to_string(pinned) + "]\n");
  const auto spec = flux::ros::RtSpec::load(file.path());
  const flux::ros::RtStage * st = spec.find("/rtchain_drift", "worker");
  ASSERT_NE(st, nullptr);

  std::atomic<int> tid{0};
  std::atomic<bool> ready{false};
  std::atomic<bool> stop{false};
  std::thread worker([&] {
    flux::ros::apply_checked(*st);
    tid.store(flux::rt::this_tid());
    ready.store(true);
    while (!stop.load()) std::this_thread::sleep_for(1ms);
  });
  while (!ready.load()) std::this_thread::sleep_for(1ms);

  const flux::rt::Report before = flux::ros::verify(*st, tid.load());
  EXPECT_TRUE(before.ok()) << before.to_string();
  ASSERT_NE(before.find("observed-cpus"), nullptr);
  EXPECT_EQ(before.find("observed-cpus")->verdict, flux::rt::Verdict::Ok);

  // Somebody else moves the thread, which is the whole failure mode: no flux call is involved.
  // SCHED_BATCH because the move must be possible everywhere this test runs -- it needs neither
  // RT privilege nor a second cpu to drift to, so there is no host on which this reduces to a
  // skip.
  sched_param zero{};
  ASSERT_EQ(::sched_setscheduler(tid.load(), SCHED_BATCH, &zero), 0);

  const flux::rt::Report after = flux::ros::verify(*st, tid.load());
  stop.store(true);
  worker.join();

  EXPECT_FALSE(after.ok()) << "the thread left the declared policy and verify called it fine:\n"
                           << after.to_string();
  ASSERT_NE(after.find("observed-policy"), nullptr);
  EXPECT_EQ(after.find("observed-policy")->verdict, flux::rt::Verdict::Fail);
}

// The only thing flux ever does for a thread it does not own. The file records what the stage
// should already be running at; without this it records it to nobody.
TEST(RtChain, AnExternalStageIsCheckedNotAssumed)
{
  SpecFile file(R"(
chains:
  sensor_in:
    target: soft
    stages:
      - node: /rtchain_foreign
        group: listener
        external: true
        expect_priority: 60
)");
  const auto spec = flux::ros::RtSpec::load(file.path());
  const flux::ros::RtStage * st = spec.find("/rtchain_foreign", "listener");
  ASSERT_NE(st, nullptr);

  flux::rt::Options as_owner_would;
  as_owner_would.policy = flux::rt::Policy::Fifo;
  as_owner_would.priority = 60;
  const flux::rt::Report perm_rep = flux::rt::preflight(as_owner_would);
  const flux::rt::Finding * perm = perm_rep.find("policy-permission");
  ASSERT_NE(perm, nullptr);
  const bool privileged = perm->verdict == flux::rt::Verdict::Ok;

  std::atomic<int> tid{0};
  std::atomic<bool> ready{false};
  std::atomic<bool> stop{false};
  std::thread foreign([&] {
    // Stands in for the thread's real owner: an rmw listener, a driver, a CUDA worker. flux never
    // sets this one, which is what `external` declares.
    if (privileged) flux::rt::apply(as_owner_would);
    tid.store(flux::rt::this_tid());
    ready.store(true);
    while (!stop.load()) std::this_thread::sleep_for(1ms);
  });
  while (!ready.load()) std::this_thread::sleep_for(1ms);

  const flux::rt::Report seen = flux::ros::verify(*st, tid.load());
  ASSERT_NE(seen.find("observed-priority"), nullptr);

  if (!privileged) {
    // The declaration is simply unmet here, and saying so is the correct answer -- an unmet
    // expectation must not read the same as a met one just because flux could not have helped.
    EXPECT_FALSE(seen.ok()) << "an external stage nobody put on SCHED_FIFO passed:\n"
                            << seen.to_string();
    stop.store(true);
    foreign.join();
    return;
  }

  EXPECT_TRUE(seen.ok()) << seen.to_string();
  EXPECT_EQ(seen.find("observed-priority")->verdict, flux::rt::Verdict::Ok);

  sched_param wrong{};
  wrong.sched_priority = 50;
  ASSERT_EQ(::sched_setscheduler(tid.load(), SCHED_FIFO, &wrong), 0);
  const flux::rt::Report drifted = flux::ros::verify(*st, tid.load());
  stop.store(true);
  foreign.join();

  EXPECT_FALSE(drifted.ok());
  EXPECT_EQ(drifted.find("observed-priority")->verdict, flux::rt::Verdict::Fail);
}

// A stage whose thread is not there is the case a declaration exists to catch: it means nothing
// ever applied it, or what did apply it has exited. Reading that as a thread at rest would turn
// the loudest failure into the quietest pass.
TEST(RtChain, AStageWhoseThreadIsGoneIsAFailureNotASilence)
{
  SpecFile file(R"(
chains:
  gone:
    target: soft
    stages:
      - node: /rtchain_gone
        group: worker
        policy: fifo
        priority: 20
)");
  const auto spec = flux::ros::RtSpec::load(file.path());
  const flux::ros::RtStage * st = spec.find("/rtchain_gone", "worker");
  ASSERT_NE(st, nullptr);

  std::ifstream f("/proc/sys/kernel/pid_max");
  int pid_max = 0;
  ASSERT_TRUE(f >> pid_max);

  const flux::rt::Report rep = flux::ros::verify(*st, pid_max + 1);
  EXPECT_FALSE(rep.ok());
  ASSERT_NE(rep.find("observed-thread"), nullptr);
  EXPECT_EQ(rep.find("observed-thread")->verdict, flux::rt::Verdict::Fail);
}

// An external stage is the file saying "this thread is somebody else's". Applying or scheduling it
// would put that thread under flux after all, which is the one thing the marking rules out.
TEST(RtChain, AnExternalStageIsRefusedByBothEntryPoints)
{
  SpecFile file(R"(
chains:
  sensor_in:
    target: soft
    stages:
      - node: /rtchain_ext
        group: dds_listener
        external: true
        expect_priority: 60
      - node: /rtchain_ext
        group: work
        policy: fifo
        priority: 70
)");
  const auto spec = flux::ros::RtSpec::load(file.path());

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("rtchain_ext");
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  const flux::ros::RtStage & ext = spec.stage(*node, "dds_listener");
  EXPECT_THROW(ex.schedule(group, ext), std::invalid_argument);
  EXPECT_THROW(flux::ros::apply_checked(ext), std::invalid_argument);
  EXPECT_NO_THROW(ex.schedule(group, spec.stage(*node, "work")));
  rclcpp::shutdown();
}
