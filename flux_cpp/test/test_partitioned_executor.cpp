#include "flux/ros/executor.hpp"
#include "flux/ros/partitioned_executor.hpp"
#include "flux/ros/publisher.hpp"
#include "flux/rt.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>

#include <gtest/gtest.h>
#include <sched.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{

constexpr std::uint64_t kFingerprint = 0xE9EC0Full;
constexpr std::uint32_t kSlotSize = 64 * 1024;
constexpr std::uint32_t kSlots = 8;

}  // namespace

// The point of the whole class: a callback blocking in one group must not
// delay delivery in another group. On the single Executor the fast channel would starve behind
// the blocked callback; here each group has its own thread.
TEST(PartitionedExecutor, BlockedGroupDoesNotStallOthers)
{
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("cie_iso_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("cie_iso_sub");

  auto g_slow = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto g_fast = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  flux::ros::Publisher slow_pub(*pub_node, "/part/slow", kFingerprint, kSlotSize, kSlots);
  flux::ros::Publisher fast_pub(*pub_node, "/part/fast", kFingerprint, kSlotSize, kSlots);

  std::atomic<int> slow_entered{0};
  std::atomic<int> fast_count{0};
  std::atomic<bool> release{false};

  flux::ros::Subscription slow_sub(
    *sub_node, "/part/slow", kFingerprint, [&](const flux::FrameView &) {
      slow_entered.fetch_add(1);
      while (!release.load()) {
        std::this_thread::sleep_for(1ms);
      }
    });
  flux::ros::Subscription fast_sub(
    *sub_node, "/part/fast", kFingerprint,
    [&](const flux::FrameView &) { fast_count.fetch_add(1); });

  flux::ros::PartitionedExecutor ex;
  ex.add(slow_sub, g_slow);
  ex.add(fast_sub, g_fast);
  ex.add_ros_node(sub_node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });

  std::vector<std::byte> buf(256);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (slow_entered.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    slow_pub.publish(buf.data(), buf.size());
    std::this_thread::sleep_for(2ms);
  }

  // g_slow's thread is now parked inside its callback; g_fast must keep delivering.
  deadline = std::chrono::steady_clock::now() + 5s;
  while (fast_count.load() < 20 && std::chrono::steady_clock::now() < deadline) {
    fast_pub.publish(buf.data(), buf.size());
    std::this_thread::sleep_for(2ms);
  }
  const int slow_blocked = slow_entered.load();
  const int fast_while_blocked = fast_count.load();

  release.store(true);
  run.store(false);
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  ASSERT_GT(slow_blocked, 0) << "the slow group never received its frame";
  EXPECT_GE(fast_while_blocked, 20) << "the fast group starved while the slow group was blocked";
}

// Two flux subscriptions in ONE group, so they share a thread and a dispatch pass. The priority
// argument decides which is visited first within it; without it the pass follows the order they
// were added. Priorities never cross groups -- those are separate threads, ordered by the thread
// schedule instead.
TEST(PartitionedExecutor, PriorityOrdersWithinAGroup)
{
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("cie_prio_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("cie_prio_sub");

  auto group = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  flux::ros::Publisher lo_pub(*pub_node, "/part/prio_lo", kFingerprint, kSlotSize, kSlots);
  flux::ros::Publisher hi_pub(*pub_node, "/part/prio_hi", kFingerprint, kSlotSize, kSlots);

  std::mutex m;
  std::vector<int> order;
  auto note = [&m, &order](int who) {
    std::lock_guard<std::mutex> lock(m);
    order.push_back(who);
  };

  flux::ros::Subscription lo_sub(
    *sub_node, "/part/prio_lo", kFingerprint, [&](const flux::FrameView &) { note(1); });
  flux::ros::Subscription hi_sub(
    *sub_node, "/part/prio_hi", kFingerprint, [&](const flux::FrameView &) { note(2); });

  flux::ros::PartitionedExecutor ex;
  ex.add(lo_sub, group);                   // added first, default priority
  ex.add(hi_sub, group, /*priority=*/10);  // added second, visited first
  ex.add_ros_node(sub_node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });

  // Both published before either is delivered, so one pass sees both ready and the order it
  // visits them in is the thing under test. Retried until a pass catches the pair together.
  std::vector<std::byte> buf(256);
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  bool paired = false;
  while (!paired && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(m);
      order.clear();
    }
    ASSERT_EQ(lo_pub.publish(buf.data(), buf.size()), flux::Published::Ok);
    ASSERT_EQ(hi_pub.publish(buf.data(), buf.size()), flux::Published::Ok);
    std::this_thread::sleep_for(50ms);
    std::lock_guard<std::mutex> lock(m);
    paired = order.size() == 2;
  }
  std::vector<int> seen;
  {
    std::lock_guard<std::mutex> lock(m);
    seen = order;
  }

  run.store(false);
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  ASSERT_TRUE(paired) << "no pass ever saw both frames ready together";
  EXPECT_EQ(seen, (std::vector<int>{2, 1}));
}

// A PartitionedExecutor with no flux subscriptions at all: every group is pure ROS and every child
// is a stock SingleThreadedExecutor. Delivery must work exactly as with rclcpp alone.
TEST(PartitionedExecutor, PureRosGroupsAreServed)
{
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("cie_ros_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("cie_ros_sub");

  auto ros_pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/part/notify", 10);
  std::atomic<int> ros_count{0};
  auto ros_sub = sub_node->create_subscription<std_msgs::msg::UInt64>(
    "/part/notify", 10, [&](const std_msgs::msg::UInt64 &) { ros_count.fetch_add(1); });

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(sub_node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  std_msgs::msg::UInt64 m;
  while (ros_count.load() < 5 && std::chrono::steady_clock::now() < deadline) {
    ros_pub->publish(m);
    std::this_thread::sleep_for(2ms);
  }

  run.store(false);
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_GE(ros_count.load(), 5)
    << "pure ROS group delivered nothing under the PartitionedExecutor";
}

// A callback group created after spin() started is picked up by the parent's
// tick re-scan and gets its own child within one tick.
TEST(PartitionedExecutor, GroupCreatedAfterSpinGetsAChild)
{
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("cie_late_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("cie_late_sub");

  auto ros_pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/part/late", 10);

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(sub_node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });
  std::this_thread::sleep_for(100ms);  // spin is up with only the default group

  auto g_late = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = g_late;
  std::atomic<int> late_count{0};
  auto late_sub = sub_node->create_subscription<std_msgs::msg::UInt64>(
    "/part/late", 10, [&](const std_msgs::msg::UInt64 &) { late_count.fetch_add(1); }, opts);

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  std_msgs::msg::UInt64 m;
  while (late_count.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    ros_pub->publish(m);
    std::this_thread::sleep_for(2ms);
  }

  run.store(false);
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_GT(late_count.load(), 0) << "the group created after spin never got a child executor";
}

// Every spin entry point inherited from rclcpp::Executor throws instead of
// running rclcpp's implementation, which would service ROS entities and silently skip flux
// channels.
// spin(), spin_once(timeout) and cancel() are implemented, so they are not here -- nor is
// spin_node_once, which rclcpp builds out of spin_once and which therefore now means "add the
// node, run one merged pass, remove it". What remains carries a duration budget over a wait set
// this executor does not have, and an approximation would silently skip flux channels.
TEST(PartitionedExecutor, TheInheritedSpinVariantsWithoutAFluxMeaningThrow)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_guard_node");

  flux::ros::Executor ex;
  EXPECT_THROW(ex.spin_some(), std::runtime_error);
  EXPECT_THROW(ex.spin_all(1ms), std::runtime_error);
  EXPECT_THROW(ex.spin_node_some(node), std::runtime_error);
  EXPECT_THROW(ex.spin_node_all(node, 1ms), std::runtime_error);

  std::promise<int> p;
  std::future<int> fut = p.get_future();
  EXPECT_THROW(ex.spin_until_future_complete(fut, 1ms), std::runtime_error);

  rclcpp::shutdown();
}

// A flux subscription assigned to a group whose node was never handed over: serving only the flux
// side would let the group's ROS callbacks run on another executor, breaking the group's mutual
// exclusion. spin() rejects it.
TEST(PartitionedExecutor, RejectsFluxGroupNotOnAnAddedNode)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_reject_node");

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flux::ros::Subscription sub(*node, "/part/reject", kFingerprint, [](const flux::FrameView &) {});

  flux::ros::PartitionedExecutor ex;
  ex.add(sub, g);

  std::atomic<bool> run{true};
  EXPECT_THROW(ex.spin(run), std::invalid_argument);

  rclcpp::shutdown();
}

// schedule(): the child thread serving the group applies the declared scheduling to itself
// before its first callback. Affinity needs no privilege, so it proves end to end that the
// options reach the child thread: the callback reads its own affinity back from the kernel.
TEST(PartitionedExecutor, ScheduleReachesTheChildThread)
{
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("cie_sched_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("cie_sched_sub");

  auto g = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto ros_pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/part/sched", 10);

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = g;
  std::atomic<bool> seen{false};
  std::vector<std::uint32_t> cpus_in_callback;
  auto ros_sub = sub_node->create_subscription<std_msgs::msg::UInt64>(
    "/part/sched", 10,
    [&](const std_msgs::msg::UInt64 &) {
      if (!seen.exchange(true)) cpus_in_callback = flux::rt::current().cpus;
    },
    opts);

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(sub_node);
  flux::rt::Options sched;
  sched.cpus = {0};
  ex.schedule(g, sched);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  std_msgs::msg::UInt64 m;
  while (!seen.load() && std::chrono::steady_clock::now() < deadline) {
    ros_pub->publish(m);
    std::this_thread::sleep_for(2ms);
  }

  run.store(false);
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  ASSERT_TRUE(seen.load()) << "the scheduled group never received a message";
  EXPECT_EQ(cpus_in_callback, (std::vector<std::uint32_t>{0}))
    << "the callback did not run on the pinned cpu set";
}

// An RT-policy schedule follows the rt::apply contract through the executor: where preflight
// says the policy is permitted the child thread must actually hold it, and where it is not the
// refusal must surface as the spin() error -- never as a thread silently left at the default.
TEST(PartitionedExecutor, ScheduleFifoMatchesPreflight)
{
  flux::rt::Options sched;
  sched.policy = flux::rt::Policy::Fifo;
  sched.priority = 1;
  const flux::rt::Report rep = flux::rt::preflight(sched);
  const flux::rt::Finding * perm = rep.find("policy-permission");
  ASSERT_NE(perm, nullptr);

  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("cie_fifo_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("cie_fifo_sub");

  auto g = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto ros_pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/part/fifo", 10);

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = g;
  std::atomic<bool> seen{false};
  int policy_in_callback = -1;
  auto ros_sub = sub_node->create_subscription<std_msgs::msg::UInt64>(
    "/part/fifo", 10,
    [&](const std_msgs::msg::UInt64 &) {
      if (!seen.exchange(true)) policy_in_callback = flux::rt::current().policy;
    },
    opts);

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(sub_node);
  ex.schedule(g, sched);

  std::atomic<bool> run{true};
  if (perm->verdict != flux::rt::Verdict::Ok) {
    // std::runtime_error, not system_error: the child goes through rt::apply_checked, so the
    // refusal now comes from preflight before the thread is touched rather than from the
    // syscall afterwards. system_error derives from runtime_error, so this covers both.
    EXPECT_THROW(ex.spin(run, 20'000'000), std::runtime_error);
    rclcpp::shutdown();
    return;
  }

  std::thread spinner([&] { ex.spin(run, 20'000'000); });
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  std_msgs::msg::UInt64 m;
  while (!seen.load() && std::chrono::steady_clock::now() < deadline) {
    ros_pub->publish(m);
    std::this_thread::sleep_for(2ms);
  }
  run.store(false);
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  ASSERT_TRUE(seen.load()) << "the scheduled group never received a message";
  EXPECT_EQ(policy_in_callback, SCHED_FIFO) << "preflight said Ok but the child is not FIFO";
}

// The schedule surface rejects everything it cannot honor: a null group, options validate()
// refuses, a second schedule for the same group, a schedule for a group no child will serve,
// and any schedule() once spinning.
TEST(PartitionedExecutor, ScheduleRejects)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_sched_reject");

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);

  EXPECT_THROW(ex.schedule(nullptr, flux::rt::Options{}), std::invalid_argument);

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flux::rt::Options bad;
  bad.policy = flux::rt::Policy::Fifo;  // priority 0 with an RT policy: reinterpretable
  EXPECT_THROW(ex.schedule(g, bad), std::invalid_argument);

  ex.schedule(g, flux::rt::Options{});
  EXPECT_THROW(
    ex.schedule(g, flux::rt::Options{}), std::invalid_argument);  // one schedule per group

  // A group of a node never handed over: no child will serve it, so the schedule would
  // silently never apply. spin() refuses to start.
  auto stray_node = std::make_shared<rclcpp::Node>("cie_sched_stray");
  auto stray = stray_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  ex.schedule(stray, flux::rt::Options{});
  std::atomic<bool> run{true};
  EXPECT_THROW(ex.spin(run, 20'000'000), std::invalid_argument);

  flux::ros::PartitionedExecutor ex2;
  ex2.add_ros_node(node);
  std::thread spinner([&] { ex2.spin(run, 20'000'000); });
  std::this_thread::sleep_for(100ms);
  EXPECT_THROW(ex2.schedule(g, flux::rt::Options{}), std::logic_error);
  run.store(false);
  ex2.stop();
  spinner.join();
  rclcpp::shutdown();
}

// Registration is a pre-spin affair: the spawn scan reads the registration lists without a lock,
// so a concurrent add would race it. Reject instead of racing.
TEST(PartitionedExecutor, AddAfterSpinThrows)
{
  rclcpp::init(0, nullptr);
  auto sub_node = std::make_shared<rclcpp::Node>("cie_lockout_node");

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(sub_node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });
  std::this_thread::sleep_for(100ms);

  auto g = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flux::ros::Subscription sub(
    *sub_node, "/part/lockout", kFingerprint, [](const flux::FrameView &) {});
  EXPECT_THROW(ex.add(sub, g), std::logic_error);
  EXPECT_THROW(ex.add_ros_node(sub_node), std::logic_error);

  run.store(false);
  ex.stop();
  spinner.join();
  rclcpp::shutdown();
}

// One thread per group is what makes a group schedulable, and it is exactly what a Reentrant
// group asks not to have. Serving it anyway would serialize callbacks that declared they may
// overlap -- and deadlock a service callback waiting on a sibling. Refuse at spin instead.
TEST(PartitionedExecutor, ReentrantGroupIsRefusedNotSerialized)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_reentrant");

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  auto reentrant = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  EXPECT_NE(reentrant, nullptr);

  std::atomic<bool> run{true};
  EXPECT_THROW(ex.spin(run, 20'000'000), std::invalid_argument);

  rclcpp::shutdown();
}

// The child applies through rt::apply_checked, so a host that cannot hold a deadline is refused
// when the caller declared Hard rather than surfacing as a thread that quietly runs non-RT.
TEST(PartitionedExecutor, HardStrictnessSurfacesAsASpinError)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_strict");

  flux::rt::Options opts;
  opts.cpus = {0};
  const auto rep = flux::rt::preflight(opts);
  bool warned = false;
  for (const auto & f : rep.findings) {
    if (f.verdict == flux::rt::Verdict::Warn) warned = true;
  }
  if (!rep.ok() || !warned) {
    rclcpp::shutdown();
    GTEST_SKIP() << "flux-cap:rt-warn-verdict this host has no Warn-only verdict to strengthen";
  }

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  ex.schedule(g, opts, flux::rt::Strictness::Hard);

  std::atomic<bool> run{true};
  EXPECT_THROW(ex.spin(run, 20'000'000), std::runtime_error);

  rclcpp::shutdown();
}

// ---- the declaration file's name for a thread, checked against the group handed over ----
//
// A stage names its thread (node, label); an rclcpp callback group has no name at all. The only
// thing linking the two is the pairing at the schedule() call, and nothing looked at it, so a
// stage could be applied to a thread the file was not describing.

namespace
{

class SpecFile
{
public:
  explicit SpecFile(const std::string & body)
  : path_(
      "/tmp/flux_cie_spec_" + std::to_string(::getpid()) + "_" + std::to_string(++counter_) +
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

std::string spin_refusal(flux::ros::PartitionedExecutor & ex)
{
  std::atomic<bool> run{true};
  try {
    ex.spin(run, 20'000'000);
  } catch (const std::invalid_argument & e) {
    return e.what();
  } catch (...) {
    return "<wrong exception type>";
  }
  return "<no exception>";
}

// Affinity only: these cases are about which thread a stage names, and applying a real RT policy
// would need a privilege the check does not.
constexpr const char * kLabelSpec = R"(
chains:
  c:
    target: soft
    stages:
      - node: /cie_label
        group: infer
        cpus: [0]
      - node: /cie_label_other
        cpus: [0]
)";

}  // namespace

TEST(ScheduleLabel, RejectsAStageWhoseNodeIsNotTheGroupsNode)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_label");
  SpecFile f(kLabelSpec);
  const auto spec = flux::ros::RtSpec::load(f.path());

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.schedule(g, *spec.find("/cie_label_other"));  // a stage about a different node

  const std::string why = spin_refusal(ex);
  rclcpp::shutdown();

  EXPECT_NE(why.find("but the group belongs to"), std::string::npos) << why;
}

TEST(ScheduleLabel, RejectsALabelledStageOnTheDefaultGroup)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_label");
  SpecFile f(kLabelSpec);
  const auto spec = flux::ros::RtSpec::load(f.path());

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.schedule(
    node->get_node_base_interface()->get_default_callback_group(),
    *spec.find("/cie_label", "infer"));

  const std::string why = spin_refusal(ex);
  rclcpp::shutdown();

  EXPECT_NE(why.find("the node's default one"), std::string::npos) << why;
}

TEST(ScheduleLabel, RejectsAnUnlabelledStageOnANamedGroup)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_label_other");
  SpecFile f(kLabelSpec);
  const auto spec = flux::ros::RtSpec::load(f.path());

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.schedule(g, *spec.find("/cie_label_other"));  // the file omits the label: default group

  const std::string why = spin_refusal(ex);
  rclcpp::shutdown();

  EXPECT_NE(why.find("names no group"), std::string::npos) << why;
}

TEST(ScheduleLabel, AcceptsAStageOnTheGroupItNames)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_label");
  SpecFile f(kLabelSpec);
  const auto spec = flux::ros::RtSpec::load(f.path());

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.schedule(g, *spec.find("/cie_label", "infer"));

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });
  std::this_thread::sleep_for(100ms);
  ex.stop();
  spinner.join();
  rclcpp::shutdown();
}

// One stage is one thread. Handing it to two groups would leave the file describing neither.
TEST(ScheduleLabel, RejectsOneStageOnTwoGroups)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("cie_label");
  SpecFile f(kLabelSpec);
  const auto spec = flux::ros::RtSpec::load(f.path());

  auto ga = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto gb = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  const auto & stage = *spec.find("/cie_label", "infer");
  ex.schedule(ga, stage);

  std::string why = "<no exception>";
  try {
    ex.schedule(gb, stage);
  } catch (const std::invalid_argument & e) {
    why = e.what();
  }
  rclcpp::shutdown();

  EXPECT_NE(why.find("one stage declares one thread"), std::string::npos) << why;
}
