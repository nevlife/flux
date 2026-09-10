#include "flux/io_uring_waiter.hpp"
#include "flux/ros/executor.hpp"
#include "flux/ros/publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{

// Only a missing opcode is a legitimate skip. A ring that could not be created means this host
// refuses io_uring on a kernel that has it -- a container seccomp profile, an fd or memlock
// limit -- and skipping there reports the wait layer as untestable while blaming a kernel that
// is fine. The engine refuses to fold those two together and neither does this. A macro,
// not a helper, so the skip and the failure return from the test body itself.
#define FLUX_REQUIRE_IO_URING()                                                          \
  switch (flux::IoUringWaiter::support()) {                                              \
    case flux::IoUringSupport::Yes:                                                      \
      break;                                                                             \
    case flux::IoUringSupport::NoOpcode:                                                 \
      GTEST_SKIP() << "flux-cap:io-uring kernel lacks io_uring FUTEX_WAIT (needs 6.7+)"; \
    case flux::IoUringSupport::RingFailed:                                               \
      FAIL() << "io_uring is denied on this host, not missing from this kernel: the "    \
                "ring could not be created (container seccomp profile, RLIMIT_NOFILE, "  \
                "RLIMIT_MEMLOCK). The merged wait layer went unchecked.";                \
  }

constexpr std::uint64_t kFingerprint = 0xE9EC0Full;
constexpr std::uint32_t kSlotSize = 64 * 1024;
constexpr std::uint32_t kSlots = 8;

// The frame's tag is its payload: the descriptor is derived from the byte count, so there is no
// field left to smuggle a sequence number through. A torn frame still shows as a byte that
// disagrees with the rest of the run.
std::uint8_t frame_tag(const flux::FrameView & v) noexcept
{
  return v.size() == 0 ? 0u : *static_cast<const std::uint8_t *>(v.data());
}

}  // namespace

// The flux Executor is THE executor here (no rclcpp spin): a single io_uring_enter blocks
// on both the flux channel's futex and the ROS subscription's readiness eventfd. A plain
// publisher thread emits on both topics; the executor thread receives both. This exercises
// the merged wait end to end.
TEST(FluxExecutor, MergesFluxAndRosInOneRing)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);

  auto pub_node = std::make_shared<rclcpp::Node>("flux_exec_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("flux_exec_sub");

  auto ros_pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/exec/notify", 10);
  flux::ros::Publisher flux_pub(*pub_node, "/exec/bulk", kFingerprint, kSlotSize, kSlots);

  std::atomic<int> flux_count{0};
  std::atomic<int> ros_count{0};
  std::atomic<int> torn{0};

  // No poll period: nothing drives this subscription until the executor does.
  flux::ros::Subscription flux_sub(
    *sub_node, "/exec/bulk", kFingerprint, [&](const flux::FrameView & v) {
      const auto * p = static_cast<const std::uint8_t *>(v.data());
      const auto tag = frame_tag(v);
      for (std::size_t i = 0; i < v.size(); ++i) {
        if (p[i] != tag) {
          torn.fetch_add(1);
          break;
        }
      }
      flux_count.fetch_add(1);
    });

  // The ROS subscription keeps its own create-time callback: the executor takes the message and
  // hands it back, exactly as an rclcpp executor would.
  auto ros_sub = sub_node->create_subscription<std_msgs::msg::UInt64>(
    "/exec/notify", 10, [&](const std_msgs::msg::UInt64 &) { ros_count.fetch_add(1); });

  flux::ros::Executor exec;
  exec.add(flux_sub);
  exec.add_ros_node(sub_node);

  std::atomic<bool> run{true};
  std::thread pub([&] {
    std::uint64_t seq = 0;
    std::vector<std::byte> buf(kSlotSize);
    while (run.load() && seq < 500) {
      ++seq;
      std::memset(buf.data(), static_cast<int>(seq & 0xFF), buf.size());
      flux_pub.publish(buf.data(), buf.size());
      std_msgs::msg::UInt64 m;
      m.data = seq;
      ros_pub->publish(m);
      std::this_thread::sleep_for(2ms);
    }
  });
  std::thread spinner([&] { exec.spin(run, 50'000'000); });

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline &&
         (ros_count.load() < 25 || flux_count.load() < 25)) {
    std::this_thread::sleep_for(10ms);
  }

  run.store(false);
  exec.stop();  // break the executor out of its blocking wait
  pub.join();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_EQ(torn.load(), 0) << "flux delivered a torn frame";
  EXPECT_GT(flux_count.load(), 0) << "flux channel delivered nothing through the executor";
  EXPECT_GT(ros_count.load(), 0) << "ROS subscription delivered nothing through the executor";
}

// A publisher restart moves the subscription to a new segment, killing everything the executor
// armed on the old one. Before that was handled the channel went silent
// for good here.
TEST(FluxExecutor, KeepsDeliveringAcrossPublisherRestart)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);

  auto pub_node = std::make_shared<rclcpp::Node>("flux_restart_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("flux_restart_sub");

  std::atomic<int> got{0};
  flux::ros::Subscription sub(
    *sub_node, "/exec/restart", kFingerprint, [&](const flux::FrameView &) { got.fetch_add(1); });

  flux::ros::Executor exec;
  exec.add(sub);

  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 10'000'000); });  // 10 ms tick

  std::vector<std::byte> buf(256);
  int before = 0;
  {
    flux::ros::Publisher first(*pub_node, "/exec/restart", kFingerprint, kSlotSize, kSlots);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (got.load() == 0 && std::chrono::steady_clock::now() < deadline) {
      first.publish(buf.data(), buf.size());
      std::this_thread::sleep_for(2ms);
    }
    before = got.load();
  }  // last publisher out: the name is unlinked here

  flux::ros::Publisher second(*pub_node, "/exec/restart", kFingerprint, kSlotSize, kSlots);
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (got.load() <= before && std::chrono::steady_clock::now() < deadline) {
    second.publish(buf.data(), buf.size());
    std::this_thread::sleep_for(2ms);
  }
  const int after = got.load();

  run.store(false);
  exec.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_GT(before, 0) << "nothing was delivered before the restart";
  EXPECT_GT(after, before) << "the executor went silent after the publisher restarted";
}

// Source::deliver() is the helper for an owner driving a subscription with no executor, and it
// stops at `flux::kMaxDrain` frames. A consumer's QoS depth is normally lower and binds first, so
// the ceiling only appears with a window deeper than it -- which is why it went untested: at the
// default depth of 1 the constant could be any value at all and nothing would notice. The
// executor does not go through here; its bound is one pass budget over all channels.
TEST(FluxExecutor, DeliverStopsAtTheDrainCeiling)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_drain_cap");

  constexpr std::uint32_t kDeepSlots = 128;
  constexpr int kPublished = 100;
  static_assert(kPublished > flux::kMaxDrain, "the ceiling has to be what stops this drain");

  flux::QoS qos;
  qos.depth = kDeepSlots;  // deeper than the ceiling, so depth is not what stops it

  int calls = 0;
  flux::ros::Publisher pub(*node, "/exec/drain_cap", kFingerprint, 256, kDeepSlots);
  flux::ros::Subscription sub(
    *node, "/exec/drain_cap", kFingerprint, [&](const flux::FrameView &) { ++calls; }, qos);
  ASSERT_TRUE(sub.attach());  // joins the stream before anything is published

  std::vector<std::byte> buf(256);
  for (int i = 0; i < kPublished; ++i) {
    ASSERT_EQ(pub.publish(buf.data(), buf.size()), flux::Published::Ok);
  }

  EXPECT_EQ(sub.deliver(), flux::kMaxDrain);
  EXPECT_EQ(calls, flux::kMaxDrain);
  EXPECT_EQ(sub.deliver(), kPublished - flux::kMaxDrain);  // the remainder on the next pass

  rclcpp::shutdown();
}

// The MemoryPolicy argument reaches the mapping through both wrappers. What flux_core proves is
// that a committed mapping is resident; what can break here is the argument being dropped on the
// way through, and a dropped one looks like nothing at all -- the channel works, it just still
// faults on every first touch.
TEST(FluxExecutor, MemoryPolicyReachesBothWrappers)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mem");

  flux::MemoryPolicy mem;
  mem.precommit = true;

  flux::ros::Publisher pub(*node, "/exec/mem", kFingerprint, 4096, 8, flux::Device::Cpu, mem);
  flux::ros::Subscription sub(
    *node, "/exec/mem", kFingerprint, [](const flux::FrameView &) {}, flux::QoS{},
    flux::Device::Cpu, mem);

  EXPECT_TRUE(pub.pages_committed());
  EXPECT_FALSE(pub.pages_locked()) << "a commit is not a lock";
  ASSERT_TRUE(sub.attached()) << "the subscriber must be attached for its policy to have applied";
  EXPECT_TRUE(sub.pages_committed());

  std::vector<std::byte> buf(256);
  EXPECT_EQ(pub.publish(buf.data(), buf.size()), flux::Published::Ok);
  EXPECT_EQ(sub.deliver(), 1);

  rclcpp::shutdown();
}

// The priority argument reaches flux::Executor::add through this wrapper. The ordering itself is
// flux_core's contract and is tested there; what can break here is the argument being dropped on
// the way through, which would leave every source at the default and look like nothing at all.
TEST(FluxExecutor, PriorityReachesTheCore)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_prio");

  std::vector<int> order;
  flux::ros::Publisher pub_lo(*node, "/exec/prio_lo", kFingerprint, 256, 8);
  flux::ros::Publisher pub_hi(*node, "/exec/prio_hi", kFingerprint, 256, 8);
  flux::ros::Subscription lo(
    *node, "/exec/prio_lo", kFingerprint, [&](const flux::FrameView &) { order.push_back(1); });
  flux::ros::Subscription hi(
    *node, "/exec/prio_hi", kFingerprint, [&](const flux::FrameView &) { order.push_back(2); });

  flux::ros::Executor ex;
  ex.add(lo);  // registered first, default priority
  ex.add(hi, /*priority=*/10);

  std::vector<std::byte> buf(256);
  ASSERT_EQ(ex.dispatch(), 0);  // both attached before anything is published
  ASSERT_EQ(pub_lo.publish(buf.data(), buf.size()), flux::Published::Ok);
  ASSERT_EQ(pub_hi.publish(buf.data(), buf.size()), flux::Published::Ok);

  EXPECT_EQ(ex.dispatch(), 2);
  EXPECT_EQ(order, (std::vector<int>{2, 1}));

  rclcpp::shutdown();
}

// The io_uring is sized for max_channels + 1, so registering past it would have to overwrite an
// unsubmitted entry. Reject instead, the way flux_py's Executor already does -- silently dropping
// an arm leaves that one channel served only by the tick, with no error anywhere.
TEST(FluxExecutor, AddRejectsPastMaxChannels)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_cap_node");

  std::vector<std::unique_ptr<flux::ros::Subscription>> subs;
  for (int i = 0; i < 3; ++i) {
    subs.push_back(std::make_unique<flux::ros::Subscription>(
      *node, "/exec/cap" + std::to_string(i), kFingerprint, [](const flux::FrameView &) {}));
  }

  flux::ros::Executor ex(/*max_channels=*/2);
  ex.add(*subs[0]);
  ex.add(*subs[1]);
  EXPECT_THROW(ex.add(*subs[2]), std::length_error);

  rclcpp::shutdown();
}

// The reason the take is delegated to rclcpp: a generic (serialized) subscription is taken with a
// different rcl call into a different buffer type. Taking it by hand -- which this executor used
// to do -- wrote a message struct into a byte buffer and nothing complained. rosbag-style nodes
// are exactly this shape.
TEST(FluxExecutor, DeliversASerializedSubscription)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("flux_ser_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("flux_ser_sub");

  auto ros_pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/exec/serialized", 10);
  std::atomic<int> got{0};
  std::atomic<std::size_t> bytes{0};
  auto gen_sub = sub_node->create_generic_subscription(
    "/exec/serialized", "std_msgs/msg/UInt64", rclcpp::QoS(10),
    [&](const rclcpp::SerializedMessage & m) {
      bytes.store(m.size());
      got.fetch_add(1);
    });

  flux::ros::Executor exec;
  exec.add_ros_node(sub_node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 20'000'000); });

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  for (std::uint64_t i = 1; got.load() == 0 && std::chrono::steady_clock::now() < deadline; ++i) {
    std_msgs::msg::UInt64 m;
    m.data = i;
    ros_pub->publish(m);
    std::this_thread::sleep_for(5ms);
  }

  run.store(false);
  exec.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_GT(got.load(), 0) << "a serialized subscription never reached its callback";
  EXPECT_GT(bytes.load(), 0u) << "the serialized payload was empty";
}

// Intra-process readiness bypasses the rmw queue: with IPC on and every subscriber local,
// rclcpp never publishes to rmw at all, so the on-new-message hook alone sees nothing. Only the
// intra-process hook can wake the ring. The tick is set far past the deadline so a pass on the
// tick cannot mask a missing wake.
TEST(FluxExecutor, DeliversIntraProcessWithoutATick)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "flux_ipc_node", rclcpp::NodeOptions().use_intra_process_comms(true));

  auto pub = node->create_publisher<std_msgs::msg::UInt64>("/exec/ipc", 10);
  std::atomic<int> got{0};
  auto sub = node->create_subscription<std_msgs::msg::UInt64>(
    "/exec/ipc", 10, [&](const std_msgs::msg::UInt64 &) { got.fetch_add(1); });

  flux::ros::Executor exec;
  exec.add_ros_node(node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 30'000'000'000); });
  std::this_thread::sleep_for(100ms);  // let the executor reach its blocking wait

  std_msgs::msg::UInt64 m;
  m.data = 1;
  pub->publish(m);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (got.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }

  // Snapshot before teardown: the shutdown signal() pumps the wait set, which would deliver
  // the message late and turn a missed wake into a pass.
  const int delivered = got.load();
  run.store(false);
  exec.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_GT(delivered, 0) << "an intra-process message did not wake the executor";
}

// A subscription created after add_ros_node() has no hook yet; the next spin pass must pick it
// up. signal() forces that pass, then delivery must be event-driven -- the 30 s tick means a
// message that only surfaces on the tick fails the 5 s deadline.
TEST(FluxExecutor, BridgesASubscriptionCreatedAfterAdd)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_late_node");
  auto pub = node->create_publisher<std_msgs::msg::UInt64>("/exec/late", 10);

  flux::ros::Executor exec;
  exec.add_ros_node(node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 30'000'000'000); });
  std::this_thread::sleep_for(100ms);

  std::atomic<int> got{0};
  auto sub = node->create_subscription<std_msgs::msg::UInt64>(
    "/exec/late", 10, [&](const std_msgs::msg::UInt64 &) { got.fetch_add(1); });
  exec.interrupt();  // wake the executor so its next pass bridges the new subscription
  // Let that pass finish before publishing: its pump_ros() polls the whole rclcpp wait set,
  // which would deliver a racing publish even without the bridge and mask a missing hook.
  std::this_thread::sleep_for(200ms);

  // Republish until delivery: discovery may still be matching the first sends.
  std_msgs::msg::UInt64 m;
  m.data = 1;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (got.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    pub->publish(m);
    std::this_thread::sleep_for(20ms);
  }

  // Snapshot before teardown: the shutdown signal() pumps the wait set, which would deliver
  // the queued messages late and turn a missed wake into a pass.
  const int delivered = got.load();
  run.store(false);
  exec.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_GT(delivered, 0) << "a subscription created after add_ros_node was never bridged";
}

// The fallback the kernel selects on Jetson Orin (5.15, no io_uring FUTEX_WAIT). It cannot be
// reached on a 6.7+ machine unless it is forced, which is why it went untested until now: one
// thread per channel parks in the kernel and pokes the same eventfd the ROS bridge uses.
TEST(FluxExecutor, FallbackDeliversWithoutIoUring)
{
  ::setenv("FLUX_DISABLE_IO_URING", "1", 1);
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("flux_fb_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("flux_fb_sub");

  auto ros_pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/exec/fb_notify", 10);
  flux::ros::Publisher flux_pub(*pub_node, "/exec/fb_bulk", kFingerprint, kSlotSize, kSlots);

  std::atomic<int> flux_count{0};
  std::atomic<int> ros_count{0};
  std::atomic<int> torn{0};

  flux::ros::Subscription flux_sub(
    *sub_node, "/exec/fb_bulk", kFingerprint, [&](const flux::FrameView & v) {
      const auto * p = static_cast<const std::uint8_t *>(v.data());
      const auto tag = frame_tag(v);
      for (std::size_t i = 0; i < v.size(); ++i) {
        if (p[i] != tag) {
          torn.fetch_add(1);
          break;
        }
      }
      flux_count.fetch_add(1);
    });
  auto ros_sub = sub_node->create_subscription<std_msgs::msg::UInt64>(
    "/exec/fb_notify", 10, [&](const std_msgs::msg::UInt64 &) { ros_count.fetch_add(1); });
  (void)ros_sub;

  flux::ros::Executor exec;
  ASSERT_FALSE(exec.uses_io_uring()) << "the fallback was not selected";
  exec.add(flux_sub);
  exec.add_ros_node(sub_node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 100'000'000); });

  std::vector<std::byte> buf(1024);
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  for (std::uint64_t seq = 1; std::chrono::steady_clock::now() < deadline; ++seq) {
    if (flux_count.load() >= 20 && ros_count.load() >= 20) break;
    std::memset(buf.data(), static_cast<int>(seq & 0xFF), buf.size());
    flux_pub.publish(buf.data(), buf.size());
    std_msgs::msg::UInt64 m;
    m.data = seq;
    ros_pub->publish(m);
    std::this_thread::sleep_for(2ms);
  }

  run.store(false);
  exec.stop();
  spinner.join();
  rclcpp::shutdown();
  ::unsetenv("FLUX_DISABLE_IO_URING");

  EXPECT_EQ(torn.load(), 0) << "the fallback delivered a torn frame";
  EXPECT_GT(flux_count.load(), 0) << "no flux frame reached the callback on the fallback path";
  EXPECT_GT(ros_count.load(), 0) << "no ROS message reached the callback on the fallback path";
}

// tick_ns is an upper bound on how long the wait may block, not the resolution of everything the
// executor drives: a ROS timer that asked for 2 ms must still get 2 ms under a 200 ms tick. Teeth:
// with the deadline left out of the wait timeout this fires about twice in half a second.
TEST(FluxExecutor, RosTimerKeepsItsPeriodUnderALongTick)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_timer_tick");
  std::atomic<int> fired{0};
  auto timer = node->create_wall_timer(2ms, [&fired]() { fired.fetch_add(1); });
  (void)timer;

  flux::ros::Executor exec;
  exec.add_ros_node(node);
  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 200'000'000); });
  std::this_thread::sleep_for(500ms);
  run.store(false);
  exec.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_GT(fired.load(), 50) << "the 200 ms tick drove the timer instead of its own 2 ms period";
}

// The same timer must keep its period while a flux channel in the
// same executor is saturated. Before the dispatch yielded between callbacks it did not -- a 10 ms
// timer measured 128 ms at the default budget, because every pass ran up to pass_budget flux
// callbacks first and the deadline was sampled before the wait rather than during the dispatch.
TEST(FluxExecutor, RosTimerKeepsItsPeriodUnderASaturatedFluxChannel)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_timer_saturated");

  std::atomic<int> fired{0};
  auto timer = node->create_wall_timer(10ms, [&fired]() { fired.fetch_add(1); });
  (void)timer;

  flux::ros::Publisher pub(*node, "/flux/timer_sat", kFingerprint, 4096, 16);
  // Each callback burns real CPU: this is the WCET term the bound is written in.
  flux::ros::Subscription sub(*node, "/flux/timer_sat", kFingerprint, [](const flux::FrameView &) {
    const auto until = std::chrono::steady_clock::now() + 1ms;
    while (std::chrono::steady_clock::now() < until) {
    }
  });

  flux::ros::Executor exec;  // default pass_budget: the shape that measured 128 ms
  exec.add(sub);
  exec.add_ros_node(node);

  std::atomic<bool> flooding{true};
  std::thread flood([&] {
    const std::uint8_t payload[64] = {};
    while (flooding.load(std::memory_order_relaxed)) {
      pub.publish(payload, sizeof(payload));
      std::this_thread::sleep_for(100us);
    }
  });

  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 100'000'000); });
  std::this_thread::sleep_for(2s);
  run.store(false);
  exec.stop();
  spinner.join();
  flooding.store(false);
  flood.join();
  rclcpp::shutdown();

  // 2 s at 10 ms is ~200 fires. The pre-yield behaviour produced ~15. The threshold sits far from
  // both so a loaded machine does not decide the outcome.
  EXPECT_GT(fired.load(), 60) << "the flux dispatch is still holding the timer for a whole pass";
}

// The fallback blocks in poll() rather than io_uring, and it bounds that wait the same way.
TEST(FluxExecutor, FallbackRosTimerKeepsItsPeriodUnderALongTick)
{
  ::setenv("FLUX_DISABLE_IO_URING", "1", 1);
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_fb_timer_tick");
  std::atomic<int> fired{0};
  auto timer = node->create_wall_timer(2ms, [&fired]() { fired.fetch_add(1); });
  (void)timer;

  flux::ros::Executor exec;
  ASSERT_FALSE(exec.uses_io_uring()) << "the fallback was not selected";
  exec.add_ros_node(node);
  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 200'000'000); });
  std::this_thread::sleep_for(500ms);
  run.store(false);
  exec.stop();
  spinner.join();
  rclcpp::shutdown();
  ::unsetenv("FLUX_DISABLE_IO_URING");

  EXPECT_GT(fired.load(), 50) << "the 200 ms tick drove the timer instead of its own 2 ms period";
}

// Registration is pre-spin only: the spin thread iterates the entry lists lock-free, so a
// concurrent add() is a data race, not a late registration. Same contract as the
// PartitionedExecutor.
TEST(FluxExecutor, RegistrationDuringSpinThrows)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_exec_addguard");
  flux::ros::Publisher pub(*node, "/exec/addguard", kFingerprint, kSlotSize, kSlots);
  flux::ros::Subscription sub(
    *node, "/exec/addguard", kFingerprint, [](const flux::FrameView &) {});
  flux::ros::Subscription late(
    *node, "/exec/addguard_late", kFingerprint, [](const flux::FrameView &) {});

  flux::ros::Executor exec;
  exec.add(sub);

  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 20'000'000); });
  std::this_thread::sleep_for(100ms);

  EXPECT_THROW(exec.add(late), std::logic_error);
  EXPECT_THROW(exec.add_ros_node(node), std::logic_error);
  std::atomic<bool> run2{true};
  EXPECT_THROW(exec.spin(run2, 20'000'000), std::logic_error);

  run.store(false);
  exec.stop();
  spinner.join();

  exec.add(late);  // after spin() returned, registration works again
  rclcpp::shutdown();
}

// Destroying an executor while rmw listener threads are mid-signal must never write to a
// recycled fd: the hooks reach the eventfd through a weak_ptr and the close is deferred to the
// last in-flight hook. Smoke: repeated create/destroy under continuous ROS traffic.
TEST(FluxExecutor, DestructionUnderRosTraffic)
{
  rclcpp::init(0, nullptr);
  auto pub_node = std::make_shared<rclcpp::Node>("flux_exec_dtor_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("flux_exec_dtor_sub");
  auto ros_pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/exec/dtor", 10);
  auto ros_sub = sub_node->create_subscription<std_msgs::msg::UInt64>(
    "/exec/dtor", 10, [](std_msgs::msg::UInt64::ConstSharedPtr) {});

  std::atomic<bool> feeding{true};
  std::thread feeder([&] {
    std_msgs::msg::UInt64 m;
    while (feeding.load(std::memory_order_relaxed)) {
      ++m.data;
      ros_pub->publish(m);
      std::this_thread::sleep_for(200us);
    }
  });

  for (int i = 0; i < 10; ++i) {
    flux::ros::Executor exec;
    exec.add_ros_node(sub_node);
    std::atomic<bool> run{true};
    std::thread spinner([&] { exec.spin(run, 20'000'000); });
    std::this_thread::sleep_for(30ms);
    run.store(false);
    exec.stop();
    spinner.join();
  }  // ~Executor races the feeder's in-flight hooks here

  feeding.store(false);
  feeder.join();
  rclcpp::shutdown();
}

// The inherited spin()/cancel() pair is what a caller holding an rclcpp::Executor& reaches, so
// both are implemented rather than refused: spin() is this executor's own loop with the default
// tick, and cancel() is stop(). Not rclcpp::Executor::cancel(), which clears `spinning` -- here
// that is the reentry guard, so clearing it would admit a second concurrent spin instead of
// ending this one.
TEST(FluxExecutor, TheInheritedSpinAndCancelDriveThisExecutor)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_cancel_node");

  flux::ros::Executor ex(/*max_channels=*/1);
  ex.add_ros_node(node);
  rclcpp::Executor & base = ex;  // the way a generic rclcpp caller holds it

  std::atomic<bool> returned{false};
  std::thread spinner([&] {
    base.spin();
    returned.store(true);
  });

  std::this_thread::sleep_for(200ms);
  EXPECT_FALSE(returned.load()) << "the inherited spin() returned without being cancelled";
  base.cancel();
  spinner.join();
  EXPECT_TRUE(returned.load());

  // Cancelling is not sticky past the spin that consumed it: the same executor spins again.
  std::atomic<bool> again{false};
  std::thread second([&] {
    ex.spin(20'000'000);
    again.store(true);
  });
  std::this_thread::sleep_for(200ms);
  EXPECT_FALSE(again.load());
  ex.stop();
  second.join();
  EXPECT_TRUE(again.load());

  rclcpp::shutdown();
}

// The refusals that remain: their contract is a duration budget over a wait set this executor
// does not have, so an approximation would silently skip flux channels.
TEST(FluxExecutor, TheSpinVariantsWithADurationBudgetAreRefused)
{
  rclcpp::init(0, nullptr);
  flux::ros::Executor ex(/*max_channels=*/1);
  rclcpp::Executor & base = ex;
  EXPECT_THROW(base.spin_some(std::chrono::nanoseconds(0)), std::runtime_error);
  EXPECT_THROW(base.spin_all(std::chrono::nanoseconds(1'000'000)), std::runtime_error);
  rclcpp::shutdown();
}

// The startup race flux_py had cannot happen here: spin() only reads the flag, so a run
// flag cleared before the spin thread enters spin() is still false when it gets there.
TEST(FluxExecutor, AClearedRunFlagIsHonouredEvenIfItLandsBeforeSpinEntry)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_run_flag_node");

  flux::ros::Subscription sub(
    *node, "/exec/run_flag", kFingerprint, [](const flux::FrameView &) {});
  flux::ros::Executor ex(/*max_channels=*/1);
  ex.add(sub);

  std::atomic<bool> run{false};  // cleared before the thread is even started
  std::atomic<bool> returned{false};
  std::thread t([&] {
    ex.spin(run, 20'000'000);
    returned.store(true);
  });
  t.join();
  EXPECT_TRUE(returned.load());

  rclcpp::shutdown();
}

// The other half of the inherited surface. rclcpp reads `spinning` to hand work over and
// again to run it, because clearing it is how cancel() discards work found before the cancel;
// a pass that never raises it services nothing and says nothing. Before the fix the flux half of
// spin_once() ran and the ROS half was silently dead.
TEST(FluxExecutor, TheInheritedSpinOnceServicesRosEntities)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_spin_once_node");

  std::atomic<int> got{0};
  auto sub = node->create_subscription<std_msgs::msg::UInt64>(
    "/exec/spin_once", 10, [&](std_msgs::msg::UInt64::SharedPtr) { got.fetch_add(1); });
  auto pub = node->create_publisher<std_msgs::msg::UInt64>("/exec/spin_once", 10);

  flux::ros::Executor ex(/*max_channels=*/1);
  ex.add_ros_node(node);
  rclcpp::Executor & base = ex;

  std_msgs::msg::UInt64 m;
  m.data = 1;
  pub->publish(m);

  for (int i = 0; i < 50 && got.load() == 0; i++) {
    base.spin_once(20ms);
  }
  EXPECT_EQ(got.load(), 1) << "spin_once() ran the flux half and skipped every ROS entity";

  // The flag is rclcpp's cancel state, not a mode: a pass leaves it exactly as it found it.
  EXPECT_FALSE(ex.is_spinning());

  // pump_ros() is public and drives the same pass on its own.
  m.data = 2;
  pub->publish(m);
  for (int i = 0; i < 50 && got.load() == 1; i++) {
    ex.wait_for_work(20'000'000);
    ex.pump_ros();
  }
  EXPECT_EQ(got.load(), 2) << "the public pump_ros() serviced nothing";

  rclcpp::shutdown();
}

// spin_node_once is rclcpp's own composition over spin_once, so it inherits whatever that does.
TEST(FluxExecutor, SpinNodeOnceServicesTheNodeItIsGiven)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_spin_node_once_node");

  std::atomic<int> got{0};
  auto sub = node->create_subscription<std_msgs::msg::UInt64>(
    "/exec/spin_node_once", 10, [&](std_msgs::msg::UInt64::SharedPtr) { got.fetch_add(1); });
  auto pub = node->create_publisher<std_msgs::msg::UInt64>("/exec/spin_node_once", 10);

  flux::ros::Executor ex(/*max_channels=*/1);
  rclcpp::Executor & base = ex;

  std_msgs::msg::UInt64 m;
  m.data = 1;
  pub->publish(m);

  for (int i = 0; i < 50 && got.load() == 0; i++) {
    base.spin_node_once(node, 20ms);
  }
  EXPECT_EQ(got.load(), 1);

  rclcpp::shutdown();
}

// `ex.add_node(node)` used to reach rclcpp's own registration, which records the node without
// hooking anything to the wake fd -- the node was serviced only as often as the tick.
TEST(FluxExecutor, TheInheritedAddNodeBridgesRatherThanJustRegistering)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_add_node_node");

  std::atomic<int> got{0};
  auto sub = node->create_subscription<std_msgs::msg::UInt64>(
    "/exec/add_node", 10, [&](std_msgs::msg::UInt64::SharedPtr) { got.fetch_add(1); });
  auto pub = node->create_publisher<std_msgs::msg::UInt64>("/exec/add_node", 10);

  // A flux channel so the merged wait has something to block on. Without one the wait returns
  // at once, the loop spins hot, and every pass would deliver whether anything was bridged or
  // not -- the tick would never be the only thing left.
  flux::ros::Subscription flux_sub(
    *node, "/exec/add_node_flux", kFingerprint, [](const flux::FrameView &) {});
  flux::ros::Executor ex(/*max_channels=*/1);
  ex.add(flux_sub);
  rclcpp::Executor & base = ex;
  base.add_node(node);  // not add_ros_node

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 5'000'000'000); });  // a 5 s tick: only a hook can wake
  std::this_thread::sleep_for(200ms);  // let the spin reach the wait before publishing

  const auto deadline = std::chrono::steady_clock::now() + 3s;
  std_msgs::msg::UInt64 m;
  m.data = 1;
  pub->publish(m);
  while (got.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  // Read before stopping. stop() ends the wait, and the pass that follows it would deliver the
  // message whether a hook woke the ring or not.
  const int woken = got.load();
  run.store(false);
  ex.stop();
  spinner.join();

  EXPECT_EQ(woken, 1) << "the node was registered but never bridged, so only the tick would have "
                         "delivered it";
  rclcpp::shutdown();
}

// Removal has to take the hooks with it. A hook left behind keeps waking the ring for an entity
// no pass will service, which is a spurious wakeup per message for the life of the executor.
TEST(FluxExecutor, RemoveNodeClearsTheHooksItInstalled)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_remove_node_node");

  std::atomic<int> got{0};
  auto sub = node->create_subscription<std_msgs::msg::UInt64>(
    "/exec/remove_node", 10, [&](std_msgs::msg::UInt64::SharedPtr) { got.fetch_add(1); });
  auto pub = node->create_publisher<std_msgs::msg::UInt64>("/exec/remove_node", 10);

  flux::ros::Executor ex(/*max_channels=*/1);
  rclcpp::Executor & base = ex;
  base.add_node(node);

  std_msgs::msg::UInt64 m;
  m.data = 1;
  pub->publish(m);
  for (int i = 0; i < 50 && got.load() == 0; i++) {
    base.spin_once(20ms);
  }
  ASSERT_EQ(got.load(), 1);

  base.remove_node(node);
  m.data = 2;
  pub->publish(m);
  for (int i = 0; i < 10; i++) {
    base.spin_once(20ms);
  }
  EXPECT_EQ(got.load(), 1) << "a removed node was still serviced";

  rclcpp::shutdown();
}

// The two meanings that used to share one flag. A spin_once() from another thread raises
// rclcpp's gate for its pass; that must not read as a second spin, and must not clear the gate
// the running spin() is holding up.
TEST(FluxExecutor, ASpinOnceDuringASpinNeitherThrowsNorEndsTheSpin)
{
  FLUX_REQUIRE_IO_URING();
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_concurrent_pass_node");

  flux::ros::Subscription sub(
    *node, "/exec/concurrent", kFingerprint, [](const flux::FrameView &) {});
  flux::ros::Executor ex(/*max_channels=*/1);
  ex.add(sub);

  std::atomic<bool> run{true};
  std::atomic<bool> returned{false};
  std::thread spinner([&] {
    ex.spin(run, 20'000'000);
    returned.store(true);
  });
  std::this_thread::sleep_for(200ms);
  ASSERT_TRUE(ex.is_spinning());

  EXPECT_NO_THROW(ex.spin_once(1'000'000));
  std::this_thread::sleep_for(100ms);
  EXPECT_FALSE(returned.load()) << "a concurrent pass ended the spin";
  EXPECT_TRUE(ex.is_spinning()) << "a concurrent pass cleared the gate the spin was holding";

  // A second spin() is still refused: that guard is its own flag now.
  EXPECT_THROW(ex.spin(20'000'000), std::logic_error);

  run.store(false);
  ex.stop();
  spinner.join();
  EXPECT_TRUE(returned.load());
  rclcpp::shutdown();
}
