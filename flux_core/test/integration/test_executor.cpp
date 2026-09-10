#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/executor.hpp"
#include "flux/io_uring_waiter.hpp"
#include "support/frame_id.hpp"

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// The engine's executor without ROS and without Python. flux_cpp and flux_py each test their own
// derived executor, so until this file the base class was covered only through a wrapper that
// needs an rclcpp context to run at all -- flux_core's standalone build compiled executor.cpp and
// verified nothing in it. What a Source is is three calls, so a fake one reaches every path here.
//
// Both wait paths run: io_uring where the kernel has FUTEX_WAIT, and the parker-thread fallback
// forced by FLUX_DISABLE_IO_URING, which is what Jetson Orin (5.15) actually runs.

namespace
{
using namespace std::chrono_literals;
using flux::test::frame_id;
using flux::test::publish_id;

constexpr std::uint32_t kSlotSize = 256;
constexpr std::uint32_t kSlotCount = 8;

std::string uniq(const std::string & base)
{
  return base + "." + std::to_string(::getpid());
}

std::int64_t ms_since(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - t0)
    .count();
}

// Parked-subscriber count as a publisher reads it: the number the wake gate skips the syscall on
// when it is 0. The executor's whole waiter bookkeeping is this one integer being balanced.
std::int32_t gate(const flux::Channel & ch) noexcept
{
  return ch.wait_handle()->ctrl->waiters.load(std::memory_order_relaxed);
}

// A Source over a heap Channel: no segment, no publisher process, no ROS. `attachable` gates the
// late-attach retry the executor does every pass while a publisher is not up yet.
//
// The window is opened past the default depth of 1. At depth 1 a consumer only ever sees the
// newest frame, so "one pass drains what arrived" would be indistinguishable from "one pass
// delivers one frame".
class FakeSource : public flux::Source
{
public:
  FakeSource() { open(); }
  explicit FakeSource(std::nullptr_t) {}  // starts with no channel: attach() decides

  bool attach() override
  {
    attach_calls.fetch_add(1, std::memory_order_relaxed);
    if (ch_) return true;
    if (!attachable.load(std::memory_order_relaxed)) return false;
    open();
    return true;
  }

  int deliver_one() override
  {
    deliver_calls.fetch_add(1, std::memory_order_relaxed);
    if (throw_next.exchange(false, std::memory_order_relaxed)) {
      throw std::runtime_error("flux test: deliver_one() threw");
    }
    if (!ch_) return 0;
    flux::FrameView v = ch_->take();
    if (!v) return 0;
    last_id.store(frame_id(v), std::memory_order_relaxed);
    if (log_) log_->push_back(tag_);
    delivered.fetch_add(1, std::memory_order_relaxed);
    if (on_deliver) on_deliver();
    return 1;
  }

  flux::Channel * channel() noexcept override { return ch_.get(); }

  void publish(std::uint8_t id)
  {
    std::vector<std::byte> buf(kSlotSize);
    ASSERT_EQ(publish_id(*ch_, buf.data(), buf.size(), id), flux::Published::Ok);
  }

  // Records this source's tag once per delivered frame, so a shared log shows the order the pass
  // visited sources in.
  void log_into(std::vector<int> & sink, int tag)
  {
    log_ = &sink;
    tag_ = tag;
  }

  // What a Subscription does when its segment is orphaned: the Channel goes away entirely, and
  // the executor has to unwind the arming and the gate count it placed on the old block.
  void orphan() { ch_.reset(); }

  // Runs inside the callback, which is where a test makes a frame arrive mid-pass.
  std::function<void()> on_deliver;

  std::atomic<bool> attachable{true};
  std::atomic<bool> throw_next{false};
  std::atomic<int> attach_calls{0};
  std::atomic<int> deliver_calls{0};
  std::atomic<int> delivered{0};
  std::atomic<std::uint8_t> last_id{0};

private:
  void open()
  {
    ch_ = std::make_unique<flux::Channel>(kSlotSize, kSlotCount);
    flux::QoS q;
    q.depth = kSlotCount;
    ch_->qos(q);
  }

  std::unique_ptr<flux::Channel> ch_;
  std::vector<int> * log_ = nullptr;
  int tag_ = 0;
};

// A Source over a real shm subscriber Channel, for the one path a heap channel cannot reach: a
// publisher restart, which re-attaches the same Channel onto a new mapping and bumps its
// generation. The executor must notice that and re-arm, or the channel goes silent for good.
class ShmSource : public flux::Source
{
public:
  ShmSource(std::string name, std::uint64_t fp) : name_(std::move(name)), fp_(fp) {}

  bool attach() override
  {
    if (ch_) return true;
    try {
      ch_.emplace(flux::Channel::open(name_, fp_));
    } catch (const std::exception &) {
      return false;  // publisher not up yet
    }
    return true;
  }

  int deliver_one() override
  {
    if (!ch_) return 0;
    flux::FrameView v = ch_->take();
    if (!v) return 0;
    last_id.store(frame_id(v), std::memory_order_relaxed);
    delivered.fetch_add(1, std::memory_order_relaxed);
    return 1;
  }

  flux::Channel * channel() noexcept override { return ch_ ? &*ch_ : nullptr; }

  std::atomic<std::uint8_t> last_id{0};
  std::atomic<int> delivered{0};

private:
  std::string name_;
  std::uint64_t fp_;
  std::optional<flux::Channel> ch_;
};

// A worker released once per pass, so no thread is created inside the window a pass is aiming
// at: thread startup is four orders of magnitude wider than the gap being swept.
class Shot
{
public:
  explicit Shot(std::function<void()> action) : action_(std::move(action))
  {
    th_ = std::thread([this] {
      for (;;) {
        while (!fire_.load(std::memory_order_acquire)) {
          if (quit_.load(std::memory_order_relaxed)) return;
        }
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - t0 < std::chrono::nanoseconds(delay_)) {
        }
        action_();
        fire_.store(false, std::memory_order_release);
      }
    });
  }
  ~Shot()
  {
    quit_.store(true, std::memory_order_relaxed);
    th_.join();
  }
  Shot(const Shot &) = delete;
  Shot & operator=(const Shot &) = delete;

  void go(std::int64_t delay_ns)
  {
    delay_ = delay_ns;
    fire_.store(true, std::memory_order_release);
  }
  void settle()
  {
    while (fire_.load(std::memory_order_acquire)) {
    }
  }

private:
  std::function<void()> action_;
  std::int64_t delay_ = 0;
  std::atomic<bool> fire_{false};
  std::atomic<bool> quit_{false};
  std::thread th_;
};

class CountingExecutor : public flux::Executor
{
public:
  using flux::Executor::Executor;
  std::atomic<int> passes{0};

protected:
  void on_pass() override { passes.fetch_add(1, std::memory_order_relaxed); }
};

// Both wait paths, one body. The path is picked in the constructor from the environment, so it is
// selected here rather than per-executor. Only a missing opcode is a legitimate skip: a ring that
// could not be created on a kernel that has the opcode is a resource limit on this host, and the
// engine refuses to fold those two together.
class ExecutorTest : public ::testing::TestWithParam<bool>
{
protected:
  void SetUp() override
  {
    if (GetParam()) {
      ::setenv("FLUX_DISABLE_IO_URING", "1", 1);
      return;
    }
    ::unsetenv("FLUX_DISABLE_IO_URING");
    if (flux::IoUringWaiter::support() == flux::IoUringSupport::NoOpcode) {
      GTEST_SKIP() << "flux-cap:io-uring kernel lacks io_uring FUTEX_WAIT (needs 6.7+)";
    }
  }
  void TearDown() override { ::unsetenv("FLUX_DISABLE_IO_URING"); }

  bool fallback() const { return GetParam(); }
};

INSTANTIATE_TEST_SUITE_P(
  WaitPath, ExecutorTest, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & i) { return i.param ? "Fallback" : "IoUring"; });

}  // namespace

TEST_P(ExecutorTest, DispatchDrainsEveryRegisteredSource)
{
  FakeSource a, b;
  flux::Executor ex;
  ex.add(a);
  ex.add(b);
  EXPECT_EQ(ex.size(), 2u);
  EXPECT_EQ(ex.uses_io_uring(), !fallback());

  a.publish(1);
  a.publish(2);
  b.publish(3);

  EXPECT_EQ(ex.dispatch(), 3);
  EXPECT_EQ(a.delivered.load(), 2);
  EXPECT_EQ(b.delivered.load(), 1);
  EXPECT_EQ(ex.dispatch(), 0);  // nothing left, and dispatch never blocks to find that out
}

// The dispatch order contract: a pass visits sources in add() order, not in
// publish order and not in whatever order a hash of their addresses would give. It is the only
// ordering a caller can express, so it has to be the one that holds.
TEST_P(ExecutorTest, DispatchVisitsSourcesInRegistrationOrder)
{
  FakeSource first, second;
  std::vector<int> order;
  first.log_into(order, 1);
  second.log_into(order, 2);

  flux::Executor ex;
  ex.add(first);
  ex.add(second);

  second.publish(1);  // published first, registered second
  second.publish(2);
  first.publish(3);
  first.publish(4);

  ASSERT_EQ(ex.dispatch(), 4);
  EXPECT_EQ(order, (std::vector<int>{1, 1, 2, 2}));
}

// A higher priority is visited first regardless of when it was registered. Order priority only:
// nothing preempts, so the whole of the higher source's drain still precedes the lower one.
TEST_P(ExecutorTest, AHigherPriorityIsVisitedFirst)
{
  FakeSource low, high;
  std::vector<int> order;
  low.log_into(order, 1);
  high.log_into(order, 2);

  flux::Executor ex;
  ex.add(low, 0);
  ex.add(high, 10);  // registered second, visited first

  low.publish(1);
  low.publish(2);
  high.publish(3);
  high.publish(4);

  ASSERT_EQ(ex.dispatch(), 4);
  EXPECT_EQ(order, (std::vector<int>{2, 2, 1, 1}));
}

// Equal priorities fall back to registration order, which is what keeps the default (every
// source at 0) the contract the previous test states.
TEST_P(ExecutorTest, EqualPrioritiesKeepRegistrationOrder)
{
  FakeSource first, second, third;
  std::vector<int> order;
  first.log_into(order, 1);
  second.log_into(order, 2);
  third.log_into(order, 3);

  flux::Executor ex;
  ex.add(first, 5);
  ex.add(second, 5);
  ex.add(third, 5);

  third.publish(1);
  second.publish(2);
  first.publish(3);

  ASSERT_EQ(ex.dispatch(), 3);
  EXPECT_EQ(order, (std::vector<int>{1, 2, 3}));
}

// A negative priority is below the default, so a source can be pushed behind sources registered
// after it without renumbering them.
TEST_P(ExecutorTest, ANegativePriorityGoesLast)
{
  FakeSource background, normal;
  std::vector<int> order;
  background.log_into(order, 1);
  normal.log_into(order, 2);

  flux::Executor ex;
  ex.add(background, -1);
  ex.add(normal);

  background.publish(1);
  normal.publish(2);

  ASSERT_EQ(ex.dispatch(), 2);
  EXPECT_EQ(order, (std::vector<int>{2, 1}));
}

// Reversing the visit order must not cost a wake. Every publish here lands while the executor is
// already blocked, which is the only path where the arm tags matter at all: a ready channel is
// drained without ever consulting one.
//
// This is end-to-end coverage, not a probe of the tag mapping itself. A consistent permutation of
// the tags is benign -- arming, cancelling and clearing all read the same one -- and a mutation
// that indexes by visit position instead of registration position passes this. What kills that
// class of edit is ClearLeavesTheExecutorInert, where a stale index is out of range.
TEST_P(ExecutorTest, WakesKeepReachingBothEntriesWhenPriorityReversesTheVisit)
{
  FakeSource low, high;
  std::vector<int> order;
  low.log_into(order, 1);
  high.log_into(order, 2);

  flux::Executor ex;
  ex.add(low, 0);
  ex.add(high, 10);  // visit order is the reverse of the registration order the tags name
  ASSERT_EQ(ex.dispatch(), 0);  // both armed, nothing ready

  // Alternating and repeated: a tag naming the visit position corrupts the other entry's arming
  // first, so the starvation it causes only shows once the mismatch has been through both sides.
  std::vector<int> want;
  for (int round = 0; round < 4; ++round) {
    for (FakeSource * src : {&low, &high}) {
      const std::uint32_t id = static_cast<std::uint32_t>(0x100 * (round + 1)) + (src == &high);
      std::thread pub([src, id] {
        std::this_thread::sleep_for(30ms);
        std::vector<std::byte> buf(kSlotSize);
        publish_id(*src->channel(), buf.data(), buf.size(), id);
      });
      const auto t0 = std::chrono::steady_clock::now();
      const int n = ex.spin_once(/*timeout_ns=*/3'000'000'000);
      const auto elapsed = ms_since(t0);
      pub.join();
      want.push_back(src == &high ? 2 : 1);

      EXPECT_EQ(n, 1) << "round " << round;
      EXPECT_EQ(order, want) << "round " << round;
      EXPECT_LT(elapsed, 1500) << "round " << round
                               << ": the wake never reached this entry; the wait timed out";
    }
  }
}

// And a source drains fully before the pass moves on, rather than the pass interleaving one frame
// each. That is what makes the per-pass bound a sum of per-channel terms.
TEST_P(ExecutorTest, ASourceDrainsBeforeThePassMovesOn)
{
  FakeSource a, b;
  std::vector<int> order;
  a.log_into(order, 1);
  b.log_into(order, 2);

  flux::Executor ex;
  ex.add(a);
  ex.add(b);
  for (std::uint8_t i = 1; i <= 3; ++i) {
    a.publish(i);
    b.publish(i);
  }

  ASSERT_EQ(ex.dispatch(), 6);
  EXPECT_EQ(order, (std::vector<int>{1, 1, 1, 2, 2, 2}));
}

// The point of one frame per visit: a frame that lands on a higher-priority channel while a
// lower one's callback runs is served before that lower channel's next frame. Draining a channel
// before moving on would deliver 1, 1, 2 instead -- the arrival would wait out the rest of b.
TEST_P(ExecutorTest, AFrameArrivingDuringACallbackGoesBeforeTheNextLowerOne)
{
  FakeSource high, low;
  std::vector<int> order;
  high.log_into(order, 1);
  low.log_into(order, 2);

  flux::Executor ex;
  ex.add(high, 10);
  ex.add(low, 0);

  low.publish(1);
  low.publish(2);
  bool once = true;
  low.on_deliver = [&] {
    if (!once) return;
    once = false;
    high.publish(3);
  };

  ASSERT_EQ(ex.dispatch(), 3);
  EXPECT_EQ(order, (std::vector<int>{2, 1, 2}));
}

// The budget covers the pass, not each channel: two sources with two frames each and a budget of
// 3 run three callbacks, and the fourth waits for the next pass.
TEST_P(ExecutorTest, ThePassBudgetCoversEveryChannelTogether)
{
  FakeSource a, b;
  std::vector<int> order;
  a.log_into(order, 1);
  b.log_into(order, 2);

  flux::Executor ex;
  ex.set_pass_budget(3);
  ex.add(a, 10);
  ex.add(b, 0);
  for (std::uint8_t i = 1; i <= 2; ++i) {
    a.publish(i);
    b.publish(i);
  }

  ASSERT_EQ(ex.dispatch(), 3);
  EXPECT_EQ(order, (std::vector<int>{1, 1, 2}));
  ASSERT_EQ(ex.dispatch(), 1);
  EXPECT_EQ(order, (std::vector<int>{1, 1, 2, 2}));
}

// An embedder sharing this thread reports through the yield predicate
// that its own work has come due, and the pass ends at the next callback boundary rather than at
// the budget. Asked between callbacks, never before the first, so the frame that woke the wait is
// always delivered.
TEST_P(ExecutorTest, AYieldEndsThePassAtTheNextCallbackBoundary)
{
  FakeSource a;
  std::vector<int> order;
  a.log_into(order, 1);

  flux::Executor ex;
  ex.set_pass_budget(64);
  ex.add(a);
  for (std::uint8_t i = 1; i <= 8; ++i) a.publish(i);

  int asked = 0;
  EXPECT_EQ(
    ex.dispatch([&] {
      ++asked;
      return true;
    }),
    1)
    << "the pass ran past the yield";
  EXPECT_EQ(asked, 1) << "the predicate was asked before the first callback, or more than once";
  EXPECT_TRUE(ex.has_more()) << "frames are still ready, so the caller must skip its next wait";

  // Nothing else changes: the rest is still there and a pass without a yield takes it.
  EXPECT_GT(ex.dispatch(), 1);
}

// The same predicate returning false must leave the pass exactly as it was without one.
TEST_P(ExecutorTest, AYieldThatNeverFiresChangesNothing)
{
  FakeSource a, b;
  std::vector<int> order;
  a.log_into(order, 1);
  b.log_into(order, 2);

  flux::Executor ex;
  ex.set_pass_budget(3);
  ex.add(a, 10);
  ex.add(b, 0);
  for (std::uint8_t i = 1; i <= 2; ++i) {
    a.publish(i);
    b.publish(i);
  }

  ASSERT_EQ(ex.dispatch([] { return false; }), 3);
  EXPECT_EQ(order, (std::vector<int>{1, 1, 2}));
}

// What an embedder driving wait_for_work()/dispatch() itself needs: the ring is not poked again
// for a frame the budget already left behind, so without this it sleeps a whole tick on work it
// is holding.
TEST_P(ExecutorTest, ABudgetCutPassSaysThereIsMore)
{
  FakeSource a;
  flux::Executor ex;
  ex.set_pass_budget(2);
  ex.add(a);
  for (std::uint8_t i = 1; i <= 3; ++i) a.publish(i);

  ASSERT_EQ(ex.dispatch(), 2);
  EXPECT_TRUE(ex.has_more());
  ASSERT_EQ(ex.dispatch(), 1);
  EXPECT_FALSE(ex.has_more()) << "the pass ended on an empty channel, not on the budget";
  ASSERT_EQ(ex.dispatch(), 0);
  EXPECT_FALSE(ex.has_more());
}

TEST_P(ExecutorTest, ThePassBudgetRejectsAnEmptyPass)
{
  flux::Executor ex;
  EXPECT_EQ(ex.pass_budget(), flux::kMaxDrain);
  EXPECT_THROW(ex.set_pass_budget(0), std::invalid_argument);
  ex.set_pass_budget(1);
  EXPECT_EQ(ex.pass_budget(), 1);
}

// A Source whose publisher is not up refuses to attach, and the executor must keep asking. Before
// this is true a channel registered before its publisher started would never be served.
TEST_P(ExecutorTest, DispatchRetriesASourceThatCannotAttachYet)
{
  FakeSource s{nullptr};
  s.attachable.store(false);
  flux::Executor ex;
  ex.add(s);

  EXPECT_EQ(ex.dispatch(), 0);
  EXPECT_EQ(ex.dispatch(), 0);
  EXPECT_GE(s.attach_calls.load(), 2) << "an unattached source must be retried every pass";
  EXPECT_EQ(s.channel(), nullptr);

  s.attachable.store(true);
  EXPECT_EQ(ex.dispatch(), 0);  // attaches this pass; nothing published yet
  ASSERT_NE(s.channel(), nullptr);
  s.publish(7);
  EXPECT_EQ(ex.dispatch(), 1);
  EXPECT_EQ(s.last_id.load(), 7u);
}

TEST_P(ExecutorTest, SpinOnceNeverBlocksWhenWorkIsAlreadyReady)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);
  s.publish(9);

  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(ex.spin_once(/*timeout_ns=*/2'000'000'000), 1);
  EXPECT_LT(ms_since(t0), 500) << "spin_once waited despite dispatch() finding work";
}

// The bounded wait means what it says on both paths: the fallback sleeps one attach tick while a
// publisher is absent, and a wake fd poke can be spurious, so an early return has to re-wait.
TEST_P(ExecutorTest, SpinOnceWaitsOutItsTimeoutWithNothingReady)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);

  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(ex.spin_once(/*timeout_ns=*/300'000'000), 0);
  EXPECT_GE(ms_since(t0), 250) << "the wait returned early with nothing ready";
}

TEST_P(ExecutorTest, APublishFromAnotherThreadEndsTheWait)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);
  ex.dispatch();  // arm the wait on an empty channel first

  std::thread pub([&] {
    std::this_thread::sleep_for(100ms);
    std::vector<std::byte> buf(kSlotSize);
    publish_id(*s.channel(), buf.data(), buf.size(), 0x42);
  });

  const auto t0 = std::chrono::steady_clock::now();
  const int n = ex.spin_once(/*timeout_ns=*/5'000'000'000);
  const auto elapsed = ms_since(t0);
  pub.join();

  EXPECT_EQ(n, 1);
  EXPECT_EQ(s.last_id.load(), 0x42u);
  EXPECT_LT(elapsed, 3000) << "the publish did not wake the executor; it timed out instead";
}

// One interrupt ends at most one call. A flag left set would end a later wait nobody asked to end,
// which on the spin() loop is a busy loop rather than a bounded wait.
TEST_P(ExecutorTest, AnInterruptEndsExactlyOneWait)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);

  ex.interrupt();
  auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(ex.spin_once(/*timeout_ns=*/2'000'000'000), 0);
  EXPECT_LT(ms_since(t0), 1000) << "interrupt() did not end the wait";

  t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(ex.spin_once(/*timeout_ns=*/300'000'000), 0);
  EXPECT_GE(ms_since(t0), 250) << "the consumed interrupt ended a second wait too";
}

// waker() is the seam an embedder merges its own readiness through: it ends the blocking half so
// the next pass runs, which is where that embedder's work gets serviced.
TEST_P(ExecutorTest, TheWakerBreaksTheWaitAndSurvivesTheExecutor)
{
  std::function<void()> wake;
  {
    FakeSource s;
    flux::Executor ex;
    ex.add(s);
    wake = ex.waker();
    ex.dispatch();  // arm the wait first

    std::thread poker([&] {
      std::this_thread::sleep_for(100ms);
      wake();
    });
    const auto t0 = std::chrono::steady_clock::now();
    ex.wait_for_work(/*timeout_ns=*/5'000'000'000);
    const auto elapsed = ms_since(t0);
    poker.join();
    EXPECT_LT(elapsed, 3000) << "waker() did not break the wait";
  }
  // The hook a foreign runtime still holds after the executor is gone: it reaches the fd weakly,
  // so this writes to a still-open fd or does nothing, never to a recycled fd number.
  wake();
}

// And breaking the wait is all it does. A poke with nothing ready is spurious as far as
// spin_once is concerned, so the bounded wait re-waits to its deadline rather than returning
// early: only interrupt() and stop() shorten it. An embedder that wants the pass to end without
// work has interrupt(); waker() is not a second spelling of it.
TEST_P(ExecutorTest, AWakerPokeDoesNotShortenSpinOnce)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);
  auto wake = ex.waker();

  std::thread poker([&] {
    std::this_thread::sleep_for(50ms);
    wake();
  });
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(ex.spin_once(/*timeout_ns=*/400'000'000), 0);
  const auto elapsed = ms_since(t0);
  poker.join();
  EXPECT_GE(elapsed, 350) << "a spurious wake cut the bounded wait short";
}

TEST_P(ExecutorTest, AddRejectsPastMaxChannels)
{
  FakeSource a, b, c;
  flux::Executor ex(/*max_channels=*/2);
  ex.add(a);
  ex.add(b);
  EXPECT_THROW(ex.add(c), std::length_error);
  EXPECT_EQ(ex.size(), 2u);
}

// Registration is pre-spin only: the spin thread walks the entry vector without a lock, so a
// concurrent add() is a data race and not a late registration.
TEST_P(ExecutorTest, AddDuringSpinThrows)
{
  FakeSource a, b;
  flux::Executor ex;
  ex.add(a);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, /*tick_ns=*/20'000'000); });
  while (!ex.is_spinning()) std::this_thread::sleep_for(1ms);

  EXPECT_THROW(ex.add(b), std::logic_error);

  ex.stop();
  spinner.join();
  EXPECT_FALSE(ex.is_spinning());
}

TEST_P(ExecutorTest, SpinRefusesReentry)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, /*tick_ns=*/20'000'000); });
  while (!ex.is_spinning()) std::this_thread::sleep_for(1ms);

  EXPECT_THROW(ex.spin(/*tick_ns=*/20'000'000), std::logic_error);

  ex.stop();
  spinner.join();
}

// "am I spinning" and "should I stop" are two flags. With one, entering spin() arms it and
// that arming erases a stop() that landed while the caller was still starting the thread -- the
// caller has moved on to join(), so nobody asks again and the loop never ends.
TEST_P(ExecutorTest, AStopBeforeSpinEndsItAndIsClearedOnTheWayOut)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);

  ex.stop();  // lands before spin() is ever entered
  const auto t0 = std::chrono::steady_clock::now();
  ex.spin(/*tick_ns=*/5'000'000'000);
  EXPECT_LT(ms_since(t0), 2000) << "a stop() placed before spin() was lost";

  // And the flag did not stay set: the executor is spinnable again.
  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, /*tick_ns=*/20'000'000); });
  while (!ex.is_spinning()) std::this_thread::sleep_for(1ms);
  s.publish(3);
  ex.stop();
  spinner.join();
  EXPECT_EQ(s.last_id.load(), 3u);
}

TEST_P(ExecutorTest, AClearedRunFlagEndsTheSpin)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);

  std::atomic<bool> run{false};  // cleared before the spin is even entered
  const auto t0 = std::chrono::steady_clock::now();
  ex.spin(run, /*tick_ns=*/5'000'000'000);
  EXPECT_LT(ms_since(t0), 2000);
}

TEST_P(ExecutorTest, OnPassRunsOncePerSpinPass)
{
  FakeSource s;
  CountingExecutor ex;
  ex.add(s);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, /*tick_ns=*/10'000'000); });
  while (ex.passes.load() < 3) std::this_thread::sleep_for(2ms);
  ex.stop();
  spinner.join();
  EXPECT_GE(ex.passes.load(), 3);
}

// A throw out of a callback leaves through the same guard a normal exit does, so the executor is
// not left marked as spinning by an exception that unwound past the loop.
TEST_P(ExecutorTest, AThrowFromDeliverLeavesTheExecutorSpinnable)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);
  s.publish(1);
  s.throw_next.store(true);

  EXPECT_THROW(ex.spin(/*tick_ns=*/20'000'000), std::runtime_error);
  EXPECT_FALSE(ex.is_spinning());

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, /*tick_ns=*/10'000'000); });
  while (!ex.is_spinning()) std::this_thread::sleep_for(1ms);
  ex.stop();
  spinner.join();
}

TEST_P(ExecutorTest, ClearLeavesTheExecutorInert)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);
  s.publish(1);
  ASSERT_EQ(ex.dispatch(), 1);

  ex.clear();
  EXPECT_EQ(ex.size(), 0u);
  EXPECT_EQ(gate(*s.channel()), 0) << "clear() left this executor counted as a parked waiter";

  s.publish(2);
  EXPECT_EQ(ex.dispatch(), 0) << "a cleared executor still read through a dropped Source";
  EXPECT_EQ(s.last_id.load(), 1u);
}

// clear() used to leave a stop request behind. The flag is cleared by the Session destructor, so
// with no session alive it stayed set and the next spin() returned before running a single pass.
TEST_P(ExecutorTest, ClearDoesNotStopTheNextSpin)
{
  FakeSource s;
  flux::Executor ex;
  ex.add(s);
  ex.clear();

  FakeSource again;
  ex.add(again);
  again.publish(1);
  std::atomic<bool> run{true};
  std::thread t([&] { ex.spin(run, 2000000); });
  for (int i = 0; i < 200 && again.last_id.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  run.store(false);
  ex.stop();
  t.join();
  EXPECT_EQ(again.last_id.load(), 1u) << "the first spin after clear() delivered nothing";
}

// The gate is what a publisher reads to decide whether to pay for a wake syscall at all. An
// unbalanced +1 costs every publisher a syscall forever; an unbalanced -1 drives it to 0 or below
// and the wake is skipped for everyone, which is a channel that goes silent.
TEST_P(ExecutorTest, TheWaiterGateIsBalancedAcrossTheExecutorLifetime)
{
  FakeSource s;
  ASSERT_EQ(gate(*s.channel()), 0);
  auto held = s.channel()->wait_handle();
  {
    flux::Executor ex;
    ex.add(s);
    ex.dispatch();
    // io_uring parks in the kernel for the whole wait, so the gate is held for the duration.
    // The fallback's parker announces itself only while inside wait(), so it is not asserted here.
    if (!fallback()) EXPECT_EQ(gate(*s.channel()), 1);
    ex.dispatch();
    ex.dispatch();
    if (!fallback()) EXPECT_EQ(gate(*s.channel()), 1) << "repeated passes stacked waiter counts";
  }
  EXPECT_EQ(held->ctrl->waiters.load(std::memory_order_relaxed), 0)
    << "the executor did not release its waiter count on destruction";
}

// The Source dropped its Channel: the arming and the gate count sit on a block the Channel no
// longer names, so removal has to hit that exact block rather than whatever the Source attaches
// next. Getting it wrong drives a live segment's gate negative.
TEST_P(ExecutorTest, AnOrphanedChannelRestartsTheEntryWithoutLeakingTheGate)
{
  FakeSource s;
  auto old_block = s.channel()->wait_handle();
  flux::Executor ex;
  ex.add(s);
  s.publish(1);
  ASSERT_EQ(ex.dispatch(), 1);

  s.orphan();  // the segment went away under the executor
  EXPECT_EQ(ex.dispatch(), 0) << "the entry did not restart cleanly on an orphaned channel";
  EXPECT_EQ(old_block->ctrl->waiters.load(std::memory_order_relaxed), 0)
    << "the waiter count stayed on the block the Channel no longer names";

  ASSERT_NE(s.channel(), nullptr) << "attach() should have replaced the channel";
  EXPECT_NE(s.channel()->wait_handle().get(), old_block.get());
  s.publish(2);
  EXPECT_EQ(ex.dispatch(), 1);
  EXPECT_EQ(s.last_id.load(), 2u);
}

// A publisher restart moves the subscriber to a new mapping and bumps its attach generation. A
// wait armed on the old mapping's word would stay pending in the kernel forever, so the executor
// has to reap it and arm the new word the same pass. Same property
// flux_cpp checks through a ROS subscription, here with nothing but the engine.
TEST_P(ExecutorTest, DeliverySurvivesAPublisherRestart)
{
  const std::uint64_t fp = 0xE0E0E0u;
  const std::string name = flux::segment_name(uniq("/flux_exec_restart"), fp);
  ::shm_unlink(name.c_str());
  std::vector<std::byte> buf(128);

  std::optional<flux::Channel> pub1;
  pub1.emplace(flux::open_publisher_segment(name, 128, 4, fp));
  ASSERT_EQ(publish_id(*pub1, buf.data(), buf.size(), 1), flux::Published::Ok);

  ShmSource s(name, fp);
  flux::Executor ex;
  ex.add(s);
  ASSERT_EQ(ex.dispatch(), 0);  // attaches; take() only delivers frames newer than the join
  ASSERT_NE(s.channel(), nullptr);
  const std::uint32_t gen1 = s.channel()->attach_generation();

  ASSERT_EQ(publish_id(*pub1, buf.data(), buf.size(), 1), flux::Published::Ok);
  ASSERT_EQ(ex.dispatch(), 1);
  ASSERT_EQ(s.last_id.load(), 1u);

  pub1.reset();  // last publisher out unlinks its unique segment
  std::optional<flux::Channel> pub2;
  pub2.emplace(flux::open_publisher_segment(name, 128, 4, fp));  // rotates the signpost

  // The re-attach happens when a take stalls on the dead mapping, and the frame that proves
  // delivery resumed has to be newer than that re-attach: keep publishing while dispatching.
  bool recovered = false;
  for (int i = 0; i < 400 && !recovered; ++i) {
    publish_id(*pub2, buf.data(), buf.size(), 2);
    if (ex.dispatch() > 0 && s.last_id.load() == 2u) recovered = true;
  }
  EXPECT_TRUE(recovered) << "the executor stopped delivering across a publisher restart";
  EXPECT_GT(s.channel()->attach_generation(), gen1)
    << "no re-attach happened; the test proved nothing";

  // Teeth: delivery resuming only proves dispatch() reconciled. The wait has to be armed on the
  // NEW word too, so a publish alone -- no tick, no further dispatch -- ends the wait.
  std::thread pub([&] {
    std::this_thread::sleep_for(100ms);
    publish_id(*pub2, buf.data(), buf.size(), 3);
  });
  const auto t0 = std::chrono::steady_clock::now();
  const int woke = ex.spin_once(/*timeout_ns=*/5'000'000'000);
  const auto elapsed = ms_since(t0);
  pub.join();
  EXPECT_GT(woke, 0);
  EXPECT_LT(elapsed, 3000) << "the wait was still armed on the replaced segment's word";

  pub2.reset();
  ::shm_unlink(name.c_str());
}

// A channel's futex wait and the control eventfd armed on the same ring, completing against
// each other. The offset is
// swept a nanosecond at a time across the pass's entry path, which is where a wait that is decided
// on but not yet entered can miss a signal.
//
// Three kinds of pass, cycled, because an interrupt on every pass masks a lost frame re-arm: the
// executor wakes for the wrong reason and delivers anyway. Deleting the re-arm in dispatch() must
// make the frame-only kind stall, or this test is watching nothing.
TEST_P(ExecutorTest, AMergedWaitLosesNeitherAFrameNorAnInterrupt)
{
  constexpr int kPerKind = 150;
  constexpr std::int64_t kTimeoutNs = 100'000'000;  // 600x the worst wait measured on this path

  const std::uint64_t fp = 0xEF0011u;
  const std::string name = flux::segment_name(uniq("/flux_exec_merged"), fp);
  ::shm_unlink(name.c_str());

  std::optional<flux::Channel> pub;
  pub.emplace(flux::open_publisher_segment(name, kSlotSize, kSlotCount, fp));
  std::vector<std::byte> buf(kSlotSize);

  ShmSource s(name, fp);
  flux::Executor ex;
  ex.add(s);
  ex.dispatch();  // attach before the sweep, so every offset lands on a live wait
  ASSERT_NE(s.channel(), nullptr);

  int published = 0;
  std::atomic<int> ok{0};
  Shot publisher([&] {
    if (publish_id(*pub, buf.data(), buf.size(), 1) == flux::Published::Ok) {
      ok.fetch_add(1, std::memory_order_relaxed);
    }
  });
  Shot interrupter([&] { ex.interrupt(); });

  enum Kind { kFrameOnly, kCtlOnly, kBoth, kKinds };
  const char * kind_name[kKinds] = {"frame only", "control only", "collision"};
  int stalls[kKinds] = {0, 0, 0};

  for (int i = 0; i < kPerKind * kKinds; ++i) {
    const int kind = i % kKinds;
    if (kind != kCtlOnly) {
      publisher.go(i);
      ++published;
    }
    if (kind != kFrameOnly) interrupter.go(i + 1);  // a nanosecond behind, into the same wait

    const auto t0 = std::chrono::steady_clock::now();
    ex.spin_once(kTimeoutNs);
    const std::int64_t took =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0)
        .count();

    if (kind != kCtlOnly) publisher.settle();
    if (kind != kFrameOnly) interrupter.settle();
    if (took >= kTimeoutNs) ++stalls[kind];
  }

  for (int i = 0; i < 200 && s.delivered.load() < ok.load(); ++i) ex.spin_once(5'000'000);

  for (int k = 0; k < kKinds; ++k) {
    EXPECT_EQ(stalls[k], 0) << "a " << kind_name[k] << " pass ran its wait out: a wake was lost";
  }
  // Delivery is best effort, so only what the ring did not withhold on purpose is a lost wake.
  EXPECT_EQ(ok.load() - s.delivered.load() - static_cast<int>(s.channel()->lost()), 0)
    << "published " << ok.load() << ", delivered " << s.delivered.load() << ", lost "
    << s.channel()->lost();
  EXPECT_EQ(published, ok.load()) << "the ring refused a publish; the sweep did not run as written";

  pub.reset();
  ::shm_unlink(name.c_str());
}
