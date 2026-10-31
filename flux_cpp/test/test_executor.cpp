#include "flux/detail/executor.hpp"
#include "flux/detail/subscriber.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace
{

class ExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override { ex_.emplace(); }
  std::optional<flux::detail::Executor> ex_;
};

struct Feed : flux::detail::Source
{
  flux::detail::Channel ch{16, 4};
  bool attached = true;
  bool fail = false;
  std::atomic<int> delivered{0};

  auto attach() -> bool override
  {
    if (fail && !attached) {
      throw std::runtime_error("attach failed");
    }
    return attached;
  }
  auto deliver_one() -> bool override
  {
    if (fail) {
      throw std::runtime_error("deliver failed");
    }
    if (!ch.take()) {
      return false;
    }
    ++delivered;
    return true;
  }
  [[nodiscard]] auto channel() const -> const flux::detail::Channel * override
  {
    return attached ? &ch : nullptr;
  }
};

// A subscription made before its publisher, as flux::Subscription is.
struct Late : flux::detail::Source
{
  explicit Late(const std::string & name) : sub(name, flux::kNoSchema, flux::QoS{}, {}) {}
  flux::detail::Subscriber sub;
  std::atomic<int> delivered{0};

  auto attach() -> bool override { return sub.attach_for_executor(); }
  auto deliver_one() -> bool override
  {
    if (!sub.take_for_executor()) {
      return false;
    }
    ++delivered;
    return true;
  }
  [[nodiscard]] auto channel() const -> const flux::detail::Channel * override
  {
    return sub.channel();
  }
  [[nodiscard]] auto watch() const -> const flux::detail::Watch * override { return sub.watch(); }
};

auto parker_threads() -> int
{
  int n = 0;
  for (const auto & task : std::filesystem::directory_iterator("/proc/self/task")) {
    std::ifstream comm(task.path() / "comm");
    std::string name;
    std::getline(comm, name);
    n += name == "flux-parker" ? 1 : 0;
  }
  return n;
}

// test_executor_parked runs this file again with FLUX_DISABLE_FUTEX_WAITV set: the fallback a
// kernel below 5.16 takes.
TEST_F(ExecutorTest, TheWaitTakesThePathTheEnvironmentChose)
{
  flux::detail::Executor & ex = *ex_;
  Feed f;
  ex.add(f);
  ex.dispatch();
  ex.wait_for_work(0);
  const bool parked = std::getenv("FLUX_DISABLE_FUTEX_WAITV") != nullptr;
  // A parker names itself once it runs.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (parked && parker_threads() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  EXPECT_EQ(parker_threads() > 0, parked);
}

// The executor sleeps on the subscription's watch, which the publisher coming up wakes: with a
// tick this long, a retry once a tick would deliver nothing in time.
TEST_F(ExecutorTest, APublisherComingUpWakesASubscriptionWaitingForIt)
{
  flux::detail::Executor & ex = *ex_;
  const std::string name =
    flux::detail::signpost_name("/cppx" + std::to_string(getpid()) + "_late");
  Late late(name);
  ex.add(late);
  const flux::detail::ExecutorHandle h = ex.handle();
  std::thread spin([&] { ex.spin(10'000'000'000); });
  while (!h.is_spinning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  auto pub = flux::detail::Channel::create(name, 16, 4);
  const auto t0 = std::chrono::steady_clock::now();
  const auto since = [&] { return std::chrono::steady_clock::now() - t0; };
  while (late.delivered == 0 && since() < std::chrono::seconds(2)) {
    ASSERT_EQ(pub.publish("a", 1), flux::Published::Ok);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto took = since();
  h.stop();
  spin.join();
  EXPECT_LT(took, std::chrono::milliseconds(500));
}

TEST_F(ExecutorTest, DispatchDeliversToASource)
{
  flux::detail::Executor & ex = *ex_;
  Feed f;
  ex.add(f);
  EXPECT_EQ(ex.size(), 1u);
  ASSERT_EQ(f.ch.publish("a", 1), flux::Published::Ok);
  EXPECT_EQ(ex.dispatch(), 1u);
  EXPECT_EQ(f.delivered, 1);
  EXPECT_EQ(ex.dispatch(), 0u);
}

TEST_F(ExecutorTest, AnExceptionFromDeliverLeavesTheSpin)
{
  flux::detail::Executor & ex = *ex_;
  Feed f;
  f.ch.qos(flux::QoS(2));
  ex.add(f);
  ASSERT_EQ(f.ch.publish("a", 1), flux::Published::Ok);
  f.fail = true;
  EXPECT_THROW(ex.spin(10'000'000), std::runtime_error);
  f.fail = false;
  ASSERT_EQ(f.ch.publish("b", 1), flux::Published::Ok);
  EXPECT_EQ(ex.dispatch(), 2u);
}

TEST_F(ExecutorTest, AnExceptionFromAttachLeavesTheSpin)
{
  flux::detail::Executor & ex = *ex_;
  Feed f;
  f.attached = false;
  f.fail = true;
  ex.add(f);
  EXPECT_THROW(ex.spin(10'000'000), std::runtime_error);
}

struct BrokenChannel : Feed
{
  [[nodiscard]] auto channel() const -> const flux::detail::Channel * override
  {
    throw std::runtime_error("channel failed");
  }
};

// channel() is called through the C ABI too, which an exception must not cross.
TEST_F(ExecutorTest, AnExceptionFromChannelLeavesTheCall)
{
  flux::detail::Executor & ex = *ex_;
  BrokenChannel f;
  ex.add(f);
  ASSERT_EQ(f.ch.publish("a", 1), flux::Published::Ok);
  EXPECT_THROW(ex.dispatch(), std::runtime_error);
}

TEST_F(ExecutorTest, OnPassRunsAndItsExceptionLeavesTheSpin)
{
  flux::detail::Executor & ex = *ex_;
  int passes = 0;
  ex.on_pass([&] {
    if (++passes == 3) {
      throw std::logic_error("third pass");
    }
  });
  EXPECT_THROW(ex.spin(1'000'000), std::logic_error);
  EXPECT_EQ(passes, 3);
}

TEST_F(ExecutorTest, AHandleStopsASpinOnAnotherThread)
{
  flux::detail::Executor & ex = *ex_;
  Feed f;
  ex.add(f);
  flux::detail::ExecutorHandle h = ex.handle();
  std::thread t([&] { ex.spin(); });
  while (!h.is_spinning()) {
    std::this_thread::yield();
  }
  ASSERT_EQ(f.ch.publish("a", 1), flux::Published::Ok);
  while (f.delivered == 0) {
    std::this_thread::yield();
  }
  h.stop();
  t.join();
  EXPECT_FALSE(h.is_spinning());
}

TEST_F(ExecutorTest, ARunFlagEndsTheSpin)
{
  flux::detail::Executor & ex = *ex_;
  std::atomic<bool> run{true};
  int passes = 0;
  ex.on_pass([&] {
    if (++passes == 2) {
      run = false;
    }
  });
  ex.spin(run, 1'000'000);
  EXPECT_EQ(passes, 2);
}

}  // namespace
