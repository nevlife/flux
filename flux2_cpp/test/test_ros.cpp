#include "flux2/executor.hpp"
#include "flux2/publisher.hpp"
#include "flux2/subscription.hpp"
#include "flux2/topics.hpp"
#include "flux2/trace.h"

// Humble has no EventsExecutor, the other executor that hooks entities.
#if __has_include(<rclcpp/experimental/executors/events_executor/events_executor.hpp>)
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#endif
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>
#include <rcl/time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// Each endpoint_init call, recorded in place of libflux2_ffi's (an executable's definition comes
// first in the lookup the calls from libflux2_cpp make).
struct EndpointInit
{
  const void * endpoint;
  std::string node, key, signpost;
  std::uint64_t fingerprint;
  std::int32_t role, lang;
};
std::vector<EndpointInit> & endpoint_inits()
{
  static std::vector<EndpointInit> v;
  return v;
}

extern "C" void flux2_trace_endpoint_init(
  const void * endpoint, const char * node, const char * key, const char * signpost,
  std::uint64_t fingerprint, std::int32_t role, std::int32_t lang)
{
  endpoint_inits().push_back({endpoint, node, key, signpost, fingerprint, role, lang});
}

namespace
{

using namespace std::chrono_literals;

// Small, so a test fits a container's 64 MB /dev/shm.
constexpr std::uint32_t kSlot = 4096;
constexpr std::uint32_t kSlots = 4;

std::string topic(const char * name)
{
  return "/t" + std::to_string(getpid()) + "_" + name;
}

template <class P>
bool eventually(P done, std::chrono::milliseconds limit = 2000ms)
{
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!done()) {
    if (std::chrono::steady_clock::now() > end) return false;
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

std::string text(const flux2::FrameView & v)
{
  return std::string(static_cast<const char *>(v.data()), v.size());
}

class Ros : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("flux2_test");
    ex_.emplace();
  }

  // Spins on a thread until the test body returns.
  class Spinning
  {
  public:
    explicit Spinning(flux2::Executor & ex) : ex_(ex), t_([this] { ex_.spin(); }) {}
    Spinning(const Spinning &) = delete;
    Spinning & operator=(const Spinning &) = delete;
    Spinning(Spinning &&) = delete;
    Spinning & operator=(Spinning &&) = delete;
    ~Spinning()
    {
      ex_.cancel();
      t_.join();
    }

  private:
    flux2::Executor & ex_;
    std::thread t_;
  };

  std::shared_ptr<rclcpp::Node> node_;
  std::optional<flux2::Executor> ex_;
};

TEST_F(Ros, AFrameReachesTheCallback)
{
  flux2::Publisher pub(*node_, topic("cb"), flux2::kNoSchema, kSlot, kSlots);
  std::atomic<int> got{0};
  flux2::Subscription sub(
    *node_, topic("cb"), flux2::kNoSchema, 4, [&](const flux2::FrameView & v) {
      if (text(v) == "frame") ++got;
    });
  ex_->add(sub);
  const Spinning spin{*ex_};
  ASSERT_EQ(pub.publish("frame", 5), flux2::Published::Ok);
  EXPECT_TRUE(eventually([&] { return got == 1; }));
}

TEST_F(Ros, TopicsAreResolvedAgainstTheNode)
{
  auto in_ns = std::make_shared<rclcpp::Node>("n", "/ns" + std::to_string(getpid()));
  flux2::Publisher pub(*in_ns, "cam", flux2::kNoSchema, kSlot, kSlots);
  EXPECT_EQ(pub.topic_name(), "/ns" + std::to_string(getpid()) + "/cam");
  flux2::Subscription sub(*node_, pub.topic_name());
  EXPECT_EQ(sub.signpost_name(), pub.signpost_name());
}

TEST_F(Ros, EachEndpointIsTracedOnceWithItsNode)
{
  auto in_ns = std::make_shared<rclcpp::Node>("traced", "/ns" + std::to_string(getpid()));
  endpoint_inits().clear();
  flux2::Publisher pub(*in_ns, topic("traced"), 9, kSlot, kSlots);
  const flux2::Subscription sub(*in_ns, topic("traced"), 9);
  ASSERT_EQ(endpoint_inits().size(), 2U);
  for (const EndpointInit & e : endpoint_inits()) {
    EXPECT_EQ(e.node, in_ns->get_fully_qualified_name());
    EXPECT_EQ(e.key, pub.topic_name());
    EXPECT_EQ(e.signpost, pub.signpost_name());
    EXPECT_EQ(e.fingerprint, 9U);
    EXPECT_EQ(e.lang, FLUX2_TRACE_LANG_CPP);
  }
  EXPECT_EQ(endpoint_inits()[0].endpoint, &pub);
  EXPECT_EQ(endpoint_inits()[0].role, FLUX2_TRACE_ROLE_PUBLISHER);
  EXPECT_EQ(endpoint_inits()[1].endpoint, &sub);
  EXPECT_EQ(endpoint_inits()[1].role, FLUX2_TRACE_ROLE_SUBSCRIPTION);
  EXPECT_EQ(sub.endpoint(), &sub);
}

TEST_F(Ros, EndpointsAreListedWithTheirNode)
{
  flux2::Publisher pub(*node_, topic("listed"), 7, kSlot, kSlots);
  const flux2::Subscription sub(*node_, topic("listed"), 7);
  const std::vector<flux2::Topic> all = flux2::enumerate_topics();
  const auto it = std::find_if(all.begin(), all.end(), [&](const flux2::Topic & t) {
    return t.signpost == pub.signpost_name();
  });
  ASSERT_NE(it, all.end());
  EXPECT_EQ(it->key, pub.topic_name());
  EXPECT_TRUE(it->key_exact);
  EXPECT_EQ(it->fingerprint, 7U);
  ASSERT_EQ(it->endpoints.size(), 2U);
  for (const flux2::Endpoint & e : it->endpoints) {
    EXPECT_EQ(e.pid, static_cast<std::uint32_t>(getpid()));
    EXPECT_EQ(e.label, node_->get_fully_qualified_name());
  }
  const flux2::ChannelStats s = flux2::read_channel_stats(pub.signpost_name());
  EXPECT_TRUE(s.live);
  EXPECT_EQ(s.slot_size, kSlot);
  EXPECT_FALSE(flux2::read_channel_stats(pub.signpost_name() + "x").live);
}

TEST_F(Ros, TakeBlockingWaitsForALatePublisher)
{
  flux2::Subscription sub(*node_, topic("late"), flux2::kNoSchema, flux2::QoS().max_borrow(2));
  EXPECT_FALSE(sub.attached());
  EXPECT_FALSE(sub.take());
  std::optional<flux2::Publisher> pub;
  std::thread late([&] {
    std::this_thread::sleep_for(150ms);
    pub.emplace(*node_, topic("late"), flux2::kNoSchema, kSlot, kSlots);
    std::this_thread::sleep_for(150ms);
    (void)pub->publish("hi", 2);
  });
  const flux2::FrameView v = sub.take_blocking(2'000'000'000);
  late.join();
  ASSERT_TRUE(v);
  EXPECT_EQ(text(v), "hi");
  EXPECT_TRUE(sub.attached());

  // Attached, the publish itself ends the wait: a poll would show as a median of tens of ms.
  std::array<std::chrono::nanoseconds, 9> lat{};
  for (auto & l : lat) {
    std::chrono::steady_clock::time_point sent;
    std::thread later([&] {
      std::this_thread::sleep_for(20ms);
      sent = std::chrono::steady_clock::now();
      (void)pub->publish("w", 1);
    });
    const flux2::FrameView w = sub.take_blocking(2'000'000'000);
    const auto got = std::chrono::steady_clock::now();
    later.join();
    ASSERT_TRUE(w);
    l = got - sent;
  }
  std::sort(lat.begin(), lat.end());
  EXPECT_LT(lat[lat.size() / 2], 10ms);
}

// As rclcpp::wait_for_message: a shut down context ends the wait, attached or not.
TEST_F(Ros, TakeBlockingEndsWhenTheContextShutsDown)
{
  auto ctx = std::make_shared<rclcpp::Context>();
  ctx->init(0, nullptr);
  auto other = std::make_shared<rclcpp::Node>("flux2_ctx", rclcpp::NodeOptions().context(ctx));
  flux2::Publisher pub(*node_, topic("ctx_on"), flux2::kNoSchema, kSlot, kSlots);
  flux2::Subscription attached(*other, topic("ctx_on"));
  flux2::Subscription waiting(*other, topic("ctx_off"));
  ASSERT_TRUE(attached.attached());
  auto a = std::async(std::launch::async, [&] { return attached.take_blocking(); });
  auto w = std::async(std::launch::async, [&] { return waiting.take_blocking(3'000'000'000); });
  std::this_thread::sleep_for(150ms);
  ctx->shutdown("test");
  EXPECT_EQ(a.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(w.wait_for(1s), std::future_status::ready);
  // Ends the wait if the shutdown did not.
  (void)pub.publish("x", 1);
  EXPECT_FALSE(a.get());
  EXPECT_FALSE(w.get());
}

// GoogleTest before 1.12 (Humble) has no GTEST_FLAG_SET.
#ifndef GTEST_FLAG_SET
#define GTEST_FLAG_SET(name, value) (void)(::testing::GTEST_FLAG(name) = value)
#endif

// rclcpp's handler shuts the context down on SIGINT and SIGTERM.
TEST_F(Ros, ASignalEndsATakeBlockingLoop)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  for (const int sig : {SIGINT, SIGTERM}) {
    EXPECT_EXIT(
      {
        alarm(10);
        flux2::Subscription sub(*node_, topic("signal"));
        std::thread([sig] {
          std::this_thread::sleep_for(200ms);
          std::raise(sig);
        }).detach();
        while (rclcpp::ok()) (void)sub.take_blocking();
        std::exit(0);
      },
      ::testing::ExitedWithCode(0), "")
      << strsignal(sig);
  }
}

TEST_F(Ros, TakeBlockingIsEmptyAtOnceWhileMaxBorrowViewsAreHeld)
{
  flux2::Publisher pub(*node_, topic("held"), flux2::kNoSchema, kSlot, kSlots);
  flux2::Subscription sub(*node_, topic("held"), flux2::kNoSchema, flux2::QoS(4).max_borrow(1));
  ASSERT_EQ(pub.publish("a", 1), flux2::Published::Ok);
  ASSERT_EQ(pub.publish("b", 1), flux2::Published::Ok);
  const flux2::FrameView held = sub.take();
  ASSERT_TRUE(held);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(sub.take_blocking());
  EXPECT_FALSE(sub.take_blocking(2'000'000'000));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms);
}

TEST_F(Ros, LostCountsOverTheSubscriptionsLife)
{
  std::optional<flux2::Publisher> pub;
  pub.emplace(*node_, topic("lost"), flux2::kNoSchema, kSlot, kSlots);
  flux2::Subscription sub(*node_, topic("lost"), flux2::kNoSchema, flux2::QoS(1));
  for (int i = 0; i < 3; ++i) ASSERT_EQ(pub->publish("a", 1), flux2::Published::Ok);
  ASSERT_TRUE(sub.take());
  ASSERT_EQ(sub.lost(), 2U);
  pub.reset();
  ASSERT_TRUE(eventually([&] {
    (void)sub.take();
    return !sub.attached();
  }));
  EXPECT_EQ(sub.lost(), 2U);
  pub.emplace(*node_, topic("lost"), flux2::kNoSchema, kSlot, kSlots);
  ASSERT_TRUE(eventually([&] {
    (void)sub.take();
    return sub.attached();
  }));
  for (int i = 0; i < 3; ++i) ASSERT_EQ(pub->publish("b", 1), flux2::Published::Ok);
  ASSERT_TRUE(sub.take());
  EXPECT_EQ(sub.lost(), 4U);
}

TEST_F(Ros, ASubscriptionFollowsARestartedPublisher)
{
  std::optional<flux2::Publisher> pub;
  pub.emplace(*node_, topic("restart"), flux2::kNoSchema, kSlot, kSlots);
  std::atomic<int> got{0};
  flux2::Subscription sub(
    *node_, topic("restart"), flux2::kNoSchema, 4, [&](const flux2::FrameView &) { ++got; });
  const std::uint32_t first = sub.attach_generation();
  ex_->add(sub);
  const Spinning spin{*ex_};
  ASSERT_EQ(pub->publish("a", 1), flux2::Published::Ok);
  ASSERT_TRUE(eventually([&] { return got == 1; }));
  pub.reset();
  pub.emplace(*node_, topic("restart"), flux2::kNoSchema, kSlot, kSlots);
  // Republished until the re-attach catches one.
  EXPECT_TRUE(eventually([&] {
    (void)pub->publish("b", 1);
    return got >= 2;
  }));
  EXPECT_NE(sub.attach_generation(), first);
}

TEST_F(Ros, RosAndFluxShareOneSpin)
{
  flux2::Publisher pub(*node_, topic("mixed"), flux2::kNoSchema, kSlot, kSlots);
  std::atomic<int> frames{0};
  std::atomic<int> msgs{0};
  std::atomic<int> ticks{0};
  flux2::Subscription sub(
    *node_, topic("mixed"), flux2::kNoSchema, 4, [&](const flux2::FrameView &) { ++frames; });
  auto ros_sub = node_->create_subscription<std_msgs::msg::String>(
    topic("ros"), 10, [&](const std_msgs::msg::String &) { ++msgs; });
  auto ros_pub = node_->create_publisher<std_msgs::msg::String>(topic("ros"), 10);
  // Far shorter than the 100 ms rescan: only the timer deadline can wake for it in time.
  auto timer = node_->create_wall_timer(10ms, [&] { ++ticks; });
  ex_->add(sub);
  ex_->add_node(node_);
  const Spinning spin{*ex_};
  ASSERT_EQ(pub.publish("x", 1), flux2::Published::Ok);
  ros_pub->publish(std_msgs::msg::String());
  EXPECT_TRUE(eventually([&] { return frames == 1 && msgs == 1; }));
  const int before = ticks;
  std::this_thread::sleep_for(300ms);
  EXPECT_GE(ticks - before, 15);
}

TEST_F(Ros, OverloadedRosWorkDoesNotStarveFlux)
{
  flux2::Publisher pub(*node_, topic("starve"), flux2::kNoSchema, kSlot, kSlots);
  std::atomic<int> frames{0};
  flux2::Subscription sub(
    *node_, topic("starve"), flux2::kNoSchema, 1, [&](const flux2::FrameView &) { ++frames; });
  // Always behind: a 5 ms callback on a 1 ms timer keeps ROS work ready for the whole test.
  auto busy = node_->create_wall_timer(1ms, [] { std::this_thread::sleep_for(5ms); });
  ex_->add(sub);
  ex_->add_node(node_);
  const Spinning spin{*ex_};
  ASSERT_TRUE(eventually([&] {
    (void)pub.publish("x", 1);
    return sub.attached();
  }));
  frames = 0;
  for (int i = 0; i < 20; ++i) {
    ASSERT_EQ(pub.publish("x", 1), flux2::Published::Ok);
    std::this_thread::sleep_for(15ms);
  }
  // Each frame waits out at most one ROS callback, not a run of them.
  EXPECT_GE(frames.load(), 18);
}

TEST_F(Ros, OverloadedFluxWorkDoesNotStarveRos)
{
  flux2::Publisher pub(*node_, topic("busyflux"), flux2::kNoSchema, kSlot, kSlots);
  std::atomic<int> ticks{0};
  flux2::Subscription sub(
    *node_, topic("busyflux"), flux2::kNoSchema, 1,
    [](const flux2::FrameView &) { std::this_thread::sleep_for(20ms); });
  // Three cheap entities due every 10 ms: each turn between flux callbacks must run them all.
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  timers.reserve(3);
  for (int i = 0; i < 3; ++i) timers.push_back(node_->create_wall_timer(10ms, [&] { ++ticks; }));
  ex_->add(sub);
  ex_->add_node(node_);
  const Spinning spin{*ex_};
  ASSERT_TRUE(eventually([&] {
    (void)pub.publish("x", 1);
    return sub.attached();
  }));
  ticks = 0;
  const auto end = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < end) {
    (void)pub.publish("x", 1);
    std::this_thread::sleep_for(5ms);
  }
  // A flux callback every 20 ms leaves room for all three timers each time: about 150.
  EXPECT_GE(ticks.load(), 100);
}

TEST_F(Ros, ACallbackExceptionLeavesTheSpin)
{
  flux2::Publisher pub(*node_, topic("throw"), flux2::kNoSchema, kSlot, kSlots);
  flux2::Subscription sub(
    *node_, topic("throw"), flux2::kNoSchema, 4,
    [](const flux2::FrameView &) { throw std::logic_error("callback"); });
  ex_->add(sub);
  ASSERT_EQ(pub.publish("x", 1), flux2::Published::Ok);
  EXPECT_THROW(ex_->spin(), std::logic_error);
  EXPECT_FALSE(ex_->is_spinning());
}

TEST_F(Ros, FutureCompletionServesFluxAndSpinOnceIsRefused)
{
  flux2::Publisher pub(*node_, topic("once"), flux2::kNoSchema, kSlot, kSlots);
  std::promise<std::string> first;
  flux2::Subscription sub(
    *node_, topic("once"), flux2::kNoSchema, 4,
    [&](const flux2::FrameView & v) { first.set_value(text(v)); });
  ex_->add(sub);
  ASSERT_EQ(pub.publish("one", 3), flux2::Published::Ok);
  auto f = first.get_future();
  EXPECT_EQ(ex_->spin_until_future_complete(f, 2s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(f.get(), "one");
  EXPECT_THROW(ex_->spin_some(), std::runtime_error);
  EXPECT_THROW(ex_->spin_once(), std::runtime_error);
  // A future nothing completes runs out its timeout.
  std::promise<void> never;
  EXPECT_EQ(
    ex_->spin_until_future_complete(never.get_future(), 50ms), rclcpp::FutureReturnCode::TIMEOUT);
}

TEST_F(Ros, AZeroTimeoutFutureSpinRunsTheReadyWork)
{
  flux2::Publisher pub(*node_, topic("zero"), flux2::kNoSchema, kSlot, kSlots);
  std::promise<void> got;
  flux2::Subscription sub(
    *node_, topic("zero"), flux2::kNoSchema, 4, [&](const flux2::FrameView &) { got.set_value(); });
  ex_->add(sub);
  ASSERT_EQ(pub.publish("x", 1), flux2::Published::Ok);
  // As rclcpp: one pass of work before the timeout is looked at.
  EXPECT_EQ(
    ex_->spin_until_future_complete(got.get_future(), 0s), rclcpp::FutureReturnCode::SUCCESS);
}

TEST_F(Ros, ARosTimeTimerFollowsTheRosClock)
{
  const rclcpp::Clock::SharedPtr clock = node_->get_clock();
  rcl_clock_t * rcl = clock->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(rcl), RCL_RET_OK);
  rcl_time_point_value_t now = 1'000'000'000;
  const auto set = [&](rcl_time_point_value_t t) {
    const std::lock_guard<std::mutex> lk(clock->get_clock_mutex());
    ASSERT_EQ(rcl_set_ros_time_override(rcl, t), RCL_RET_OK);
  };
  set(now);
  std::atomic<int> ticks{0};
  auto timer = rclcpp::create_timer(node_, clock, 50ms, [&] { ++ticks; });
  ex_->add_node(node_);
  {
    const Spinning spin{*ex_};
    // As /clock at ten times real time: 10 s of ROS time, 200 periods.
    for (int i = 0; i < 1000; ++i) {
      now += 10'000'000;
      set(now);
      std::this_thread::sleep_for(1ms);
    }
  }
  EXPECT_GE(ticks.load(), 150);
}

TEST_F(Ros, AMovedFromPublisherRefusesToPublish)
{
  flux2::Publisher pub(*node_, topic("moved"), flux2::kNoSchema, kSlot, kSlots);
  flux2::Publisher moved = std::move(pub);
  // NOLINTNEXTLINE(bugprone-use-after-move): what a moved-from publisher does is the test
  EXPECT_EQ(pub.publish("x", 1), flux2::Published::InvalidArgument);
  EXPECT_EQ(moved.publish("x", 1), flux2::Published::Ok);
}

TEST_F(Ros, MisuseIsRefusedUpFront)
{
  flux2::Publisher pub(*node_, topic("misuse"), flux2::kNoSchema, kSlot, kSlots);
  try {
    flux2::Publisher second(*node_, topic("misuse"), flux2::kNoSchema, kSlot, kSlots);
    FAIL() << "a second publisher was accepted";
  } catch (const flux2::Error & e) {
    EXPECT_EQ(e.status(), FLUX2_PUBLISHER_EXISTS);
  }
  EXPECT_THROW(
    flux2::Subscription(
      *node_, topic("misuse"), flux2::kNoSchema, flux2::QoS(2).transient_local(3)),
    flux2::Error);
  flux2::Subscription pull(*node_, topic("misuse"));
  EXPECT_THROW(ex_->add(pull), std::invalid_argument);
}

TEST_F(Ros, ASecondThreadInOneSubscriptionIsRefused)
{
  flux2::Subscription sub(*node_, topic("two"));
  // With no publisher, take_blocking stays inside the subscription until its timeout.
  std::thread parked([&] { EXPECT_FALSE(sub.take_blocking(300'000'000)); });
  std::this_thread::sleep_for(100ms);
  EXPECT_THROW(sub.take(), std::logic_error);
  parked.join();
  EXPECT_FALSE(sub.take());
}

TEST_F(Ros, ASubscriptionAnExecutorServesRefusesTake)
{
  flux2::Publisher pub(*node_, topic("served"), flux2::kNoSchema, kSlot, kSlots);
  std::atomic<int> got{0};
  flux2::Subscription sub(
    *node_, topic("served"), flux2::kNoSchema, 4, [&](const auto &) { ++got; });
  ex_->add(sub);
  {
    const Spinning spin{*ex_};
    ASSERT_EQ(pub.publish("a", 1), flux2::Published::Ok);
    ASSERT_TRUE(eventually([&] { return got == 1; }));
  }
  EXPECT_THROW(sub.take(), std::logic_error);
  EXPECT_THROW(sub.take_blocking(0), std::logic_error);
}

#if __has_include(<rclcpp/experimental/executors/events_executor/events_executor.hpp>)
// Another executor hooks a node this one let go; dropping this one must leave those hooks alone.
TEST_F(Ros, ANodeMovedToAnotherExecutorKeepsItsHooks)
{
  std::atomic<int> got{0};
  auto sub = node_->create_subscription<std_msgs::msg::String>(
    topic("moved"), 10, [&](const std_msgs::msg::String &) { ++got; });
  auto pub = node_->create_publisher<std_msgs::msg::String>(topic("moved"), 10);
  ex_->add_node(node_);
  {
    const Spinning spin{*ex_};
    pub->publish(std_msgs::msg::String());
    ASSERT_TRUE(eventually([&] { return got == 1; }));
  }
  ex_->remove_node(node_);

  rclcpp::experimental::executors::EventsExecutor other;
  other.add_node(node_);
  std::thread t([&] { other.spin(); });
  pub->publish(std_msgs::msg::String());
  EXPECT_TRUE(eventually([&] { return got == 2; }));
  ex_.reset();
  pub->publish(std_msgs::msg::String());
  EXPECT_TRUE(eventually([&] { return got == 3; }));
  other.cancel();
  t.join();
}
#endif

}  // namespace
