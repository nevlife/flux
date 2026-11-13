#include "flux/executor.hpp"
#include "flux/publisher.hpp"
#include "flux/subscription.hpp"
#include "flux/topics.hpp"
#include "flux/trace.h"

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
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
#include <cstdio>
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

// Each endpoint_init call, recorded in place of libflux_ffi's (an executable's definition comes
// first in the lookup the calls from libflux_cpp make).
struct EndpointInit
{
  const void * endpoint;
  std::string node, key, signpost;
  std::uint64_t fingerprint;
  std::int32_t role, lang;
};
auto endpoint_inits() -> std::vector<EndpointInit> &
{
  static std::vector<EndpointInit> v;
  return v;
}

extern "C" void flux_trace_endpoint_init(
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

auto topic(const char * name) -> std::string
{
  return "/t" + std::to_string(getpid()) + "_" + name;
}

template <class P>
auto eventually(P done, std::chrono::milliseconds limit = 2000ms) -> bool
{
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!done()) {
    if (std::chrono::steady_clock::now() > end) {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

auto text(const flux::FrameView & v) -> std::string
{
  return std::string(static_cast<const char *>(v.data()), v.size());
}

class Ros : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // Left to the default context's destructor at exit, the shutdown would use rclcpp's
  // thread_local `g_contexts_in_shutdown` after it is destroyed, and crash now and then.
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("flux_test");
    ex_.emplace();
  }

  // Spins on a thread until the test body returns.
  class Spinning
  {
  public:
    explicit Spinning(flux::Executor & ex) : ex_(ex), t_([this] { ex_.spin(); }) {}
    Spinning(const Spinning &) = delete;
    auto operator=(const Spinning &) -> Spinning & = delete;
    Spinning(Spinning &&) = delete;
    auto operator=(Spinning &&) -> Spinning & = delete;
    ~Spinning()
    {
      ex_.cancel();
      t_.join();
    }

  private:
    flux::Executor & ex_;
    std::thread t_;
  };

  std::shared_ptr<rclcpp::Node> node_;
  std::optional<flux::Executor> ex_;
};

TEST_F(Ros, AFrameReachesTheCallback)
{
  flux::Publisher pub(*node_, topic("cb"), flux::kNoSchema, kSlot, kSlots);
  std::atomic<int> got{0};
  flux::Subscription sub(*node_, topic("cb"), flux::kNoSchema, 4, [&](const flux::FrameView & v) {
    if (text(v) == "frame") {
      ++got;
    }
  });
  ex_->add(sub);
  const Spinning spin{*ex_};
  ASSERT_EQ(pub.publish("frame", 5), flux::Published::Ok);
  EXPECT_TRUE(eventually([&] { return got == 1; }));
}

// Seven frames after the kept one go through a ring of four around its slot.
TEST_F(Ros, AFrameMovedOutOfTheCallbackKeepsItsSlot)
{
  flux::Publisher pub(*node_, topic("kept"), flux::kNoSchema, kSlot, kSlots);
  std::optional<flux::FrameView> kept;
  int seen = 0;
  flux::Subscription sub(
    *node_, topic("kept"), flux::kNoSchema, flux::QoS(8).max_borrow(2), [&](flux::FrameView && v) {
      ++seen;
      if (!kept) {
        kept = std::move(v);
      }
    });
  for (const char c : std::string("abcdefgh")) {
    ASSERT_EQ(pub.publish(&c, 1), flux::Published::Ok);
    ASSERT_TRUE(sub.deliver_one());
  }
  EXPECT_EQ(seen, 8);
  ASSERT_TRUE(kept);
  EXPECT_EQ(text(*kept), "a");
}

TEST_F(Ros, NoFrameIsDeliveredWhileMaxBorrowFramesAreKept)
{
  flux::Publisher pub(*node_, topic("full"), flux::kNoSchema, kSlot, kSlots);
  std::vector<flux::FrameView> kept;
  flux::Subscription sub(
    *node_, topic("full"), flux::kNoSchema, flux::QoS(8).max_borrow(2),
    [&](flux::FrameView && v) { kept.push_back(std::move(v)); });
  for (const char c : std::string("abc")) {
    ASSERT_EQ(pub.publish(&c, 1), flux::Published::Ok);
  }
  ASSERT_TRUE(sub.deliver_one());
  ASSERT_TRUE(sub.deliver_one());
  EXPECT_FALSE(sub.deliver_one());
  EXPECT_EQ(kept.size(), 2U);
  kept.erase(kept.begin());
  ASSERT_TRUE(sub.deliver_one());
  ASSERT_EQ(kept.size(), 2U);
  EXPECT_EQ(text(kept.back()), "c");
}

TEST_F(Ros, APublisherWithFewerSlotsThanRequiredIsRefused)
{
  flux::Publisher pub(*node_, topic("few"), flux::kNoSchema, kSlot, kSlots);
  flux::Subscription sub(*node_, topic("few"));
  sub.require_publisher_slots(kSlots);
  try {
    sub.require_publisher_slots(kSlots + 1);
    FAIL() << "attached to " << kSlots << " slots where " << kSlots + 1 << " are required";
  } catch (const flux::Error & e) {
    EXPECT_EQ(e.status(), FLUX_INVALID_ARGUMENT);
  }
}

TEST_F(Ros, ASubscriptionInAGroupIsServedWithItsNode)
{
  flux::Publisher pub(*node_, topic("member"), flux::kNoSchema, kSlot, kSlots);
  std::atomic<int> got{0};
  flux::Subscription sub(
    *node_, topic("member"), flux::kNoSchema, 4, [&](const flux::FrameView &) { ++got; });
  sub.add_to_callback_group(*node_);
  // Made after the spin starts, but it outlives the executor serving it, as `sub` does.
  std::atomic<int> late{0};
  std::optional<flux::Subscription> later;
  ex_->add_node(node_);
  const Spinning spin{*ex_};
  ASSERT_EQ(pub.publish("x", 1), flux::Published::Ok);
  EXPECT_TRUE(eventually([&] { return got == 1; }));

  // Found by the next rescan.
  later.emplace(
    *node_, topic("member"), flux::kNoSchema, 4, [&](const flux::FrameView &) { ++late; });
  later->add_to_callback_group(*node_);
  EXPECT_TRUE(eventually([&] {
    (void)pub.publish("x", 1);
    return late > 0;
  }));
}

auto logged() -> std::vector<std::string> &
{
  static std::vector<std::string> lines;
  return lines;
}

TEST_F(Ros, AnRclcppExecutorServingAFluxSubscriptionSaysSoOnce)
{
  flux::Subscription sub(
    *node_, topic("stock"), flux::kNoSchema, 4, [](const flux::FrameView &) {});
  sub.add_to_callback_group(*node_);
  logged().clear();
  const auto previous = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler([](
                                       const rcutils_log_location_t *, int, const char *,
                                       rcutils_time_point_value_t, const char * format,
                                       va_list * args) {
    std::array<char, 512> line{};
    std::vsnprintf(line.data(), line.size(), format, *args);
    logged().emplace_back(line.data());
  });
  rclcpp::executors::SingleThreadedExecutor stock;
  stock.add_node(node_);
  stock.spin_some(10ms);
  stock.spin_some(10ms);
  rcutils_logging_set_output_handler(previous);
  const auto said = std::count_if(logged().begin(), logged().end(), [](const std::string & l) {
    return l.find(topic("stock")) != std::string::npos &&
           l.find("no frame of it is delivered") != std::string::npos;
  });
  EXPECT_EQ(said, 1);
}

TEST_F(Ros, TopicsAreResolvedAgainstTheNode)
{
  auto in_ns = std::make_shared<rclcpp::Node>("n", "/ns" + std::to_string(getpid()));
  flux::Publisher pub(*in_ns, "cam", flux::kNoSchema, kSlot, kSlots);
  EXPECT_EQ(pub.topic_name(), "/ns" + std::to_string(getpid()) + "/cam");
  flux::Subscription sub(*node_, pub.topic_name());
  EXPECT_EQ(sub.signpost_name(), pub.signpost_name());
}

TEST_F(Ros, EachEndpointIsTracedOnceWithItsNode)
{
  auto in_ns = std::make_shared<rclcpp::Node>("traced", "/ns" + std::to_string(getpid()));
  endpoint_inits().clear();
  flux::Publisher pub(*in_ns, topic("traced"), 9, kSlot, kSlots);
  const flux::Subscription sub(*in_ns, topic("traced"), 9);
  ASSERT_EQ(endpoint_inits().size(), 2U);
  for (const EndpointInit & e : endpoint_inits()) {
    EXPECT_EQ(e.node, in_ns->get_fully_qualified_name());
    EXPECT_EQ(e.key, pub.topic_name());
    EXPECT_EQ(e.signpost, pub.signpost_name());
    EXPECT_EQ(e.fingerprint, 9U);
    EXPECT_EQ(e.lang, FLUX_TRACE_LANG_CPP);
  }
  EXPECT_EQ(endpoint_inits()[0].endpoint, &pub);
  EXPECT_EQ(endpoint_inits()[0].role, FLUX_TRACE_ROLE_PUBLISHER);
  EXPECT_EQ(endpoint_inits()[1].endpoint, &sub);
  EXPECT_EQ(endpoint_inits()[1].role, FLUX_TRACE_ROLE_SUBSCRIPTION);
  EXPECT_EQ(sub.endpoint(), &sub);
}

TEST_F(Ros, EndpointsAreListedWithTheirNode)
{
  flux::Publisher pub(*node_, topic("listed"), 7, kSlot, kSlots);
  const flux::Subscription sub(*node_, topic("listed"), 7);
  const std::vector<flux::Topic> all = flux::enumerate_topics();
  const auto it = std::find_if(all.begin(), all.end(), [&](const flux::Topic & t) {
    return t.signpost == pub.signpost_name();
  });
  ASSERT_NE(it, all.end());
  EXPECT_EQ(it->key, pub.topic_name());
  EXPECT_TRUE(it->key_exact);
  EXPECT_EQ(it->fingerprint, 7U);
  ASSERT_EQ(it->endpoints.size(), 2U);
  for (const flux::Endpoint & e : it->endpoints) {
    EXPECT_EQ(e.pid, static_cast<std::uint32_t>(getpid()));
    EXPECT_EQ(e.label, node_->get_fully_qualified_name());
  }
  const flux::ChannelStats s = flux::read_channel_stats(pub.signpost_name());
  EXPECT_TRUE(s.live);
  EXPECT_EQ(s.slot_size, kSlot);
  EXPECT_FALSE(flux::read_channel_stats(pub.signpost_name() + "x").live);
}

TEST_F(Ros, TakeBlockingWaitsForALatePublisher)
{
  flux::Subscription sub(*node_, topic("late"), flux::kNoSchema, flux::QoS().max_borrow(2));
  EXPECT_FALSE(sub.attached());
  EXPECT_FALSE(sub.take());
  std::optional<flux::Publisher> pub;
  std::thread late([&] {
    std::this_thread::sleep_for(150ms);
    pub.emplace(*node_, topic("late"), flux::kNoSchema, kSlot, kSlots);
    std::this_thread::sleep_for(150ms);
    (void)pub->publish("hi", 2);
  });
  const flux::FrameView v = sub.take_blocking(2'000'000'000);
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
    const flux::FrameView w = sub.take_blocking(2'000'000'000);
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
  auto other = std::make_shared<rclcpp::Node>("flux_ctx", rclcpp::NodeOptions().context(ctx));
  flux::Publisher pub(*node_, topic("ctx_on"), flux::kNoSchema, kSlot, kSlots);
  flux::Subscription attached(*other, topic("ctx_on"));
  flux::Subscription waiting(*other, topic("ctx_off"));
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

// rclcpp's handler shuts the context down on SIGINT and SIGTERM.
TEST_F(Ros, ASignalEndsATakeBlockingLoop)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  for (const int sig : {SIGINT, SIGTERM}) {
    EXPECT_EXIT(
      {
        alarm(10);
        flux::Subscription sub(*node_, topic("signal"));
        std::thread([sig] {
          std::this_thread::sleep_for(200ms);
          std::raise(sig);
        }).detach();
        while (rclcpp::ok()) {
          (void)sub.take_blocking();
        }
        std::exit(0);
      },
      ::testing::ExitedWithCode(0), "")
      << strsignal(sig);
  }
}

TEST_F(Ros, TakeBlockingIsEmptyAtOnceWhileMaxBorrowViewsAreHeld)
{
  flux::Publisher pub(*node_, topic("held"), flux::kNoSchema, kSlot, kSlots);
  flux::Subscription sub(*node_, topic("held"), flux::kNoSchema, flux::QoS(4).max_borrow(1));
  ASSERT_EQ(pub.publish("a", 1), flux::Published::Ok);
  ASSERT_EQ(pub.publish("b", 1), flux::Published::Ok);
  const flux::FrameView held = sub.take();
  ASSERT_TRUE(held);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(sub.take_blocking());
  EXPECT_FALSE(sub.take_blocking(2'000'000'000));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms);
}

TEST_F(Ros, SubscriberCountFollowsTheSubscribingProcesses)
{
  flux::Publisher pub(*node_, topic("count"), flux::kNoSchema, kSlot, kSlots);
  EXPECT_EQ(pub.subscriber_count(), 0U);
  std::optional<flux::Subscription> a;
  a.emplace(*node_, topic("count"));
  EXPECT_EQ(pub.subscriber_count(), 1U);
  flux::Subscription b(*node_, topic("count"));
  EXPECT_EQ(pub.subscriber_count(), 1U);
}

TEST_F(Ros, LostCountsOverTheSubscriptionsLife)
{
  std::optional<flux::Publisher> pub;
  pub.emplace(*node_, topic("lost"), flux::kNoSchema, kSlot, kSlots);
  flux::Subscription sub(*node_, topic("lost"), flux::kNoSchema, flux::QoS(1));
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(pub->publish("a", 1), flux::Published::Ok);
  }
  ASSERT_TRUE(sub.take());
  ASSERT_EQ(sub.lost(), 2U);
  pub.reset();
  ASSERT_TRUE(eventually([&] {
    (void)sub.take();
    return !sub.attached();
  }));
  EXPECT_EQ(sub.lost(), 2U);
  pub.emplace(*node_, topic("lost"), flux::kNoSchema, kSlot, kSlots);
  ASSERT_TRUE(eventually([&] {
    (void)sub.take();
    return sub.attached();
  }));
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(pub->publish("b", 1), flux::Published::Ok);
  }
  ASSERT_TRUE(sub.take());
  EXPECT_EQ(sub.lost(), 4U);
}

TEST_F(Ros, ASubscriptionFollowsARestartedPublisher)
{
  std::optional<flux::Publisher> pub;
  pub.emplace(*node_, topic("restart"), flux::kNoSchema, kSlot, kSlots);
  std::atomic<int> got{0};
  flux::Subscription sub(
    *node_, topic("restart"), flux::kNoSchema, 4, [&](const flux::FrameView &) { ++got; });
  const std::uint32_t first = sub.attach_generation();
  ex_->add(sub);
  const Spinning spin{*ex_};
  ASSERT_EQ(pub->publish("a", 1), flux::Published::Ok);
  ASSERT_TRUE(eventually([&] { return got == 1; }));
  pub.reset();
  pub.emplace(*node_, topic("restart"), flux::kNoSchema, kSlot, kSlots);
  // Republished until the re-attach catches one.
  EXPECT_TRUE(eventually([&] {
    (void)pub->publish("b", 1);
    return got >= 2;
  }));
  EXPECT_NE(sub.attach_generation(), first);
}

TEST_F(Ros, RosAndFluxShareOneSpin)
{
  flux::Publisher pub(*node_, topic("mixed"), flux::kNoSchema, kSlot, kSlots);
  std::atomic<int> frames{0};
  std::atomic<int> msgs{0};
  std::atomic<int> ticks{0};
  flux::Subscription sub(
    *node_, topic("mixed"), flux::kNoSchema, 4, [&](const flux::FrameView &) { ++frames; });
  auto ros_sub = node_->create_subscription<std_msgs::msg::String>(
    topic("ros"), 10, [&](const std_msgs::msg::String &) { ++msgs; });
  auto ros_pub = node_->create_publisher<std_msgs::msg::String>(topic("ros"), 10);
  // Far shorter than the 100 ms rescan: only the timer deadline can wake for it in time.
  auto timer = node_->create_wall_timer(10ms, [&] { ++ticks; });
  ex_->add(sub);
  ex_->add_node(node_);
  const Spinning spin{*ex_};
  ASSERT_EQ(pub.publish("x", 1), flux::Published::Ok);
  ros_pub->publish(std_msgs::msg::String());
  EXPECT_TRUE(eventually([&] { return frames == 1 && msgs == 1; }));
  const int before = ticks;
  std::this_thread::sleep_for(300ms);
  EXPECT_GE(ticks - before, 15);
}

TEST_F(Ros, OverloadedRosWorkDoesNotStarveFlux)
{
  flux::Publisher pub(*node_, topic("starve"), flux::kNoSchema, kSlot, kSlots);
  std::atomic<int> frames{0};
  flux::Subscription sub(
    *node_, topic("starve"), flux::kNoSchema, 1, [&](const flux::FrameView &) { ++frames; });
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
    ASSERT_EQ(pub.publish("x", 1), flux::Published::Ok);
    std::this_thread::sleep_for(15ms);
  }
  // Each frame waits out at most one ROS callback, not a run of them.
  EXPECT_GE(frames.load(), 18);
}

TEST_F(Ros, OverloadedFluxWorkDoesNotStarveRos)
{
  flux::Publisher pub(*node_, topic("busyflux"), flux::kNoSchema, kSlot, kSlots);
  std::atomic<int> ticks{0};
  flux::Subscription sub(
    *node_, topic("busyflux"), flux::kNoSchema, 1,
    [](const flux::FrameView &) { std::this_thread::sleep_for(20ms); });
  // Three cheap entities due every 10 ms: each turn between flux callbacks must run them all.
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  timers.reserve(3);
  for (int i = 0; i < 3; ++i) {
    timers.push_back(node_->create_wall_timer(10ms, [&] { ++ticks; }));
  }
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
  flux::Publisher pub(*node_, topic("throw"), flux::kNoSchema, kSlot, kSlots);
  flux::Subscription sub(*node_, topic("throw"), flux::kNoSchema, 4, [](const flux::FrameView &) {
    throw std::logic_error("callback");
  });
  ex_->add(sub);
  ASSERT_EQ(pub.publish("x", 1), flux::Published::Ok);
  EXPECT_THROW(ex_->spin(), std::logic_error);
  EXPECT_FALSE(ex_->is_spinning());
}

TEST_F(Ros, FutureCompletionServesFluxAndSpinOnceIsRefused)
{
  flux::Publisher pub(*node_, topic("once"), flux::kNoSchema, kSlot, kSlots);
  std::promise<std::string> first;
  flux::Subscription sub(*node_, topic("once"), flux::kNoSchema, 4, [&](const flux::FrameView & v) {
    first.set_value(text(v));
  });
  ex_->add(sub);
  ASSERT_EQ(pub.publish("one", 3), flux::Published::Ok);
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
  flux::Publisher pub(*node_, topic("zero"), flux::kNoSchema, kSlot, kSlots);
  std::promise<void> got;
  flux::Subscription sub(
    *node_, topic("zero"), flux::kNoSchema, 4, [&](const flux::FrameView &) { got.set_value(); });
  ex_->add(sub);
  ASSERT_EQ(pub.publish("x", 1), flux::Published::Ok);
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
  flux::Publisher pub(*node_, topic("moved"), flux::kNoSchema, kSlot, kSlots);
  flux::Publisher moved = std::move(pub);
  // NOLINTNEXTLINE(bugprone-use-after-move): what a moved-from publisher does is the test
  EXPECT_EQ(pub.publish("x", 1), flux::Published::InvalidArgument);
  EXPECT_EQ(moved.publish("x", 1), flux::Published::Ok);
}

TEST_F(Ros, MisuseIsRefusedUpFront)
{
  flux::Publisher pub(*node_, topic("misuse"), flux::kNoSchema, kSlot, kSlots);
  try {
    flux::Publisher second(*node_, topic("misuse"), flux::kNoSchema, kSlot, kSlots);
    FAIL() << "a second publisher was accepted";
  } catch (const flux::Error & e) {
    EXPECT_EQ(e.status(), FLUX_PUBLISHER_EXISTS);
  }
  EXPECT_THROW(
    flux::Subscription(*node_, topic("misuse"), flux::kNoSchema, flux::QoS(2).transient_local(3)),
    flux::Error);
  flux::Subscription pull(*node_, topic("misuse"));
  EXPECT_THROW(ex_->add(pull), std::invalid_argument);
}

TEST_F(Ros, ASecondThreadInOneSubscriptionIsRefused)
{
  flux::Subscription sub(*node_, topic("two"));
  // With no publisher, take_blocking stays inside the subscription until its timeout.
  std::thread parked([&] { EXPECT_FALSE(sub.take_blocking(300'000'000)); });
  std::this_thread::sleep_for(100ms);
  EXPECT_THROW(sub.take(), std::logic_error);
  parked.join();
  EXPECT_FALSE(sub.take());
}

TEST_F(Ros, ASubscriptionAnExecutorServesRefusesTake)
{
  flux::Publisher pub(*node_, topic("served"), flux::kNoSchema, kSlot, kSlots);
  std::atomic<int> got{0};
  flux::Subscription sub(*node_, topic("served"), flux::kNoSchema, 4, [&](const auto &) { ++got; });
  ex_->add(sub);
  {
    const Spinning spin{*ex_};
    ASSERT_EQ(pub.publish("a", 1), flux::Published::Ok);
    ASSERT_TRUE(eventually([&] { return got == 1; }));
  }
  EXPECT_THROW(sub.take(), std::logic_error);
  EXPECT_THROW(sub.take_blocking(0), std::logic_error);
}

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

}  // namespace
