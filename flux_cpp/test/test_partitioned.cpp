#include "flux/partitioned_executor.hpp"
#include "flux/publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using namespace std::chrono_literals;

constexpr std::uint32_t kSlot = 4096;
constexpr std::uint32_t kSlots = 4;

std::string topic(const char * name)
{
  return "/p" + std::to_string(getpid()) + "_" + name;
}

std::size_t threads()
{
  const std::filesystem::directory_iterator tasks("/proc/self/task");
  return static_cast<std::size_t>(std::distance(begin(tasks), end(tasks)));
}

template <class P>
bool eventually(P done, std::chrono::milliseconds limit = 3000ms)
{
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!done()) {
    if (std::chrono::steady_clock::now() > end) return false;
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

class Partitioned : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("flux_part_test");
    a_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    b_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  }

  flux::Publisher pub(const char * name)
  {
    return flux::Publisher(*node_, topic(name), flux::kNoSchema, kSlot, kSlots);
  }

  // Spins on a thread until the test body returns, then rethrows nothing: failures are asserted.
  class Spinning
  {
  public:
    explicit Spinning(flux::PartitionedExecutor & ex)
    : ex_(ex), t_([this] {
        try {
          ex_.spin();
        } catch (const std::exception & e) {
          error_ = e.what();
        }
      })
    {
    }
    Spinning(const Spinning &) = delete;
    Spinning & operator=(const Spinning &) = delete;
    Spinning(Spinning &&) = delete;
    Spinning & operator=(Spinning &&) = delete;
    ~Spinning() { stop(); }
    std::string stop()
    {
      if (t_.joinable()) {
        ex_.cancel();
        t_.join();
      }
      return error_;
    }

  private:
    flux::PartitionedExecutor & ex_;
    std::string error_;
    std::thread t_;
  };

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::CallbackGroup::SharedPtr a_;
  rclcpp::CallbackGroup::SharedPtr b_;
};

TEST_F(Partitioned, ABlockedGroupDoesNotStallAnother)
{
  auto pa = pub("blocka");
  auto pb = pub("blockb");
  std::atomic<bool> release{false};
  std::atomic<int> a_in{0};
  std::atomic<int> b_got{0};
  flux::Subscription sa(*node_, topic("blocka"), flux::kNoSchema, 4, [&](const flux::FrameView &) {
    ++a_in;
    while (!release) std::this_thread::sleep_for(1ms);
  });
  flux::Subscription sb(
    *node_, topic("blockb"), flux::kNoSchema, 4, [&](const flux::FrameView &) { ++b_got; });
  flux::PartitionedExecutor ex;
  ex.add(sa, a_);
  ex.add(sb, b_);
  ex.add_node(node_);
  Spinning spin(ex);
  ASSERT_EQ(pa.publish("x", 1), flux::Published::Ok);
  ASSERT_TRUE(eventually([&] { return a_in == 1; }));
  ASSERT_EQ(pb.publish("y", 1), flux::Published::Ok);
  EXPECT_TRUE(eventually([&] { return b_got == 1; }));
  release = true;
  EXPECT_EQ(spin.stop(), "");
}

TEST_F(Partitioned, AGroupRunsItsFluxAndRosCallbacksOnItsOwnThread)
{
  auto pa = pub("threads");
  std::atomic<std::thread::id> flux_thread{};
  std::atomic<std::thread::id> ros_thread{};
  std::atomic<std::thread::id> default_thread{};
  flux::Subscription sa(*node_, topic("threads"), flux::kNoSchema, 4, [&](const flux::FrameView &) {
    flux_thread = std::this_thread::get_id();
  });
  rclcpp::SubscriptionOptions in_a;
  in_a.callback_group = a_;
  auto ros_a = node_->create_subscription<std_msgs::msg::String>(
    topic("ros_a"), 10,
    [&](const std_msgs::msg::String &) { ros_thread = std::this_thread::get_id(); }, in_a);
  // The node's default group is served too, on a thread of its own.
  auto timer = node_->create_wall_timer(5ms, [&] { default_thread = std::this_thread::get_id(); });
  auto ros_pub = node_->create_publisher<std_msgs::msg::String>(topic("ros_a"), 10);
  flux::PartitionedExecutor ex;
  ex.add(sa, a_);
  ex.add_node(node_);
  Spinning spin(ex);
  const std::thread::id none{};
  EXPECT_TRUE(eventually([&] {
    (void)pa.publish("x", 1);
    ros_pub->publish(std_msgs::msg::String());
    return flux_thread.load() != none && ros_thread.load() != none && default_thread.load() != none;
  }));
  EXPECT_EQ(flux_thread.load(), ros_thread.load());
  EXPECT_NE(flux_thread.load(), default_thread.load());
  EXPECT_EQ(spin.stop(), "");
}

TEST_F(Partitioned, AGroupCreatedWhileSpinningIsServed)
{
  flux::PartitionedExecutor ex;
  ex.add_node(node_);
  Spinning spin(ex);
  std::this_thread::sleep_for(50ms);
  auto late = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
  std::atomic<int> ticks{0};
  auto timer = node_->create_wall_timer(5ms, [&] { ++ticks; }, late);
  EXPECT_TRUE(eventually([&] { return ticks > 0; }));
  EXPECT_EQ(spin.stop(), "");
}

TEST_F(Partitioned, AThreadStartHookRunsOnTheGroupsThread)
{
  auto pa = pub("hook");
  std::atomic<std::thread::id> hook_thread{};
  std::atomic<std::thread::id> cb_thread{};
  flux::Subscription sa(*node_, topic("hook"), flux::kNoSchema, 4, [&](const flux::FrameView &) {
    cb_thread = std::this_thread::get_id();
  });
  flux::PartitionedExecutor ex;
  ex.add(sa, a_);
  ex.add_node(node_);
  ex.on_thread_start(a_, [&] { hook_thread = std::this_thread::get_id(); });
  EXPECT_THROW(ex.on_thread_start(a_, [] {}), std::invalid_argument);
  Spinning spin(ex);
  ASSERT_TRUE(eventually([&] {
    (void)pa.publish("x", 1);
    return cb_thread.load() != std::thread::id{};
  }));
  EXPECT_EQ(hook_thread.load(), cb_thread.load());
  EXPECT_EQ(spin.stop(), "");
}

TEST_F(Partitioned, TheThreadOfADroppedGroupEnds)
{
  flux::PartitionedExecutor ex;
  ex.add_node(node_);
  Spinning spin(ex);
  std::this_thread::sleep_for(200ms);
  const std::size_t before = threads();
  std::atomic<int> ticks{0};
  for (int i = 0; i < 5; ++i) {
    ticks = 0;
    auto g = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
    auto timer = node_->create_wall_timer(1ms, [&] { ++ticks; }, g);
    ASSERT_TRUE(eventually([&] { return ticks > 0; }));
  }
  EXPECT_TRUE(eventually([&] { return threads() <= before; }));
  EXPECT_EQ(spin.stop(), "");
}

TEST_F(Partitioned, AThreadStartHookKeepsItsGroup)
{
  std::atomic<int> ran{0};
  flux::PartitionedExecutor ex;
  ex.add_node(node_);
  {
    auto g = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
    ex.on_thread_start(g, [&] { ++ran; });
  }
  Spinning spin(ex);
  EXPECT_TRUE(eventually([&] { return ran == 1; }));
  EXPECT_EQ(spin.stop(), "");
}

// A signal handler calling cancel() can interrupt spin() while it holds its lock, as here.
TEST_F(Partitioned, CancelTakesNoLockSpinMayHold)
{
  flux::PartitionedExecutor ex;
  struct CancelsWhileAdded
  {
    flux::PartitionedExecutor * ex;
    rclcpp::SubscriptionBase::SharedPtr getSubscriber()
    {
      ex->cancel();
      return nullptr;
    }
  };
  CancelsWhileAdded a{&ex};
  CancelsWhileAdded b{&ex};
  ex.add_sync_group(a, b);
}

TEST_F(Partitioned, AnErrorOnOneThreadStopsAllAndReachesSpin)
{
  auto pa = pub("throw");
  flux::Subscription sa(*node_, topic("throw"), flux::kNoSchema, 4, [](const flux::FrameView &) {
    throw std::logic_error("in group a");
  });
  flux::PartitionedExecutor ex;
  ex.add(sa, a_);
  ex.add_node(node_);
  ASSERT_EQ(pa.publish("x", 1), flux::Published::Ok);
  EXPECT_THROW(ex.spin(), std::logic_error);
  ex.cancel();
  ex.spin();
}

TEST_F(Partitioned, MisuseIsRefused)
{
  flux::Subscription pull(*node_, topic("misuse"));
  flux::PartitionedExecutor ex;
  EXPECT_THROW(ex.add(pull, a_), std::invalid_argument);

  flux::Subscription sa(
    *node_, topic("misuse"), flux::kNoSchema, 4, [](const flux::FrameView &) {});
  ex.add(sa, a_);
  EXPECT_THROW(ex.add(sa, b_), std::invalid_argument);
  EXPECT_THROW(ex.spin(), std::invalid_argument);  // a_ belongs to no added node

  flux::PartitionedExecutor reentrant;
  auto r = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
  reentrant.add_node(node_);
  EXPECT_THROW(reentrant.spin(), std::invalid_argument);
}

}  // namespace
