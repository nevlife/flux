#include "flux/executor.hpp"
#include "flux/message_filters/subscriber.hpp"
#include "flux/partitioned_executor.hpp"
#include "flux/publisher.hpp"
#include "flux_test/flux/stamped.hpp"

#include <message_filters/cache.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/temperature.hpp>

#include <gtest/gtest.h>
#include <rclcpp/version.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using namespace std::chrono_literals;

using Stamped = flux_test::flux_msg::Stamped;

using In = flux::message_filters::Subscriber<Stamped>;
using Msg = In::Message;
using MsgPtr = std::shared_ptr<const Msg>;
// Jazzy's synchronizer calls back with nine arguments, which only a bind expression absorbs.
using std::placeholders::_1;
using std::placeholders::_2;

std::string topic(const char * name)
{
  return "/m" + std::to_string(getpid()) + "_" + name;
}

flux::Published send(flux::Publisher & p, std::int32_t sec, std::uint8_t payload)
{
  auto b = Stamped::build__(p);
  b.set__header__stamp(sec, 0);
  b.set__payload(payload);
  return b.commit__();
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

// rclcpp's message_filters took an rmw profile up to Jazzy and an rclcpp::QoS after.
auto ros_qos()
{
#if RCLCPP_VERSION_MAJOR >= 29
  return rclcpp::QoS(10);
#else
  return rmw_qos_profile_default;
#endif
}

class Filters : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  void SetUp() override { node_ = std::make_shared<rclcpp::Node>("flux_mf_test"); }

  flux::Publisher pub(const char * name, std::uint32_t slots = 8)
  {
    return flux::Publisher(*node_, topic(name), Stamped::kFingerprint, 64, slots);
  }

  // Queued frames hold slots: the borrow limit covers the synchronizer's queue.
  static flux::QoS qos() { return flux::QoS(8).max_borrow(8); }

  std::shared_ptr<rclcpp::Node> node_;
};

class Spinner
{
public:
  explicit Spinner(std::function<void()> spin, std::function<void()> stop)
  : stop_(std::move(stop)), t_(std::move(spin))
  {
  }
  Spinner(const Spinner &) = delete;
  Spinner & operator=(const Spinner &) = delete;
  Spinner(Spinner &&) = delete;
  Spinner & operator=(Spinner &&) = delete;
  ~Spinner()
  {
    stop_();
    t_.join();
  }

private:
  std::function<void()> stop_;
  std::thread t_;
};

TEST_F(Filters, TwoFluxTopicsArePairedByStamp)
{
  auto pa = pub("a");
  auto pb = pub("b");
  In a(*node_, topic("a"), 4, qos());
  In b(*node_, topic("b"), 4, qos());
  ::message_filters::TimeSynchronizer<Msg, Msg> sync(4);
  sync.connectInput(a, b);
  std::mutex m;
  std::vector<std::pair<int, int>> pairs;
  const std::function<void(const MsgPtr &, const MsgPtr &)> on_pair =
    [&](const MsgPtr & x, const MsgPtr & y) {
      const std::lock_guard<std::mutex> lk(m);
      pairs.emplace_back(x->view().payload(), y->view().payload());
    };
  sync.registerCallback(std::bind(on_pair, _1, _2));
  flux::Executor ex;
  ex.add(a);
  ex.add(b);
  const Spinner s([&] { ex.spin(); }, [&] { ex.cancel(); });
  ASSERT_EQ(send(pa, 1, 0xA0), flux::Published::Ok);
  ASSERT_EQ(send(pb, 2, 0xB2), flux::Published::Ok);
  ASSERT_EQ(send(pb, 1, 0xB0), flux::Published::Ok);
  EXPECT_TRUE(eventually([&] {
    const std::lock_guard<std::mutex> lk(m);
    return !pairs.empty();
  }));
  const std::lock_guard<std::mutex> lk(m);
  EXPECT_EQ(pairs.front(), std::make_pair(0xA0, 0xB0));
  EXPECT_EQ(a.unreadable() + b.unreadable(), 0U);
}

TEST_F(Filters, AFrameWhoseHeaderDoesNotReadIsCountedNotForwarded)
{
  auto pa = pub("bad");
  In a(*node_, topic("bad"), 4, qos());
  ASSERT_EQ(pa.publish("abc", 3), flux::Published::Ok);
  EXPECT_TRUE(a.deliver_one());
  EXPECT_EQ(a.unreadable(), 1U);
  EXPECT_EQ(a.forwarded(), 0U);
  ASSERT_EQ(send(pa, 1, 1), flux::Published::Ok);
  EXPECT_TRUE(a.deliver_one());
  EXPECT_EQ(a.forwarded(), 1U);
}

// tf2_ros::MessageFilter drops every message whose header names no frame.
TEST_F(Filters, TheHeaderCarriesTheFrameId)
{
  auto pa = pub("frame");
  In a(*node_, topic("frame"), 4, qos());
  std::string frame_id;
  a.registerCallback(std::function<void(const MsgPtr &)>(
    [&](const MsgPtr & msg) { frame_id = msg->header.frame_id; }));
  auto b = Stamped::build__(pa);
  b.set__header__stamp(1, 0);
  b.set__header__frame_id("camera");
  ASSERT_EQ(b.commit__(), flux::Published::Ok);
  ASSERT_TRUE(a.deliver_one());
  EXPECT_EQ(frame_id, "camera");
}

// rclcpp::Time cannot hold it, so it is counted as unreadable, as in Python, and the spin goes on.
TEST_F(Filters, ANegativeStampIsCountedNotForwarded)
{
  auto pa = pub("neg");
  In a(*node_, topic("neg"), 4, qos());
  ::message_filters::Cache<Msg> cache(a, 4);
  ASSERT_EQ(send(pa, -1, 1), flux::Published::Ok);
  EXPECT_TRUE(a.deliver_one());
  EXPECT_EQ(a.unreadable(), 1U);
  ASSERT_EQ(send(pa, 2, 2), flux::Published::Ok);
  EXPECT_TRUE(a.deliver_one());
  EXPECT_EQ(a.forwarded(), 1U);
  EXPECT_EQ(cache.getLatestTime().seconds(), 2.0);
}

TEST_F(Filters, AQueuedFrameOutlivesTheDeliveryThatQueuedIt)
{
  auto pa = pub("keep");
  In a(*node_, topic("keep"), 4, qos());
  std::vector<std::shared_ptr<const Msg>> kept;
  a.registerCallback(std::function<void(const std::shared_ptr<const Msg> &)>(
    [&](const std::shared_ptr<const Msg> & msg) {
      if (kept.size() < 3) kept.push_back(msg);
    }));
  for (std::uint8_t i = 0; i < 20; ++i) {
    ASSERT_EQ(send(pa, i, i), flux::Published::Ok);
    ASSERT_TRUE(a.deliver_one());
  }
  ASSERT_EQ(kept.size(), 3U);
  for (std::uint8_t i = 0; i < 3; ++i) EXPECT_EQ(kept[i]->view().payload(), i);
}

TEST_F(Filters, ACacheWithTheDefaultQoSKeepsUp)
{
  auto pa = pub("cache", 16);
  In a(*node_, topic("cache"), 10);
  ::message_filters::Cache<Msg> cache(a, 10);
  for (std::uint8_t i = 0; i < 30; ++i) {
    ASSERT_EQ(send(pa, i, i), flux::Published::Ok);
    ASSERT_TRUE(a.deliver_one()) << "stalled at frame " << int{i};
  }
  EXPECT_EQ(cache.getLatestTime().seconds(), 29.0);
}

TEST_F(Filters, APublisherWithNoMoreSlotsThanTheQueueIsRefused)
{
  auto pa = pub("few");
  try {
    In a(*node_, topic("few"), 8);
    a.attach();
    FAIL() << "attached to 8 slots for a queue of 8";
  } catch (const flux::Error & e) {
    EXPECT_EQ(e.status(), FLUX_INVALID_ARGUMENT);
  }
}

TEST_F(Filters, AFluxTopicPairsWithARosTopicOnOneExecutor)
{
  auto pa = pub("mixed");
  In a(*node_, topic("mixed"), 4, qos());
  ::message_filters::Subscriber<sensor_msgs::msg::Temperature> r(node_, topic("temp"), ros_qos());
  ::message_filters::TimeSynchronizer<Msg, sensor_msgs::msg::Temperature> sync(4);
  sync.connectInput(a, r);
  std::atomic<int> paired{0};
  const std::function<void(const MsgPtr &, const sensor_msgs::msg::Temperature::ConstSharedPtr &)>
    on_pair = [&](const auto &, const auto &) { ++paired; };
  sync.registerCallback(std::bind(on_pair, _1, _2));
  auto temp = node_->create_publisher<sensor_msgs::msg::Temperature>(topic("temp"), 10);
  flux::Executor ex;
  ex.add(a);
  ex.add_node(node_);
  const Spinner s([&] { ex.spin(); }, [&] { ex.cancel(); });
  EXPECT_TRUE(eventually([&] {
    (void)send(pa, 5, 1);
    sensor_msgs::msg::Temperature t;
    t.header.stamp.sec = 5;
    temp->publish(t);
    std::this_thread::sleep_for(10ms);
    return paired > 0;
  }));
}

TEST_F(Filters, APartitionedSynchronizerKeepsItsInputsInOneGroup)
{
  auto pa = pub("pa");
  auto pb = pub("pb");
  auto g = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto other = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  In a(*node_, topic("pa"), 4, qos());
  In b(*node_, topic("pb"), 4, qos());
  {
    flux::PartitionedExecutor split;
    split.add(a, g);
    split.add(b, other);
    split.add_node(node_);
    split.add_sync_group(a, b);
    EXPECT_THROW(split.spin(), std::invalid_argument);
  }
  {
    flux::PartitionedExecutor unassigned;
    unassigned.add(a, g);
    unassigned.add_node(node_);
    unassigned.add_sync_group(a, b);
    EXPECT_THROW(unassigned.spin(), std::invalid_argument);
  }
  ::message_filters::TimeSynchronizer<Msg, Msg> sync(4);
  sync.connectInput(a, b);
  std::atomic<int> paired{0};
  const std::function<void(const MsgPtr &, const MsgPtr &)> on_pair =
    [&](const auto &, const auto &) { ++paired; };
  sync.registerCallback(std::bind(on_pair, _1, _2));
  flux::PartitionedExecutor one;
  one.add(a, g);
  one.add(b, g);
  one.add_node(node_);
  one.add_sync_group(a, b);
  const Spinner s([&] { one.spin(); }, [&] { one.cancel(); });
  EXPECT_TRUE(eventually([&] {
    (void)send(pa, 7, 1);
    (void)send(pb, 7, 2);
    std::this_thread::sleep_for(10ms);
    return paired > 0;
  }));
}

}  // namespace
