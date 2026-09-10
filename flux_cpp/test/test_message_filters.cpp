#include "flux/ros/executor.hpp"
#include "flux/ros/message_filters/subscriber.hpp"
#include "flux/ros/partitioned_executor.hpp"
#include "flux/ros/publisher.hpp"

#include <message_filters/sync_policies/approximate_time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <gtest/gtest.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

// flux topics inside a message_filters graph. The claim under test is that a flux topic and a
// ROS topic can be synchronized in one Synchronizer without the payload being copied: what the
// filter queues is a StampedFrame, which holds the borrow.

using namespace std::chrono_literals;
using sensor_msgs::flux_msg::Image;

namespace mf = ::message_filters;
namespace fmf = flux::ros::message_filters;

namespace
{
using Frame = fmf::StampedFrame<Image>;

constexpr std::uint32_t kSlotSize = 4096;
constexpr std::uint32_t kSlots = 16;

std::string uniq(const std::string & base)
{
  return base + "_" + std::to_string(::getpid());
}

// A frame whose payload is one repeated byte, so a torn or stale read shows without a descriptor
// alongside. The stamp is the synchronization key.
void publish_at(flux::ros::Publisher & pub, std::int32_t sec, std::uint8_t tag)
{
  Image::Builder b = Image::build__(pub);
  ASSERT_TRUE(b);
  b.set__header__stamp(sec, 0);
  b.set__header__frame_id("cam");
  b.set__width(8);
  b.set__height(1);
  b.set__encoding("mono8");
  b.set__step(8);
  auto d = b.alloc__data(8);
  for (std::size_t i = 0; i < d.size(); ++i) d[i] = tag;
  ASSERT_EQ(b.commit__(), flux::Published::Ok);
}

bool wait_until(const std::function<bool()> & done, std::chrono::milliseconds timeout = 5s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) return true;
    std::this_thread::sleep_for(2ms);
  }
  return done();
}
}  // namespace

// Two flux topics into one ExactTime synchronizer. The matched callback must see both frames and
// must be able to read their payloads, which is only true if the borrows outlived the delivery
// that produced them.
TEST(MessageFilters, SynchronizesTwoFluxTopics)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mf_two");
  const std::string left_topic = uniq("/mf/left");
  const std::string right_topic = uniq("/mf/right");

  flux::QoS qos;
  qos.depth = 4;
  qos.max_borrow = 16;  // inputs x queue_size, plus the one being delivered

  fmf::Subscriber<Image> left(*node, left_topic, qos);
  fmf::Subscriber<Image> right(*node, right_topic, qos);

  std::atomic<int> matched{0};
  std::atomic<int> left_tag{0};
  std::atomic<int> right_tag{0};
  mf::TimeSynchronizer<Frame, Frame> sync(left, right, 10);
  // std::bind rather than a bare lambda: Signal9 binds nine placeholders to whatever it is given
  // (signal9.h:274), and a bind expression discards the arguments it was not given placeholders
  // for. This is the form the ROS documentation uses.
  sync.registerCallback(std::bind(
    [&](const std::shared_ptr<const Frame> & a, const std::shared_ptr<const Frame> & b) {
      left_tag.store(static_cast<const std::uint8_t *>(a->view().data().data())[0]);
      right_tag.store(static_cast<const std::uint8_t *>(b->view().data().data())[0]);
      matched.fetch_add(1);
    },
    std::placeholders::_1, std::placeholders::_2));

  flux::ros::Publisher lpub(*node, left_topic, Image::kFingerprint, kSlotSize, kSlots);
  flux::ros::Publisher rpub(*node, right_topic, Image::kFingerprint, kSlotSize, kSlots);

  flux::ros::Executor ex;
  ex.add(left);
  ex.add(right);
  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 10'000'000); });

  for (std::int32_t t = 1; t <= 5 && matched.load() == 0; ++t) {
    publish_at(lpub, t, 0xA0);
    publish_at(rpub, t, 0xB0);
    std::this_thread::sleep_for(20ms);
  }
  const bool ok = wait_until([&] { return matched.load() > 0; });

  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_TRUE(ok) << "no pair was matched; forwarded left=" << left.forwarded()
                  << " right=" << right.forwarded();
  EXPECT_EQ(left_tag.load(), 0xA0);
  EXPECT_EQ(right_tag.load(), 0xB0);
  EXPECT_EQ(left.unreadable(), 0u);
  EXPECT_EQ(right.unreadable(), 0u);
}

// The headline: one synchronizer whose inputs are a flux topic and an ordinary ROS topic. Both
// are serviced by the same flux::ros::Executor thread, which is what the policy's lock requires.
TEST(MessageFilters, SynchronizesAFluxTopicWithARosTopic)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mf_mixed");
  const std::string flux_topic = uniq("/mf/mixed_flux");
  const std::string ros_topic = uniq("/mf/mixed_ros");

  flux::QoS qos;
  qos.depth = 4;
  qos.max_borrow = 16;

  fmf::Subscriber<Image> flux_in(*node, flux_topic, qos);
  mf::Subscriber<sensor_msgs::msg::Image> ros_in(node, ros_topic);

  std::atomic<int> matched{0};
  using Policy = mf::sync_policies::ApproximateTime<Frame, sensor_msgs::msg::Image>;
  mf::Synchronizer<Policy> sync(Policy(10), flux_in, ros_in);
  sync.registerCallback(std::bind(
    [&](
      const std::shared_ptr<const Frame> &,
      const std::shared_ptr<const sensor_msgs::msg::Image> &) { matched.fetch_add(1); },
    std::placeholders::_1, std::placeholders::_2));

  flux::ros::Publisher fpub(*node, flux_topic, Image::kFingerprint, kSlotSize, kSlots);
  auto rpub = node->create_publisher<sensor_msgs::msg::Image>(ros_topic, 10);

  flux::ros::Executor ex;
  ex.add(flux_in);
  ex.add_ros_node(node);
  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 10'000'000); });

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  for (std::int32_t t = 1; matched.load() == 0 && std::chrono::steady_clock::now() < deadline;
       ++t) {
    publish_at(fpub, t, 0xC0);
    sensor_msgs::msg::Image m;
    m.header.stamp.sec = t;
    m.header.stamp.nanosec = 0;
    m.width = 8;
    m.height = 1;
    m.step = 8;
    m.encoding = "mono8";
    m.data.assign(8, 0xD0);
    rpub->publish(m);
    std::this_thread::sleep_for(20ms);
  }
  const bool ok = matched.load() > 0;

  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_TRUE(ok) << "flux and ROS never paired; flux forwarded=" << flux_in.forwarded();
}

// A subscriber declared before its node exists, which is why the default constructor is there.
TEST(MessageFilters, SubscribesLate)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mf_late");
  const std::string topic = uniq("/mf/late");

  fmf::Subscriber<Image> sub;
  EXPECT_FALSE(sub.subscribed());
  EXPECT_FALSE(sub.attach()) << "an unsubscribed source must not claim to be attached";
  EXPECT_EQ(sub.deliver(), 0);
  EXPECT_EQ(sub.channel(), nullptr);

  sub.subscribe(*node, topic);
  EXPECT_TRUE(sub.subscribed());

  std::atomic<int> got{0};
  sub.registerCallback([&](const std::shared_ptr<const Frame> &) { got.fetch_add(1); });

  flux::ros::Publisher pub(*node, topic, Image::kFingerprint, kSlotSize, kSlots);
  flux::ros::Executor ex;
  ex.add(sub);
  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 10'000'000); });

  for (std::int32_t t = 1; t <= 5 && got.load() == 0; ++t) {
    publish_at(pub, t, 0xE0);
    std::this_thread::sleep_for(20ms);
  }
  const bool ok = wait_until([&] { return got.load() > 0; });

  ex.stop();
  spinner.join();

  sub.unsubscribe();
  EXPECT_FALSE(sub.subscribed());
  rclcpp::shutdown();

  EXPECT_TRUE(ok);
  EXPECT_GT(sub.forwarded(), 0u);
}

// ---- the same-thread rule, checked rather than documented ----
//
// The sync policies run the matched callback holding their own std::mutex, which has no priority
// inheritance, so inputs serviced by two threads couple those threads' priorities. Under
// flux::ros::Executor that cannot happen -- one thread. Under PartitionedExecutor the assignment
// is the user's, and these are the cases spin() now judges.

// Type alone is not a verdict: several rejections in this executor are std::invalid_argument, so
// a test that only names the type passes on the wrong refusal. Each case below pins the reason.
namespace
{
std::string refusal(const std::function<void()> & f)
{
  try {
    f();
  } catch (const std::invalid_argument & e) {
    return e.what();
  } catch (...) {
    return "<wrong exception type>";
  }
  return "<no exception>";
}
}  // namespace

TEST(SyncGroup, InputsInDifferentGroupsAreRefused)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mf_split");

  auto ga = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto gb = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  fmf::Subscriber<Image> left(*node, uniq("/mf/split_l"));
  fmf::Subscriber<Image> right(*node, uniq("/mf/split_r"));

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.add(left, ga);
  ex.add(right, gb);
  ex.add_sync_group(left, right);

  std::atomic<bool> run{true};
  const std::string why = refusal([&] { ex.spin(run, 20'000'000); });
  rclcpp::shutdown();

  EXPECT_NE(why.find("different callback groups"), std::string::npos) << why;
}

TEST(SyncGroup, InputsInOneGroupAreAccepted)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mf_together");

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  fmf::Subscriber<Image> left(*node, uniq("/mf/together_l"));
  fmf::Subscriber<Image> right(*node, uniq("/mf/together_r"));

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.add(left, g);
  ex.add(right, g);
  ex.add_sync_group(left, right);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });
  std::this_thread::sleep_for(100ms);
  const std::size_t unplaced = ex.unplaced_sync_inputs();
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_EQ(unplaced, 0u);
}

// The case the check exists for. A flux input and a DDS input are serviced by different
// machinery -- dispatch() and pump_ros() -- and only a shared callback group puts those on one
// thread. Nothing about the graph says so, which is why it has to be declared and checked.
TEST(SyncGroup, AMixedGraphSplitAcrossGroupsIsRefused)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mf_mixed_split");

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  fmf::Subscriber<Image> flux_in(*node, uniq("/mf/mixsplit_flux"));
  // No options, so this lands in the node's default group -- not `g`.
  mf::Subscriber<sensor_msgs::msg::Image> ros_in(node, uniq("/mf/mixsplit_ros"));

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.add(flux_in, g);
  ex.add_sync_group(flux_in, ros_in);

  std::atomic<bool> run{true};
  const std::string why = refusal([&] { ex.spin(run, 20'000'000); });
  rclcpp::shutdown();

  EXPECT_NE(why.find("different callback groups"), std::string::npos) << why;
}

TEST(SyncGroup, AMixedGraphInOneGroupIsAccepted)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mf_mixed_together");

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = g;

  fmf::Subscriber<Image> flux_in(*node, uniq("/mf/mixok_flux"));
  mf::Subscriber<sensor_msgs::msg::Image> ros_in(
    node, uniq("/mf/mixok_ros"), rmw_qos_profile_default, opts);

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.add(flux_in, g);
  ex.add_sync_group(flux_in, ros_in);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });
  std::this_thread::sleep_for(100ms);
  const std::size_t unplaced = ex.unplaced_sync_inputs();
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_EQ(unplaced, 0u);
}

// A flux input declared as a synchronizer input but never assigned is not an unknown: no child
// would drive it, so the partner it is being synchronized with would wait forever.
TEST(SyncGroup, AnUnassignedFluxInputIsRefused)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mf_unassigned");

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  fmf::Subscriber<Image> left(*node, uniq("/mf/unassigned_l"));
  fmf::Subscriber<Image> right(*node, uniq("/mf/unassigned_r"));

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);
  ex.add(left, g);  // right is never assigned
  ex.add_sync_group(left, right);

  std::atomic<bool> run{true};
  const std::string why = refusal([&] { ex.spin(run, 20'000'000); });
  rclcpp::shutdown();

  EXPECT_NE(why.find("never assigned to a callback group"), std::string::npos) << why;
}

// An input this executor cannot place is reported, not judged and not silently dropped: the
// subscription belongs to a node nobody handed over, so which thread serves it is not knowable
// from here.
TEST(SyncGroup, AnUnplaceableInputIsCountedNotJudged)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_mf_unplaceable");
  auto other = std::make_shared<rclcpp::Node>("flux_mf_elsewhere");

  auto g = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  fmf::Subscriber<Image> flux_in(*node, uniq("/mf/unplaceable_flux"));
  mf::Subscriber<sensor_msgs::msg::Image> ros_in(other, uniq("/mf/unplaceable_ros"));

  flux::ros::PartitionedExecutor ex;
  ex.add_ros_node(node);  // `other` is deliberately not registered
  ex.add(flux_in, g);
  ex.add_sync_group(flux_in, ros_in);

  std::atomic<bool> run{true};
  std::thread spinner([&] { ex.spin(run, 20'000'000); });
  std::this_thread::sleep_for(100ms);
  const std::size_t unplaced = ex.unplaced_sync_inputs();
  ex.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_EQ(unplaced, 1u);
}
