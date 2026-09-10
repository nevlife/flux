#include "flux/discovery.hpp"
#include "flux/ros/executor.hpp"
#include "flux/ros/publisher.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>
#include <std_srvs/srv/empty.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{

constexpr std::uint64_t kFingerprint = 0xF10C0DEull;
constexpr std::uint32_t kSlotSize = 64 * 1024;  // 64 KiB bulk payload
constexpr std::uint32_t kSlots = 8;

// The frame's tag is its payload: the descriptor is derived from the byte count, so there is no
// field left to smuggle a sequence number through. A torn frame still shows as a byte that
// disagrees with the rest of the run.
std::uint8_t frame_tag(const flux::FrameView & v) noexcept
{
  return v.size() == 0 ? 0u : *static_cast<const std::uint8_t *>(v.data());
}

}  // namespace

// One publisher node emits, per tick, a large payload on a flux topic (zero-copy shm)
// and a small sequence id on a normal ROS topic. A separate subscriber node receives
// both, on the same executor. This is flux's intended shape -- the ROS topic carries the
// lightweight notification, flux carries the bulk data -- and it proves the two coexist
// inside ordinary rclcpp nodes without a transport swap.
//
// Both transports run on one flux::ros::Executor, which is now the only arrangement: a
// subscription no longer drives itself off a wall timer. Counters rather than the containers
// decide when to stop, because the callbacks run on the spin thread and the containers are only
// safe to read once that thread has joined.
TEST(RosCoexist, FluxAndRosTopicsDeliverTogether)
{
  rclcpp::init(0, nullptr);

  auto pub_node = std::make_shared<rclcpp::Node>("flux_pub_node");
  auto sub_node = std::make_shared<rclcpp::Node>("flux_sub_node");

  // normal ROS topic (the notification / baseline)
  auto ros_pub = pub_node->create_publisher<std_msgs::msg::UInt64>("/demo/notify", 10);
  std::vector<std::uint64_t> ros_ids;
  std::atomic<int> ros_seen{0};
  auto ros_sub = sub_node->create_subscription<std_msgs::msg::UInt64>(
    "/demo/notify", 10, [&](const std_msgs::msg::UInt64 & m) {
      ros_ids.push_back(m.data);
      ros_seen.fetch_add(1);
    });

  // flux topic (the bulk zero-copy payload). Publisher created first so the segment
  // exists before the subscription attaches.
  flux::ros::Publisher flux_pub(*pub_node, "/demo/bulk", kFingerprint, kSlotSize, kSlots);

  std::set<std::uint64_t> flux_ids;
  std::atomic<int> torn{0};
  std::atomic<int> flux_seen{0};
  flux::ros::Subscription flux_sub(
    *sub_node, "/demo/bulk", kFingerprint,
    [&](const flux::FrameView & v) {
      const auto * p = static_cast<const std::uint8_t *>(v.data());
      const auto tag = frame_tag(v);
      for (std::size_t i = 0; i < v.size(); ++i) {
        if (p[i] != tag) {
          torn.fetch_add(1);
          break;
        }
      }
      flux_ids.insert(frame_tag(v));
      flux_seen.fetch_add(1);
    },
    flux::QoS{});

  // publish loop driven by a wall timer on the publisher node
  std::uint64_t seq = 0;
  std::vector<std::byte> buf(kSlotSize);
  auto pub_timer = pub_node->create_wall_timer(2ms, [&] {
    ++seq;
    std::memset(buf.data(), static_cast<int>(seq & 0xFF), buf.size());
    flux_pub.publish(buf.data(), buf.size());
    std_msgs::msg::UInt64 m;
    m.data = seq;
    ros_pub->publish(m);
  });

  flux::ros::Executor exec;
  exec.add(flux_sub);
  exec.add_ros_node(pub_node);
  exec.add_ros_node(sub_node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 5'000'000); });

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline &&
         (ros_seen.load() < 25 || flux_seen.load() < 25)) {
    std::this_thread::sleep_for(10ms);
  }
  run.store(false);
  exec.stop();
  spinner.join();  // the containers below are this thread's only after the join

  rclcpp::shutdown();

  EXPECT_EQ(torn.load(), 0) << "flux delivered a torn frame";
  EXPECT_GT(ros_ids.size(), 0u) << "normal ROS topic delivered nothing";
  EXPECT_GT(flux_ids.size(), 0u) << "flux topic delivered nothing";
  for (std::uint64_t id : flux_ids) {  // every flux id must be one actually published
    EXPECT_GE(id, 1u);
    EXPECT_LE(id, seq);
  }
}

// X-020. Both ends report the domain they resolved, and both resolve it the one way core does.
// Previously a core caller took "0" while these wrappers took ROS_DOMAIN_ID, so on a host with
// a domain set the two never met and neither said anything. The name is the thing that decides
// whether they meet, so the check ends there rather than at the string.
TEST(RosDomain, EndpointsReportTheDomainTheyResolvedAndCoreAgrees)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_domain_node");

  flux::ros::Publisher pub(*node, "/demo/domain", kFingerprint, kSlotSize, kSlots);
  flux::ros::Subscription sub(*node, "/demo/domain", kFingerprint, [](const flux::FrameView &) {});

  const std::string core = flux::process_domain();
  EXPECT_EQ(pub.domain(), core);
  EXPECT_EQ(sub.domain(), core);
  EXPECT_EQ(pub.segment_name(), sub.segment_name());
  EXPECT_EQ(pub.segment_name(), flux::signpost_name("/demo/domain", kFingerprint, core));

  rclcpp::shutdown();
}

// 0-copy publish through the ROS wrapper (docs/en/copy_model.en.md): the bytes the subscriber sees
// were written into the slot itself, and an uncommitted handle publishes nothing.
TEST(RosPublisher, LoanFillsTheSlotInPlace)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_loan_node");

  flux::ros::Publisher pub(*node, "/demo/loan", kFingerprint, kSlotSize, kSlots);

  std::vector<std::uint64_t> ids;
  std::atomic<int> torn{0};
  // No poll period: driven through the executor hooks here, so each publish is observed at a
  // known point.
  flux::ros::Subscription sub(*node, "/demo/loan", kFingerprint, [&](const flux::FrameView & v) {
    const auto * p = static_cast<const std::uint8_t *>(v.data());
    const auto tag = frame_tag(v);
    for (std::size_t i = 0; i < v.size(); ++i) {
      if (p[i] != tag) {
        torn.fetch_add(1);
        break;
      }
    }
    ids.push_back(frame_tag(v));
  });
  ASSERT_TRUE(sub.attached());

  constexpr std::size_t kBytes = 4096;
  for (std::uint64_t seq = 1; seq <= 3; ++seq) {
    flux::WriteSlot w = pub.loan();
    ASSERT_TRUE(w) << "loan found no free slot";
    ASSERT_GE(w.capacity(), kBytes);
    std::memset(w.data(), static_cast<int>(seq & 0xFF), kBytes);  // straight into the segment
    EXPECT_EQ(w.commit(kBytes), flux::Published::Ok);
    EXPECT_EQ(sub.deliver(), 1);
  }

  const std::uint64_t dropped_before = pub.dropped();
  {
    flux::WriteSlot w = pub.loan();
    EXPECT_TRUE(w);
  }  // destroyed uncommitted: the claim is reverted
  EXPECT_EQ(sub.deliver(), 0) << "an uncommitted loan published a frame";
  EXPECT_EQ(pub.dropped(), dropped_before) << "an aborted loan is not a dropped frame";

  rclcpp::shutdown();

  EXPECT_EQ(torn.load(), 0) << "loan delivered a torn frame";
  EXPECT_EQ(ids, (std::vector<std::uint64_t>{1, 2, 3}));
}

// A publisher that goes away without a successor leaves the subscription mapped to a dead
// stream. The subscription must drop that mapping (attached() -> false) instead of pinning it,
// and a returning publisher is picked up by the ordinary lazy attach.
TEST(RosCoexist, SubscriptionDropsADeadPublisherMapping)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux_orphan_node");

  auto pub =
    std::make_unique<flux::ros::Publisher>(*node, "/demo/orphan", kFingerprint, kSlotSize, kSlots);
  flux::ros::Subscription sub(*node, "/demo/orphan", kFingerprint, [](const flux::FrameView &) {});
  ASSERT_TRUE(sub.attached());

  pub.reset();
  for (int i = 0; i < 64 && sub.attached(); ++i) sub.deliver();
  EXPECT_FALSE(sub.attached());

  pub =
    std::make_unique<flux::ros::Publisher>(*node, "/demo/orphan", kFingerprint, kSlotSize, kSlots);
  EXPECT_TRUE(sub.attach());

  rclcpp::shutdown();
}

// A service request must reach the executor when it arrives, not when the tick next comes round.
// The tick here is 3 s and the deadline 500 ms, so this passes only because the service and the
// client are hooked to the readiness eventfd like subscriptions are. Without those hooks the
// request sits in the queue until the tick expires and both waits below time out.
//
// An action is one rclcpp::Waitable, and waitables are hooked on the same pass, so this covers
// the arrangement that made the gap worth closing: a node that answers goals while flux carries
// the bulk frames.
TEST(RosCoexist, ServiceAndClientWakeTheExecutorOnArrival)
{
  rclcpp::init(0, nullptr);
  // The node is stripped of its parameter services and parameter-event publisher on purpose.
  // pump_ros() drains everything rclcpp reports as ready, so ANY bridged entity waking the ring
  // would also carry the service request out with it -- and a stock node's internal subscriptions
  // do exactly that. With them gone, the service's own hook is the only thing that can end the
  // wait, which is what this test is here to prove.
  auto node = std::make_shared<rclcpp::Node>(
    "flux_service_node",
    rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));

  std::atomic<int> served{0};
  auto service = node->create_service<std_srvs::srv::Empty>(
    "/demo/svc", [&served](
                   const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>) { served.fetch_add(1); });
  auto client = node->create_client<std_srvs::srv::Empty>("/demo/svc");

  flux::ros::Executor exec;
  exec.add_ros_node(node);

  std::atomic<bool> run{true};
  std::thread spinner([&] { exec.spin(run, 3'000'000'000); });

  ASSERT_TRUE(client->wait_for_service(2s)) << "service never came up";
  const auto sent = std::chrono::steady_clock::now();
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  const bool answered = future.wait_for(500ms) == std::future_status::ready;
  const auto elapsed = std::chrono::steady_clock::now() - sent;

  run.store(false);
  exec.stop();
  spinner.join();
  rclcpp::shutdown();

  EXPECT_TRUE(answered) << "the response did not arrive within 500 ms of a 3 s tick: the service "
                           "or the client is not hooked to the readiness eventfd";
  EXPECT_EQ(served.load(), 1);
  EXPECT_LT(elapsed, 500ms);
}
