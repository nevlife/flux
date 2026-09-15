// The pull half of the ROS surface: peek(), take() and take_blocking() on a flux::ros
// Subscription. The callback half is covered by test_flux_executor; this file exists because
// nothing else in this package exercises a Subscription without an executor driving it.
#include "flux/ros/publisher.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <sys/resource.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <system_error>
#include <thread>

using namespace std::chrono_literals;

namespace
{

constexpr std::uint64_t kFingerprint = 0x9011000111ull;
constexpr std::uint32_t kSlotSize = 4096;
constexpr std::uint32_t kSlots = 4;

std::string uniq(const std::string & stem)
{
  return stem + std::to_string(::getpid());
}

std::uint8_t first_byte(const flux::FrameView & v)
{
  return v.valid() ? *static_cast<const std::uint8_t *>(v.data()) : 0u;
}

class PullSurface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("pull_surface_test");
  }
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }
  rclcpp::Node::SharedPtr node_;
};

}  // namespace

TEST_F(PullSurface, PeekReturnsTheNewestFrameRepeatedlyAndTakeConsumesOnce)
{
  const std::string topic = uniq("peek_repeat_");
  flux::ros::Publisher pub(*node_, topic, kFingerprint, kSlotSize, kSlots);
  flux::ros::Subscription sub(*node_, topic, kFingerprint);

  EXPECT_FALSE(sub.peek().valid()) << "nothing published yet";

  const std::uint8_t one[4] = {0x11, 0x11, 0x11, 0x11};
  ASSERT_EQ(pub.publish(one, sizeof(one)), flux::Published::Ok);

  EXPECT_EQ(first_byte(sub.peek()), 0x11);
  EXPECT_EQ(first_byte(sub.peek()), 0x11) << "peek consumed the frame";

  EXPECT_EQ(first_byte(sub.take()), 0x11);
  EXPECT_FALSE(sub.take().valid()) << "take handed the same frame out twice";
  EXPECT_EQ(first_byte(sub.peek()), 0x11) << "peek must still see the newest frame";
}

TEST_F(PullSurface, TakeBlockingReturnsEmptyOnTimeoutAndTheFrameWhenOneArrives)
{
  const std::string topic = uniq("take_blocking_");
  flux::ros::Publisher pub(*node_, topic, kFingerprint, kSlotSize, kSlots);
  flux::ros::Subscription sub(*node_, topic, kFingerprint);

  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(sub.take_blocking(50'000'000).valid()) << "no publisher wrote anything";
  EXPECT_GE(std::chrono::steady_clock::now() - t0, 40ms) << "it returned without parking";

  std::thread writer([&] {
    std::this_thread::sleep_for(100ms);
    const std::uint8_t v[4] = {0x22, 0x22, 0x22, 0x22};
    pub.publish(v, sizeof(v));
  });
  const flux::FrameView got = sub.take_blocking(2'000'000'000);
  writer.join();
  EXPECT_EQ(first_byte(got), 0x22) << "the park did not wake on the publish";
}

TEST_F(PullSurface, TakeFollowsPublishOrderAndCountsWhatTheRingLapped)
{
  const std::string topic = uniq("take_order_");
  flux::ros::Publisher pub(*node_, topic, kFingerprint, kSlotSize, kSlots);
  flux::QoS qos;
  qos.depth = kSlots;
  flux::ros::Subscription sub(*node_, topic, kFingerprint, {}, qos);

  for (std::uint8_t i = 1; i <= 3; ++i) {
    const std::uint8_t v[4] = {i, i, i, i};
    ASSERT_EQ(pub.publish(v, sizeof(v)), flux::Published::Ok);
  }
  EXPECT_EQ(first_byte(sub.take()), 1);
  EXPECT_EQ(first_byte(sub.take()), 2);
  EXPECT_EQ(first_byte(sub.take()), 3);
  EXPECT_FALSE(sub.take().valid());
  EXPECT_EQ(sub.lost(), 0u) << "nothing was lapped in a ring this size";
}

// docs/en/api.en.md, MemoryPolicy: a refused mlock is a std::system_error out of attach, not an
// unattached subscription that keeps retrying as if no publisher were up.
TEST_F(PullSurface, ARefusedMemoryPolicyThrowsInsteadOfLookingUnattached)
{
  struct rlimit saved;
  ASSERT_EQ(::getrlimit(RLIMIT_MEMLOCK, &saved), 0);
  if (saved.rlim_cur == RLIM_INFINITY && ::geteuid() == 0) {
    GTEST_SKIP() << "running as root with no memlock limit: the kernel refuses nothing to lower";
  }
  const std::string topic = uniq("mem_refused_");
  flux::ros::Publisher pub(*node_, topic, kFingerprint, kSlotSize, kSlots);

  struct rlimit tiny = saved;
  tiny.rlim_cur = 4096;  // one page, far below the segment
  ASSERT_EQ(::setrlimit(RLIMIT_MEMLOCK, &tiny), 0);
  flux::MemoryPolicy mem;
  mem.lock = true;
  bool threw = false;
  bool attached = false;
  try {
    flux::ros::Subscription sub(
      *node_, topic, kFingerprint, {}, flux::QoS{}, flux::Device::Cpu, mem);
    attached = sub.attached();
  } catch (const std::system_error &) {
    threw = true;
  }
  ASSERT_EQ(::setrlimit(RLIMIT_MEMLOCK, &saved), 0);  // before any assertion can leave it lowered

  EXPECT_TRUE(threw) << "the refusal came back as an " << (attached ? "attached" : "unattached")
                     << " subscription instead of a throw";
}
