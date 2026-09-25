// The pull half of the ROS surface: peek(), take() and take_blocking() on a flux::ros
// Subscription. The callback half is covered by test_flux_executor; this file exists because
// nothing else in this package exercises a Subscription without an executor driving it.
#include "flux/owner.hpp"
#include "flux/ros/publisher.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <dirent.h>
#include <gtest/gtest.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

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

// A shape known only at run time (a tensor's sizes) publishes through the pointer form, as on
// flux::Channel; a literal shape takes the compile-time checked one.
TEST_F(PullSurface, APublisherTakesARuntimeShapeAndALiteralOne)
{
  const std::string topic = uniq("runtime_shape_");
  flux::ros::Publisher pub(*node_, topic, kFingerprint, kSlotSize, kSlots);
  flux::ros::Subscription sub(*node_, topic, kFingerprint);

  const float vals[6] = {1, 2, 3, 4, 5, 6};
  const std::vector<std::uint64_t> sizes = {2, 3};
  ASSERT_EQ(pub.publish(vals, flux::DType::F32, sizes.data(), sizes.size()), flux::Published::Ok);
  flux::FrameView v = sub.take();
  ASSERT_TRUE(v.valid());
  EXPECT_EQ(v.meta().ndim, 2u);
  EXPECT_EQ(v.meta().shape[1], 3u);

  ASSERT_EQ(pub.publish(vals, flux::DType::F32, {6}), flux::Published::Ok);
  EXPECT_EQ(sub.take().meta().ndim, 1u);
}

// lost() restarts with a new stream, so a rate needs to know when that happened (api.md).
TEST_F(PullSurface, AttachGenerationChangesWhenThePublisherRestarts)
{
  const std::string topic = uniq("attach_gen_");
  auto pub = std::make_unique<flux::ros::Publisher>(*node_, topic, kFingerprint, kSlotSize, kSlots);
  flux::ros::Subscription sub(*node_, topic, kFingerprint);
  const std::uint8_t one[4] = {1, 1, 1, 1};
  ASSERT_EQ(pub->publish(one, sizeof(one)), flux::Published::Ok);
  ASSERT_TRUE(sub.take().valid());
  const std::uint32_t first = sub.attach_generation();
  pub.reset();
  flux::ros::Publisher pub2(*node_, topic, kFingerprint, kSlotSize, kSlots);
  const std::uint8_t two[4] = {2, 2, 2, 2};
  bool seen = false;
  for (int i = 0; i < 100 && !seen; ++i) {
    ASSERT_EQ(pub2.publish(two, sizeof(two)), flux::Published::Ok);
    seen = first_byte(sub.take()) == 2;
  }
  ASSERT_TRUE(seen) << "the subscription never followed the restart";
  EXPECT_NE(sub.attach_generation(), first);
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
    (void)pub.publish(v, sizeof(v));
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
  qos.keep_last(kSlots);
  flux::ros::Subscription sub(*node_, topic, kFingerprint, qos);

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
      *node_, topic, kFingerprint, flux::QoS{}, {}, flux::Device::Cpu, mem);
    attached = sub.attached();
  } catch (const std::system_error &) {
    threw = true;
  }
  ASSERT_EQ(::setrlimit(RLIMIT_MEMLOCK, &saved), 0);  // before any assertion can leave it lowered

  EXPECT_TRUE(threw) << "the refusal came back as an " << (attached ? "attached" : "unattached")
                     << " subscription instead of a throw";
}

// A publisher that is up but whose segment this process cannot open is not "not yet": no retry
// fixes a permission. The constructor throws rather than leaving a subscription that looks
// unattached forever. A publisher that is simply absent still waits quietly.
TEST_F(PullSurface, ASegmentThatCannotBeOpenedWhileItsPublisherLivesThrows)
{
  if (::geteuid() == 0) GTEST_SKIP() << "root opens a read-only file anyway";
  const std::string topic = uniq("unopenable_");
  flux::ros::Publisher pub(*node_, topic, kFingerprint, kSlotSize, kSlots);
  // The segment is the signpost's name plus an owner suffix; the signpost stays readable, so the
  // publisher still reads as live.
  const std::string prefix = pub.segment_name().substr(1) + ".";
  int locked = 0;
  if (DIR * d = ::opendir("/dev/shm")) {
    while (dirent * e = ::readdir(d)) {
      if (std::strncmp(e->d_name, prefix.c_str(), prefix.size()) == 0) {
        locked += ::chmod((std::string("/dev/shm/") + e->d_name).c_str(), 0400) == 0;
      }
    }
    ::closedir(d);
  }
  ASSERT_EQ(locked, 1);
  EXPECT_THROW(flux::ros::Subscription(*node_, topic, kFingerprint), std::runtime_error);

  EXPECT_FALSE(flux::ros::Subscription(*node_, uniq("absent_"), kFingerprint).attached());
}

TEST_F(PullSurface, ATopicPastTheNameLimitIsRefused)
{
  EXPECT_THROW(
    flux::ros::Publisher(*node_, "/" + std::string(185, 'k'), kFingerprint, kSlotSize, kSlots),
    std::invalid_argument);
}

// The same holds one step earlier, when the signpost itself cannot be read.
TEST_F(PullSurface, ASignpostThatCannotBeReadThrows)
{
  if (::geteuid() == 0) GTEST_SKIP() << "root reads a mode-0 file anyway";
  const std::string topic = uniq("unreadable_");
  flux::ros::Publisher pub(*node_, topic, kFingerprint, kSlotSize, kSlots);
  const std::string file = "/dev/shm" + pub.segment_name();
  ASSERT_EQ(::chmod(file.c_str(), 0), 0);
  EXPECT_THROW(flux::ros::Subscription(*node_, topic, kFingerprint), std::runtime_error);
  ::chmod(file.c_str(), 0600);
}

// A subscription its QoS refused never existed, so this process's manifest does not list it.
TEST_F(PullSurface, ASubscriptionItsQosRefusedIsNotAnnounced)
{
  const std::string topic = uniq("refused_qos_");
  flux::ros::Publisher pub(*node_, topic, kFingerprint, kSlotSize, kSlots);
  const flux::QoS replay_past_depth = flux::QoS(2).transient_local(3);
  EXPECT_THROW(
    flux::ros::Subscription(*node_, topic, kFingerprint, replay_past_depth), std::invalid_argument);
  for (const auto & e :
       flux::OwnerFile::read_manifest(flux::owner_file_name(flux::OwnerFile::self()))) {
    EXPECT_FALSE(e.signpost == pub.segment_name() && !e.publisher);
  }
}
