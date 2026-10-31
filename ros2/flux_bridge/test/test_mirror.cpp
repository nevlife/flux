// flux::Publisher::mirror_to_ros against an rclcpp subscriber in the same process.

#include "flux/publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/imu.hpp"
#include "sensor_msgs/flux/imu_ros.hpp"

#include <dirent.h>
#include <gtest/gtest.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace
{

using namespace std::chrono_literals;
using Imu = sensor_msgs::flux_msg::Imu;

constexpr std::uint32_t kSlot = 4096;
constexpr std::uint32_t kSlots = 4;

auto topic(const char * name) -> std::string
{
  return "/m" + std::to_string(getpid()) + "_" + name;
}

class Mirror : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override { node_ = std::make_shared<rclcpp::Node>("flux_mirror_test"); }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(Mirror, AFrameReachesARosSubscriber)
{
  flux::Publisher pub(*node_, topic("imu"), Imu::kFingerprint, kSlot, kSlots);
  pub.mirror_to_ros();
  EXPECT_EQ(node_->count_publishers(pub.topic_name()), 1U);

  std::mutex m;
  std::vector<sensor_msgs::msg::Imu> got;
  auto sub = node_->create_subscription<sensor_msgs::msg::Imu>(
    pub.topic_name(), rclcpp::QoS(10).best_effort(), [&](const sensor_msgs::msg::Imu & msg) {
      const std::lock_guard<std::mutex> lock(m);
      got.push_back(msg);
    });
  sensor_msgs::msg::Imu msg;
  msg.header.frame_id = "imu";
  msg.linear_acceleration.z = 9.8;
  const auto end = std::chrono::steady_clock::now() + 10s;
  for (;;) {
    {
      const std::lock_guard<std::mutex> lock(m);
      if (!got.empty() || std::chrono::steady_clock::now() > end) {
        break;
      }
    }
    Imu::Builder b = Imu::build__(pub);
    sensor_msgs::flux_msg::msg_to_frame(msg, b);
    ASSERT_FALSE(flux::faulted(b.commit__()));
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got[0].header.frame_id, "imu");
  EXPECT_EQ(got[0].linear_acceleration.z, 9.8);
}

TEST_F(Mirror, AnUntypedPublisherIsRefused)
{
  flux::Publisher pub(*node_, topic("raw"), flux::kNoSchema, kSlot, kSlots);
  try {
    pub.mirror_to_ros();
    FAIL() << "an untyped mirror was accepted";
  } catch (const flux::Error & e) {
    EXPECT_EQ(e.status(), FLUX_INVALID_ARGUMENT);
  }
}

TEST_F(Mirror, AMovedPublisherKeepsItsMirror)
{
  flux::Publisher pub(*node_, topic("moved"), Imu::kFingerprint, kSlot, kSlots);
  pub.mirror_to_ros();
  flux::Publisher moved = std::move(pub);
  EXPECT_EQ(node_->count_publishers(moved.topic_name()), 1U);
}

auto threads() -> std::size_t
{
  std::size_t n = 0;
  DIR * d = opendir("/proc/self/task");
  while (const dirent * e = readdir(d)) {
    n += e->d_name[0] != '.';
  }
  closedir(d);
  return n;
}

template <class P>
auto eventually(P done, std::chrono::milliseconds limit = 2000ms) -> bool
{
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!done()) {
    if (std::chrono::steady_clock::now() > end) {
      return false;
    }
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

TEST_F(Mirror, DroppingThePublisherStopsItsThread)
{
  const std::size_t before = threads();
  {
    flux::Publisher pub(*node_, topic("drop"), Imu::kFingerprint, kSlot, kSlots);
    pub.mirror_to_ros();
    EXPECT_EQ(threads(), before + 1);
  }
  EXPECT_EQ(threads(), before);
}

// The only subscriber place stays free while no ROS 2 subscriber wants the mirror. The place is
// per process, so the subscriber that takes it runs in another one.
TEST_F(Mirror, AnUnwatchedMirrorHoldsNoReader)
{
  flux::Publisher pub(
    *node_, topic("idle"), Imu::kFingerprint, kSlot, kSlots, flux::MemoryPolicy{}, 1);
  pub.mirror_to_ros();
  std::this_thread::sleep_for(300ms);
  const std::string script =
    "import sys; from flux import _flux as f; "
    "f._Subscription(sys.argv[1], sys.argv[2], '', int(sys.argv[3]), f.QoS(), f.MemoryPolicy())";
  std::vector<std::string> args{"python3",        "-c",
                                script,           pub.signpost_name(),
                                pub.topic_name(), std::to_string(Imu::kFingerprint)};
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (std::string & a : args) {
    argv.push_back(a.data());
  }
  argv.push_back(nullptr);
  pid_t pid = 0;
  ASSERT_EQ(posix_spawnp(&pid, "python3", nullptr, nullptr, argv.data(), environ), 0);
  int status = 0;
  ASSERT_EQ(waitpid(pid, &status, 0), pid);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
}

TEST_F(Mirror, ADeviceBackedPublisherIsRefused)
{
  if (flux::gpu::probe().route == flux::gpu::Route::None) {
    GTEST_SKIP() << "no GPU route";
  }
  flux::Publisher pub(*node_, topic("cuda"), Imu::kFingerprint, kSlot, kSlots, flux::Device::Cuda);
  try {
    pub.mirror_to_ros();
    FAIL() << "a device-backed mirror was accepted";
  } catch (const flux::Error & e) {
    EXPECT_EQ(e.status(), FLUX_INVALID_ARGUMENT);
  }
}

}  // namespace
