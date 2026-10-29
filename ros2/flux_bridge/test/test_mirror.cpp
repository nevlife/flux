// flux::Publisher::mirror_to_ros against an rclcpp subscriber in the same process.

#include "flux/publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/imu.hpp"
#include "sensor_msgs/flux/imu_ros.hpp"

#include <gtest/gtest.h>
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

std::string topic(const char * name)
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
      if (!got.empty() || std::chrono::steady_clock::now() > end) break;
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

}  // namespace
