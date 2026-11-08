#include "flux_wrapper/flux_wrapper.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

auto expect_flux() -> bool
{
  const char * v = std::getenv("FLUX_WRAPPER_EXPECT_FLUX");
  return v != nullptr && std::string_view(v) == "1";
}

class WrapperTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ =
      std::make_shared<rclcpp::Node>("wrapper", "/flux_wrapper_test_" + std::to_string(::getpid()));
  }
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  // Spins the node while `publish_once` runs every 10 ms, until `done` or 10 s.
  template <class F>
  void run_until(const std::atomic<bool> & done, F publish_once)
  {
    flux_wrapper::Executor ex;
    ex.add_node(node_);
    std::thread spinner([&ex] { ex.spin(); });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!done && std::chrono::steady_clock::now() < deadline) {
      publish_once();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ex.cancel();
    spinner.join();
  }

  rclcpp::Node::SharedPtr node_;
};

auto header(std::int32_t sec, const std::string & frame) -> std_msgs::msg::Header
{
  std_msgs::msg::Header h;
  h.stamp.sec = sec;
  h.stamp.nanosec = 7;
  h.frame_id = frame;
  return h;
}

TEST_F(WrapperTest, RunSwitchPicksThePath)
{
  EXPECT_EQ(flux_wrapper::use_flux(), expect_flux());
  flux_wrapper::Publisher<Image> pub(*node_, "switch", rclcpp::QoS(1));
  EXPECT_EQ(pub.uses_flux(), expect_flux());
}

TEST_F(WrapperTest, ImageThroughALoan)
{
  struct Got
  {
    std::int32_t sec;
    std::string frame, encoding;
    std::uint32_t height, width, step;
    std::vector<std::uint8_t> data;
  };
  std::optional<Got> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Image> sub(
    *node_, "image", rclcpp::QoS(4), [&](const flux_wrapper::View<Image> & v) {
      got = Got{v.stamp().sec,
                std::string(v.frame_id()),
                std::string(v.encoding()),
                v.height(),
                v.width(),
                v.step(),
                {v.data().begin(), v.data().end()}};
      done = true;
    });
  EXPECT_EQ(sub.uses_flux(), expect_flux());
  flux_wrapper::Publisher<Image> pub(*node_, "image", rclcpp::QoS(4));

  run_until(done, [&] {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    loan.set_header(header(42, "camera"));
    loan.set_height(2);
    loan.set_width(3);
    loan.set_encoding("mono8");
    loan.set_is_bigendian(0);
    loan.set_step(3);
    auto data = loan.alloc_data(6);
    ASSERT_EQ(data.size(), 6u);
    for (std::size_t i = 0; i < data.size(); ++i) {
      data[i] = static_cast<std::uint8_t>(i + 1);
    }
    EXPECT_TRUE(pub.publish(std::move(loan)));
  });

  ASSERT_TRUE(got);
  EXPECT_EQ(got->sec, 42);
  EXPECT_EQ(got->frame, "camera");
  EXPECT_EQ(got->encoding, "mono8");
  EXPECT_EQ(got->height, 2u);
  EXPECT_EQ(got->width, 3u);
  EXPECT_EQ(got->step, 3u);
  EXPECT_EQ(got->data, (std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6}));
}

TEST_F(WrapperTest, PointCloudFromARosMessage)
{
  std::optional<PointCloud2> got;
  std::atomic<bool> done{false};
  const auto group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  flux_wrapper::Subscription<PointCloud2> sub(
    *node_, "points", rclcpp::QoS(4),
    [&](const flux_wrapper::View<PointCloud2> & v) {
      ASSERT_EQ(v.fields_size(), 2u);
      EXPECT_EQ(v.field(1).name, "y");
      EXPECT_EQ(v.field(1).offset, 4u);
      got = v.to_msg();
      done = true;
    },
    {group});
  flux_wrapper::Publisher<PointCloud2> pub(*node_, "points", rclcpp::QoS(4));

  PointCloud2 cloud;
  cloud.header = header(5, "lidar");
  cloud.height = 1;
  cloud.width = 2;
  for (const char * name : {"x", "y"}) {
    PointField f;
    f.name = name;
    f.offset = cloud.fields.size() * 4;
    f.datatype = PointField::FLOAT32;
    f.count = 1;
    cloud.fields.push_back(f);
  }
  cloud.point_step = 8;
  cloud.row_step = 16;
  cloud.data.assign(16, 9);
  cloud.is_dense = true;
  run_until(done, [&] { EXPECT_TRUE(pub.publish(std::make_unique<PointCloud2>(cloud))); });

  ASSERT_TRUE(got);
  EXPECT_EQ(*got, cloud);
}

// A plain ROS subscriber, as rviz2 and ros2 bag are, gets a flux publisher's messages only
// through the mirror; on the ROS path it gets them directly.
TEST_F(WrapperTest, RosSubscriberReceives)
{
  std::atomic<bool> done{false};
  auto ros_sub = node_->create_subscription<Image>(
    "mirrored", rclcpp::SensorDataQoS(), [&](const Image::ConstSharedPtr & m) {
      EXPECT_EQ(m->encoding, "rgb8");
      done = true;
    });
  flux_wrapper::PublisherOptions options;
  options.mirror_to_ros = true;
  flux_wrapper::Publisher<Image> pub(*node_, "mirrored", rclcpp::QoS(4), options);

  run_until(done, [&] {
    auto loan = pub.loan();
    loan.set_header(header(1, "camera"));
    loan.set_encoding("rgb8");
    loan.alloc_data(3);
    EXPECT_TRUE(pub.publish(std::move(loan)));
  });
  EXPECT_TRUE(done);
}

}  // namespace
