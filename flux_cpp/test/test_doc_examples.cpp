// The examples of the user documents, between [doc:<id>] and [doc:/<id>] anchors.
// scripts/check_doc_examples.py holds every docs/ fence tagged doc:<id> equal to its region.
#include "flux/channel.hpp"
#include "flux/executor.hpp"
#include "flux/message_filters/subscriber.hpp"
#include "flux/partitioned_executor.hpp"
#include "flux/publisher.hpp"
#include "flux/subscription.hpp"
#include "flux/topics.hpp"

#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image.hpp"
#include "sensor_msgs/flux/image_ros.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{

using namespace std::chrono_literals;
using sensor_msgs::flux_msg::Image;

using MemsetAsync = int (*)(std::uint64_t, unsigned char, std::size_t, void *);

MemsetAsync cu_memset()
{
  static const MemsetAsync f = [] {
    void * lib = dlopen("libcuda.so.1", RTLD_LAZY | RTLD_LOCAL);
    // NOLINTNEXTLINE(*-reinterpret-cast): dlsym hands back an untyped address.
    return lib == nullptr ? nullptr : reinterpret_cast<MemsetAsync>(dlsym(lib, "cuMemsetD8Async"));
  }();
  return f;
}

// Stands in for the reader's own kernels.
void launch_my_kernel(void * device, std::size_t n, void * stream)
{
  ASSERT_EQ(cu_memset()(reinterpret_cast<std::uintptr_t>(device), 1, n, stream), 0);  // NOLINT
}

void read_on_device(const void * device, std::size_t n, void * stream)
{
  EXPECT_NE(device, nullptr);
  EXPECT_GT(n, 0U);
  EXPECT_NE(stream, nullptr);
}

class Doc : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("doc", "/doc" + std::to_string(getpid()));
  }
  void TearDown() override
  {
    node.reset();
    rclcpp::shutdown();
  }

  // Publishes on `topic` from another thread until the context is shut down, which ends a spin.
  std::thread feed(const std::string & topic, std::uint64_t fingerprint = flux::kNoSchema)
  {
    return std::thread([this, topic, fingerprint] {
      flux::Publisher pub(*node, topic, fingerprint, 1U << 20U, 16);
      for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        std::this_thread::sleep_for(10ms);
        if (fingerprint == flux::kNoSchema) {
          const std::vector<std::uint8_t> bytes(64, 1);
          (void)pub.publish(bytes.data(), flux::DType::U8, {8, 8});
        } else {
          Image::Builder b = Image::build__(pub);
          b.set__header__stamp(i, 0);
          b.set__width(640);
          (void)b.commit__();
        }
      }
      rclcpp::shutdown();
    });
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(Doc, PublishAndLoan)
{
  // [doc:cpp_publish]
  flux::Publisher pub(*node, "image");

  constexpr std::size_t kHeight = 480;
  constexpr std::size_t kWidth = 640;
  std::vector<std::uint8_t> pixels(kHeight * kWidth * 3);
  const flux::Published p = pub.publish(pixels.data(), flux::DType::U8, {kHeight, kWidth, 3});
  if (flux::faulted(p)) {
    RCLCPP_ERROR(node->get_logger(), "publish: %s", flux::to_string(p));
  }
  // [doc:/cpp_publish]
  EXPECT_EQ(p, flux::Published::Ok);
  flux::Subscription image(*node, "image", flux::kNoSchema, flux::QoS(1).transient_local(1));
  const flux::FrameView sent = image.take();
  ASSERT_TRUE(sent);
  EXPECT_EQ(sent.meta().shape[2], 3U);

  flux::Publisher points_pub(*node, "points", flux::kNoSchema, 1U << 20U, 4);
  flux::Subscription check(*node, "points");
  {
    flux::Publisher & pub = points_pub;
    // [doc:cpp_loan]
    constexpr std::size_t kPoints = 1024;
    flux::WriteSlot slot = pub.loan(flux::DType::F32, {kPoints, 3});
    if (slot) {
      auto * xyz = static_cast<float *>(slot.data());
      std::fill_n(xyz, kPoints * 3, 0.0F);
      if (flux::faulted(slot.commit())) {
        RCLCPP_ERROR(node->get_logger(), "commit failed");
      }
    }
    // [doc:/cpp_loan]
  }
  const flux::FrameView v = check.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.size(), std::size_t{1024} * 3 * sizeof(float));
}

TEST_F(Doc, SubscribeWithAnExecutor)
{
  std::thread publisher = feed("image");
  testing::internal::CaptureStdout();

  // [doc:cpp_subscribe]
  flux::Subscription sub(
    *node, "image", flux::kNoSchema, flux::QoS{}, [](const flux::FrameView & frame) {
      std::cout << frame.size() << " bytes, " << unsigned{frame.meta().ndim} << " dimensions\n";
    });

  flux::Executor ex;
  ex.add(sub);
  ex.add_node(node);
  ex.spin();
  // [doc:/cpp_subscribe]

  publisher.join();
  EXPECT_NE(
    testing::internal::GetCapturedStdout().find("64 bytes, 2 dimensions"), std::string::npos);
}

TEST_F(Doc, TakeWithoutAnExecutor)
{
  flux::Publisher pub(*node, "image", flux::kNoSchema, 4096, 4);
  testing::internal::CaptureStdout();
  // [doc:cpp_take]
  flux::Subscription sub(*node, "image");
  // [doc:/cpp_take]
  ASSERT_EQ(pub.publish("a", 1), flux::Published::Ok);
  ASSERT_EQ(pub.publish("b", 1), flux::Published::Ok);
  // [doc:cpp_take_frames]
  while (flux::FrameView frame = sub.take()) {
    std::cout << "frame " << frame.ticket() << ", " << frame.size() << " bytes\n";
  }
  if (flux::FrameView frame = sub.take_blocking(100'000'000)) {
    std::cout << "one more, " << frame.size() << " bytes\n";
  }
  // [doc:/cpp_take_frames]
  EXPECT_NE(testing::internal::GetCapturedStdout().find("frame 2, 1 bytes"), std::string::npos);
}

TEST_F(Doc, Qos)
{
  flux::Publisher pub(*node, "cmd", flux::kNoSchema, 4096, 8);
  for (const char * c : {"a", "b", "c"}) ASSERT_EQ(pub.publish(c, 1), flux::Published::Ok);
  // [doc:cpp_qos]
  const flux::QoS qos = flux::QoS(4).transient_local(2).max_borrow(2);
  flux::Subscription sub(*node, "cmd", flux::kNoSchema, qos);
  // [doc:/cpp_qos]
  const flux::FrameView first = sub.take();
  ASSERT_TRUE(first);
  EXPECT_EQ(*static_cast<const char *>(first.data()), 'b');
}

TEST_F(Doc, Adapter)
{
  // [doc:cpp_adapter_publish]
  flux::Publisher pub(*node, "image", Image::kFingerprint);

  constexpr std::uint32_t kHeight = 480;
  constexpr std::uint32_t kWidth = 640;

  Image::Builder b = Image::build__(pub);
  if (b) {
    const builtin_interfaces::msg::Time stamp = node->now();
    b.set__header__stamp(stamp.sec, stamp.nanosec);
    b.set__header__frame_id("camera");
    b.set__height(kHeight);
    b.set__width(kWidth);
    b.set__encoding("rgb8");
    b.set__step(kWidth * 3);
    flux::wire::Span<std::uint8_t> data = b.alloc__data(std::size_t{kHeight} * kWidth * 3);
    std::fill(data.begin(), data.end(), 0);
    if (flux::faulted(b.commit__())) {
      RCLCPP_ERROR(node->get_logger(), "image not published");
    }
  }
  // [doc:/cpp_adapter_publish]
  flux::Subscription sub(*node, "image", Image::kFingerprint, flux::QoS(1).transient_local(1));
  testing::internal::CaptureStdout();
  if (flux::FrameView frame = sub.take()) {
    // [doc:cpp_adapter_read]
    const Image::View img(frame);
    const std::string_view encoding = img.encoding();
    const flux::wire::Span<const std::uint8_t> data = img.data();
    if (img.ok__()) {
      std::cout << img.width() << 'x' << img.height() << ' ' << encoding << ", " << data.size()
                << " bytes\n";
    }
    // [doc:/cpp_adapter_read]
  }
  EXPECT_NE(
    testing::internal::GetCapturedStdout().find("640x480 rgb8, 921600 bytes"), std::string::npos);
}

TEST_F(Doc, RosMessageBridge)
{
  flux::Publisher pub(*node, "image", Image::kFingerprint);
  flux::Subscription sub(*node, "image", Image::kFingerprint);
  sensor_msgs::msg::Image msg;
  msg.width = 2;
  msg.height = 1;
  msg.encoding = "mono8";
  msg.data = {7, 9};
  {
    // [doc:cpp_ros_bridge_publish]
    Image::Builder b = Image::build__(pub);
    sensor_msgs::flux_msg::msg_to_frame(msg, b);
    if (flux::faulted(b.commit__())) {
      RCLCPP_ERROR(node->get_logger(), "image not published");
    }
    // [doc:/cpp_ros_bridge_publish]
  }
  const flux::FrameView frame = sub.take();
  ASSERT_TRUE(frame);
  // [doc:cpp_ros_bridge_read]
  const sensor_msgs::msg::Image copy = sensor_msgs::flux_msg::frame_to_msg(Image::View(frame));
  // [doc:/cpp_ros_bridge_read]
  EXPECT_EQ(copy.data, msg.data);
  EXPECT_EQ(copy.encoding, "mono8");
}

TEST_F(Doc, Partitioned)
{
  std::thread publisher = feed("image");
  testing::internal::CaptureStdout();

  // [doc:cpp_partitioned]
  auto camera = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  flux::Subscription sub(
    *node, "image", flux::kNoSchema, flux::QoS{}, [](const flux::FrameView & frame) {
      std::array<char, 16> name{};
      pthread_getname_np(pthread_self(), name.data(), name.size());
      std::cout << name.data() << ": " << frame.size() << " bytes\n";
    });

  flux::PartitionedExecutor ex;
  ex.add(sub, camera);
  ex.add_node(node);
  ex.on_thread_start(camera, [] { pthread_setname_np(pthread_self(), "camera"); });
  ex.spin();
  // [doc:/cpp_partitioned]

  publisher.join();
  EXPECT_NE(testing::internal::GetCapturedStdout().find("camera: 64 bytes"), std::string::npos);
}

// [doc:cpp_component]
class Camera : public rclcpp::Node
{
public:
  explicit Camera(const rclcpp::NodeOptions & options)
  : Node("camera", options),
    group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    sub_(*this, "image", flux::kNoSchema, flux::QoS{}, [](const flux::FrameView & frame) {
      std::cout << "camera: " << frame.size() << " bytes\n";
    })
  {
    sub_.add_to_callback_group(*this, group_);
  }

private:
  rclcpp::CallbackGroup::SharedPtr group_;
  flux::Subscription sub_;
};
// [doc:/cpp_component]

// A container serves it the same way; this one never names the subscription.
TEST_F(Doc, AComponentsSubscriptionIsServedThroughItsGroup)
{
  std::thread publisher = feed("image");
  testing::internal::CaptureStdout();
  const auto camera = std::make_shared<Camera>(
    rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/doc" + std::to_string(getpid())}));
  flux::PartitionedExecutor ex;
  ex.add_node(camera);
  ex.spin();
  publisher.join();
  EXPECT_NE(testing::internal::GetCapturedStdout().find("camera: 64 bytes"), std::string::npos);
}

TEST_F(Doc, MessageFilters)
{
  std::thread left_pub = feed("left/image", Image::kFingerprint);
  std::thread right_pub = feed("right/image", Image::kFingerprint);
  testing::internal::CaptureStdout();

  // [doc:cpp_message_filters]
  using Input = flux::message_filters::Subscriber<Image>;
  using Frame = std::shared_ptr<const Input::Message>;
  using Policy = message_filters::sync_policies::ApproximateTime<Input::Message, Input::Message>;

  Input left(*node, "left/image", 10);
  Input right(*node, "right/image", 10);
  message_filters::Synchronizer<Policy> sync(Policy(10), left, right);
  const auto on_pair = [](const Frame & l, const Frame & r) {
    std::cout << "pair at " << l->header.stamp.sec << " s: " << l->view().width() << ", "
              << r->view().width() << '\n';
  };
  sync.registerCallback(std::bind(on_pair, std::placeholders::_1, std::placeholders::_2));

  flux::Executor ex;
  ex.add(left);
  ex.add(right);
  ex.spin();
  // [doc:/cpp_message_filters]

  left_pub.join();
  right_pub.join();
  EXPECT_NE(testing::internal::GetCapturedStdout().find(": 640, 640"), std::string::npos);
}

TEST_F(Doc, Enumerate)
{
  flux::Publisher pub(*node, "image", flux::kNoSchema, 4096, 4);
  testing::internal::CaptureStdout();
  // [doc:cpp_enumerate]
  for (const flux::Topic & t : flux::enumerate_topics()) {
    const flux::ChannelStats s = flux::read_channel_stats(t.signpost);
    std::cout << t.key << ": " << (s.live ? "up" : "down") << ", " << t.endpoints.size()
              << " endpoints\n";
  }
  // [doc:/cpp_enumerate]
  const std::string key = "/doc" + std::to_string(getpid()) + "/image: up";
  EXPECT_NE(testing::internal::GetCapturedStdout().find(key), std::string::npos);
}

TEST_F(Doc, Gpu)
{
  if (flux::gpu::probe().route == flux::gpu::Route::None || cu_memset() == nullptr) {
    GTEST_SKIP() << "no GPU route on this host";
  }
  // [doc:cpp_gpu_publish]
  flux::Publisher pub(*node, "depth", flux::kNoSchema, 4U << 20U, 4, flux::Device::Cuda);

  flux::WriteSlot slot = pub.loan(flux::DType::F32, {480, 640});
  if (slot) {
    launch_my_kernel(slot.device_ptr(), slot.capacity(), slot.stream());
    if (flux::faulted(slot.commit())) {
      RCLCPP_ERROR(node->get_logger(), "commit: the stream could not be waited for");
    }
  }
  // [doc:/cpp_gpu_publish]
  flux::Subscription sub(
    *node, "depth", flux::kNoSchema, flux::QoS(1).transient_local(1), {}, flux::Device::Cuda);
  // [doc:cpp_gpu_read]
  if (flux::FrameView frame = sub.take()) {
    read_on_device(frame.device_ptr(), frame.size(), frame.stream());
  }
  // [doc:/cpp_gpu_read]
  EXPECT_EQ(sub.lost(), 0U);
}

}  // namespace
