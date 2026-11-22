#include "flux_wrapper/flux_wrapper.hpp"
#include "flux_wrapper/message_filters.hpp"

#include <flux_wrapper_test_msgs/msg/bar.hpp>
#include <flux_wrapper_test_msgs/msg/blob.hpp>
#include <flux_wrapper_test_msgs/msg/foo.hpp>
#include <sensor_msgs/msg/imu.hpp>

// A message package's own adapters, as a driver includes them: only in a build with flux.
#ifdef USE_FLUX_ENABLED
#include <flux_wrapper_test_msgs/flux/bar_ros.hpp>
#include <flux_wrapper_test_msgs/flux/blob_ros.hpp>
#include <flux_wrapper_test_msgs/flux/foo_ros.hpp>
#endif

#include <gtest/gtest.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rcutils/logging.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#ifdef FLUX_WRAPPER_CUDA
#include <cuda_runtime_api.h>
#endif

#include <array>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

// A replacement operator new is malloc and free by definition.
// NOLINTBEGIN(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory)
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
namespace
{
// The largest allocation of this thread while it watches, so that a test sees a payload copy.
thread_local bool watching = false;
thread_local std::size_t largest = 0;
}  // namespace

auto operator new(std::size_t n) -> void *
{
  if (watching && n > largest) {
    largest = n;
  }
  if (void * p = std::malloc(n != 0 ? n : 1)) {
    return p;
  }
  throw std::bad_alloc();
}
void operator delete(void * p) noexcept
{
  std::free(p);
}
void operator delete(void * p, std::size_t) noexcept
{
  std::free(p);
}
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
// NOLINTEND(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory)

namespace
{

using flux_wrapper_test_msgs::msg::Bar;
using flux_wrapper_test_msgs::msg::Blob;
using flux_wrapper_test_msgs::msg::Foo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

auto expect_flux() -> bool
{
  const char * v = std::getenv("FLUX_WRAPPER_EXPECT_FLUX");
  return v != nullptr && std::string_view(v) == "1";
}

class WrapperTest : public ::testing::Test
{
public:
  [[nodiscard]] auto node() const -> rclcpp::Node & { return *node_; }

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

#ifdef USE_FLUX_ENABLED
  // Sends each of `bad` and then `good` (aux bytes) from a raw publisher on MsgT's GPU channel:
  // each bad one is counted in rejected() and none is delivered, and the good one, whose frame_id
  // is "ok", is.
  template <class MsgT>
  void expect_bad_aux_skipped(
    const std::string & topic, const std::vector<std::uint8_t> & good,
    const std::vector<std::vector<std::uint8_t>> & bad)
  {
    std::atomic<int> delivered{0};
    flux_wrapper::Subscription<MsgT> sub(
      *node_, topic, rclcpp::QoS(4), [&](const flux_wrapper::View<MsgT> & v) {
        EXPECT_EQ(v.frame_id(), "ok");
        ++delivered;
      });
    // A host channel will do: the subscription parses the aux bytes the same way.
    flux::PublisherOptions options;
    options.slot_size = 4096;
    options.slot_count = 4;
    options.aux_size = flux_wrapper::detail::Gpu<MsgT>::kAuxBytes;
    flux::Publisher raw(*node_, topic, options, flux_wrapper::detail::Gpu<MsgT>::kFingerprint);
    const auto send = [&](const std::vector<std::uint8_t> & a) {
      flux::Loan w = raw.loan();
      if (!w) {
        return;
      }
      std::memcpy(w.aux().data(), a.data(), a.size());
      w.set_aux_len(a.size());
      EXPECT_FALSE(flux::faulted(w.commit(0)));
    };
    for (const auto & aux : bad) {
      const std::uint64_t before = sub.rejected();
      std::atomic<bool> done{false};
      run_until(done, [&] {
        send(aux);
        done = sub.rejected() > before;
      });
      EXPECT_GT(sub.rejected(), before);
      EXPECT_EQ(delivered, 0);
    }
    std::atomic<bool> done{false};
    run_until(done, [&] {
      send(good);
      done = delivered > 0;
    });
    EXPECT_GT(delivered, 0);
  }
#endif

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

// What the accessors of a View read, as a message.
auto capture(const flux_wrapper::View<Image> & v) -> Image
{
  Image m;
  m.header.stamp = v.stamp();
  m.header.frame_id = std::string(v.frame_id());
  m.height = v.height();
  m.width = v.width();
  m.encoding = std::string(v.encoding());
  m.is_bigendian = v.is_bigendian();
  m.step = v.step();
  m.data.assign(v.data().begin(), v.data().end());
  return m;
}

auto image(std::int32_t sec, std::vector<std::uint8_t> data) -> Image
{
  Image m;
  m.header = header(sec, "camera");
  m.height = 2;
  m.width = 3;
  m.encoding = "mono8";
  m.is_bigendian = 1;
  m.step = 3;
  m.data = std::move(data);
  return m;
}

// `fields` FLOAT32 fields named by `name(i)`, two points.
auto cloud(
  std::int32_t sec, std::size_t fields,
  const std::function<std::string(std::size_t)> & name = [](std::size_t i) {
    return "f" + std::to_string(i);
  }) -> PointCloud2
{
  PointCloud2 c;
  c.header = header(sec, "lidar");
  c.height = 1;
  c.width = 2;
  for (std::size_t i = 0; i < fields; ++i) {
    PointField f;
    f.name = name(i);
    f.offset = static_cast<std::uint32_t>(4 * i);
    f.datatype = PointField::FLOAT32;
    f.count = 1;
    c.fields.push_back(f);
  }
  c.is_bigendian = true;
  c.point_step = static_cast<std::uint32_t>(4 * fields);
  c.row_step = 2 * c.point_step;
  c.data.resize(c.row_step);
  for (std::size_t i = 0; i < c.data.size(); ++i) {
    c.data[i] = static_cast<std::uint8_t>(sec * 16 + static_cast<std::int32_t>(i));
  }
  c.is_dense = true;
  return c;
}

// A custom type with every kind of field the wrapper carries besides its root data.
auto foo(std::int32_t sec, std::vector<std::uint8_t> data) -> Foo
{
  Foo m;
  m.header = header(sec, "foo_frame");
  m.width = 3;
  m.scale = 0.5;
  m.valid = true;
  m.label = "label";
  m.tags = {"a", "bc"};
  m.values = {1.5F, -2.0F};
  m.origin.x = 1.0;
  m.origin.y = 2.0;
  m.origin.z = 3.0;
  m.corners.resize(2);
  m.corners[1].x = 4.0;
  m.data = std::move(data);
  return m;
}

auto without(Foo m) -> Foo
{
  m.data.clear();
  return m;
}

auto bar(std::int32_t sec) -> Bar
{
  Bar m;
  m.header = header(sec, "bar_frame");
  m.value = -4;
  m.name = "bar";
  m.samples = {0.25, 0.5};
  return m;
}

auto imu(std::int32_t sec) -> Imu
{
  Imu m;
  m.header = header(sec, "imu");
  m.orientation.w = 1.0;
  m.angular_velocity.z = 0.5;
  m.linear_acceleration_covariance[4] = 2.0;
  return m;
}

#ifdef FLUX_WRAPPER_CUDA
auto capture(const flux_wrapper::View<PointCloud2> & v) -> PointCloud2
{
  PointCloud2 m;
  m.header.stamp = v.stamp();
  m.header.frame_id = std::string(v.frame_id());
  m.height = v.height();
  m.width = v.width();
  for (std::size_t i = 0; i < v.fields_size(); ++i) {
    const flux_wrapper::Field f = v.field(i);
    PointField p;
    p.name = std::string(f.name);
    p.offset = f.offset;
    p.datatype = f.datatype;
    p.count = f.count;
    m.fields.push_back(p);
  }
  m.is_bigendian = v.is_bigendian();
  m.point_step = v.point_step();
  m.row_step = v.row_step();
  m.data.assign(v.data().begin(), v.data().end());
  m.is_dense = v.is_dense();
  return m;
}

// Empty when Device::Cuda can be had on this run's path, else why not.
auto no_cuda() -> std::string
{
#ifdef USE_FLUX_ENABLED
  if (flux_wrapper::use_flux()) {
    const flux::gpu::Platform p = flux::gpu::probe();
    return p.route == flux::gpu::Route::None ? "no flux GPU route: " + p.reason : "";
  }
#endif
  int n = 0;
  return cudaGetDeviceCount(&n) == cudaSuccess && n > 0 ? "" : "no CUDA device";
}

class DeviceBytes
{
public:
  explicit DeviceBytes(const std::vector<std::uint8_t> & bytes)
  {
    EXPECT_EQ(cudaMalloc(&p_, bytes.size()), cudaSuccess);
    EXPECT_EQ(cudaMemcpy(p_, bytes.data(), bytes.size(), cudaMemcpyHostToDevice), cudaSuccess);
  }
  ~DeviceBytes() { (void)cudaFree(p_); }
  DeviceBytes(const DeviceBytes &) = delete;
  auto operator=(const DeviceBytes &) -> DeviceBytes & = delete;
  DeviceBytes(DeviceBytes &&) = delete;
  auto operator=(DeviceBytes &&) -> DeviceBytes & = delete;

  [[nodiscard]] auto get() const -> const void * { return p_; }

private:
  void * p_ = nullptr;
};

#define SKIP_WITHOUT_CUDA()                              \
  if (const std::string why = no_cuda(); !why.empty()) { \
    GTEST_SKIP() << why;                                 \
  }
#else
#define SKIP_WITHOUT_CUDA() GTEST_SKIP() << "built without CUDA"
#endif

TEST_F(WrapperTest, RunSwitchPicksThePath)
{
  EXPECT_EQ(flux_wrapper::use_flux(), expect_flux());
  flux_wrapper::Publisher<Image> pub(*node_, "switch", rclcpp::QoS(1));
  EXPECT_EQ(pub.uses_flux(), expect_flux());
}

TEST_F(WrapperTest, SubscriptionCountSeesASubscriber)
{
  flux_wrapper::Publisher<Image> pub(*node_, "counted", rclcpp::QoS(1));
  EXPECT_EQ(pub.get_subscription_count(), 0u);
  flux_wrapper::Subscription<Image> sub(
    *node_, "counted", rclcpp::QoS(1), [](const flux_wrapper::View<Image> &) {});
  // The ROS graph learns of a subscriber through discovery, so it may take a moment.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (pub.get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(pub.get_subscription_count(), 1u);
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

  // The same message again, kept by the caller.
  got.reset();
  done = false;
  cloud.header = header(6, "lidar");
  run_until(done, [&] { EXPECT_TRUE(pub.publish(cloud)); });

  ASSERT_TRUE(got);
  EXPECT_EQ(*got, cloud);
}

// A message far larger than its slot goes out: its data field goes to the publisher's arena.
TEST_F(WrapperTest, AMessageLargerThanItsSlotGoesOut)
{
  constexpr std::size_t kBytes = 9u << 20;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Image> sub(
    *node_, "large", rclcpp::QoS(1), [&](const flux_wrapper::View<Image> & v) {
      EXPECT_EQ(v.data().size(), kBytes);
      done = true;
    });
  flux_wrapper::PublisherOptions options;
  options.slot_count = 2;
  flux_wrapper::Publisher<Image> pub(*node_, "large", rclcpp::QoS(1), options);

  run_until(done, [&] {
    auto loan = pub.loan();
    if (loan && loan.alloc_data(kBytes).size() == kBytes) {
      (void)pub.publish(std::move(loan));
    }
  });
  EXPECT_TRUE(done);
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

// The mirror publishes with the driver's QoS, as the ROS path does: a reliable ROS subscriber, the
// rclcpp default, gets it too.
TEST_F(WrapperTest, AReliableRosSubscriberReceivesTheMirror)
{
  std::atomic<bool> done{false};
  auto ros_sub = node_->create_subscription<Image>(
    "mirrored_reliable", rclcpp::QoS(10), [&](const Image::ConstSharedPtr &) { done = true; });
  flux_wrapper::PublisherOptions options;
  options.mirror_to_ros = true;
  flux_wrapper::Publisher<Image> pub(*node_, "mirrored_reliable", rclcpp::QoS(4), options);
  run_until(done, [&] {
    auto loan = pub.loan();
    loan.set_header(header(1, "camera"));
    loan.set_encoding("rgb8");
    loan.alloc_data(3);
    EXPECT_TRUE(pub.publish(std::move(loan)));
  });
  EXPECT_TRUE(done);
}

// Kept Views still read their own frame after the ring went round many times past them.
TEST_F(WrapperTest, HeldViewsOutliveTheirCallback)
{
  std::vector<std::shared_ptr<const flux_wrapper::View<Image>>> kept;
  std::vector<Image> seen;
  int after = 0;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.max_borrow = 3;
  flux_wrapper::Subscription<Image> sub(
    *node_, "held", rclcpp::QoS(4),
    [&](std::shared_ptr<const flux_wrapper::View<Image>> v) {
      if (kept.size() < 2) {
        seen.push_back(capture(*v));
        kept.push_back(std::move(v));
      } else if (++after == 12) {
        done = true;
      }
    },
    sub_options);
  flux_wrapper::PublisherOptions options;
  options.slot_count = 4;
  flux_wrapper::Publisher<Image> pub(*node_, "held", rclcpp::QoS(4), options);

  std::int32_t sec = 0;
  run_until(done, [&] {
    ++sec;
    EXPECT_TRUE(
      pub.publish(image(sec, std::vector<std::uint8_t>(6, static_cast<std::uint8_t>(sec)))));
  });
  ASSERT_TRUE(done);
  ASSERT_EQ(kept.size(), 2u);
  for (std::size_t i = 0; i < kept.size(); ++i) {
    EXPECT_EQ(capture(*kept[i]), seen[i]);
  }
}

TEST_F(WrapperTest, APublisherWithNoMoreSlotsThanMaxBorrowIsRefused)
{
  const auto ignore = [](const flux_wrapper::View<Image> &) {};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.max_borrow = 0;
  EXPECT_THROW(
    flux_wrapper::Subscription<Image>(*node_, "few_slots", rclcpp::QoS(4), ignore, sub_options),
    std::invalid_argument);
#ifdef USE_FLUX_ENABLED
  if (!expect_flux()) {
    GTEST_SKIP() << "slots are a flux publisher's";
  }
  flux_wrapper::PublisherOptions options;
  options.slot_count = 4;
  flux_wrapper::Publisher<Image> pub(*node_, "few_slots", rclcpp::QoS(4), options);
  sub_options.max_borrow = 4;
  try {
    flux_wrapper::Subscription<Image> sub(*node_, "few_slots", rclcpp::QoS(4), ignore, sub_options);
    FAIL() << "attached to 4 slots for max_borrow 4";
  } catch (const flux::Error & e) {
    EXPECT_EQ(e.status(), flux::Status::InvalidArgument);
  }
  sub_options.max_borrow = 3;
  const flux_wrapper::Subscription<Image> sub(
    *node_, "few_slots", rclcpp::QoS(4), ignore, sub_options);
  EXPECT_TRUE(sub.uses_flux());
#endif
}

// Written through a device buffer from host and from device memory, read by a host subscription:
// over flux a GPU channel, over ROS the publisher's device buffer copied into the message.
TEST_F(WrapperTest, CudaImageFromHostAndDeviceSources)
{
  SKIP_WITHOUT_CUDA();
#ifdef FLUX_WRAPPER_CUDA
  // Frames left from an earlier phase have another stamp and are passed over.
  std::atomic<std::int32_t> want_sec{1};
  std::optional<Image> got;
  bool got_on_device = true;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Image> sub(
    *node_, "cuda_image", rclcpp::QoS(4), [&](const flux_wrapper::View<Image> & v) {
      if (v.stamp().sec != want_sec) {
        return;
      }
      got = capture(v);
      got_on_device = v.on_device();
      done = true;
    });
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<Image> pub(*node_, "cuda_image", rclcpp::QoS(4), options);
  ASSERT_TRUE(pub.uses_device());
  EXPECT_EQ(pub.uses_flux(), expect_flux());

  Image want = image(1, {1, 2, 3, 4, 5, 6});
  const auto write = [&](const void * src) {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    loan.set_header(want.header);
    loan.set_height(want.height);
    loan.set_width(want.width);
    loan.set_encoding(want.encoding);
    loan.set_is_bigendian(want.is_bigendian);
    loan.set_step(want.step);
    const flux_wrapper::Buffer buf = loan.alloc_buffer(want.data.size());
    ASSERT_TRUE(buf);
    EXPECT_TRUE(buf.on_device());
    EXPECT_NE(buf.device_ptr(), nullptr);
    EXPECT_NE(buf.stream(), nullptr);
    EXPECT_FALSE(buf.copy_from(src, want.data.size() + 1));
    ASSERT_TRUE(buf.copy_from(src, want.data.size()));
    EXPECT_TRUE(pub.publish(std::move(loan)));
  };
  run_until(done, [&] { write(want.data.data()); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, want);
  EXPECT_FALSE(got_on_device);

  got.reset();
  done = false;
  want = image(2, {9, 8, 7, 6, 5, 4});
  want_sec = 2;
  const DeviceBytes source(want.data);
  run_until(done, [&] { write(source.get()); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, want);
#endif
}

TEST_F(WrapperTest, CudaImageFromARosMessage)
{
  SKIP_WITHOUT_CUDA();
  std::atomic<std::int32_t> want_sec{1};
  std::optional<Image> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Image> sub(
    *node_, "cuda_message", rclcpp::QoS(4), [&](const flux_wrapper::View<Image> & v) {
      if (v.stamp().sec == want_sec) {
        got = v.to_msg();
        done = true;
      }
    });
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<Image> pub(*node_, "cuda_message", rclcpp::QoS(4), options);
  ASSERT_TRUE(pub.uses_device());

  const Image kept = image(1, {4, 5, 6, 7, 8, 9});
  run_until(done, [&] { EXPECT_TRUE(pub.publish(kept)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, kept);

  got.reset();
  done = false;
  want_sec = 2;
  const Image owned = image(2, {3, 2, 1, 0, 1, 2});
  run_until(done, [&] { EXPECT_TRUE(pub.publish(std::make_unique<Image>(owned))); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, owned);
}

// A Cuda subscription gets the frame's device memory, and over flux no host copy on a discrete GPU.
TEST_F(WrapperTest, CudaSubscriptionKeepsFramesOnTheDevice)
{
  SKIP_WITHOUT_CUDA();
#ifdef FLUX_WRAPPER_CUDA
  struct Got
  {
    bool on_device = false;
    std::size_t host_bytes = 0;
    std::size_t size = 0;
    bool stream = false;
    std::vector<std::uint8_t> device_bytes;
    Image fields;
    Image msg;
  };
  std::optional<Got> got;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Subscription<Image> sub(
    *node_, "on_device", rclcpp::QoS(4),
    [&](const flux_wrapper::View<Image> & v) {
      Got g;
      g.on_device = v.on_device();
      g.host_bytes = v.data().size();
      g.size = v.size();
      g.stream = v.stream() != nullptr;
      g.device_bytes.resize(v.size());
      EXPECT_EQ(
        cudaMemcpyAsync(
          g.device_bytes.data(), v.device_ptr(), v.size(), cudaMemcpyDeviceToHost, v.stream()),
        cudaSuccess);
      EXPECT_EQ(cudaStreamSynchronize(v.stream()), cudaSuccess);
      g.fields = capture(v);
      g.msg = v.to_msg();
      got = std::move(g);
      done = true;
    },
    sub_options);
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<Image> pub(*node_, "on_device", rclcpp::QoS(4), options);
  ASSERT_TRUE(pub.uses_device());

  const Image want = image(3, {10, 20, 30, 40, 50, 60});
  run_until(done, [&] { EXPECT_TRUE(pub.publish(want)); });

  ASSERT_TRUE(got);
  EXPECT_TRUE(got->on_device);
#ifdef USE_FLUX_ENABLED
  if (expect_flux() && flux::gpu::probe().route == flux::gpu::Route::DeviceHandle) {
    EXPECT_EQ(got->host_bytes, 0u);
  }
#endif
  EXPECT_TRUE(got->stream);
  EXPECT_EQ(got->size, want.data.size());
  EXPECT_EQ(got->device_bytes, want.data);
  Image fields = want;
  fields.data = got->fields.data;
  EXPECT_EQ(got->fields, fields);
  EXPECT_EQ(got->msg, want);
#endif
}

// encoding and frame_id travel in the frame's aux bytes, 4096 of them; the ROS path has no cap.
TEST_F(WrapperTest, CudaFieldsOverTheAuxFailTheLoan)
{
  SKIP_WITHOUT_CUDA();
  if (!expect_flux()) {
    GTEST_SKIP() << "the aux is a flux GPU frame's";
  }
  std::optional<Image> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Image> sub(
    *node_, "capped", rclcpp::QoS(4), [&](const flux_wrapper::View<Image> & v) {
      got = capture(v);
      done = true;
    });
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<Image> pub(*node_, "capped", rclcpp::QoS(4), options);
  ASSERT_TRUE(pub.uses_device());

  auto long_frame = pub.loan();
  ASSERT_TRUE(long_frame);
  long_frame.set_header(header(1, std::string(5000, 'f')));
  EXPECT_FALSE(long_frame);
  EXPECT_FALSE(pub.publish(std::move(long_frame)));

  // Data larger than the slot fails the loan the same way, not an empty frame.
  auto too_large = pub.loan();
  ASSERT_TRUE(too_large);
  EXPECT_FALSE(too_large.alloc_buffer(std::size_t{1} << 40).size());
  EXPECT_FALSE(too_large);
  EXPECT_FALSE(pub.publish(std::move(too_large)));

  Image msg = image(1, {1});
  msg.header.frame_id = std::string(5000, 'f');
  EXPECT_FALSE(pub.publish(msg));

  // Past the old caps (a 256-byte frame_id, a 32-byte encoding) a frame goes out whole.
  msg.header.frame_id = std::string(300, 'f');
  msg.encoding = std::string(40, 'e');
  run_until(done, [&] { EXPECT_TRUE(pub.publish(msg)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, msg);
}

#ifdef FLUX_WRAPPER_CUDA
// What a Cuda subscription got: the data field read back from the device, and the host fields.
template <class MsgT>
struct OnDevice
{
  bool on_device = false;
  bool stream = false;
  std::size_t size = 0;
  std::vector<std::uint8_t> device_bytes;
  MsgT msg;
};

template <class MsgT>
auto read_device(const flux_wrapper::View<MsgT> & v) -> OnDevice<MsgT>
{
  OnDevice<MsgT> g;
  g.on_device = v.on_device();
  g.stream = v.stream() != nullptr;
  g.size = v.size();
  g.device_bytes.resize(v.size());
  if (v.on_device()) {
    EXPECT_EQ(
      cudaMemcpyAsync(
        g.device_bytes.data(), v.device_ptr(), v.size(), cudaMemcpyDeviceToHost, v.stream()),
      cudaSuccess);
    EXPECT_EQ(cudaStreamSynchronize(v.stream()), cudaSuccess);
  }
  g.msg = v.to_msg();
  return g;
}
#endif

// A Cuda subscription gets device memory from a Host publisher too: over flux the frame's copy on
// the device, over ROS the subscription's device buffer.
TEST_F(WrapperTest, CudaSubscriptionGetsAHostImageOnTheDevice)
{
  SKIP_WITHOUT_CUDA();
#ifdef FLUX_WRAPPER_CUDA
  std::optional<OnDevice<Image>> got;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Subscription<Image> sub(
    *node_, "host_image", rclcpp::QoS(4),
    [&](const flux_wrapper::View<Image> & v) {
      got = read_device(v);
      done = true;
    },
    sub_options);
  flux_wrapper::Publisher<Image> pub(*node_, "host_image", rclcpp::QoS(4));
  ASSERT_FALSE(pub.uses_device());
  ASSERT_TRUE(sub.uses_device());

  const Image want = image(4, {11, 22, 33, 44, 55, 66});
  run_until(done, [&] { EXPECT_TRUE(pub.publish(want)); });

  ASSERT_TRUE(got);
  EXPECT_TRUE(got->on_device);
  EXPECT_TRUE(got->stream);
  EXPECT_EQ(got->size, want.data.size());
  EXPECT_EQ(got->device_bytes, want.data);
  EXPECT_EQ(got->msg, want);
#endif
}

// Alloc::Slot keeps the data in the slot, which an integrated GPU reads in place.
// On the flux path a publisher that allocs on subscriber sends nothing before one attaches.
TEST_F(WrapperTest, ASlotPublisherThatAllocsOnSubscriberPublishesOnceOneAttaches)
{
  flux_wrapper::PublisherOptions options;
  options.alloc = flux_wrapper::Alloc::Slot;
  options.alloc_on_subscriber = true;
  auto pub = flux_wrapper::create_publisher<Image>(*node_, "on_sub", rclcpp::QoS(4), options);
  const Image want = image(4, {1, 2, 3});
  if (expect_flux()) {
    EXPECT_FALSE(pub->publish(want));
  }
  std::atomic<bool> done{false};
  auto sub = flux_wrapper::create_subscription<Image>(
    *node_, "on_sub", rclcpp::QoS(4), [&](const flux_wrapper::View<Image> &) { done = true; });
  run_until(done, [&] { (void)pub->publish(want); });
  EXPECT_TRUE(done);
}

// A Cuda publisher with an image format puts the data at the start of an image slot.
TEST_F(WrapperTest, ACudaImagePublisherWithAFormatReachesASubscriber)
{
  SKIP_WITHOUT_CUDA();
#if defined(FLUX_WRAPPER_CUDA) && defined(USE_FLUX_ENABLED)
  if (!expect_flux() || !flux::gpu::probe().integrated) {
    GTEST_SKIP() << "image slots are flux's, on an integrated GPU";
  }
  std::atomic<bool> done{false};
  std::vector<std::uint8_t> got;
  auto sub = flux_wrapper::create_subscription<Image>(
    *node_, "image_fmt", rclcpp::QoS(4), [&](const flux_wrapper::View<Image> & v) {
      got.assign(v.data().begin(), v.data().end());
      done = true;
    });
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  options.image_encoding = "mono8";
  options.image_width = 256;
  options.image_height = 2;
  auto pub = flux_wrapper::create_publisher<Image>(*node_, "image_fmt", rclcpp::QoS(4), options);
  ASSERT_TRUE(pub->uses_device());
  Image want = image(4, std::vector<std::uint8_t>(512, 7));
  want.width = 256;
  want.step = 256;
  run_until(done, [&] { EXPECT_TRUE(pub->publish(want)); });
  ASSERT_TRUE(done);
  EXPECT_EQ(got, want.data);
  want.width = 200;
  want.step = 200;
  want.data.resize(400);
  EXPECT_FALSE(pub->publish(want)) << "rows 200 B apart in a slot of a 256 B pitch";
  options.image_encoding = "rgb8";
  EXPECT_THROW(
    flux_wrapper::create_publisher<Image>(*node_, "image_rgb8", rclcpp::QoS(4), options),
    flux::Error);
  options.image_encoding = "mono8";
  options.image_slots = flux_wrapper::ImageSlots::Off;
  EXPECT_THROW(
    flux_wrapper::create_publisher<Image>(*node_, "image_off", rclcpp::QoS(4), options),
    flux::Error);
  options.image_slots = flux_wrapper::ImageSlots::Auto;
  options.device = flux_wrapper::Device::Cpu;
  options.alloc = flux_wrapper::Alloc::Slot;
  EXPECT_THROW(
    flux_wrapper::create_publisher<Image>(*node_, "image_host", rclcpp::QoS(4), options),
    flux::Error);
#endif
}

TEST_F(WrapperTest, CudaSubscriptionGetsASlotHostImageOnTheDevice)
{
  SKIP_WITHOUT_CUDA();
#ifdef FLUX_WRAPPER_CUDA
  std::optional<OnDevice<Image>> got;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  auto sub = flux_wrapper::create_subscription<Image>(
    *node_, "slot_host_image", rclcpp::QoS(4),
    [&](const flux_wrapper::View<Image> & v) {
      got = read_device(v);
      done = true;
    },
    sub_options);
  flux_wrapper::PublisherOptions options;
  options.alloc = flux_wrapper::Alloc::Slot;
  auto pub =
    flux_wrapper::create_publisher<Image>(*node_, "slot_host_image", rclcpp::QoS(4), options);
  ASSERT_FALSE(pub->uses_device());

  const Image want = image(4, {11, 22, 33, 44, 55, 66});
  run_until(done, [&] { EXPECT_TRUE(pub->publish(want)); });

  ASSERT_TRUE(got);
  EXPECT_TRUE(got->on_device);
  EXPECT_EQ(got->device_bytes, want.data);
  EXPECT_EQ(got->msg, want);
#endif
}

TEST_F(WrapperTest, CudaSubscriptionGetsAHostPointCloudOnTheDevice)
{
  SKIP_WITHOUT_CUDA();
#ifdef FLUX_WRAPPER_CUDA
  std::optional<OnDevice<PointCloud2>> got;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Subscription<PointCloud2> sub(
    *node_, "host_cloud", rclcpp::QoS(4),
    [&](const flux_wrapper::View<PointCloud2> & v) {
      got = read_device(v);
      done = true;
    },
    sub_options);
  flux_wrapper::Publisher<PointCloud2> pub(*node_, "host_cloud", rclcpp::QoS(4));

  const PointCloud2 want = cloud(4, 3);
  run_until(done, [&] { EXPECT_TRUE(pub.publish(want)); });

  ASSERT_TRUE(got);
  EXPECT_TRUE(got->on_device);
  EXPECT_TRUE(got->stream);
  EXPECT_EQ(got->device_bytes, want.data);
  EXPECT_EQ(got->msg, want);
#endif
}

// Data past a small slot goes to the publisher's arena; a Cuda subscription reads it on the device
// as it reads data in the slot.
TEST_F(WrapperTest, CudaSubscriptionGetsHostDataPastItsSlotOnTheDevice)
{
  SKIP_WITHOUT_CUDA();
#ifdef FLUX_WRAPPER_CUDA
  std::optional<OnDevice<Image>> image_got;
  std::optional<OnDevice<PointCloud2>> cloud_got;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Subscription<Image> image_sub(
    *node_, "big_image", rclcpp::QoS(4),
    [&](const flux_wrapper::View<Image> & v) {
      image_got = read_device(v);
      done = image_got && cloud_got;
    },
    sub_options);
  flux_wrapper::Subscription<PointCloud2> cloud_sub(
    *node_, "big_cloud", rclcpp::QoS(4),
    [&](const flux_wrapper::View<PointCloud2> & v) {
      cloud_got = read_device(v);
      done = image_got && cloud_got;
    },
    sub_options);
  flux_wrapper::Publisher<Image> image_pub(*node_, "big_image", rclcpp::QoS(4));
  flux_wrapper::Publisher<PointCloud2> cloud_pub(*node_, "big_cloud", rclcpp::QoS(4));

  std::vector<std::uint8_t> pixels(64 * 1024);
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    pixels[i] = static_cast<std::uint8_t>(i * 7);
  }
  const Image image_want = image(5, pixels);
  PointCloud2 cloud_want = cloud(5, 3);
  cloud_want.width = 8192;
  cloud_want.row_step = cloud_want.width * cloud_want.point_step;
  cloud_want.data.assign(pixels.begin(), pixels.end());
  cloud_want.data.resize(cloud_want.row_step);
  run_until(done, [&] {
    EXPECT_TRUE(image_pub.publish(image_want));
    EXPECT_TRUE(cloud_pub.publish(cloud_want));
  });

  ASSERT_TRUE(image_got && cloud_got);
  EXPECT_TRUE(image_got->on_device);
  EXPECT_EQ(image_got->device_bytes, image_want.data);
  EXPECT_EQ(image_got->msg, image_want);
  EXPECT_TRUE(cloud_got->on_device);
  EXPECT_EQ(cloud_got->device_bytes, cloud_want.data);
  EXPECT_EQ(cloud_got->msg, cloud_want);
#endif
}

// Held Views keep their device bytes: over flux their slots, over ROS their device buffers. Read
// on another stream, after() holds them back until that read is done.
TEST_F(WrapperTest, CudaHeldViewsKeepTheirDeviceBytes)
{
  SKIP_WITHOUT_CUDA();
#ifdef FLUX_WRAPPER_CUDA
  std::vector<std::shared_ptr<const flux_wrapper::View<Image>>> kept;
  std::vector<std::vector<std::uint8_t>> seen;
  int after = 0;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  sub_options.max_borrow = 3;
  flux_wrapper::Subscription<Image> sub(
    *node_, "cuda_held", rclcpp::QoS(4),
    [&](std::shared_ptr<const flux_wrapper::View<Image>> v) {
      if (kept.size() < 2) {
        seen.push_back(read_device(*v).device_bytes);
        kept.push_back(std::move(v));
      } else if (++after == 12) {
        done = true;
      }
    },
    sub_options);
  flux_wrapper::PublisherOptions options;
  options.slot_count = 4;
  flux_wrapper::Publisher<Image> pub(*node_, "cuda_held", rclcpp::QoS(4), options);

  std::int32_t sec = 0;
  run_until(done, [&] {
    ++sec;
    EXPECT_TRUE(
      pub.publish(image(sec, std::vector<std::uint8_t>(6, static_cast<std::uint8_t>(sec)))));
  });
  ASSERT_TRUE(done);
  ASSERT_EQ(kept.size(), 2u);

  cudaStream_t own = nullptr;
  ASSERT_EQ(cudaStreamCreateWithFlags(&own, cudaStreamNonBlocking), cudaSuccess);
  std::vector<std::vector<std::uint8_t>> read(kept.size());
  for (std::size_t i = 0; i < kept.size(); ++i) {
    ASSERT_TRUE(kept[i]->on_device());
    read[i].resize(kept[i]->size());
    EXPECT_EQ(
      cudaMemcpyAsync(
        read[i].data(), kept[i]->device_ptr(), read[i].size(), cudaMemcpyDeviceToHost, own),
      cudaSuccess);
    EXPECT_TRUE(kept[i]->after(own));
  }
  kept.clear();
  EXPECT_EQ(cudaStreamSynchronize(own), cudaSuccess);
  EXPECT_EQ(cudaStreamDestroy(own), cudaSuccess);
  EXPECT_EQ(read, seen);
#endif
}

// Over ROS a message has nowhere to wait: with its one device buffer held, the next are skipped.
TEST_F(WrapperTest, CudaRosPathSkipsMessagesWhileItsBuffersAreHeld)
{
  SKIP_WITHOUT_CUDA();
  if (expect_flux()) {
    GTEST_SKIP() << "over flux a frame past max_borrow waits in the ring";
  }
#ifdef FLUX_WRAPPER_CUDA
  std::shared_ptr<const flux_wrapper::View<Image>> kept;
  std::atomic<int> calls{0};
  std::atomic<int> sent{0};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Subscription<Image> sub(
    *node_, "cuda_skip", rclcpp::QoS(4),
    [&](std::shared_ptr<const flux_wrapper::View<Image>> v) {
      ++calls;
      kept = std::move(v);
    },
    sub_options);
  auto counter = node_->create_subscription<Image>(
    "cuda_skip", rclcpp::QoS(4), [&](const Image::ConstSharedPtr &) { ++sent; });
  flux_wrapper::Publisher<Image> pub(*node_, "cuda_skip", rclcpp::QoS(4));
  const Image want = image(5, {1, 2, 3, 4, 5, 6});

  std::atomic<bool> done{false};
  run_until(done, [&] {
    EXPECT_TRUE(pub.publish(want));
    done = calls > 0 && sent >= 10;
  });
  EXPECT_EQ(calls, 1);
  ASSERT_TRUE(kept);
  EXPECT_EQ(kept->to_msg(), want);

  kept.reset();
  done = false;
  run_until(done, [&] {
    EXPECT_TRUE(pub.publish(want));
    done = calls > 1;
  });
  EXPECT_GT(calls, 1);
#endif
}

// A Cuda publisher's cloud, written from host and device memory and sent as ROS messages, reaches
// a Host subscription in host memory and a Cuda one in device memory.
TEST_F(WrapperTest, CudaPointCloudReachesHostAndCudaSubscriptions)
{
  SKIP_WITHOUT_CUDA();
#ifdef FLUX_WRAPPER_CUDA
  // Frames left from an earlier phase have another stamp and are passed over.
  std::atomic<std::int32_t> want_sec{1};
  std::optional<PointCloud2> host_got;
  std::optional<PointCloud2> host_msg;
  bool host_on_device = true;
  std::optional<OnDevice<PointCloud2>> device_got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<PointCloud2> host_sub(
    *node_, "cuda_cloud", rclcpp::QoS(4), [&](const flux_wrapper::View<PointCloud2> & v) {
      if (v.stamp().sec == want_sec) {
        host_got = capture(v);
        host_msg = v.to_msg();
        host_on_device = v.on_device();
        done = device_got.has_value();
      }
    });
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Subscription<PointCloud2> device_sub(
    *node_, "cuda_cloud", rclcpp::QoS(4),
    [&](const flux_wrapper::View<PointCloud2> & v) {
      if (v.stamp().sec == want_sec) {
        device_got = read_device(v);
        done = host_got.has_value();
      }
    },
    sub_options);
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<PointCloud2> pub(*node_, "cuda_cloud", rclcpp::QoS(4), options);
  ASSERT_TRUE(pub.uses_device());

  PointCloud2 want;
  const auto phase = [&](std::int32_t sec, const std::function<void()> & publish_once) {
    want = cloud(sec, 3);
    want_sec = sec;
    host_got.reset();
    host_msg.reset();
    device_got.reset();
    done = false;
    run_until(done, publish_once);
    ASSERT_TRUE(host_got && device_got) << "phase " << sec;
    EXPECT_EQ(*host_got, want);
    EXPECT_EQ(*host_msg, want);
    EXPECT_FALSE(host_on_device);
    EXPECT_TRUE(device_got->on_device);
    EXPECT_EQ(device_got->device_bytes, want.data);
    EXPECT_EQ(device_got->msg, want);
  };
  const auto write = [&](const void * src) {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    loan.set_header(want.header);
    loan.set_height(want.height);
    loan.set_width(want.width);
    loan.set_fields(want.fields);
    loan.set_is_bigendian(want.is_bigendian);
    loan.set_point_step(want.point_step);
    loan.set_row_step(want.row_step);
    loan.set_is_dense(want.is_dense);
    const flux_wrapper::Buffer buf = loan.alloc_buffer(want.data.size());
    ASSERT_TRUE(buf);
    EXPECT_TRUE(buf.on_device());
    ASSERT_TRUE(buf.copy_from(src, want.data.size()));
    EXPECT_TRUE(pub.publish(std::move(loan)));
  };
  phase(1, [&] { write(want.data.data()); });
  std::optional<DeviceBytes> source;
  phase(2, [&] {
    if (!source) {
      source.emplace(want.data);
    }
    write(source->get());
  });
  phase(3, [&] { EXPECT_TRUE(pub.publish(want)); });
  phase(4, [&] { EXPECT_TRUE(pub.publish(std::make_unique<PointCloud2>(want))); });
#endif
}

// The header and fields travel in the frame's aux bytes, 4096 of them; the ROS path has no cap.
TEST_F(WrapperTest, CudaPointCloudOverItsAuxFailsTheLoan)
{
  SKIP_WITHOUT_CUDA();
  if (!expect_flux()) {
    GTEST_SKIP() << "the aux is a flux GPU frame's";
  }
  std::optional<PointCloud2> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<PointCloud2> sub(
    *node_, "capped_cloud", rclcpp::QoS(4), [&](const flux_wrapper::View<PointCloud2> & v) {
      got = v.to_msg();
      done = true;
    });
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<PointCloud2> pub(*node_, "capped_cloud", rclcpp::QoS(4), options);
  ASSERT_TRUE(pub.uses_device());

  auto long_frame = pub.loan();
  ASSERT_TRUE(long_frame);
  long_frame.set_header(header(1, std::string(5000, 'f')));
  EXPECT_FALSE(long_frame);
  EXPECT_FALSE(pub.publish(std::move(long_frame)));

  // Past the old caps (32 fields, 32-byte names, a 256-byte frame_id) a cloud goes out whole.
  PointCloud2 msg =
    cloud(1, 40, [](std::size_t i) { return std::string(40, 'n') + std::to_string(10 + i); });
  msg.header.frame_id = std::string(300, 'f');
  run_until(done, [&] { EXPECT_TRUE(pub.publish(msg)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, msg);
}

// Where Cuda cannot be had (a build without CUDA, no device, no flux GPU route), a Cuda publisher
// and subscription work as Host ones.
TEST_F(WrapperTest, CudaThatCannotBeHadIsHost)
{
#ifdef FLUX_WRAPPER_CUDA
  const bool cuda = no_cuda().empty();
#else
  const bool cuda = false;
#endif
  std::optional<PointCloud2> got;
  bool got_on_device = !cuda;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Subscription<PointCloud2> sub(
    *node_, "fallback", rclcpp::QoS(4),
    [&](const flux_wrapper::View<PointCloud2> & v) {
      got = v.to_msg();
      got_on_device = v.on_device();
      done = true;
    },
    sub_options);
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<PointCloud2> pub(*node_, "fallback", rclcpp::QoS(4), options);
  EXPECT_EQ(pub.uses_device(), cuda);
  EXPECT_EQ(sub.uses_device(), cuda);

  const PointCloud2 want = cloud(1, 2);
  run_until(done, [&] { EXPECT_TRUE(pub.publish(want)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, want);
  EXPECT_EQ(got_on_device, cuda);
}

// Warnings this process logged, while a Logs is alive.
class Logs
{
public:
  Logs() : previous_(rcutils_logging_get_output_handler())
  {
    lines().clear();
    rcutils_logging_set_output_handler(&Logs::keep);
  }
  ~Logs() { rcutils_logging_set_output_handler(previous_); }
  Logs(const Logs &) = delete;
  auto operator=(const Logs &) -> Logs & = delete;
  Logs(Logs &&) = delete;
  auto operator=(Logs &&) -> Logs & = delete;

  [[nodiscard]] static auto count(std::string_view text) -> int
  {
    int n = 0;
    for (const std::string & l : lines()) {
      n += static_cast<int>(l.find(text) != std::string::npos);
    }
    return n;
  }

private:
  static auto lines() -> std::vector<std::string> &
  {
    static std::vector<std::string> l;
    return l;
  }
  static void keep(
    const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
    const char * format, va_list * args)
  {
    if (severity < RCUTILS_LOG_SEVERITY_WARN) {
      return;
    }
    std::array<char, 1024> buf{};
    va_list copy;
    va_copy(copy, *args);
    (void)std::vsnprintf(buf.data(), buf.size(), format, copy);
    va_end(copy);
    lines().emplace_back(buf.data());
  }

  rcutils_logging_output_handler_t previous_;
};

#if defined(FLUX_WRAPPER_CUDA) && defined(USE_FLUX_ENABLED)
// Sends `want` from a Cuda publisher through test_flux_wrapper.py's echo(), which reads it in
// Python and publishes it again with the Python flux_wrapper, and reads that with a Cuda
// subscription.
template <class MsgT>
auto echo_through_python(
  WrapperTest & t, const rclcpp::Node & node, const char * script, const char * type,
  const MsgT & want) -> std::optional<OnDevice<MsgT>>
{
  const std::string in = std::string(node.get_namespace()) + "/py_in_" + type;
  const std::string out = std::string(node.get_namespace()) + "/py_out_" + type;
  std::optional<OnDevice<MsgT>> got;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Subscription<MsgT> sub(
    t.node(), out, rclcpp::QoS(4),
    [&](const flux_wrapper::View<MsgT> & v) {
      got = read_device(v);
      done = true;
    },
    sub_options);
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<MsgT> pub(t.node(), in, rclcpp::QoS(4), options);

  std::vector<std::string> args = {"python3", script, type, in, out};
  std::vector<char *> argv;
  for (std::string & a : args) {
    argv.push_back(a.data());
  }
  argv.push_back(nullptr);
  pid_t pid = 0;
  const int spawned = posix_spawnp(&pid, "python3", nullptr, nullptr, argv.data(), environ);
  EXPECT_EQ(spawned, 0);
  // Without a child, pid 0 would signal this process group.
  if (spawned != 0) {
    return got;
  }
  t.run_until(done, [&] { (void)pub.publish(want); });
  EXPECT_EQ(::kill(pid, SIGINT), 0);
  int status = 0;
  EXPECT_EQ(::waitpid(pid, &status, 0), pid);
  return got;
}

// A header past the aux is logged once per publisher, with its field and the message's stamp, so
// the aux can be sized by it.
TEST_F(WrapperTest, AHeaderPastTheAuxIsLoggedOnceWithItsField)
{
  SKIP_WITHOUT_CUDA();
  if (!expect_flux()) {
    GTEST_SKIP() << "the aux is a flux GPU frame's";
  }
  const Logs logs;
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<Image> pub(*node_, "over_logged", rclcpp::QoS(4), options);
  ASSERT_TRUE(pub.uses_device());
  for (int i = 0; i < 3; ++i) {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    loan.set_header(header(5, std::string(5000, 'f')));
    EXPECT_FALSE(pub.publish(std::move(loan)));
  }
  flux_wrapper::Publisher<PointCloud2> clouds(*node_, "over_logged_cloud", rclcpp::QoS(4), options);
  EXPECT_FALSE(clouds.publish(
    cloud(1, 200, [](std::size_t i) { return std::string(40, 'n') + std::to_string(i); })));
  EXPECT_EQ(
    Logs::count(
      "over_logged: a message did not go out on Device::Cuda: header.frame_id (5000 B) did not "
      "fit the aux (4096 B, 48 B used before it); header.stamp 5.000000007"),
    1);
  EXPECT_EQ(
    Logs::count("over_logged_cloud: a message did not go out on Device::Cuda: fields (200 entries, "
                "names 8490 B)"),
    1);
}

// On Host the message's own strings go after its fixed fields in the slot. One past the slot fails
// the message, set by hand or copied from a ROS message, and is logged once per publisher.
TEST_F(WrapperTest, AStringPastTheSlotIsLoggedOnceWithItsField)
{
  if (!expect_flux()) {
    GTEST_SKIP() << "the slot is a flux one";
  }
  const Logs logs;
  flux_wrapper::Publisher<Image> pub(*node_, "slot_over_logged", rclcpp::QoS(4));
  for (int i = 0; i < 2; ++i) {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    loan.set_header(header(5, std::string(9000, 'f')));
    EXPECT_FALSE(loan);
    EXPECT_FALSE(pub.publish(std::move(loan)));
  }
  Image msg = image(5, {1, 2, 3});
  msg.header.frame_id = std::string(9000, 'f');
  EXPECT_FALSE(pub.publish(msg));
  EXPECT_EQ(
    Logs::count(
      "slot_over_logged: a message did not go out: header.frame_id (9000 B) did not fit the slot "
      "(8192 B, 48 B used before it); header.stamp 5.000000007"),
    1);
  msg.header.frame_id = std::string(4096, 'f');
  std::optional<Image> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Image> sub(
    *node_, "slot_over_logged", rclcpp::QoS(4), [&](const flux_wrapper::View<Image> & v) {
      got = v.to_msg();
      done = true;
    });
  run_until(done, [&] { EXPECT_TRUE(pub.publish(msg)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, msg);
}

// Data past a Cuda publisher's slot is logged once as too large, not as fields over the aux.
TEST_F(WrapperTest, CudaDataPastTheSlotIsLoggedAsTooLarge)
{
  SKIP_WITHOUT_CUDA();
  if (!expect_flux()) {
    GTEST_SKIP() << "the slot is a flux GPU channel's";
  }
  const Logs logs;
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  options.slot_size = 4096;
  flux_wrapper::Publisher<Image> pub(*node_, "data_over_logged", rclcpp::QoS(4), options);
  ASSERT_TRUE(pub.uses_device());
  for (int i = 0; i < 2; ++i) {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    EXPECT_FALSE(loan.alloc_buffer(8192));
    EXPECT_FALSE(pub.publish(std::move(loan)));
  }
  EXPECT_FALSE(pub.publish(image(1, std::vector<std::uint8_t>(8192, 1))));
  EXPECT_EQ(Logs::count("data_over_logged failed: too large"), 1);
  EXPECT_EQ(Logs::count("over the aux"), 0);
}

// Each publisher and subscription that cannot have Cuda says so, with its topic.
TEST_F(WrapperTest, EveryCudaFallbackIsLoggedWithItsTopic)
{
#ifdef FLUX_WRAPPER_CUDA
  if (no_cuda().empty()) {
    GTEST_SKIP() << "Cuda can be had here: run with CUDA_VISIBLE_DEVICES= or build without CUDA";
  }
#endif
  const Logs logs;
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  const flux_wrapper::Publisher<Image> first(*node_, "fallback_first", rclcpp::QoS(1), options);
  const flux_wrapper::Publisher<Image> second(*node_, "fallback_second", rclcpp::QoS(1), options);
  const flux_wrapper::Subscription<Image> sub(
    *node_, "fallback_sub", rclcpp::QoS(1), [](const flux_wrapper::View<Image> &) {}, sub_options);
  EXPECT_EQ(Logs::count("fallback_first"), 1);
  EXPECT_EQ(Logs::count("fallback_second"), 1);
  EXPECT_EQ(Logs::count("fallback_sub"), 1);
}

// A C++ GPU frame read and written again by the Python flux_wrapper: each side reads the other's
// aux layout, of the standard types and of a custom one.
TEST_F(WrapperTest, GpuFramesGoThroughPython)
{
  SKIP_WITHOUT_CUDA();
  const char * script = std::getenv("FLUX_WRAPPER_PY_TEST");
  if (!expect_flux() || script == nullptr) {
    GTEST_SKIP() << "flux path, with FLUX_WRAPPER_PY_TEST set (ctest sets it)";
  }
  const Image image_want = image(5, {5, 6, 7, 8, 9, 10});
  const auto image_got = echo_through_python(*this, *node_, script, "Image", image_want);
  ASSERT_TRUE(image_got);
  EXPECT_TRUE(image_got->on_device);
  EXPECT_EQ(image_got->device_bytes, image_want.data);
  EXPECT_EQ(image_got->msg, image_want);

  const PointCloud2 cloud_want = cloud(5, 3);
  const auto cloud_got = echo_through_python(*this, *node_, script, "PointCloud2", cloud_want);
  ASSERT_TRUE(cloud_got);
  EXPECT_TRUE(cloud_got->on_device);
  EXPECT_EQ(cloud_got->device_bytes, cloud_want.data);
  EXPECT_EQ(cloud_got->msg, cloud_want);

  const Foo foo_want = foo(5, {7, 7, 8});
  const auto foo_got = echo_through_python(*this, *node_, script, "Foo", foo_want);
  ASSERT_TRUE(foo_got);
  EXPECT_TRUE(foo_got->on_device);
  EXPECT_EQ(foo_got->device_bytes, foo_want.data);
  EXPECT_EQ(foo_got->msg, foo_want);
}
#endif

#ifdef USE_FLUX_ENABLED
// The aux bytes come from another process. A frame whose aux does not parse is not delivered.
// A good aux is a frame of the type with data empty, written by its Builder.
auto aux_of_image(std::string_view frame_id) -> std::vector<std::uint8_t>
{
  std::vector<std::uint8_t> buf(512);
  sensor_msgs::flux_msg::Image::Builder b(buf.data(), buf.size());
  b.set__header__frame_id(frame_id);
  b.set__encoding("rgb8");
  buf.resize(b.size__());
  return buf;
}

auto aux_of_cloud(std::string_view frame_id) -> std::vector<std::uint8_t>
{
  std::vector<std::uint8_t> buf(512);
  sensor_msgs::flux_msg::PointCloud2::Builder b(buf.data(), buf.size());
  b.set__header__frame_id(frame_id);
  auto f = b.alloc__fields(1);
  f[0].set__name("x");
  buf.resize(b.size__());
  return buf;
}

// The same aux with its first descriptor (frame_id) pointed past it.
auto past_the_end(std::vector<std::uint8_t> aux) -> std::vector<std::uint8_t>
{
  const std::uint32_t far = 1U << 20;
  std::memcpy(aux.data() + 8, &far, sizeof(far));
  return aux;
}

TEST_F(WrapperTest, GpuImageFrameWithABadAuxIsSkipped)
{
  if (!expect_flux()) {
    GTEST_SKIP() << "flux path only";
  }
  const auto good = aux_of_image("ok");
  expect_bad_aux_skipped<Image>(
    "bad_aux", good,
    {std::vector<std::uint8_t>(good.begin(), good.begin() + 10), past_the_end(good)});
}

TEST_F(WrapperTest, GpuPointCloudFrameWithABadAuxIsSkipped)
{
  if (!expect_flux()) {
    GTEST_SKIP() << "flux path only";
  }
  const auto good = aux_of_cloud("ok");
  expect_bad_aux_skipped<PointCloud2>(
    "bad_cloud_aux", good,
    {std::vector<std::uint8_t>(good.begin(), good.end() - 1), past_the_end(good)});
}

// The C ABI, the C++ constant and the Python module agree; a split would leave a channel unread.
TEST(GpuChannel, FingerprintsAreTheTypesGpuFingerprint)
{
  EXPECT_EQ(
    flux_wrapper::detail::Gpu<Image>::kFingerprint,
    flux_gpu_fingerprint(sensor_msgs::flux_msg::Image::kFingerprint));
  EXPECT_EQ(
    flux_wrapper::detail::Gpu<PointCloud2>::kFingerprint,
    flux_gpu_fingerprint(sensor_msgs::flux_msg::PointCloud2::kFingerprint));
}
#endif

// A type without accessors of its own is set from its ROS message and read as one, and its root
// data is written and read in place, on either path.
TEST_F(WrapperTest, AnyTypeThroughALoanWithItsDataInPlace)
{
  static_assert(flux_wrapper::detail::kRootData<Foo> && flux_wrapper::detail::kRootHeader<Foo>);
  struct Got
  {
    std::int32_t sec;
    std::string frame;
    Foo fields;
    std::vector<std::uint8_t> data;
    std::size_t size;
    Foo msg;
  };
  std::optional<Got> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Foo> sub(
    *node_, "foo", rclcpp::QoS(4), [&](const flux_wrapper::View<Foo> & v) {
      got = Got{v.stamp().sec,    std::string(v.frame_id()),
                v.without_data(), {v.data().begin(), v.data().end()},
                v.size(),         v.to_msg()};
      done = true;
    });
  flux_wrapper::Publisher<Foo> pub(*node_, "foo", rclcpp::QoS(4));
  EXPECT_EQ(pub.uses_flux(), expect_flux());

  // set_from() leaves data alone: what alloc_data() wrote goes out, not the message's data.
  const Foo fields = foo(8, {1, 2, 3});
  run_until(done, [&] {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    const auto data = loan.alloc_data(4);
    ASSERT_EQ(data.size(), 4u);
    std::fill(data.begin(), data.end(), std::uint8_t{9});
    loan.set_from(fields);
    EXPECT_TRUE(pub.publish(std::move(loan)));
  });

  ASSERT_TRUE(got);
  EXPECT_EQ(got->sec, 8);
  EXPECT_EQ(got->frame, "foo_frame");
  EXPECT_EQ(got->fields, without(fields));
  EXPECT_EQ(got->data, (std::vector<std::uint8_t>{9, 9, 9, 9}));
  EXPECT_EQ(got->size, 4u);
  Foo whole = fields;
  whole.data = {9, 9, 9, 9};
  EXPECT_EQ(got->msg, whole);
}

TEST_F(WrapperTest, AnyTypeFromARosMessage)
{
  std::atomic<std::int32_t> want_sec{1};
  std::optional<Foo> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Foo> sub(
    *node_, "foo_message", rclcpp::QoS(4), [&](const flux_wrapper::View<Foo> & v) {
      if (v.stamp().sec == want_sec) {
        got = v.to_msg();
        done = true;
      }
    });
  flux_wrapper::Publisher<Foo> pub(*node_, "foo_message", rclcpp::QoS(4));
  const Foo kept = foo(1, {4, 5, 6});
  run_until(done, [&] { EXPECT_TRUE(pub.publish(kept)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, kept);

  got.reset();
  done = false;
  want_sec = 2;
  const Foo owned = foo(2, {});
  run_until(done, [&] { EXPECT_TRUE(pub.publish(std::make_unique<Foo>(owned))); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, owned);
}

// Without a root data the loan sets and the view reads every field; a root header still has
// set_header(), stamp() and frame_id().
TEST_F(WrapperTest, ATypeWithoutDataIsSetAndReadWhole)
{
  static_assert(!flux_wrapper::detail::kRootData<Bar>);
  std::optional<Bar> got;
  std::optional<Bar> fields;
  std::string frame;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Bar> sub(
    *node_, "bar", rclcpp::QoS(4), [&](const flux_wrapper::View<Bar> & v) {
      got = v.to_msg();
      fields = v.without_data();
      frame = std::string(v.frame_id());
      done = v.stamp().sec == 9;
    });
  flux_wrapper::Publisher<Bar> pub(*node_, "bar", rclcpp::QoS(4));
  Bar want = bar(3);
  run_until(done, [&] {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    loan.set_from(want);
    loan.set_header(header(9, "late"));
    EXPECT_TRUE(pub.publish(std::move(loan)));
  });
  ASSERT_TRUE(got);
  want.header = header(9, "late");
  EXPECT_EQ(*got, want);
  EXPECT_EQ(*fields, want);
  EXPECT_EQ(frame, "late");
}

// A field the loan does not set is the type's default, not the last loan's. The Python test of the
// same name checks the same.
TEST_F(WrapperTest, ALoanStartsFromAnEmptyMessage)
{
  for (const flux_wrapper::Device device :
       {flux_wrapper::Device::Cpu, flux_wrapper::Device::Cuda}) {
    const bool cuda = device == flux_wrapper::Device::Cuda;
    const std::string topic = cuda ? "foo_fresh_cuda" : "foo_fresh_host";
    std::optional<Foo> got;
    std::atomic<bool> done{false};
    flux_wrapper::Subscription<Foo> sub(
      *node_, topic, rclcpp::QoS(4), [&](const flux_wrapper::View<Foo> & v) {
        if (v.stamp().sec == 2) {
          got = v.without_data();
          done = true;
        }
      });
    flux_wrapper::PublisherOptions options;
    options.device = device;
    flux_wrapper::Publisher<Foo> pub(*node_, topic, rclcpp::QoS(4), options);
    run_until(done, [&] {
      auto whole = pub.loan();
      whole.set_from(foo(1, {}));
      EXPECT_TRUE(pub.publish(std::move(whole)));
      auto header_only = pub.loan();
      header_only.set_header(header(2, "only_header"));
      EXPECT_TRUE(pub.publish(std::move(header_only)));
    });
    ASSERT_TRUE(got) << topic;
    Foo want;
    want.header = header(2, "only_header");
    EXPECT_EQ(*got, want) << topic;
  }
}

// A standard type flux_std_adapters generates needs no include of the driver's.
TEST_F(WrapperTest, AStandardTypeIsCarried)
{
  std::optional<Imu> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Imu> sub(
    *node_, "imu", rclcpp::QoS(4), [&](const flux_wrapper::View<Imu> & v) {
      got = v.to_msg();
      done = true;
    });
  flux_wrapper::Publisher<Imu> pub(*node_, "imu", rclcpp::QoS(4));
  EXPECT_EQ(pub.uses_flux(), expect_flux());
  const Imu want = imu(4);
  run_until(done, [&] { EXPECT_TRUE(pub.publish(want)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, want);
}

// A root data of byte[] is written in place as uint8[] is; a type without a header has no stamp().
TEST_F(WrapperTest, AByteDataWithoutAHeader)
{
  static_assert(flux_wrapper::detail::kRootData<Blob> && !flux_wrapper::detail::kRootHeader<Blob>);
  std::optional<Blob> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Blob> sub(
    *node_, "blob", rclcpp::QoS(4), [&](const flux_wrapper::View<Blob> & v) {
      got = v.to_msg();
      done = v.size() == 3;
    });
  flux_wrapper::Publisher<Blob> pub(*node_, "blob", rclcpp::QoS(4));
  Blob want;
  want.seq = 11;
  run_until(done, [&] {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    loan.set_from(want);
    const auto data = loan.alloc_data(3);
    ASSERT_EQ(data.size(), 3u);
    data[0] = 1;
    data[1] = 2;
    data[2] = 3;
    EXPECT_TRUE(pub.publish(std::move(loan)));
  });
  ASSERT_TRUE(got);
  want.data = {1, 2, 3};
  EXPECT_EQ(*got, want);
}

// set_from() and without_data() copy the other fields and never the payload: no allocation the
// size of data, on either path, where to_msg() makes one.
TEST_F(WrapperTest, WithoutDataAndSetFromDoNotCopyThePayload)
{
  const std::size_t payload = std::size_t{4} << 20;
  const Foo big = foo(6, std::vector<std::uint8_t>(payload, 7));
  flux_wrapper::Publisher<Foo> pub(*node_, "foo_big", rclcpp::QoS(4));
  {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    largest = 0;
    watching = true;
    loan.set_from(big);
    watching = false;
    EXPECT_LT(largest, payload / 4);
  }

  struct Got
  {
    Foo fields;
    std::size_t fields_alloc = 0;
    Foo msg;
    std::size_t msg_alloc = 0;
  };
  std::optional<Got> got;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Foo> sub(
    *node_, "foo_big", rclcpp::QoS(4), [&](const flux_wrapper::View<Foo> & v) {
      Got g;
      largest = 0;
      watching = true;
      g.fields = v.without_data();
      g.fields_alloc = largest;
      largest = 0;
      g.msg = v.to_msg();
      g.msg_alloc = largest;
      watching = false;
      got = std::move(g);
      done = true;
    });
  run_until(done, [&] { EXPECT_TRUE(pub.publish(big)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(got->fields, without(big));
  EXPECT_LT(got->fields_alloc, payload / 4);
  EXPECT_EQ(got->msg, big);
  EXPECT_GE(got->msg_alloc, payload);
}

// Device::Cuda needs a root data: without one the publisher and the subscription are Host, each
// saying so with its topic.
TEST_F(WrapperTest, CudaOnATypeWithoutDataIsHost)
{
  const Logs logs;
  std::optional<Bar> got;
  std::atomic<bool> done{false};
  flux_wrapper::SubscriptionOptions sub_options;
  sub_options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Subscription<Bar> sub(
    *node_, "bar_cuda", rclcpp::QoS(4),
    [&](const flux_wrapper::View<Bar> & v) {
      got = v.to_msg();
      done = true;
    },
    sub_options);
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<Bar> pub(*node_, "bar_cuda", rclcpp::QoS(4), options);
  EXPECT_FALSE(pub.uses_device());
  const Bar want = bar(2);
  run_until(done, [&] { EXPECT_TRUE(pub.publish(want)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, want);
  EXPECT_EQ(Logs::count("bar_cuda: Device::Cuda, but the type has no root uint8[] data"), 2);
}

#ifdef FLUX_WRAPPER_CUDA
// What a subscription of a Foo got: as read_device(), and the host bytes and the other fields.
struct FooGot
{
  OnDevice<Foo> device;
  std::vector<std::uint8_t> host;
  Foo fields;
};
#endif

// A custom type's data in each pairing of publisher and subscription devices: a Cuda subscription
// has it in device memory, a Host one in host memory (of a Cuda publisher on a discrete GPU, the
// copy flux makes at take).
TEST_F(WrapperTest, CudaAnyTypeInEveryDevicePairing)
{
  SKIP_WITHOUT_CUDA();
#ifdef FLUX_WRAPPER_CUDA
  using flux_wrapper::Device;
  std::int32_t sec = 0;
  for (const Device pub_device : {Device::Cpu, Device::Cuda}) {
    for (const Device sub_device : {Device::Cpu, Device::Cuda}) {
      ++sec;
      const std::string topic = "foo_pair" + std::to_string(sec);
      const Foo want = foo(sec, {3, 1, 4, 1, 5, static_cast<std::uint8_t>(sec)});
      std::optional<FooGot> got;
      std::atomic<bool> done{false};
      flux_wrapper::SubscriptionOptions sub_options;
      sub_options.device = sub_device;
      flux_wrapper::Subscription<Foo> sub(
        *node_, topic, rclcpp::QoS(4),
        [&](const flux_wrapper::View<Foo> & v) {
          got = FooGot{read_device(v), {v.data().begin(), v.data().end()}, v.without_data()};
          done = true;
        },
        sub_options);
      flux_wrapper::PublisherOptions options;
      options.device = pub_device;
      flux_wrapper::Publisher<Foo> pub(*node_, topic, rclcpp::QoS(4), options);
      const bool cuda_pub = pub_device == Device::Cuda;
      ASSERT_EQ(pub.uses_device(), cuda_pub);
      const DeviceBytes source(want.data);
      run_until(done, [&] {
        auto loan = pub.loan();
        ASSERT_TRUE(loan);
        loan.set_from(want);
        const flux_wrapper::Buffer buf = loan.alloc_buffer(want.data.size());
        ASSERT_TRUE(buf);
        EXPECT_EQ(buf.on_device(), cuda_pub);
        ASSERT_TRUE(buf.copy_from(cuda_pub ? source.get() : want.data.data(), want.data.size()));
        EXPECT_TRUE(pub.publish(std::move(loan)));
      });
      const std::string pairing = (cuda_pub ? "Cuda->" : "Host->") +
                                  std::string(sub_device == Device::Cuda ? "Cuda" : "Host");
      ASSERT_TRUE(got) << pairing;
      if (sub_device == Device::Cuda) {
        EXPECT_TRUE(got->device.on_device) << pairing;
        EXPECT_EQ(got->device.device_bytes, want.data) << pairing;
      } else {
        EXPECT_FALSE(got->device.on_device) << pairing;
        EXPECT_EQ(got->host, want.data) << pairing;
      }
      EXPECT_EQ(got->device.msg, want) << pairing;
      EXPECT_EQ(got->fields, without(want)) << pairing;
    }
  }
#endif
}

// Of a type without accessors the log cannot name the field that did not fit, only the size.
TEST_F(WrapperTest, AnyTypePastTheSlotIsLoggedOnceWithItsSize)
{
  if (!expect_flux()) {
    GTEST_SKIP() << "the slot is a flux one";
  }
  const Logs logs;
  flux_wrapper::Publisher<Foo> pub(*node_, "foo_over_logged", rclcpp::QoS(4));
  Foo m = foo(5, {1});
  m.label = std::string(9000, 'l');
  for (int i = 0; i < 2; ++i) {
    auto loan = pub.loan();
    ASSERT_TRUE(loan);
    loan.set_from(m);
    EXPECT_FALSE(loan);
    EXPECT_FALSE(pub.publish(std::move(loan)));
  }
  EXPECT_FALSE(pub.publish(m));
  EXPECT_EQ(
    Logs::count("foo_over_logged: a message did not go out: the fields set from it did not fit "
                "the slot ("),
    1);
  EXPECT_EQ(Logs::count("B used before them); header.stamp 5.000000007. Later ones"), 1);
#ifdef FLUX_WRAPPER_CUDA
  if (!no_cuda().empty()) {
    return;
  }
  flux_wrapper::PublisherOptions options;
  options.device = flux_wrapper::Device::Cuda;
  flux_wrapper::Publisher<Foo> gpu(*node_, "foo_over_aux", rclcpp::QoS(4), options);
  ASSERT_TRUE(gpu.uses_device());
  m.label = std::string(5000, 'l');
  EXPECT_FALSE(gpu.publish(m));
  EXPECT_EQ(
    Logs::count("foo_over_aux: a message did not go out on Device::Cuda: the fields set from it "
                "did not fit the aux (4096 B, "),
    1);
#ifdef USE_FLUX_ENABLED
  // The header set alone is named; the Python test checks the same line.
  flux_wrapper::Publisher<Foo> header_gpu(*node_, "foo_header_aux", rclcpp::QoS(4), options);
  auto loan = header_gpu.loan();
  loan.set_header(header(5, std::string(5000, 'f')));
  EXPECT_FALSE(loan);
  EXPECT_FALSE(header_gpu.publish(std::move(loan)));
  const std::string used = std::to_string(flux_wrapper_test_msgs::flux_msg::Foo::kScalarBytes);
  EXPECT_EQ(
    Logs::count(
      "foo_header_aux: a message did not go out on Device::Cuda: header.frame_id (5000 B) did not "
      "fit the aux (4096 B, " +
      used + " B used before it); header.stamp 5.000000007. Later ones are not logged"),
    1);
#endif
#endif
}

// A Loan that found no free slot is false, and filling it anyway is harmless: publish() sends
// nothing. On Device::Cuda it has no aux to write the fields to.
TEST_F(WrapperTest, ALoanWithNoFreeSlotIsHarmlessToFill)
{
  if (!expect_flux()) {
    GTEST_SKIP() << "the slots are flux's";
  }
  for (const flux_wrapper::Device device :
       {flux_wrapper::Device::Cpu, flux_wrapper::Device::Cuda}) {
    flux_wrapper::PublisherOptions options;
    options.device = device;
    options.slot_count = 2;
    const std::string suffix = device == flux_wrapper::Device::Cuda ? "_cuda" : "_host";
    flux_wrapper::Publisher<Image> images(*node_, "full_image" + suffix, rclcpp::QoS(1), options);
    flux_wrapper::Publisher<Foo> foos(*node_, "full_foo" + suffix, rclcpp::QoS(1), options);
    std::vector<flux_wrapper::Loan<Image>> held_images;
    std::vector<flux_wrapper::Loan<Foo>> held_foos;
    for (int i = 0; i < 2; ++i) {
      held_images.push_back(images.loan());
      held_foos.push_back(foos.loan());
    }
    auto image_loan = images.loan();
    image_loan.set_from(image(1, {1}));
    EXPECT_FALSE(image_loan.alloc_buffer(1));
    EXPECT_FALSE(image_loan);
    const flux_wrapper::PublishResult r = images.publish(std::move(image_loan));
    EXPECT_FALSE(r);
    EXPECT_EQ(r.status(), flux_wrapper::Published::Backpressure);
    auto foo_loan = foos.loan();
    foo_loan.set_from(foo(1, {1}));
    EXPECT_FALSE(foo_loan.alloc_buffer(1));
    EXPECT_FALSE(foo_loan);
    EXPECT_FALSE(foos.publish(std::move(foo_loan)));
  }
}

// A false Loan hands out no data to write, as Python's None: publish() would drop it.
template <class MsgT>
void expect_no_data(flux_wrapper::Loan<MsgT> & loan, const std::string & what)
{
  EXPECT_FALSE(loan) << what;
  EXPECT_TRUE(loan.alloc_data(64).empty()) << what;
  EXPECT_FALSE(loan.alloc_buffer(64)) << what;
  EXPECT_FALSE(loan) << what;
}

// A Loan made without a publisher has no message: false on every path.
TEST_F(WrapperTest, ALoanWithoutAMessageHandsOutNoData)
{
  flux_wrapper::Loan<Image> image_loan;
  expect_no_data(image_loan, "Image");
  flux_wrapper::Loan<PointCloud2> cloud_loan;
  expect_no_data(cloud_loan, "PointCloud2");
  flux_wrapper::Loan<Foo> foo_loan;
  expect_no_data(foo_loan, "Foo");
}

// After a field past the slot (Host) or the aux (Cuda), or data past where it goes, the Loan is
// false and the data asked of it after is empty.
TEST_F(WrapperTest, AFalseLoanHandsOutNoData)
{
  if (!expect_flux()) {
    GTEST_SKIP() << "the slots are flux's";
  }
  std::vector<flux_wrapper::Device> devices{flux_wrapper::Device::Cpu};
#ifdef FLUX_WRAPPER_CUDA
  if (no_cuda().empty()) {
    devices.push_back(flux_wrapper::Device::Cuda);
  }
#endif
  for (const flux_wrapper::Device device : devices) {
    flux_wrapper::PublisherOptions options;
    options.device = device;
    const bool cuda = device == flux_wrapper::Device::Cuda;
    const std::string suffix = cuda ? "_cuda" : "_host";
    // Past the 8192-byte slot of a Host publisher and the 4096-byte aux of a Cuda one.
    const std::size_t over = 9000;
    flux_wrapper::Publisher<Image> images(*node_, "false_image" + suffix, rclcpp::QoS(1), options);
    flux_wrapper::Publisher<PointCloud2> clouds(
      *node_, "false_cloud" + suffix, rclcpp::QoS(1), options);
    flux_wrapper::Publisher<Foo> foos(*node_, "false_foo" + suffix, rclcpp::QoS(1), options);
    ASSERT_EQ(images.uses_device(), cuda);

    auto image_loan = images.loan();
    ASSERT_TRUE(image_loan);
    image_loan.set_header(header(1, std::string(over, 'f')));
    expect_no_data(image_loan, "Image header" + suffix);
    EXPECT_FALSE(images.publish(std::move(image_loan)));

    auto cloud_loan = clouds.loan();
    ASSERT_TRUE(cloud_loan);
    cloud_loan.set_header(header(1, std::string(over, 'f')));
    expect_no_data(cloud_loan, "PointCloud2 header" + suffix);
    EXPECT_FALSE(clouds.publish(std::move(cloud_loan)));

    Foo m = foo(1, {1});
    m.label = std::string(over, 'l');
    auto foo_loan = foos.loan();
    ASSERT_TRUE(foo_loan);
    foo_loan.set_from(m);
    expect_no_data(foo_loan, "Foo set_from" + suffix);
    EXPECT_FALSE(foos.publish(std::move(foo_loan)));

    // Data past the arena (Host) or the slot (Cuda) fails the Loan; smaller data after is empty.
    auto too_large = images.loan();
    ASSERT_TRUE(too_large);
    EXPECT_FALSE(too_large.alloc_buffer(std::size_t{1} << 40));
    expect_no_data(too_large, "Image data" + suffix);
    EXPECT_FALSE(images.publish(std::move(too_large)));
  }
}

}  // namespace

// The same code pairs two topics by stamp over flux and over ROS: each pair's Views read the
// frames they came from, and a View the synchronizer queued still reads after its callback.
TEST_F(WrapperTest, AMessageFiltersSynchronizerPairsTwoTopicsByStamp)
{
  using flux_wrapper::message_filters::Stamped;
  using flux_wrapper::message_filters::Subscriber;
  using Policy = ::message_filters::sync_policies::ApproximateTime<Stamped<Image>, Stamped<Image>>;
  const rclcpp::QoS qos(10);
  flux_wrapper::Publisher<Image> lpub(node(), "sync_left", qos);
  flux_wrapper::Publisher<Image> rpub(node(), "sync_right", qos);
  Subscriber<Image> left(node(), "sync_left", qos, 10);
  Subscriber<Image> right(node(), "sync_right", qos, 10);
  ::message_filters::Synchronizer<Policy> sync(Policy(10), left, right);
  std::vector<std::pair<std::int32_t, std::uint8_t>> got;
  std::atomic<bool> done{false};
  using Ptr = std::shared_ptr<const Stamped<Image>>;
  const auto on_pair = [&](const Ptr & l, const Ptr & r) {
    EXPECT_EQ(l->header.stamp, r->header.stamp);
    EXPECT_EQ(l->header.frame_id, "camera");
    EXPECT_EQ(l->view->stamp(), l->header.stamp);
    EXPECT_EQ(l->view->data()[0] + 100, r->view->data()[0]);
    got.emplace_back(l->header.stamp.sec, l->view->data()[0]);
    done = got.size() >= 3;
  };
  sync.registerCallback(std::bind(on_pair, std::placeholders::_1, std::placeholders::_2));
  std::int32_t sec = 0;
  run_until(done, [&] {
    ++sec;
    const auto v = static_cast<std::uint8_t>(sec);
    EXPECT_TRUE(lpub.publish(image(sec, {v, v, v, v, v, v})));
    EXPECT_TRUE(rpub.publish(image(sec, std::vector<std::uint8_t>(6, v + 100))));
  });
  ASSERT_GE(got.size(), 3U);
  for (const auto & [s, v] : got) {
    EXPECT_EQ(static_cast<std::uint8_t>(s), v);
  }
  EXPECT_EQ(left.subscription().uses_flux(), expect_flux());
  EXPECT_GE(left.forwarded(), 3U);
  EXPECT_EQ(left.unreadable(), 0U);
}

// A negative stamp is no ROS time: the filter counts it instead of forwarding it to a synchronizer
// that would throw on it.
TEST_F(WrapperTest, AMessageFiltersInputDropsANegativeStamp)
{
  const rclcpp::QoS qos(10);
  flux_wrapper::Publisher<Image> pub(node(), "sync_negative", qos);
  flux_wrapper::message_filters::Subscriber<Image> in(node(), "sync_negative", qos, 4);
  std::atomic<bool> done{false};
  run_until(done, [&] {
    EXPECT_TRUE(pub.publish(image(-1, {1, 2, 3, 4, 5, 6})));
    done = in.unreadable() > 0;
  });
  EXPECT_GT(in.unreadable(), 0U);
  EXPECT_EQ(in.forwarded(), 0U);
}
