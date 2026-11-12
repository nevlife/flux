#include "flux_wrapper/flux_wrapper.hpp"

#include <gtest/gtest.h>
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
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
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
    flux::Publisher raw(
      *node_, topic, flux_wrapper::detail::Gpu<MsgT>::kFingerprint, 4096, 4, flux::MemoryPolicy{},
      flux::Publisher::kDefaultMaxSubscribers, flux_wrapper::detail::Gpu<MsgT>::kAuxBytes);
    const auto send = [&](const std::vector<std::uint8_t> & a) {
      flux::WriteSlot w = raw.loan();
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
      pub.publish(std::move(loan));
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
    EXPECT_EQ(e.status(), FLUX_INVALID_ARGUMENT);
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

  const PointCloud2 want = cloud(1, 2);
  run_until(done, [&] { EXPECT_TRUE(pub.publish(want)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, want);
  EXPECT_EQ(got_on_device, cuda);
}

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

// A C++ GPU frame read and written again by the Python flux_wrapper: each side reads the other's
// aux layout.
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

// Each publisher and subscription that cannot have Cuda says so, with its topic.
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

}  // namespace
