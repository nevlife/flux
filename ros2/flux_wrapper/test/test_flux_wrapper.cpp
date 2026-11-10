#include "flux_wrapper/flux_wrapper.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#ifdef FLUX_WRAPPER_CUDA
#include <cuda_runtime_api.h>
#endif

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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

#ifdef FLUX_WRAPPER_CUDA
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

// A message over the default slot size goes out once max_message_size makes room for it.
TEST_F(WrapperTest, LargerSlotTakesALargerMessage)
{
  constexpr std::size_t kBytes = 9u << 20;
  std::atomic<bool> done{false};
  flux_wrapper::Subscription<Image> sub(
    *node_, "large", rclcpp::QoS(1), [&](const flux_wrapper::View<Image> & v) {
      EXPECT_EQ(v.data().size(), kBytes);
      done = true;
    });
  flux_wrapper::PublisherOptions options;
  options.max_message_size = 10u << 20;
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

// A Cuda subscription gets the frame's device memory and no host copy on a discrete GPU.
TEST_F(WrapperTest, CudaSubscriptionKeepsFramesOnTheDevice)
{
  SKIP_WITHOUT_CUDA();
  if (!expect_flux()) {
    GTEST_SKIP() << "frames stay on the device only over flux";
  }
#if defined(FLUX_WRAPPER_CUDA) && defined(USE_FLUX_ENABLED)
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
  if (flux::gpu::probe().route == flux::gpu::Route::DeviceHandle) {
    EXPECT_EQ(got->host_bytes, 0u);
  }
  EXPECT_TRUE(got->stream);
  EXPECT_EQ(got->size, want.data.size());
  EXPECT_EQ(got->device_bytes, want.data);
  Image fields = want;
  fields.data = got->fields.data;
  EXPECT_EQ(got->fields, fields);
  EXPECT_EQ(got->msg, want);
#endif
}

// encoding and frame_id travel in the frame's aux bytes, which cap them; the ROS path has no cap.
TEST_F(WrapperTest, CudaStringOverItsCapFailsTheLoan)
{
  SKIP_WITHOUT_CUDA();
  if (!expect_flux()) {
    GTEST_SKIP() << "the caps are of a flux GPU frame";
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
  long_frame.set_header(header(1, std::string(257, 'f')));
  EXPECT_FALSE(long_frame);
  EXPECT_FALSE(pub.publish(std::move(long_frame)));

  auto long_encoding = pub.loan();
  ASSERT_TRUE(long_encoding);
  long_encoding.set_encoding(std::string(33, 'e'));
  EXPECT_FALSE(long_encoding);
  EXPECT_FALSE(pub.publish(std::move(long_encoding)));

  Image msg = image(1, {1});
  msg.header.frame_id = std::string(257, 'f');
  EXPECT_FALSE(pub.publish(msg));

  // At the caps a frame goes out whole.
  msg.header.frame_id = std::string(256, 'f');
  msg.encoding = std::string(32, 'e');
  run_until(done, [&] { EXPECT_TRUE(pub.publish(msg)); });
  ASSERT_TRUE(got);
  EXPECT_EQ(*got, msg);
}

#ifdef USE_FLUX_ENABLED
// The aux bytes come from another process. A frame whose aux does not parse is not delivered.
TEST_F(WrapperTest, GpuImageFrameWithABadAuxIsSkipped)
{
  if (!expect_flux()) {
    GTEST_SKIP() << "flux path only";
  }
  using flux_wrapper::detail::GpuImageAux;
  std::atomic<int> delivered{0};
  flux_wrapper::Subscription<Image> sub(
    *node_, "bad_aux", rclcpp::QoS(4), [&](const flux_wrapper::View<Image> & v) {
      EXPECT_EQ(v.frame_id(), "ok");
      ++delivered;
    });
  // A host channel will do: the subscription parses the aux bytes the same way.
  flux::Publisher raw(
    *node_, "bad_aux", flux_wrapper::detail::kGpuImageFingerprint, 4096, 4, flux::MemoryPolicy{},
    flux::Publisher::kDefaultMaxSubscribers, sizeof(GpuImageAux));
  const auto send = [&](const GpuImageAux & a, std::size_t aux_len) {
    flux::WriteSlot w = raw.loan();
    if (!w) {
      return;
    }
    std::memcpy(w.aux().data(), &a, sizeof(a));
    w.set_aux_len(aux_len);
    EXPECT_FALSE(flux::faulted(w.commit(0)));
  };

  GpuImageAux good{};
  good.version = GpuImageAux::kVersion;
  good.frame_id_len = 2;
  std::memcpy(good.frame_id.data(), "ok", 2);
  GpuImageAux bad_version = good;
  bad_version.version = GpuImageAux::kVersion + 1;
  GpuImageAux bad_frame_id = good;
  bad_frame_id.frame_id_len = GpuImageAux::kFrameIdCap + 1;
  GpuImageAux bad_encoding = good;
  bad_encoding.encoding_len = GpuImageAux::kEncodingCap + 1;
  const std::vector<std::pair<GpuImageAux, std::size_t>> bad = {
    {good, 10},
    {bad_version, sizeof(GpuImageAux)},
    {bad_frame_id, sizeof(GpuImageAux)},
    {bad_encoding, sizeof(GpuImageAux)}};
  for (const auto & [aux, len] : bad) {
    const std::uint64_t before = sub.rejected();
    std::atomic<bool> done{false};
    run_until(done, [&] {
      send(aux, len);
      done = sub.rejected() > before;
    });
    EXPECT_GT(sub.rejected(), before);
    EXPECT_EQ(delivered, 0);
  }

  std::atomic<bool> done{false};
  run_until(done, [&] {
    send(good, sizeof(good));
    done = delivered > 0;
  });
  EXPECT_GT(delivered, 0);
}
#endif

}  // namespace
