// On the host GPU, skipped where there is none. The device writes and reads with the driver's
// memset and copy, so no kernel has to be compiled.
#include "flux/channel.hpp"
#include "flux/publisher.hpp"
#include "flux/subscription.hpp"
#include "flux_test/flux/blob.hpp"

#include <rclcpp/rclcpp.hpp>

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{

using MemsetAsync = int (*)(std::uint64_t, unsigned char, std::size_t, void *);
using CopyDtoH = int (*)(void *, std::uint64_t, std::size_t);

struct Driver
{
  MemsetAsync memset = nullptr;
  CopyDtoH copy = nullptr;
};

auto driver() -> const Driver *
{
  static const Driver d = [] {
    Driver out;
    void * lib = dlopen("libcuda.so.1", RTLD_LAZY | RTLD_LOCAL);
    if (lib == nullptr) {
      return out;
    }
    // NOLINTBEGIN(*-reinterpret-cast): dlsym hands back an untyped address.
    out.memset = reinterpret_cast<MemsetAsync>(dlsym(lib, "cuMemsetD8Async"));
    out.copy = reinterpret_cast<CopyDtoH>(dlsym(lib, "cuMemcpyDtoH_v2"));
    // NOLINTEND(*-reinterpret-cast)
    return out;
  }();
  return d.memset != nullptr && d.copy != nullptr ? &d : nullptr;
}

auto addr(const void * p) -> std::uint64_t
{
  return reinterpret_cast<std::uintptr_t>(p);  // NOLINT(*-reinterpret-cast)
}

auto read_back(const void * device, std::size_t n) -> std::vector<unsigned char>
{
  std::vector<unsigned char> out(n);
  EXPECT_EQ(driver()->copy(out.data(), addr(device), n), 0);
  return out;
}

auto all(const std::vector<unsigned char> & v, unsigned char b) -> bool
{
  for (const unsigned char x : v) {
    if (x != b) {
      return false;
    }
  }
  return true;
}

// The exit status of a shell command, and its standard output in out.
auto shell(const std::string & cmd, std::string & out) -> int
{
  FILE * p = popen(cmd.c_str(), "r");  // NOLINT(bugprone-command-processor): our own script
  if (p == nullptr) {
    return -1;
  }
  std::array<char, 256> buf{};
  while (std::fgets(buf.data(), buf.size(), p) != nullptr) {
    out += buf.data();
  }
  const int status = pclose(p);
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

auto unique_name(const char * test) -> std::string
{
  return flux::detail::signpost_name("/gpu" + std::to_string(getpid()) + "_" + test);
}

class Gpu : public ::testing::Test
{
protected:
  // A GPU or IOMMU fault in the kernel log (scripts/kernel_log.sh) fails the run: one can leave no
  // CUDA error behind.
  static void SetUpTestSuite()
  {
    mark_.clear();
    readable_ = shell(FLUX_KERNEL_LOG " mark", mark_) == 0;
    if (!mark_.empty() && mark_.back() == '\n') {
      mark_.pop_back();
    }
  }
  static void TearDownTestSuite()
  {
    if (!readable_) {
      std::fputs("the kernel log is not readable without root; not judged by it\n", stderr);
      return;
    }
    std::string faults;
    EXPECT_EQ(shell(FLUX_KERNEL_LOG " check '" + mark_ + "' 2>&1", faults), 0) << faults;
  }

  void SetUp() override
  {
    platform_ = flux::gpu::probe();
    if (platform_.route == flux::gpu::Route::None || driver() == nullptr) {
      GTEST_SKIP() << "no GPU route: " << platform_.reason;
    }
  }
  flux::gpu::Platform platform_;
  static inline std::string mark_;
  static inline bool readable_ = false;
};

TEST_F(Gpu, TheProbeNamesTheRouteAndTheAttributes)
{
  RecordProperty("route", static_cast<int>(platform_.route));
  EXPECT_TRUE(platform_.reason.empty());
  EXPECT_GE(platform_.device, 0);
  EXPECT_EQ(flux::gpu::wedged(), 0);
}

TEST_F(Gpu, TheLegacyDefaultStreamIsRefused)
{
  EXPECT_THROW(flux::gpu::Stream::from_native(nullptr), flux::Error);
}

// What a kernel writes into a loan, after the commit's fence, both kinds of subscriber read.
TEST_F(Gpu, AFrameWrittenOnTheDeviceReachesHostAndDeviceSubscribers)
{
  const std::string name = unique_name("loan");
  auto pub = flux::detail::Channel::create(
    name, 1 << 16, 4, flux::kNoSchema, {}, FLUX_DEFAULT_MAX_SUBSCRIBERS,
    flux::gpu::Stream::create());
  flux::WriteSlot w = pub.loan();
  ASSERT_TRUE(w);
  ASSERT_NE(w.device_ptr(), nullptr);
  ASSERT_EQ(driver()->memset(addr(w.device_ptr()), 0x6b, 4096, w.stream()), 0);
  ASSERT_EQ(w.commit(4096), flux::Published::Ok);
  EXPECT_EQ(pub.fence_wait().commit_waits, 1U);

  flux::QoS q(4);
  q.transient_local(4);
  flux::detail::Watch watch(name);
  auto cpu = flux::detail::Channel::open(watch, flux::kNoSchema, q);
  flux::FrameView v = cpu.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.device_ptr(), nullptr);
  EXPECT_EQ(static_cast<const unsigned char *>(v.data())[4095], 0x6b);
  v.release();

  auto dev =
    flux::detail::Channel::open(watch, flux::kNoSchema, q, {}, flux::gpu::Stream::create());
  v = dev.take();
  ASSERT_TRUE(v);
  ASSERT_NE(v.device_ptr(), nullptr);
  EXPECT_TRUE(all(read_back(v.device_ptr(), 4096), 0x6b));
  v.release();
  EXPECT_EQ(dev.fence_wait().release_waits, 1U);
  EXPECT_EQ(dev.fence_failed(), 0U);
}

// A loan of device memory aborted with work still queued on its stream, its channel dropped at
// once: that work must not land in device memory that is gone, which leaves a sticky
// CUDA_ERROR_LAUNCH_FAILED on the process and fails every later CUDA call.
TEST_F(Gpu, AnAbortedLoanWithWorkQueuedDoesNotOutliveItsMemory)
{
  constexpr std::size_t kBytes = std::size_t{64} << 20;
  for (int i = 0; i < 10; ++i) {
    auto pub = flux::detail::Channel::create(
      unique_name("abort") + std::to_string(i), kBytes, 2, flux::kNoSchema, {},
      FLUX_DEFAULT_MAX_SUBSCRIBERS, flux::gpu::Stream::create());
    flux::WriteSlot w = pub.loan();
    ASSERT_TRUE(w);
    ASSERT_NE(w.device_ptr(), nullptr);
    for (int k = 0; k < 20; ++k) {
      ASSERT_EQ(driver()->memset(addr(w.device_ptr()), 0x6b, kBytes, w.stream()), 0);
    }
    w.abort();
  }
  EXPECT_EQ(flux::gpu::wedged(), 0);
  EXPECT_NO_THROW(flux::gpu::Stream::create());
}

// The aux bytes stay in host memory, whatever the payload is, for both kinds of subscriber.
TEST_F(Gpu, AuxBytesReachHostAndDeviceSubscribers)
{
  const std::string name = unique_name("aux");
  auto pub = flux::detail::Channel::create(
    name, 1 << 16, 4, flux::kNoSchema, {}, FLUX_DEFAULT_MAX_SUBSCRIBERS,
    flux::gpu::Stream::create(), 64);
  flux::WriteSlot w = pub.loan();
  ASSERT_TRUE(w);
  ASSERT_EQ(w.aux().size(), 64u);
  std::memset(w.aux().data(), 0x5a, 10);
  w.set_aux_len(10);
  ASSERT_EQ(driver()->memset(addr(w.device_ptr()), 0x6b, 4096, w.stream()), 0);
  ASSERT_EQ(w.commit(4096), flux::Published::Ok);

  flux::QoS q(4);
  q.transient_local(4);
  flux::detail::Watch watch(name);
  for (const bool device : {false, true}) {
    auto sub = flux::detail::Channel::open(
      watch, flux::kNoSchema, q, {}, device ? flux::gpu::Stream::create() : flux::gpu::Stream{});
    flux::FrameView v = sub.take();
    ASSERT_TRUE(v);
    const auto aux = v.aux();
    ASSERT_EQ(aux.size(), 10u);
    for (const std::uint8_t b : aux) {
      EXPECT_EQ(b, 0x5a);
    }
  }
}

TEST_F(Gpu, PublisherAndSubscriptionTakeADevice)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("flux_gpu_test");
  const std::string topic = "/gpu" + std::to_string(getpid()) + "_node";
  flux::Publisher pub(*node, topic, flux::kNoSchema, 1 << 16, 4, flux::Device::Cuda);
  flux::QoS q(4);
  q.transient_local(4);
  flux::Subscription sub(*node, topic, flux::kNoSchema, q, {}, flux::Device::Cuda);
  flux::WriteSlot w = pub.loan();
  ASSERT_TRUE(w);
  ASSERT_EQ(driver()->memset(addr(w.device_ptr()), 0x11, 64, w.stream()), 0);
  ASSERT_EQ(w.commit(64), flux::Published::Ok);
  flux::FrameView v = sub.take_blocking(1'000'000'000);
  ASSERT_TRUE(v);
  EXPECT_TRUE(all(read_back(v.device_ptr(), 64), 0x11));
  EXPECT_NE(v.stream(), nullptr);
  v.release();
  // The counters are as of the last take: a release is not a call into the subscription.
  EXPECT_FALSE(sub.take());
  EXPECT_EQ(sub.fence_wait().release_waits, 1U);
  EXPECT_EQ(sub.fence_failed(), 0U);
}

// A generated type's publisher that declares a stream keeps host slots: the device field goes to
// the GPU arena, and the small ones stay in the slot, for both kinds of subscriber. A CPU reader
// of device memory reads its own copy.
TEST_F(Gpu, ADeviceFieldGoesThroughTheGpuArena)
{
  using Blob = flux_test::flux_msg::Blob;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("flux_gpu_arena");
  const std::string topic = "/gpu" + std::to_string(getpid()) + "_arena";
  flux::Publisher pub(*node, topic, Blob::kFingerprint, 1 << 16, 4, flux::Device::Cuda);
  EXPECT_FALSE(pub.device_backed());
  flux::QoS q(4);
  q.transient_local(4);
  flux::Subscription gpu(*node, topic, Blob::kFingerprint, q, {}, flux::Device::Cuda);
  flux::Subscription cpu(*node, topic, Blob::kFingerprint, q);
  // The second frame reaches past the chunks the subscriber imported for the first.
  for (const auto & [bytes, fill] :
       {std::pair<std::size_t, unsigned char>{3 << 20, 0x41}, {40 << 20, 0x42}}) {
    flux::WriteSlot w = pub.loan();
    ASSERT_TRUE(w);
    Blob::Builder b(w);
    b.set__name("frame");
    const flux::wire::DeviceSpan<std::uint8_t> d = b.alloc_device__data(bytes);
    ASSERT_EQ(d.size(), bytes);
    ASSERT_EQ(driver()->memset(addr(d.data()), fill, bytes, w.stream()), 0);
    ASSERT_EQ(b.commit__(), flux::Published::Ok);

    flux::FrameView v = gpu.take_blocking(1'000'000'000);
    ASSERT_TRUE(v);
    const Blob::View m(v);
    EXPECT_EQ(m.name(), "frame");
    const flux::wire::DeviceSpan<const std::uint8_t> got = m.data__device();
    ASSERT_EQ(got.size(), bytes);
    EXPECT_TRUE(all(read_back(got.data(), bytes), fill));
    EXPECT_TRUE(m.ok__());
    v.release();

    flux::FrameView c = cpu.take_blocking(1'000'000'000);
    ASSERT_TRUE(c);
    const Blob::View h(c);
    EXPECT_EQ(h.name(), "frame");
    EXPECT_TRUE(h.data__device().empty());
    ASSERT_EQ(h.data().size(), bytes);
    EXPECT_EQ(h.data()[0], fill);
    EXPECT_EQ(h.data()[bytes - 1], fill);
    EXPECT_TRUE(h.ok__());
  }
  EXPECT_EQ(gpu.fence_failed(), 0U);
}

// Each device field of a frame is a block of its own, nested ones too, and a CPU reader finds
// every one in its copy.
TEST_F(Gpu, EveryDeviceBlockOfAFrameReachesACpuReader)
{
  using Blob = flux_test::flux_msg::Blob;
  const std::string name = unique_name("arena_blocks");
  auto pub = flux::detail::Channel::create(
    name, 1 << 16, 4, Blob::kFingerprint, {}, FLUX_DEFAULT_MAX_SUBSCRIBERS,
    flux::gpu::Stream::create());
  flux::WriteSlot w = pub.loan();
  ASSERT_TRUE(w);
  Blob::Builder b(w);
  const auto data = b.alloc_device__data(5000);
  ASSERT_EQ(driver()->memset(addr(data.data()), 0x21, 5000, w.stream()), 0);
  const auto items = b.alloc__items(3);
  for (std::size_t i = 0; i < items.size(); ++i) {
    const auto ids = items[i].alloc_device__ids(100 * (i + 1));
    ASSERT_EQ(ids.size(), 100 * (i + 1));
    const auto fill = static_cast<unsigned char>(0x30 + i);
    ASSERT_EQ(driver()->memset(addr(ids.data()), fill, ids.size() * 2, w.stream()), 0);
  }
  ASSERT_EQ(b.commit__(), flux::Published::Ok);

  flux::QoS q(4);
  q.transient_local(4);
  flux::detail::Watch watch(name);
  auto cpu = flux::detail::Channel::open(watch, Blob::kFingerprint, q);
  flux::FrameView v = cpu.take();
  ASSERT_TRUE(v);
  const Blob::View m(v);
  ASSERT_EQ(m.data().size(), 5000u);
  EXPECT_EQ(m.data()[4999], 0x21);
  ASSERT_EQ(m.items__size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    const auto ids = m.items(i).ids();
    ASSERT_EQ(ids.size(), 100 * (i + 1));
    const auto fill = static_cast<std::uint16_t>((0x30 + i) * 0x101);
    EXPECT_EQ(ids[0], fill);
    EXPECT_EQ(ids[ids.size() - 1], fill);
  }
  EXPECT_TRUE(m.ok__());
  // A second View of the same frame reads the same copy.
  const Blob::View again(v);
  EXPECT_EQ(again.items(2).ids()[299], static_cast<std::uint16_t>(0x32 * 0x101));
}

// A slot's device block is freed when the slot is claimed again: frames that add up to more than
// the arena all go out.
TEST_F(Gpu, DeviceBlocksGoBackWithTheirSlots)
{
  using Blob = flux_test::flux_msg::Blob;
  const std::string name = unique_name("arena_reuse");
  auto pub = flux::detail::Channel::create(
    name, 1 << 16, 4, Blob::kFingerprint, {}, FLUX_DEFAULT_MAX_SUBSCRIBERS,
    flux::gpu::Stream::create());
  for (int i = 0; i < 40; ++i) {
    flux::WriteSlot w = pub.loan();
    ASSERT_TRUE(w);
    Blob::Builder b(w);
    ASSERT_EQ(b.alloc_device__data(std::size_t{40} << 20).size(), std::size_t{40} << 20) << i;
    ASSERT_EQ(b.commit__(), flux::Published::Ok) << i;
  }
}

// The other way: a CPU publisher's fields, in its slot and in its arena, reach a GPU reader on
// the device. A discrete GPU reads its copies, which the take and the View made.
TEST_F(Gpu, EveryCpuFieldOfAFrameReachesAGpuReader)
{
  using Blob = flux_test::flux_msg::Blob;
  if (platform_.route != flux::gpu::Route::DeviceHandle) {
    GTEST_SKIP() << "an integrated GPU has no device address for the arena yet";
  }
  const std::string name = unique_name("cpu_fields");
  auto pub = flux::detail::Channel::create(
    name, 1 << 16, 4, Blob::kFingerprint, {}, FLUX_DEFAULT_MAX_SUBSCRIBERS);
  flux::WriteSlot w = pub.loan();
  ASSERT_TRUE(w);
  Blob::Builder b(w);
  const auto data = b.alloc__data(std::size_t{3} << 20);
  ASSERT_EQ(data.size(), std::size_t{3} << 20);
  std::memset(data.data(), 0x5a, data.size());
  const std::array<std::size_t, 3> lens{10, 100'000, 20};
  const auto items = b.alloc__items(lens.size());
  for (std::size_t i = 0; i < lens.size(); ++i) {
    const auto ids = items[i].alloc__ids(lens.at(i));
    ASSERT_EQ(ids.size(), lens.at(i));
    for (auto & x : ids) {
      x = static_cast<std::uint16_t>(0x100 + i);
    }
  }
  ASSERT_EQ(b.commit__(), flux::Published::Ok);

  flux::QoS q(4);
  q.transient_local(4);
  flux::detail::Watch watch(name);
  const flux::gpu::Stream stream = flux::gpu::Stream::create();
  auto gpu = flux::detail::Channel::open(watch, Blob::kFingerprint, q, {}, stream);
  flux::FrameView v = gpu.take();
  ASSERT_TRUE(v);
  const Blob::View m(v);
  const auto d = m.data__device();
  ASSERT_EQ(d.size(), data.size());
  ASSERT_TRUE(stream.wait());
  EXPECT_TRUE(all(read_back(d.data(), d.size()), 0x5a));
  ASSERT_EQ(m.items__size(), lens.size());
  for (std::size_t i = 0; i < lens.size(); ++i) {
    const auto ids = m.items(i).ids__device();
    ASSERT_EQ(ids.size(), lens.at(i)) << i;
    const std::vector<unsigned char> got = read_back(ids.data(), ids.size() * 2);
    std::vector<std::uint16_t> back(ids.size());
    std::memcpy(back.data(), got.data(), got.size());
    for (const std::uint16_t x : back) {
      ASSERT_EQ(x, 0x100 + i) << i;
    }
  }
  EXPECT_TRUE(m.ok__());
}

// A raw tensor loan's channel keeps its device payload: the type registered nothing.
TEST_F(Gpu, AnUnregisteredFingerprintKeepsTheDevicePayload)
{
  if (platform_.route != flux::gpu::Route::DeviceHandle) {
    GTEST_SKIP() << "a host payload is the only one here";
  }
  auto pub = flux::detail::Channel::create(
    unique_name("raw"), 1 << 16, 4, 0x1234, {}, FLUX_DEFAULT_MAX_SUBSCRIBERS,
    flux::gpu::Stream::create());
  EXPECT_TRUE(pub.device_backed());
}

TEST_F(Gpu, ABoundedFenceWaitsForWorkThatEndsInTime)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("flux_gpu_bound");
  const std::string topic = "/gpu" + std::to_string(getpid()) + "_bound";
  flux::Publisher pub(*node, topic, flux::kNoSchema, 1 << 16, 4, flux::Device::Cuda);
  pub.set_fence_timeout(std::chrono::seconds(5));
  flux::QoS q(4);
  q.transient_local(4);
  flux::Subscription sub(*node, topic, flux::kNoSchema, q, {}, flux::Device::Cuda);
  sub.set_fence_timeout(std::chrono::seconds(5));
  flux::WriteSlot w = pub.loan();
  ASSERT_TRUE(w);
  ASSERT_EQ(driver()->memset(addr(w.device_ptr()), 0x22, 64, w.stream()), 0);
  ASSERT_EQ(w.commit(64), flux::Published::Ok);
  flux::FrameView v = sub.take_blocking(1'000'000'000);
  ASSERT_TRUE(v);
  EXPECT_TRUE(all(read_back(v.device_ptr(), 64), 0x22));
  v.release();
  EXPECT_FALSE(sub.take());
  EXPECT_EQ(pub.fence_failed(), 0U);
  EXPECT_EQ(sub.fence_failed(), 0U);
  EXPECT_EQ(sub.fence_wait().release_waits, 1U);
}

TEST_F(Gpu, ADeferredReleaseReturnsTheSlotOnceTheDeviceIsDone)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("flux_gpu_deferred");
  const std::string topic = "/gpu" + std::to_string(getpid()) + "_deferred";
  flux::Publisher pub(*node, topic, flux::kNoSchema, 1 << 16, 4, flux::Device::Cuda);
  flux::QoS q(4);
  q.transient_local(4);
  q.max_borrow(1);
  flux::Subscription sub(*node, topic, flux::kNoSchema, q, {}, flux::Device::Cuda);
  sub.set_release_mode(flux::gpu::ReleaseMode::Deferred);
  for (const unsigned char b : {0x31, 0x32}) {
    flux::WriteSlot w = pub.loan();
    ASSERT_TRUE(w);
    ASSERT_EQ(driver()->memset(addr(w.device_ptr()), b, 64, w.stream()), 0);
    ASSERT_EQ(w.commit(64), flux::Published::Ok);
  }
  flux::FrameView v = sub.take_blocking(1'000'000'000);
  ASSERT_TRUE(v);
  EXPECT_TRUE(all(read_back(v.device_ptr(), 64), 0x31));
  v.release();
  // max_borrow 1: the second take needs the first slot back, which the take reaps once done.
  flux::FrameView next = sub.take_blocking(1'000'000'000);
  ASSERT_TRUE(next);
  EXPECT_TRUE(all(read_back(next.device_ptr(), 64), 0x32));
  EXPECT_EQ(sub.fence_wait().release_waits, 0U);
  EXPECT_EQ(sub.fence_failed(), 0U);
}

}  // namespace
