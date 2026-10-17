// On the host GPU, skipped where there is none. The device writes and reads with the driver's
// memset and copy, so no kernel has to be compiled.
#include "flux2/channel.hpp"
#include "flux2/publisher.hpp"
#include "flux2/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
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

const Driver * driver()
{
  static const Driver d = [] {
    Driver out;
    void * lib = dlopen("libcuda.so.1", RTLD_LAZY | RTLD_LOCAL);
    if (lib == nullptr) return out;
    // NOLINTBEGIN(*-reinterpret-cast): dlsym hands back an untyped address.
    out.memset = reinterpret_cast<MemsetAsync>(dlsym(lib, "cuMemsetD8Async"));
    out.copy = reinterpret_cast<CopyDtoH>(dlsym(lib, "cuMemcpyDtoH_v2"));
    // NOLINTEND(*-reinterpret-cast)
    return out;
  }();
  return d.memset != nullptr && d.copy != nullptr ? &d : nullptr;
}

std::uint64_t addr(const void * p)
{
  return reinterpret_cast<std::uintptr_t>(p);  // NOLINT(*-reinterpret-cast)
}

std::vector<unsigned char> read_back(const void * device, std::size_t n)
{
  std::vector<unsigned char> out(n);
  EXPECT_EQ(driver()->copy(out.data(), addr(device), n), 0);
  return out;
}

bool all(const std::vector<unsigned char> & v, unsigned char b)
{
  for (const unsigned char x : v) {
    if (x != b) return false;
  }
  return true;
}

// The exit status of a shell command, and its standard output in out.
int shell(const std::string & cmd, std::string & out)
{
  FILE * p = popen(cmd.c_str(), "r");  // NOLINT(bugprone-command-processor): our own script
  if (p == nullptr) return -1;
  std::array<char, 256> buf{};
  while (std::fgets(buf.data(), buf.size(), p) != nullptr) out += buf.data();
  const int status = pclose(p);
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

std::string unique_name(const char * test)
{
  return flux2::detail::signpost_name("/gpu" + std::to_string(getpid()) + "_" + test);
}

class Gpu : public ::testing::Test
{
protected:
  // A GPU or IOMMU fault in the kernel log (scripts/kernel_log.sh) fails the run: one can leave no
  // CUDA error behind.
  static void SetUpTestSuite()
  {
    mark_.clear();
    readable_ = shell(FLUX2_KERNEL_LOG " mark", mark_) == 0;
    if (!mark_.empty() && mark_.back() == '\n') mark_.pop_back();
  }
  static void TearDownTestSuite()
  {
    if (!readable_) {
      std::fputs("the kernel log is not readable without root; not judged by it\n", stderr);
      return;
    }
    std::string faults;
    EXPECT_EQ(shell(FLUX2_KERNEL_LOG " check '" + mark_ + "' 2>&1", faults), 0) << faults;
  }

  void SetUp() override
  {
    platform_ = flux2::gpu::probe();
    if (platform_.route == flux2::gpu::Route::None || driver() == nullptr) {
      GTEST_SKIP() << "no GPU route: " << platform_.reason;
    }
  }
  flux2::gpu::Platform platform_;
  static inline std::string mark_;
  static inline bool readable_ = false;
};

TEST_F(Gpu, TheProbeNamesTheRouteAndTheAttributes)
{
  RecordProperty("route", static_cast<int>(platform_.route));
  EXPECT_TRUE(platform_.reason.empty());
  EXPECT_GE(platform_.device, 0);
  EXPECT_EQ(flux2::gpu::wedged(), 0);
}

TEST_F(Gpu, TheLegacyDefaultStreamIsRefused)
{
  EXPECT_THROW(flux2::gpu::Stream::from_native(nullptr), flux2::Error);
}

// What a kernel writes into a loan, after the commit's fence, both kinds of subscriber read.
TEST_F(Gpu, AFrameWrittenOnTheDeviceReachesHostAndDeviceSubscribers)
{
  if (platform_.route == flux2::gpu::Route::DeviceHandle) GTEST_SKIP() << "host reads need shm";
  const std::string name = unique_name("loan");
  auto pub = flux2::detail::Channel::create(
    name, 1 << 16, 4, flux2::kNoSchema, {}, FLUX2_DEFAULT_MAX_SUBSCRIBERS,
    flux2::gpu::Stream::create());
  flux2::WriteSlot w = pub.loan();
  ASSERT_TRUE(w);
  ASSERT_NE(w.device_ptr(), nullptr);
  ASSERT_EQ(driver()->memset(addr(w.device_ptr()), 0x6b, 4096, w.stream()), 0);
  ASSERT_EQ(w.commit(4096), flux2::Published::Ok);
  EXPECT_EQ(pub.fence_wait().commit_waits, 1U);

  flux2::QoS q(4);
  q.transient_local(4);
  flux2::detail::Watch watch(name);
  auto cpu = flux2::detail::Channel::open(watch, flux2::kNoSchema, q);
  flux2::FrameView v = cpu.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.device_ptr(), nullptr);
  EXPECT_EQ(static_cast<const unsigned char *>(v.data())[4095], 0x6b);
  v.release();

  auto dev =
    flux2::detail::Channel::open(watch, flux2::kNoSchema, q, {}, flux2::gpu::Stream::create());
  v = dev.take();
  ASSERT_TRUE(v);
  ASSERT_NE(v.device_ptr(), nullptr);
  EXPECT_TRUE(all(read_back(v.device_ptr(), 4096), 0x6b));
  v.release();
  EXPECT_EQ(dev.fence_wait().release_waits, 1U);
  EXPECT_EQ(dev.fence_failed(), 0U);
}

TEST_F(Gpu, PublisherAndSubscriptionTakeADevice)
{
  if (platform_.route == flux2::gpu::Route::DeviceHandle) GTEST_SKIP() << "host reads need shm";
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux2_gpu_test");
  const std::string topic = "/gpu" + std::to_string(getpid()) + "_node";
  flux2::Publisher pub(*node, topic, flux2::kNoSchema, 1 << 16, 4, flux2::Device::Cuda);
  flux2::QoS q(4);
  q.transient_local(4);
  flux2::Subscription sub(*node, topic, flux2::kNoSchema, q, {}, flux2::Device::Cuda);
  flux2::WriteSlot w = pub.loan();
  ASSERT_TRUE(w);
  ASSERT_EQ(driver()->memset(addr(w.device_ptr()), 0x11, 64, w.stream()), 0);
  ASSERT_EQ(w.commit(64), flux2::Published::Ok);
  flux2::FrameView v = sub.take_blocking(1'000'000'000);
  ASSERT_TRUE(v);
  EXPECT_TRUE(all(read_back(v.device_ptr(), 64), 0x11));
  EXPECT_NE(v.stream(), nullptr);
  v.release();
  // The counters are as of the last take: a release is not a call into the subscription.
  EXPECT_FALSE(sub.take());
  EXPECT_EQ(sub.fence_wait().release_waits, 1U);
  EXPECT_EQ(sub.fence_failed(), 0U);
}

TEST_F(Gpu, ABoundedFenceWaitsForWorkThatEndsInTime)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux2_gpu_bound");
  const std::string topic = "/gpu" + std::to_string(getpid()) + "_bound";
  flux2::Publisher pub(*node, topic, flux2::kNoSchema, 1 << 16, 4, flux2::Device::Cuda);
  pub.set_fence_timeout(std::chrono::seconds(5));
  flux2::QoS q(4);
  q.transient_local(4);
  flux2::Subscription sub(*node, topic, flux2::kNoSchema, q, {}, flux2::Device::Cuda);
  sub.set_fence_timeout(std::chrono::seconds(5));
  flux2::WriteSlot w = pub.loan();
  ASSERT_TRUE(w);
  ASSERT_EQ(driver()->memset(addr(w.device_ptr()), 0x22, 64, w.stream()), 0);
  ASSERT_EQ(w.commit(64), flux2::Published::Ok);
  flux2::FrameView v = sub.take_blocking(1'000'000'000);
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
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("flux2_gpu_deferred");
  const std::string topic = "/gpu" + std::to_string(getpid()) + "_deferred";
  flux2::Publisher pub(*node, topic, flux2::kNoSchema, 1 << 16, 4, flux2::Device::Cuda);
  flux2::QoS q(4);
  q.transient_local(4);
  q.max_borrow(1);
  flux2::Subscription sub(*node, topic, flux2::kNoSchema, q, {}, flux2::Device::Cuda);
  sub.set_release_mode(flux2::gpu::ReleaseMode::Deferred);
  for (const unsigned char b : {0x31, 0x32}) {
    flux2::WriteSlot w = pub.loan();
    ASSERT_TRUE(w);
    ASSERT_EQ(driver()->memset(addr(w.device_ptr()), b, 64, w.stream()), 0);
    ASSERT_EQ(w.commit(64), flux2::Published::Ok);
  }
  flux2::FrameView v = sub.take_blocking(1'000'000'000);
  ASSERT_TRUE(v);
  EXPECT_TRUE(all(read_back(v.device_ptr(), 64), 0x31));
  v.release();
  // max_borrow 1: the second take needs the first slot back, which the take reaps once done.
  flux2::FrameView next = sub.take_blocking(1'000'000'000);
  ASSERT_TRUE(next);
  EXPECT_TRUE(all(read_back(next.device_ptr(), 64), 0x32));
  EXPECT_EQ(sub.fence_wait().release_waits, 0U);
  EXPECT_EQ(sub.fence_failed(), 0U);
}

}  // namespace
