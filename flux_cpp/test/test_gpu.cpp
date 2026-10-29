// On the host GPU, skipped where there is none. The device writes and reads with the driver's
// memset and copy, so no kernel has to be compiled.
#include "flux/channel.hpp"
#include "flux/publisher.hpp"
#include "flux/subscription.hpp"

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
    if (!mark_.empty() && mark_.back() == '\n') mark_.pop_back();
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

TEST_F(Gpu, PublisherAndSubscriptionTakeADevice)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
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

TEST_F(Gpu, ABoundedFenceWaitsForWorkThatEndsInTime)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
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
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
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
