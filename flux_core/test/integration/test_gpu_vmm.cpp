#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/gpu_platform.hpp"
#include "flux/gpu_vmm.hpp"
#include "flux/segment.hpp"

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

// The dGPU payload region and the fd handshake behind it.
//
// These skip on every host without the DeviceHandle route, which is every hosted CI runner and
// the jetson runner both -- the first has no GPU, the second is integrated and takes ShmDirect.
// A dGPU runner is what makes them run at all.

namespace
{

// Device copies live here rather than in flux_core. The engine deliberately links no CUDA and
// calls no copy primitive; a test that needs to see bytes move opens the driver
// itself rather than growing the engine a function it must not have.
struct Memcpy
{
  int (*h2d)(unsigned long long, const void *, std::size_t) = nullptr;
  int (*d2h)(void *, unsigned long long, std::size_t) = nullptr;
  int (*sync)() = nullptr;

  static const Memcpy & get()
  {
    static const Memcpy m = [] {
      Memcpy c;
      void * lib = ::dlopen("libcuda.so.1", RTLD_LAZY | RTLD_LOCAL);
      if (lib == nullptr) return c;
      c.h2d = reinterpret_cast<decltype(c.h2d)>(::dlsym(lib, "cuMemcpyHtoD_v2"));
      c.d2h = reinterpret_cast<decltype(c.d2h)>(::dlsym(lib, "cuMemcpyDtoH_v2"));
      c.sync = reinterpret_cast<decltype(c.sync)>(::dlsym(lib, "cuCtxSynchronize"));
      return c;
    }();
    return m;
  }

  bool ok() const { return h2d != nullptr && d2h != nullptr && sync != nullptr; }
};

bool device_handle_route()
{
  return flux::gpu::probe().route == flux::gpu::Route::DeviceHandle;
}

std::vector<std::uint8_t> pattern(std::size_t n)
{
  std::vector<std::uint8_t> v(n);
  for (std::size_t i = 0; i < n; ++i) v[i] = static_cast<std::uint8_t>(i * 7u + 11u);
  return v;
}

constexpr std::size_t kBytes = 4u << 20;
constexpr const char * kEnvEndpoint = "FLUX_TEST_GPU_ENDPOINT";

flux::SegmentId fake_id(std::uint64_t ino)
{
  flux::SegmentId id;
  id.dev = 64;
  id.ino = ino;
  return id;
}

}  // namespace

TEST(GpuVmmEndpoint, NamesTheSegmentObjectRatherThanItsName)
{
  const std::string a = flux::gpu::device_endpoint(fake_id(1234));
  const std::string b = flux::gpu::device_endpoint(fake_id(1235));
  EXPECT_NE(a, b) << "two segment objects must not share an endpoint";
  EXPECT_EQ(a, flux::gpu::device_endpoint(fake_id(1234))) << "derivation must be stable";
  EXPECT_LT(a.size(), 108u) << "an abstract AF_UNIX address is bounded by sun_path";
  EXPECT_EQ(a.find('/'), std::string::npos) << "abstract: no filesystem entry to leave behind";
}

TEST(GpuVmm, ImportWithNoPublisherFails)
{
  if (!device_handle_route())
    GTEST_SKIP() << "flux-cap:gpu-device-handle host has no DeviceHandle route";
  // Nobody bound this endpoint. The failure is the transient one a subscriber retries through,
  // not a crash and not a silent empty mapping.
  EXPECT_THROW(
    flux::gpu::DeviceImport::open(flux::gpu::device_endpoint(fake_id(999999)), kBytes, 0),
    std::runtime_error);
}

TEST(GpuVmm, RegionIsRoundedToTheAllocationGranularity)
{
  if (!device_handle_route())
    GTEST_SKIP() << "flux-cap:gpu-device-handle host has no DeviceHandle route";
  const std::size_t granularity = flux::gpu::device_granularity(0);
  ASSERT_GT(granularity, 0u);

  // One byte over a granularity multiple: the region has to grow, never truncate. A truncated
  // region would put the last slot's payload outside the allocation.
  flux::gpu::DeviceAlloc a =
    flux::gpu::DeviceAlloc::create(flux::gpu::device_endpoint(fake_id(4001)), granularity + 1, 0);
  EXPECT_EQ(a.bytes(), granularity * 2);
  EXPECT_NE(a.base(), nullptr);
  EXPECT_EQ(a.device(), 0);
}

TEST(GpuVmm, ExportedRegionIsImportableAndTheBytesAreShared)
{
  if (!device_handle_route())
    GTEST_SKIP() << "flux-cap:gpu-device-handle host has no DeviceHandle route";
  const Memcpy & cp = Memcpy::get();
  if (!cp.ok())
    GTEST_SKIP() << "flux-cap:cuda-copy libcuda has no copy entry points to verify with";

  const std::string endpoint = flux::gpu::device_endpoint(fake_id(4002));
  flux::gpu::DeviceAlloc pub = flux::gpu::DeviceAlloc::create(endpoint, kBytes, 0);
  flux::gpu::DeviceImport sub = flux::gpu::DeviceImport::open(endpoint, kBytes, 0);

  ASSERT_NE(pub.base(), nullptr);
  ASSERT_NE(sub.base(), nullptr);
  EXPECT_EQ(pub.bytes(), sub.bytes()) << "both sides must size the region the same way";
  EXPECT_NE(pub.base(), sub.base()) << "an import is its own mapping, not the exporter's address";

  const std::vector<std::uint8_t> written = pattern(kBytes);
  ASSERT_EQ(cp.h2d(reinterpret_cast<unsigned long long>(pub.base()), written.data(), kBytes), 0);
  ASSERT_EQ(cp.sync(), 0);

  std::vector<std::uint8_t> read(kBytes, 0);
  ASSERT_EQ(cp.d2h(read.data(), reinterpret_cast<unsigned long long>(sub.base()), kBytes), 0);
  EXPECT_EQ(read, written) << "the importer must see the exporter's bytes, not a copy of nothing";

  // The other direction too: both mappings are read-write, which is what lets a subscriber write
  // the refcount side of the protocol into a region the publisher also holds.
  std::vector<std::uint8_t> back(kBytes, 0x5A);
  ASSERT_EQ(cp.h2d(reinterpret_cast<unsigned long long>(sub.base()), back.data(), kBytes), 0);
  ASSERT_EQ(cp.sync(), 0);
  std::vector<std::uint8_t> seen(kBytes, 0);
  ASSERT_EQ(cp.d2h(seen.data(), reinterpret_cast<unsigned long long>(pub.base()), kBytes), 0);
  EXPECT_EQ(seen, back);
}

TEST(GpuVmm, SlotOffsetsLandInsideTheRegion)
{
  if (!device_handle_route())
    GTEST_SKIP() << "flux-cap:gpu-device-handle host has no DeviceHandle route";
  const Memcpy & cp = Memcpy::get();
  if (!cp.ok())
    GTEST_SKIP() << "flux-cap:cuda-copy libcuda has no copy entry points to verify with";

  // The whole point of one allocation: slot s is base + s * stride, exactly as on the
  // host path, and every slot is addressable without a per-slot handle.
  constexpr std::uint32_t kSlots = 8;
  constexpr std::size_t kStride = 64u << 10;
  const std::string endpoint = flux::gpu::device_endpoint(fake_id(4003));
  flux::gpu::DeviceAlloc pub = flux::gpu::DeviceAlloc::create(endpoint, kStride * kSlots, 0);
  flux::gpu::DeviceImport sub = flux::gpu::DeviceImport::open(endpoint, kStride * kSlots, 0);
  ASSERT_GE(pub.bytes(), kStride * kSlots);

  for (std::uint32_t s = 0; s < kSlots; ++s) {
    const std::vector<std::uint8_t> mark(kStride, static_cast<std::uint8_t>(s + 1));
    auto * dst = static_cast<std::uint8_t *>(pub.base()) + s * kStride;
    ASSERT_EQ(cp.h2d(reinterpret_cast<unsigned long long>(dst), mark.data(), kStride), 0);
  }
  ASSERT_EQ(cp.sync(), 0);

  for (std::uint32_t s = 0; s < kSlots; ++s) {
    std::vector<std::uint8_t> got(kStride, 0);
    auto * src = static_cast<std::uint8_t *>(sub.base()) + s * kStride;
    ASSERT_EQ(cp.d2h(got.data(), reinterpret_cast<unsigned long long>(src), kStride), 0);
    EXPECT_EQ(got, std::vector<std::uint8_t>(kStride, static_cast<std::uint8_t>(s + 1)))
      << "slot " << s << " read back another slot's bytes";
  }
}

// The child half of the cross-process case. It runs only in the process the test below execs, so
// it skips when run directly. A fresh process rather than a bare fork: this one has to call CUDA,
// and a child forked from a process that already initialized the driver may not.
TEST(GpuVmmChild, ImportsAndVerifies)
{
  const char * endpoint = std::getenv(kEnvEndpoint);
  if (endpoint == nullptr) GTEST_SKIP() << "flux-cap:exec-child not the exec'd importer";

  const Memcpy & cp = Memcpy::get();
  ASSERT_TRUE(cp.ok());
  flux::gpu::DeviceImport sub = flux::gpu::DeviceImport::open(endpoint, kBytes, 0);
  ASSERT_NE(sub.base(), nullptr);

  std::vector<std::uint8_t> read(kBytes, 0);
  ASSERT_EQ(cp.d2h(read.data(), reinterpret_cast<unsigned long long>(sub.base()), kBytes), 0);
  EXPECT_EQ(read, pattern(kBytes));
}

TEST(GpuVmm, AnotherProcessImportsTheSameBytes)
{
  if (!device_handle_route())
    GTEST_SKIP() << "flux-cap:gpu-device-handle host has no DeviceHandle route";
  const Memcpy & cp = Memcpy::get();
  if (!cp.ok())
    GTEST_SKIP() << "flux-cap:cuda-copy libcuda has no copy entry points to verify with";

  const std::string endpoint = flux::gpu::device_endpoint(fake_id(4004));
  flux::gpu::DeviceAlloc pub = flux::gpu::DeviceAlloc::create(endpoint, kBytes, 0);
  const std::vector<std::uint8_t> written = pattern(kBytes);
  ASSERT_EQ(cp.h2d(reinterpret_cast<unsigned long long>(pub.base()), written.data(), kBytes), 0);
  ASSERT_EQ(cp.sync(), 0);

  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::setenv(kEnvEndpoint, endpoint.c_str(), 1);
    char arg0[] = "flux_gpu_import_child";
    char filter[] = "--gtest_filter=GpuVmmChild.ImportsAndVerifies";
    char * argv[] = {arg0, filter, nullptr};
    ::execv("/proc/self/exe", argv);
    ::_exit(127);  // exec failed: distinct from any gtest status
  }
  int status = 0;
  ASSERT_EQ(::waitpid(pid, &status, 0), pid);
  ASSERT_TRUE(WIFEXITED(status)) << "the importer process did not exit normally";
  EXPECT_EQ(WEXITSTATUS(status), 0) << "a separate process could not read the exported region";
}

// ---- the engine over a device-backed segment ----

namespace
{

constexpr std::uint64_t kFp = 0xD61CE0001ULL;

// Channel::create takes the signpost name, not a raw topic: deriving it is the caller's job
// above core.
std::string channel_name(const char * tag)
{
  return flux::signpost_name(
    std::string("/flux_test/gpu_dev/") + tag + "." + std::to_string(::getpid()), kFp);
}

}  // namespace

// The point of all of it: publish and take run unchanged over a payload that is not in the shm
// object. Nothing in the slot protocol was told the payload moved.
TEST(GpuVmmChannel, LoanCommitAndTakeCarryDeviceBytes)
{
  if (!device_handle_route())
    GTEST_SKIP() << "flux-cap:gpu-device-handle host has no DeviceHandle route";
  const Memcpy & cp = Memcpy::get();
  if (!cp.ok())
    GTEST_SKIP() << "flux-cap:cuda-copy libcuda has no copy entry points to verify with";

  constexpr std::uint32_t kSlot = 1u << 20;
  const std::string name = channel_name("roundtrip");
  flux::Channel pub = flux::Channel::create(name, kSlot, 4, kFp, flux::gpu::Stream::create());
  flux::Channel sub = flux::Channel::open(name, kFp, flux::gpu::Stream::create());

  flux::WriteSlot ws = pub.loan();
  ASSERT_TRUE(ws);
  ASSERT_NE(ws.device_ptr(), nullptr) << "a device-backed slot must name an address a kernel uses";

  const std::vector<std::uint8_t> written = pattern(kSlot);
  ASSERT_EQ(
    cp.h2d(reinterpret_cast<unsigned long long>(ws.device_ptr()), written.data(), kSlot), 0);
  ASSERT_EQ(ws.commit(kSlot), flux::Published::Ok);

  flux::FrameView v = sub.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.size(), kSlot);
  ASSERT_NE(v.device_ptr(), nullptr);

  std::vector<std::uint8_t> read(kSlot, 0);
  ASSERT_EQ(cp.d2h(read.data(), reinterpret_cast<unsigned long long>(v.device_ptr()), kSlot), 0);
  EXPECT_EQ(read, written) << "the subscriber read another region than the publisher wrote";
}

// The host copy path is closed, because the slot is a pointer no host store may follow.
TEST(GpuVmmChannel, PublishOfAHostBufferIsRefused)
{
  if (!device_handle_route())
    GTEST_SKIP() << "flux-cap:gpu-device-handle host has no DeviceHandle route";
  const std::string name = channel_name("nopublish");
  flux::Channel pub = flux::Channel::create(name, 4096, 4, kFp, flux::gpu::Stream::create());

  const std::vector<std::uint8_t> host(64, 7);
  EXPECT_EQ(pub.publish(host.data(), host.size()), flux::Published::WrongDevice)
    << "the refusal has to name itself; a caller cannot retry its way out of this one";
  // Not dropped(). That counter is a backpressure rate -- the ring was full and the frame was
  // shed -- and this frame could never have gone out as written. Counting it there would put a
  // wiring mistake into the number a caller watches for a slow consumer.
  EXPECT_EQ(pub.dropped(), 0u) << "WrongDevice is a fault, not backpressure";
}

// A device-backed channel is GPU-only, and the refusal is permanent rather than transient
// -- SegmentMismatch is the type a caller must not retry through.
TEST(GpuVmmChannel, AHostSubscriberIsRefusedAtAttach)
{
  if (!device_handle_route())
    GTEST_SKIP() << "flux-cap:gpu-device-handle host has no DeviceHandle route";
  const std::string name = channel_name("nohostsub");
  flux::Channel pub = flux::Channel::create(name, 4096, 4, kFp, flux::gpu::Stream::create());
  EXPECT_THROW(flux::Channel::open(name, kFp), flux::SegmentMismatch);
}

// A publisher restart makes a new segment, and its identity is what the endpoint is derived from,
// so a re-attach has to import a different region than the one it is holding. The host
// path re-attaches by swapping a mapping; this checks the device half follows.
TEST(GpuVmmChannel, ASubscriberFollowsARestartOntoANewDeviceRegion)
{
  if (!device_handle_route())
    GTEST_SKIP() << "flux-cap:gpu-device-handle host has no DeviceHandle route";
  const Memcpy & cp = Memcpy::get();
  if (!cp.ok())
    GTEST_SKIP() << "flux-cap:cuda-copy libcuda has no copy entry points to verify with";

  constexpr std::uint32_t kSlot = 4096;
  const std::string name = channel_name("restart");

  const auto put = [&cp](flux::Channel & ch, std::uint8_t mark) {
    flux::WriteSlot ws = ch.loan();
    if (!ws) return false;
    const std::vector<std::uint8_t> bytes(kSlot, mark);
    if (cp.h2d(reinterpret_cast<unsigned long long>(ws.device_ptr()), bytes.data(), kSlot) != 0) {
      return false;
    }
    return ws.commit(kSlot) == flux::Published::Ok;
  };
  const auto first_byte = [&cp](const flux::FrameView & v) {
    std::uint8_t b = 0;
    EXPECT_EQ(cp.d2h(&b, reinterpret_cast<unsigned long long>(v.device_ptr()), 1), 0);
    return b;
  };

  std::optional<flux::Channel> pub;
  pub.emplace(flux::Channel::create(name, kSlot, 4, kFp, flux::gpu::Stream::create()));
  flux::Channel sub = flux::Channel::open(name, kFp, flux::gpu::Stream::create());
  ASSERT_TRUE(put(*pub, 0xA1));
  {
    flux::FrameView v = sub.take();
    ASSERT_TRUE(v);
    EXPECT_EQ(first_byte(v), 0xA1);
  }  // released: a re-attach must not swap a mapping a live view still aliases
  const std::uint32_t gen_before = sub.attach_generation();

  pub.reset();  // unlinks the segment and stops its endpoint server
  flux::Channel pub2 = flux::Channel::create(name, kSlot, 4, kFp, flux::gpu::Stream::create());

  // Keep publishing while polling, as a restarted publisher does. A re-attach joins the new
  // stream, and the default Volatile durability starts from the join -- so a frame committed
  // before the subscriber got there is skipped by design, not lost by the device path.
  bool arrived = false;
  for (int i = 0; i < 200 && !arrived; ++i) {
    ASSERT_TRUE(put(pub2, 0xB2));
    flux::FrameView v = sub.take();
    if (v) {
      EXPECT_EQ(first_byte(v), 0xB2) << "read the old region after following the rotation";
      arrived = true;
    }
  }
  EXPECT_TRUE(arrived) << "the subscriber never followed the publisher onto the new segment";
  EXPECT_GT(sub.attach_generation(), gen_before) << "no re-attach happened";
}
