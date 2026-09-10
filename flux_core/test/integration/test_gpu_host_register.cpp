#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/gpu_platform.hpp"
#include "flux/segment.hpp"

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

// The registered-host route (Route::ShmRegistered): an integrated GPU
// that cannot address a pageable mapping on its own, but can once the driver is told about it.
//
// These skip on every host that does not take that route -- a Thor takes ShmDirect, a dGPU host
// takes DeviceHandle, and a host with no CUDA takes none. What they pin is the one thing that
// separates this route from ShmDirect: the device address is NOT the host address, so anything
// that hands data() to a kernel is wrong here and right there.

namespace
{

// Device copies live here rather than in flux_core, for the reason test_gpu_vmm.cpp gives: the
// engine links no CUDA and calls no copy primitive.
struct Memcpy
{
  int (*d2h)(void *, unsigned long long, std::size_t) = nullptr;
  int (*sync)() = nullptr;

  static const Memcpy & get()
  {
    static const Memcpy m = [] {
      Memcpy c;
      void * lib = ::dlopen("libcuda.so.1", RTLD_LAZY | RTLD_LOCAL);
      if (lib == nullptr) return c;
      c.d2h = reinterpret_cast<decltype(c.d2h)>(::dlsym(lib, "cuMemcpyDtoH_v2"));
      c.sync = reinterpret_cast<decltype(c.sync)>(::dlsym(lib, "cuCtxSynchronize"));
      return c;
    }();
    return m;
  }

  bool ok() const { return d2h != nullptr && sync != nullptr; }
};

bool registered_route()
{
  return flux::gpu::probe().route == flux::gpu::Route::ShmRegistered;
}

// A declaration this host can fence on any thread. The legacy default stream is refused by
// require_fenceable(), so the channel has to be handed a created one.
flux::gpu::Stream fenceable()
{
  return flux::gpu::Stream::create();
}

// Distinct per process so parallel runs of this binary do not share a segment.
std::string gpu_name(const char * base)
{
  return flux::segment_name(std::string(base) + "." + std::to_string(::getpid()), 0x6F0);
}

}  // namespace

// The registration itself, without a channel in the way. Two facts: it succeeds on this route,
// and the address it answers with is a different number from the one that went in. The second is
// the whole reason FrameView splits data() from device_ptr() -- on ShmDirect they agree, and code
// written against that agreement is silently wrong here.
TEST(GpuHostRegister, TheDeviceAddressIsNotTheHostAddress)
{
  if (!registered_route())
    GTEST_SKIP() << "flux-cap:gpu-shm-registered host does not take the ShmRegistered route";
  std::vector<std::uint8_t> buffer(64 * 1024);
  const auto reg = flux::gpu::HostRegistration::create(buffer.data(), buffer.size(), false);
  ASSERT_TRUE(static_cast<bool>(reg));
  EXPECT_NE(reg.device_base(), static_cast<void *>(buffer.data()));
}

// A read-only mapping is what a subscriber's payload is after Segment::protect_payload(), and the
// driver refuses one unless it is told. Registering it as writable must fail rather than be
// papered over: silently registering the wrong way would work here and fault in a kernel.
TEST(GpuHostRegister, AReadOnlyMappingNeedsToBeDeclaredAsOne)
{
  if (!registered_route())
    GTEST_SKIP() << "flux-cap:gpu-shm-registered host does not take the ShmRegistered route";
  flux::Segment seg = flux::Segment::create_heap(4096, 2);
  ASSERT_TRUE(seg.valid());
  // Heap segments are never PROT_READ, so this only checks the flag reaches the driver: both
  // spellings are accepted for a writable mapping. The refusal itself is a driver behaviour the
  // shm path exercises; what is pinned here is that asking for read-only is not an error.
  EXPECT_NO_THROW({
    auto reg =
      flux::gpu::HostRegistration::create(seg.payload_base(), seg.layout().payload_bytes(), true);
    EXPECT_TRUE(static_cast<bool>(reg));
  });
}

// The channel-level contract. A declared stream on this route makes every payload address
// resolvable from a kernel, and the mapping from host to device address is the same rebase for
// every slot -- if it were not, slot n's device pointer would land in slot m's bytes.
TEST(GpuHostRegister, EverySlotRebasesByTheSameOffset)
{
  if (!registered_route())
    GTEST_SKIP() << "flux-cap:gpu-shm-registered host does not take the ShmRegistered route";
  constexpr std::uint32_t kSlots = 4;
  flux::Channel ch(flux::Segment::create_heap(4096, kSlots), fenceable());

  std::vector<std::uint8_t> payload(1024, 0x7E);

  ASSERT_EQ(ch.publish(payload.data(), payload.size()), flux::Published::Ok);
  flux::FrameView view = ch.take();
  ASSERT_TRUE(static_cast<bool>(view));

  // Both are non-null and they differ: the host address stays usable (this route keeps the slot
  // a byte range) while the device address is the registered one.
  ASSERT_NE(view.data(), nullptr);
  ASSERT_NE(view.device_ptr(), nullptr);
  EXPECT_NE(view.device_ptr(), view.data());
}

// End to end, with the GPU actually reading. A publisher writes host-side bytes into a slot and
// the consumer's device_ptr() is handed to a device copy: the bytes that come back are the ones
// that went in. This is the claim the route exists to make -- one set of bytes, both processors.
TEST(GpuHostRegister, TheGpuReadsWhatTheHostWrote)
{
  if (!registered_route())
    GTEST_SKIP() << "flux-cap:gpu-shm-registered host does not take the ShmRegistered route";
  const Memcpy & cp = Memcpy::get();
  if (!cp.ok())
    GTEST_SKIP() << "flux-cap:cuda-copy libcuda has no copy entry points to verify with";

  flux::Channel ch(flux::Segment::create_heap(4096, 4), fenceable());
  std::vector<std::uint8_t> payload(2048);
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<std::uint8_t>(i * 7 + 1);
  }
  ASSERT_EQ(ch.publish(payload.data(), payload.size()), flux::Published::Ok);

  flux::FrameView view = ch.take();
  ASSERT_TRUE(static_cast<bool>(view));
  ASSERT_NE(view.device_ptr(), nullptr);

  std::vector<std::uint8_t> readback(payload.size(), 0);
  const auto device = reinterpret_cast<unsigned long long>(view.device_ptr());
  ASSERT_EQ(cp.d2h(readback.data(), device, readback.size()), 0);
  ASSERT_EQ(cp.sync(), 0);
  EXPECT_EQ(readback, payload);
}

// The subscriber half, on a real shm segment rather than a heap one. This is the case the flag
// exists for: open() hands the mapping to Segment::protect_payload(), so the payload is PROT_READ
// by the time it is registered, and a registration that did not say so would have been refused.
// A heap segment never takes that branch, so without this test the read-only path is only ever
// asserted about and never run.
TEST(GpuHostRegister, ASubscribersReadOnlyPayloadStillRegisters)
{
  if (!registered_route())
    GTEST_SKIP() << "flux-cap:gpu-shm-registered host does not take the ShmRegistered route";
  const Memcpy & cp = Memcpy::get();
  if (!cp.ok())
    GTEST_SKIP() << "flux-cap:cuda-copy libcuda has no copy entry points to verify with";

  const std::string name = gpu_name("/flux_test/gpu_host_register");
  flux::gpu::Stream pub_stream;
  flux::gpu::Stream sub_stream;
  std::string failure;
  std::thread maker([&] {
    try {
      pub_stream = flux::gpu::Stream::create();
      sub_stream = flux::gpu::Stream::create();
    } catch (const std::exception & e) {
      failure = e.what();
    }
  });
  maker.join();
  if (!failure.empty())
    GTEST_SKIP() << "flux-cap:cuda-stream host cannot create a stream: " << failure;

  flux::Channel pub = flux::Channel::create(name, 4096, 4, 0x6F0, pub_stream);
  flux::Channel sub = flux::Channel::open(name, 0x6F0, sub_stream);
  ASSERT_TRUE(sub.payload_readonly()) << "this test is about the read-only mapping";

  std::vector<std::uint8_t> payload(1024);
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<std::uint8_t>(i * 3 + 5);
  }
  ASSERT_EQ(pub.publish(payload.data(), payload.size()), flux::Published::Ok);

  flux::FrameView v = sub.take();
  ASSERT_TRUE(static_cast<bool>(v));
  ASSERT_NE(v.device_ptr(), nullptr) << "a read-only mapping still resolves to a device address";
  EXPECT_NE(v.device_ptr(), v.data());

  std::vector<std::uint8_t> readback(payload.size(), 0);
  ASSERT_EQ(
    cp.d2h(readback.data(), reinterpret_cast<unsigned long long>(v.device_ptr()), readback.size()),
    0);
  ASSERT_EQ(cp.sync(), 0);
  EXPECT_EQ(readback, payload) << "the GPU reads the subscriber's read-only mapping";
}
