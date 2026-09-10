#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/gpu_platform.hpp"
#include "flux/wire.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// Covers the GPU route probe. Every assertion here holds on a host with
// a discrete GPU, on a Jetson, and on a host with no CUDA at all -- the route is re-derived from
// the attributes the probe itself reports, so the test judges the rule rather than the machine.

namespace
{

using flux::gpu::Platform;
using flux::gpu::Route;

Route expected_route(const Platform & p)
{
  if (p.integrated && p.pageable_access) return Route::ShmDirect;
  if (p.integrated && p.host_register) return Route::ShmRegistered;
  if (p.vmm && p.posix_fd_handle) return Route::DeviceHandle;
  return Route::None;
}

}  // namespace

TEST(GpuPlatform, ProbeIsTotal)
{
  Platform p;
  ASSERT_NO_THROW(p = flux::gpu::probe());

  std::cerr << "gpu route=" << flux::gpu::to_string(p.route) << " integrated=" << p.integrated
            << " pageable=" << p.pageable_access << " host_register=" << p.host_register
            << " vmm=" << p.vmm << " fd=" << p.posix_fd_handle << " ipc_event=" << p.ipc_event
            << " reason=\"" << p.reason << "\"\n";
}

// A route and an explanation are alternatives, never both and never neither: a caller must not
// have to guess whether an empty reason means "fine" or "nobody looked".
TEST(GpuPlatform, ReasonIsSetExactlyWhenThereIsNoRoute)
{
  const Platform p = flux::gpu::probe();
  if (p.route == Route::None) {
    EXPECT_FALSE(p.reason.empty());
  } else {
    EXPECT_TRUE(p.reason.empty()) << p.reason;
  }
}

TEST(GpuPlatform, RouteFollowsTheAttributes)
{
  const Platform p = flux::gpu::probe();
  if (p.device < 0) {
    GTEST_SKIP() << "flux-cap:gpu-device no device probed: " << p.reason;
  }
  EXPECT_EQ(p.route, expected_route(p));
}

// The two shm routes are the claim that a host mapping is already the GPU buffer. Only an
// integrated device may make it -- on a discrete device the same capability bit is true and the
// answer is still no.
TEST(GpuPlatform, ShmRoutesRequireAnIntegratedDevice)
{
  const Platform p = flux::gpu::probe();
  if (p.host_memory_is_device_memory()) {
    EXPECT_TRUE(p.integrated);
    EXPECT_TRUE(p.pageable_access || p.host_register);
  }
  if (p.route == Route::DeviceHandle) {
    EXPECT_TRUE(p.vmm);
    EXPECT_TRUE(p.posix_fd_handle);
    EXPECT_FALSE(p.host_memory_is_device_memory());
  }
}

TEST(GpuPlatform, UnknownDeviceIsRefusedNotGuessed)
{
  for (int bad : {-1, 4096}) {
    const Platform p = flux::gpu::probe(bad);
    EXPECT_EQ(p.route, Route::None) << "device " << bad;
    EXPECT_FALSE(p.reason.empty()) << "device " << bad;
    EXPECT_EQ(p.device, -1) << "device " << bad;
    EXPECT_FALSE(p.host_memory_is_device_memory()) << "device " << bad;
  }
}

// The route is read once and cached by callers, so a probe that answered differently on the
// second call would put two routes in one process.
TEST(GpuPlatform, ProbeIsStable)
{
  const Platform a = flux::gpu::probe();
  const Platform b = flux::gpu::probe();
  EXPECT_EQ(a.route, b.route);
  EXPECT_EQ(a.integrated, b.integrated);
  EXPECT_EQ(a.pageable_access, b.pageable_access);
  EXPECT_EQ(a.vmm, b.vmm);
  EXPECT_EQ(a.posix_fd_handle, b.posix_fd_handle);
}

TEST(GpuPlatform, EveryRouteNamesItself)
{
  const Route all[] = {Route::None, Route::ShmDirect, Route::ShmRegistered, Route::DeviceHandle};
  for (Route r : all) {
    const std::string name = flux::gpu::to_string(r);
    EXPECT_FALSE(name.empty());
    for (Route other : all) {
      if (other != r) EXPECT_NE(name, flux::gpu::to_string(other));
    }
  }
}

// An undeclared stream is the host-only default: it fences nothing and must not depend on CUDA
// being present, since every channel that never mentions GPU carries one.
TEST(GpuStream, UndeclaredStreamFencesNothing)
{
  const flux::gpu::Stream s;
  EXPECT_FALSE(s.declared());
  EXPECT_EQ(s.native(), nullptr);
  EXPECT_TRUE(s.wait());
}

// CUDA's legacy default stream is the null handle, so a value test cannot tell it from "no
// stream". Construction is what declares.
TEST(GpuStream, ConstructingFromAHandleDeclaresEvenWhenItIsNull)
{
  const flux::gpu::Stream s(nullptr);
  EXPECT_TRUE(s.declared());
  EXPECT_EQ(s.native(), nullptr);

  int marker = 0;
  const flux::gpu::Stream t(&marker);
  EXPECT_TRUE(t.declared());
  EXPECT_EQ(t.native(), &marker);
}

// The seam this exists to hold: a fence that could not be taken must arrive as failure. This
// thread has no CUDA context, so the driver refuses -- and on a host with no driver at all the
// same call fails for a different reason. Either way the answer is false, never true.
TEST(GpuStream, AFenceThatCouldNotBeTakenIsNotAFenceThatPassed)
{
  const flux::gpu::Stream s(nullptr);
  EXPECT_FALSE(s.wait());
}

// The wiring of a declared stream into Channel/WriteSlot/FrameView.
// These run on any host: what a declaration does when it cannot be honoured is as much the
// contract as what it does when it can.

namespace
{

std::string gpu_name(const char * base)
{
  return flux::segment_name(std::string(base) + "." + std::to_string(::getpid()), 0x6F0);
}

// A declared stream whose fence fails: CUDA's legacy stream handle, which resolves against the
// calling thread's current context and so fails on a thread that has none. Measured: with a
// context current it succeeds, and every handle outside CUDA's three reserved values (0, 1, 2)
// segfaults the driver rather than returning an error -- so on a host with a working driver this
// is the only way a test can force a failing fence, and its precondition is that no test in this
// binary leaves a context current on the main thread. context_free() states that.
flux::gpu::Stream unfenceable()
{
  return flux::gpu::Stream(reinterpret_cast<void *>(0x1));
}

// Fails the test loudly when the precondition above is gone, instead of letting the fence quietly
// succeed and the assertions read as a broken engine.
::testing::AssertionResult context_free()
{
  if (unfenceable().wait()) {
    return ::testing::AssertionFailure()
           << "a CUDA context is current on this thread, so the legacy stream fences: some test "
              "ran Stream::create() on the main thread instead of a worker";
  }
  return ::testing::AssertionSuccess();
}

}  // namespace

// A channel that never mentions GPU must be indistinguishable from one built before GPU existed:
// no device pointer to hand a kernel, no stream, and nothing that can fail a fence.
TEST(GpuChannel, UndeclaredChannelIsUntouched)
{
  flux::Channel ch(256, 4);
  const std::vector<std::uint8_t> bytes(16, 7);
  ASSERT_EQ(ch.publish(bytes.data(), bytes.size()), flux::Published::Ok);

  flux::WriteSlot w = ch.loan();
  ASSERT_TRUE(w);
  EXPECT_EQ(w.device_ptr(), nullptr);
  EXPECT_FALSE(w.stream().declared());
  w.abort();

  flux::FrameView v = ch.take();
  ASSERT_TRUE(v);
  EXPECT_NE(v.data(), nullptr);
  EXPECT_EQ(v.device_ptr(), nullptr);
  EXPECT_FALSE(v.stream().declared());
  v.release();

  EXPECT_EQ(ch.slot_refcount(0), 0u);
  EXPECT_EQ(ch.fence_failed(), 0u);

  const flux::Channel::FenceWait wait = ch.fence_wait();
  EXPECT_EQ(wait.commit_count, 0u) << "an undeclared channel is not clocked, not clocked at zero";
  EXPECT_EQ(wait.release_count, 0u);
  EXPECT_EQ(wait.commit_ns, 0u);
  EXPECT_EQ(wait.release_ns, 0u);
}

// Declaring is either served or refused, never quietly accepted. Which of the two happens is the
// host's answer, so the test asserts the pairing rather than the outcome.
TEST(GpuChannel, DeclaringIsServedOrRefused)
{
  // Both shm routes are served: ShmDirect needs nothing, ShmRegistered registers the mapping.
  const bool serviceable = flux::gpu::probe().host_memory_is_device_memory();
  const flux::gpu::Stream stream = unfenceable();  // non-null: the host is what is on trial here
  if (serviceable) {
    EXPECT_NO_THROW({ flux::Channel ch(flux::Segment::create_heap(256, 4), stream); });
  } else {
    EXPECT_THROW(
      { flux::Channel ch(flux::Segment::create_heap(256, 4), stream); }, std::invalid_argument);
  }
}

// A device-backed payload has no host address, and data() says so by returning null. Every
// host-only reader is built from that pointer -- a generated flux_gen adapter hands it straight to
// wire::Reader/Writer, and the Writer's constructor memsets the scalar block -- so returning the
// device address would put a host store into GPU memory. A null base latches bad() in both, which
// is a refusal the caller can see instead of a fault at the first access.
//
// No GPU is needed to state this: attaching a region is what makes a segment device-backed, and
// the rule is about that, not about CUDA.
TEST(GpuChannel, ADeviceBackedPayloadHasNoHostAddress)
{
  auto region = std::make_shared<std::vector<std::uint8_t>>(64 * 1024);
  flux::Segment seg = flux::Segment::create_heap(256, 4);
  seg.attach_device(region, region->data());
  flux::Channel ch(std::move(seg));
  EXPECT_FALSE(ch.host_addressable());

  flux::WriteSlot w = ch.loan();
  ASSERT_TRUE(w);
  EXPECT_FALSE(w.host_addressable());
  EXPECT_EQ(w.data(), nullptr) << "a host store through this would fault";
  EXPECT_TRUE(flux::wire::Writer(w.data(), w.capacity(), 8).bad())
    << "a generated Builder must refuse rather than memset device memory";
  ASSERT_EQ(w.commit(16), flux::Published::Ok);

  flux::FrameView v = ch.take();
  ASSERT_TRUE(v);
  EXPECT_FALSE(v.host_addressable());
  EXPECT_EQ(v.data(), nullptr);
  EXPECT_EQ(v.size(), 16u) << "size is the frame's nbytes wherever those bytes live";
  EXPECT_TRUE(flux::wire::Reader(v.data(), v.size()).bad());
}

// The null is the device case alone. An ordinary host channel keeps handing out its address, so
// no caller pays a new hoop for a route it is not on.
TEST(GpuChannel, AHostPayloadStillHasItsAddress)
{
  flux::Channel ch(256, 4);
  EXPECT_TRUE(ch.host_addressable());

  flux::WriteSlot w = ch.loan();
  ASSERT_TRUE(w);
  EXPECT_NE(w.data(), nullptr);
  EXPECT_FALSE(flux::wire::Writer(w.data(), w.capacity(), 8).bad());
  ASSERT_EQ(w.commit(16), flux::Published::Ok);

  flux::FrameView v = ch.take();
  ASSERT_TRUE(v);
  EXPECT_NE(v.data(), nullptr);
}

// The seam itself. This process has no CUDA context, so the fence cannot be taken -- and a
// borrow whose fence did not hold must stay held, because the alternative is letting the
// publisher overwrite bytes a kernel may still be reading.
TEST(GpuChannel, AFailedFenceKeepsTheBorrowAndIsCounted)
{
  // ShmDirect only, and not for want of generality: on ShmRegistered a declared channel
  // registers its payload at construction, and registration needs a context -- so the main
  // thread cannot still be context-free by the time the fence runs. The scenario is not
  // constructible there rather than untested by omission.
  if (flux::gpu::probe().route != flux::gpu::Route::ShmDirect) {
    GTEST_SKIP() << "flux-cap:gpu-shm-direct host has no ShmDirect route (a registered host cannot "
                    "stay context-free)";
  }
  const std::string name = gpu_name("/flux_test/gpu_fence");

  ASSERT_TRUE(context_free());
  flux::Channel pub = flux::Channel::create(name, 256, 4, 0x6F0);
  flux::Channel sub = flux::Channel::open(name, 0x6F0, unfenceable());

  const std::vector<std::uint8_t> bytes(16, 3);
  ASSERT_EQ(pub.publish(bytes.data(), bytes.size()), flux::Published::Ok);

  {
    flux::FrameView v = sub.take();
    ASSERT_TRUE(v);
    EXPECT_EQ(v.device_ptr(), v.data()) << "ShmDirect: the slot mapping is the GPU buffer";
    EXPECT_TRUE(v.stream().declared());
  }

  EXPECT_EQ(sub.fence_failed(), 1u);
  EXPECT_EQ(sub.slot_refcount(0), 1u) << "the borrow must outlive a fence that did not hold";
  EXPECT_EQ(sub.fence_wait().release_count, 1u) << "a wait that failed still blocked its caller";
}

// The two seams are reported apart because they block different threads: commit blocks whoever
// publishes, release blocks whoever drops the view -- on the callback path, the executor's spin
// thread. A publisher's release fields and a consumer's commit fields therefore stay at zero.
TEST(GpuChannel, TheTwoSeamsAreClockedApart)
{
  // Declaring a stream needs a route flux serves, not merely a device that can make one. A dGPU
  // host makes streams and then refuses the declaration (require_route), so without this the test
  // fails there instead of skipping -- and CI cannot see it, having no GPU to make a stream with.
  // Either shm route: the slot is a byte range on both, so the seams behave the same. The
  // context this leaves current on the main thread is why the context-free tests below stay
  // ShmDirect-only -- do not widen those without moving this one off the main thread.
  if (!flux::gpu::probe().host_memory_is_device_memory()) {
    GTEST_SKIP() << "flux-cap:gpu-shm host has neither shm route";
  }
  // On a worker: Stream::create() makes a context current on the calling thread, and the failing
  // -fence tests need the main thread to stay without one. An explicit stream handle carries its
  // own context, so waiting on it here does not need one -- measured.
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
  const std::string name = gpu_name("/flux_test/gpu_clock");

  flux::Channel pub = flux::Channel::create(name, 256, 4, 0x6F0, pub_stream);
  flux::Channel sub = flux::Channel::open(name, 0x6F0, sub_stream);
  EXPECT_EQ(pub.fence_wait().commit_count, 0u) << "nothing is clocked before a seam runs";

  const std::vector<std::uint8_t> bytes(16, 5);
  ASSERT_EQ(pub.publish(bytes.data(), bytes.size()), flux::Published::Ok);
  {
    flux::FrameView v = sub.take();
    ASSERT_TRUE(v);
  }
  ASSERT_EQ(sub.fence_failed(), 0u) << "a created stream fences on any thread";

  const flux::Channel::FenceWait p = pub.fence_wait();
  const flux::Channel::FenceWait c = sub.fence_wait();
  EXPECT_EQ(p.commit_count, 1u);
  EXPECT_EQ(p.release_count, 0u) << "a publisher holds no views";
  EXPECT_EQ(c.release_count, 1u);
  EXPECT_EQ(c.commit_count, 0u) << "a consumer commits nothing";
  EXPECT_EQ(p.commit_max_ns, p.commit_ns) << "one sample: the worst is the sum";
  EXPECT_EQ(c.release_max_ns, c.release_ns);
}

// CUDA's legacy default stream is refused where it is declared, not once per frame on whichever
// thread happened to get the view. It names the current context's default stream and a
// context is current per thread, so the same declaration fences on the thread that touched CUDA
// and fails on the executor thread that did not -- and a channel cannot know which is which.
TEST(GpuChannel, DeclaringTheDefaultStreamIsRefused)
{
  // Either shm route: the slot is a byte range on both, so the seams behave the same. The
  // context this leaves current on the main thread is why the context-free tests below stay
  // ShmDirect-only -- do not widen those without moving this one off the main thread.
  if (!flux::gpu::probe().host_memory_is_device_memory()) {
    GTEST_SKIP() << "flux-cap:gpu-shm host has neither shm route";
  }
  EXPECT_THROW(
    { flux::Channel ch(flux::Segment::create_heap(256, 4), flux::gpu::Stream(nullptr)); },
    std::invalid_argument);
  EXPECT_NO_THROW({ flux::Channel ch(flux::Segment::create_heap(256, 4), unfenceable()); })
    << "only the null handle is refused; a handle the driver later rejects is the host's answer";
}

// A leaked lease is never returned, so max_borrow failed fences end this consumer for good. The
// counter that says so must be the fence one: fence_failed() stops rising at exactly that point
// (no borrow is handed out to fence any more), and reporting the refusals as max_borrow would
// point at a knob that cannot fix anything.
TEST(GpuChannel, LeakedLeasesStopTheConsumerUnderTheirOwnName)
{
  // ShmDirect only, and not for want of generality: on ShmRegistered a declared channel
  // registers its payload at construction, and registration needs a context -- so the main
  // thread cannot still be context-free by the time the fence runs. The scenario is not
  // constructible there rather than untested by omission.
  if (flux::gpu::probe().route != flux::gpu::Route::ShmDirect) {
    GTEST_SKIP() << "flux-cap:gpu-shm-direct host has no ShmDirect route (a registered host cannot "
                    "stay context-free)";
  }
  const std::string name = gpu_name("/flux_test/gpu_lease");

  flux::Channel pub = flux::Channel::create(name, 256, 8, 0x6F0);
  ASSERT_TRUE(context_free());
  flux::Channel sub = flux::Channel::open(name, 0x6F0, unfenceable());
  const std::uint32_t budget = sub.qos().max_borrow;

  const std::vector<std::uint8_t> bytes(16, 3);
  for (std::uint32_t i = 0; i < budget; ++i) {  // spend the whole lease budget on failed fences
    ASSERT_EQ(pub.publish(bytes.data(), bytes.size()), flux::Published::Ok);
    flux::FrameView v = sub.take();
    ASSERT_TRUE(v) << "borrow " << i << " must still be handed out";
  }
  EXPECT_EQ(sub.fence_failed(), budget);
  EXPECT_EQ(sub.refused().fence, 0u) << "nothing has been refused yet";

  ASSERT_EQ(pub.publish(bytes.data(), bytes.size()), flux::Published::Ok);
  EXPECT_FALSE(sub.take()) << "the leases are gone and no release can bring them back";
  EXPECT_EQ(sub.refused().fence, 1u);
  EXPECT_EQ(sub.refused().max_borrow, 0u) << "the caller is holding nothing; the fence is at fault";
  EXPECT_EQ(sub.fence_failed(), budget) << "it stops rising exactly where the refusals start";
  EXPECT_FALSE(sub.take_blocking(10'000'000)) << "and blocking must not park on it";
}

// The publisher side of the same rule: a commit whose fence did not hold does not publish, and
// does not abort either -- aborting would release the claim while a kernel may still be writing.
TEST(GpuChannel, AFailedFenceRefusesToPublish)
{
  // ShmDirect only, and not for want of generality: on ShmRegistered a declared channel
  // registers its payload at construction, and registration needs a context -- so the main
  // thread cannot still be context-free by the time the fence runs. The scenario is not
  // constructible there rather than untested by omission.
  if (flux::gpu::probe().route != flux::gpu::Route::ShmDirect) {
    GTEST_SKIP() << "flux-cap:gpu-shm-direct host has no ShmDirect route (a registered host cannot "
                    "stay context-free)";
  }
  ASSERT_TRUE(context_free());
  flux::Channel ch(flux::Segment::create_heap(256, 4), unfenceable());

  flux::WriteSlot w = ch.loan();
  ASSERT_TRUE(w);
  EXPECT_EQ(w.device_ptr(), w.data());
  EXPECT_NE(w.commit(16), flux::Published::Ok);
  EXPECT_GE(ch.fence_failed(), 1u);

  const std::vector<std::uint8_t> bytes(16, 1);
  EXPECT_NE(ch.publish(bytes.data(), bytes.size()), flux::Published::Ok);
  EXPECT_FALSE(ch.take()) << "nothing was ever committed";
}

// Device::Cpu must never reach CUDA: it is the default every existing channel already has.
TEST(GpuDevice, CpuDeclaresNothing)
{
  const flux::gpu::Stream s = flux::gpu::stream_for(flux::Device::Cpu);
  EXPECT_FALSE(s.declared());
  EXPECT_TRUE(s.wait());
}

// Device::Cuda either yields a stream that can be waited on or says why not. It never yields a
// stream that silently fences nothing, and it never falls back to the host path.
TEST(GpuDevice, CudaIsServedOrExplainsWhyNot)
{
  flux::gpu::Stream s;
  try {
    s = flux::gpu::stream_for(flux::Device::Cuda);
  } catch (const std::exception & e) {
    EXPECT_STRNE(e.what(), "") << "a refusal must carry its reason";
    SUCCEED() << "refused: " << e.what();
    return;
  }
  EXPECT_TRUE(s.declared());
  EXPECT_NE(s.native(), nullptr) << "a created stream is never the default stream handle";
  EXPECT_TRUE(s.wait()) << "a stream flux created is one flux can wait on";
}

// Ownership travels with the value, so a stream outlives the channel that declared it and any
// view still holding one stays waitable.
TEST(GpuDevice, ACreatedStreamOutlivesTheValueItCameFrom)
{
  flux::gpu::Stream copy;
  try {
    const flux::gpu::Stream original = flux::gpu::stream_for(flux::Device::Cuda);
    copy = original;
  } catch (const std::exception &) {
    GTEST_SKIP() << "flux-cap:cuda-stream host cannot create a stream";
  }
  EXPECT_TRUE(copy.declared());
  EXPECT_TRUE(copy.wait());
}
