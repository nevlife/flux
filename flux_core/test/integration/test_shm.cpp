#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/owner.hpp"
#include "support/frame_id.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace
{
using flux::test::frame_id;
using flux::test::publish_id;
// Unique per run: two concurrent test runs (or a leftover from a killed one) must not share
// /dev/shm names.
std::string uniq(const std::string & base)
{
  return base + "." + std::to_string(::getpid());
}

// A frame's identity is its payload: the descriptor is derived from the byte count, so an id in
// shape[0] would have to contradict nbytes to be an id at all. The whole buffer carries the tag,
// which is also what makes an overwrite of a held frame visible.

}  // namespace

TEST(Shm, SameProcessCreateOpenRoundTrip)
{
  const std::string name = flux::segment_name(uniq("/flux_test/roundtrip"), 0x1111);
  flux::Channel pub = flux::Channel::create(name, 256, 4, 0x1111);
  flux::Channel sub = flux::Channel::open(name, 0x1111);

  std::vector<std::byte> buf(256);
  ASSERT_EQ(publish_id(pub, buf.data(), buf.size(), 0x33), flux::Published::Ok);

  flux::FrameView v = sub.peek();
  ASSERT_TRUE(v);
  EXPECT_EQ(frame_id(v), 0x33u);
}

TEST(Shm, FingerprintMismatchRejected)
{
  const std::string name = flux::segment_name(uniq("/flux_test/fp"), 0xAAAA);
  flux::Channel pub = flux::Channel::create(name, 64, 2, 0xAAAA);
  EXPECT_THROW(flux::Channel::open(name, 0xBBBB), std::runtime_error);
}

TEST(Shm, OpenMissingRejected)
{
  const std::string name = flux::segment_name(uniq("/flux_test/absent"), 0x1234);
  EXPECT_THROW(flux::Channel::open(name, 0x1234), std::runtime_error);
}

// The two ways an attach fails are not one failure. A segment that is not up yet is a retryable
// miss the caller sits through; a live publisher on another ring config can never become right.
// Only the second is a SegmentMismatch, and a caller that cannot tell them apart either spins
// forever on a misconfiguration or gives up on a publisher that was merely slow (docs/en/api.en.md
// 1).
TEST(Shm, ConfigMismatchIsDistinctFromAnAbsentSegment)
{
  const std::string absent = flux::segment_name(uniq("/flux_test/absent2"), 0x019A);
  try {
    flux::Channel::open(absent, 0x019A);
    FAIL() << "an absent segment must still fail";
  } catch (const flux::SegmentMismatch &) {
    FAIL() << "an absent segment is retryable, not a mismatch";
  } catch (const std::runtime_error &) {
  }

  const std::string name = flux::segment_name(uniq("/flux_test/cfg"), 0x019B);
  flux::Channel pub = flux::Channel::create(name, 64, 2, 0x019B);
  EXPECT_THROW(flux::Channel::create(name, 128, 4, 0x019B), flux::SegmentMismatch);
}

// The payload region of a subscriber mapping is dropped to PROT_READ at attach (v7 page
// alignment makes that possible), so a store through an aliased view faults at the guilty
// line instead of surfacing as some other consumer's torn read. The publisher's own mapping
// stays writable, and the borrow protocol (header writes) is untouched.
TEST(Shm, SubscriberPayloadIsHardwareReadOnly)
{
  if (::sysconf(_SC_PAGESIZE) > static_cast<long>(flux::kPayloadAlign)) {
    GTEST_SKIP()
      << "flux-cap:small-page page size larger than kPayloadAlign: protection is skipped by design";
  }
  const std::string name = flux::segment_name(uniq("/flux_test/prot"), 0x5EC);
  flux::Channel pub = flux::Channel::create(name, 256, 4, 0x5EC);
  flux::Channel sub = flux::Channel::open(name, 0x5EC);
  EXPECT_FALSE(pub.payload_readonly());
  EXPECT_TRUE(sub.payload_readonly());

  std::vector<std::byte> buf(256);
  ASSERT_EQ(publish_id(pub, buf.data(), buf.size(), 0x44), flux::Published::Ok);
  flux::FrameView v = sub.peek();
  ASSERT_TRUE(v);
  EXPECT_EQ(static_cast<const std::uint8_t *>(v.data())[0], 0x44u);

  // Publishing again proves the subscriber-side protection never touched the writer's mapping.
  ASSERT_EQ(publish_id(pub, buf.data(), buf.size(), 0x44), flux::Published::Ok);

  auto * alias = const_cast<std::uint8_t *>(static_cast<const std::uint8_t *>(v.data()));
  EXPECT_DEATH({ *alias = 0xFF; }, "");
}

// A real second process attaches to the same shm and reads coherent frames. Proves
// cross-process zero-copy sharing + the borrow protocol across an address-space
// boundary. The child _exit()s with a status code the parent asserts on.
TEST(Shm, CrossProcessCoherence)
{
  const std::string name = flux::segment_name(uniq("/flux_test/xproc"), 0xC0FFEE);
  constexpr std::uint32_t slot_size = 4096;
  constexpr std::uint32_t slots = 4;
  flux::Channel pub = flux::Channel::create(name, slot_size, slots, 0xC0FFEE);

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    int code = 3;  // never attached
    try {
      flux::Channel sub = flux::Channel::open(name, 0xC0FFEE);
      int got = 0;
      for (long spins = 0; spins < 2000000000L && got < 50; ++spins) {
        flux::FrameView v = sub.peek();
        if (!v) continue;
        const auto * p = static_cast<const std::uint8_t *>(v.data());
        const std::uint8_t id = p[0];
        bool coherent = true;
        for (std::size_t i = 0; i < v.size(); ++i) {
          if (p[i] != id) {
            coherent = false;
            break;
          }
        }
        if (!coherent) {
          code = 1;  // torn read
          break;
        }
        ++got;
      }
      if (code != 1) code = (got >= 50) ? 0 : 2;  // 2 = never got enough frames
    } catch (...) {
      code = 4;  // open/exception
    }
    // Clean shutdown: unlink this subscriber's owner file (it hard-exits, bypassing atexit).
    ::shm_unlink(flux::owner_file_name(flux::OwnerFile::self()).c_str());
    _exit(code);
  }

  std::vector<std::byte> buf(slot_size);
  for (std::uint64_t f = 1; f <= 1000000; ++f) {
    std::memset(buf.data(), static_cast<int>(f & 0xFF), buf.size());
    publish_id(pub, buf.data(), buf.size(), f);
  }

  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);  // 0 = 50 coherent cross-process borrows
}

// A forked subscriber blocks in take_blocking() and is woken by the parent's publish
// through the in-segment futex. Proves the plain (non-private) futex crosses the mmap
// boundary -- the reason the wake word lives inside the shared segment.
TEST(Shm, CrossProcessWake)
{
  const std::string name = flux::segment_name(uniq("/flux_test/xwake"), 0xBEEF);
  constexpr std::uint32_t slot_size = 512;
  flux::Channel pub = flux::Channel::create(name, slot_size, 4, 0xBEEF);

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    int code = 3;
    try {
      flux::Channel sub = flux::Channel::open(name, 0xBEEF);
      flux::FrameView v = sub.take_blocking(/*timeout_ns=*/5'000'000'000);  // 5 s cap
      if (!v) {
        code = 2;  // woken never / timed out
      } else {
        code = (static_cast<const std::uint8_t *>(v.data())[0] == 0x77) ? 0 : 1;
      }
    } catch (...) {
      code = 4;
    }
    ::shm_unlink(flux::owner_file_name(flux::OwnerFile::self()).c_str());  // clean owner file
    _exit(code);
  }

  usleep(100000);  // 100 ms: let the child attach and park in take_blocking
  std::vector<std::byte> buf(slot_size);
  ASSERT_EQ(publish_id(pub, buf.data(), buf.size(), 0x77), flux::Published::Ok);

  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);  // 0 = woken across process boundary, correct payload
}

// slot_size/slot_count come out of a header another process wrote and every offset is derived
// from them. A count past the 16-bit slot field in `latest` is representable but not addressable:
// a publisher selecting slot 70000 packs 70000 & 0xFFFF into `latest`, so consumers read a
// different slot than the frame landed in. Push it further and total_bytes() wraps, and the
// "layout exceeds mapping" check below it compares against the wrapped number. Attach must reject
// the pair before deriving anything from it.
//
// The segment and its signpost are hand-built: a real publisher refuses this config at create, so
// only a corrupt or rogue one produces it. Nothing past the header is touched, so the oversized
// ftruncate costs a page on tmpfs, not the layout's 280 MiB.
TEST(Shm, AttachRejectsAnUnaddressableSlotCount)
{
  const std::string signpost = flux::segment_name(uniq("/flux_test/badcfg"), 0xBAD);
  flux::OwnerId creator;
  creator.pid = 424245;
  creator.starttime = 7;
  const std::string seg = flux::unique_segment_name(signpost, creator);
  ::shm_unlink(signpost.c_str());
  ::shm_unlink(seg.c_str());

  // Signpost advertising our segment, bootstrapped by hand.
  int sp_fd = ::shm_open(signpost.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(sp_fd, 0);
  ASSERT_EQ(::ftruncate(sp_fd, 4096), 0);
  void * sp_p = ::mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, sp_fd, 0);
  ASSERT_NE(sp_p, MAP_FAILED);
  auto * sp = reinterpret_cast<flux::Signpost *>(sp_p);
  sp->magic = flux::kSignpostMagic;
  sp->version = flux::kLayoutVersion;
  sp->cur_pid = creator.pid;
  sp->cur_starttime = creator.starttime;
  sp->epoch = 1;
  sp->seq.store(0, std::memory_order_relaxed);  // even: sealed
  sp->init_state.store(flux::kInitReady, std::memory_order_release);
  ::munmap(sp_p, 4096);
  ::close(sp_fd);

  // A segment whose header claims one slot past what `latest` can name, sized so the mapping
  // genuinely covers it -- the size check has nothing to complain about.
  constexpr std::uint32_t kBadCount = flux::kMaxSlotCount + 1;
  const flux::SegmentLayout bad{1, kBadCount};
  int fd = ::shm_open(seg.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  ASSERT_EQ(::ftruncate(fd, static_cast<off_t>(bad.total_bytes())), 0);
  void * p =
    ::mmap(nullptr, sizeof(flux::ControlHeader), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  ASSERT_NE(p, MAP_FAILED);
  auto * ctrl = reinterpret_cast<flux::ControlHeader *>(p);
  ctrl->magic = flux::kMagic;
  ctrl->version = flux::kLayoutVersion;
  ctrl->slot_size = 1;
  ctrl->slot_count = kBadCount;
  ctrl->fingerprint = 0xBAD;
  ctrl->epoch = 1;
  ctrl->init_state.store(flux::kInitReady, std::memory_order_release);
  ::munmap(p, sizeof(flux::ControlHeader));
  ASSERT_TRUE(flux::Segment::lock_read(fd));  // a live publisher, so attach gets past liveness

  EXPECT_THROW(flux::open_subscriber_segment(signpost, 0xBAD), std::runtime_error)
    << "attach adopted a slot_count `latest` cannot name";

  ::close(fd);
  ::shm_unlink(seg.c_str());
  ::shm_unlink(signpost.c_str());
}

// The publisher side of the same rule: a config it could not address is refused at create.
// invalid_argument, not runtime_error: these numbers are the caller's own, and the heap-backed
// path rejects them the same way. A config another process wrote takes the attach-side path.
TEST(Shm, CreateRejectsAnOutOfRangeLayoutConfig)
{
  const std::string name = flux::segment_name(uniq("/flux_test/badcfg2"), 0xBAD2);
  EXPECT_THROW(flux::Channel::create(name, 256, 0, 0xBAD2), std::invalid_argument);
  EXPECT_THROW(flux::Channel::create(name, 0, 4, 0xBAD2), std::invalid_argument);
  EXPECT_THROW(
    flux::Channel::create(name, 256, flux::kMaxSlotCount + 1, 0xBAD2), std::invalid_argument);
  ::shm_unlink(name.c_str());
}
