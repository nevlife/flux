#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/segment.hpp"
#include "support/frame_id.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <thread>
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

// Two publishers share one segment: the first bootstraps a unique-name
// segment and advertises it on the signpost, the second reads the signpost and joins it. Both
// write the same ring; latest tracks the newest commit across both, so a subscriber sees
// whichever published most recently.
TEST(MultiPublisher, TwoPublishersShareOneSegment)
{
  const std::uint64_t fp = 0xABCDEFu;
  const std::string name = flux::segment_name(uniq("/flux_multipub_share"), fp);
  ::shm_unlink(name.c_str());

  flux::Channel pub1 = flux::Channel::create(name, 256, 4, fp);
  flux::Channel pub2 = flux::Channel::create(name, 256, 4, fp);  // attaches to pub1's segment
  ASSERT_EQ(pub1.slot_count(), 4u);
  ASSERT_EQ(pub2.slot_count(), 4u);

  std::vector<std::byte> buf(256);
  std::memset(buf.data(), 100, buf.size());
  ASSERT_EQ(publish_id(pub1, buf.data(), buf.size(), 100), flux::Published::Ok);
  {
    flux::Channel sub = flux::Channel::open(name, fp);
    flux::FrameView v = sub.peek();
    ASSERT_TRUE(v);
    EXPECT_EQ(frame_id(v), 100u);
    EXPECT_EQ(static_cast<const std::uint8_t *>(v.data())[0], 100u);
  }

  std::memset(buf.data(), 200, buf.size());
  ASSERT_EQ(publish_id(pub2, buf.data(), buf.size(), 200), flux::Published::Ok);
  {
    flux::Channel sub = flux::Channel::open(name, fp);
    flux::FrameView v = sub.peek();
    ASSERT_TRUE(v);
    EXPECT_EQ(frame_id(v), 200u);  // latest across both publishers is pub2's frame
    EXPECT_EQ(static_cast<const std::uint8_t *>(v.data())[0], 200u);
  }

  ::shm_unlink(name.c_str());
}

// Runtime analog of the multiwriter model: two publishers CAS-claim slots in the same ring
// concurrently while subscribers borrow. Each frame's payload is one repeated byte, so a
// coherent borrow reads a single uniform value. A double-claim (two writers, one slot) or a
// byte-lock/seqlock defect would surface as a torn read.
TEST(MultiPublisher, ConcurrentTwoPublishersCoherent)
{
  const std::uint64_t fp = 0xABCD01u;
  const std::string name = flux::segment_name(uniq("/flux_multipub_stress"), fp);
  ::shm_unlink(name.c_str());

  constexpr std::uint32_t kSlotSize = 4096;
  constexpr std::uint32_t kSlots = 6;
  std::uint64_t frames = 100000;
  if (const char * e = std::getenv("FLUX_STRESS_FRAMES")) frames = std::strtoull(e, nullptr, 10);

  flux::Channel pub1 = flux::Channel::create(name, kSlotSize, kSlots, fp);
  flux::Channel pub2 = flux::Channel::create(name, kSlotSize, kSlots, fp);

  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> torn{0};
  std::atomic<std::uint64_t> borrows{0};
  std::atomic<std::uint64_t> from_a{0};  // frames observed from each publisher, by payload band
  std::atomic<std::uint64_t> from_b{0};

  auto sub_fn = [&] {
    flux::Channel sub = flux::Channel::open(name, fp);
    while (!stop.load(std::memory_order_relaxed)) {
      flux::FrameView v = sub.peek();
      if (!v) continue;
      borrows.fetch_add(1, std::memory_order_relaxed);
      const auto * p = static_cast<const std::uint8_t *>(v.data());
      const std::uint8_t id = p[0];
      (id & 0x80u ? from_b : from_a).fetch_add(1, std::memory_order_relaxed);
      for (std::size_t i = 0; i < v.size(); ++i) {
        if (p[i] != id) {
          torn.fetch_add(1, std::memory_order_relaxed);
          break;
        }
      }
    }
  };

  auto pub_fn = [&](flux::Channel & pub, std::uint8_t band) {
    std::vector<std::byte> payload(kSlotSize);
    for (std::uint64_t f = 1; f <= frames && !stop.load(std::memory_order_relaxed); ++f) {
      const std::uint8_t id = static_cast<std::uint8_t>(band | (f & 0x3F));  // per-publisher band
      std::memset(payload.data(), id, payload.size());
      publish_id(pub, payload.data(), payload.size(), id);
    }
  };

  std::thread s1(sub_fn);
  std::thread s2(sub_fn);
  std::thread p1([&] { pub_fn(pub1, 0x00); });
  std::thread p2([&] { pub_fn(pub2, 0x80); });
  p1.join();
  p2.join();
  stop.store(true, std::memory_order_relaxed);
  s1.join();
  s2.join();

  EXPECT_EQ(torn.load(), 0u);

  // Coverage floor. "no torn read" is only evidence if reads actually happened, and this test is
  // specifically about MULTI-writer coherence, so frames from both publishers must have been
  // observed -- otherwise a run where one publisher finished before the other was scheduled
  // would pass while proving nothing beyond the single-writer case.
  EXPECT_GE(borrows.load(), 100u) << "too few borrows to call this a stress test";
  EXPECT_GT(from_a.load(), 0u) << "no frame from publisher 1 was ever observed";
  EXPECT_GT(from_b.load(), 0u) << "no frame from publisher 2 was ever observed";

  // Print what the run actually reached, so a passing result is readable rather than assumed.
  std::fprintf(
    stderr, "  coverage: borrows=%llu from_pub1=%llu from_pub2=%llu dropped=%llu\n",
    static_cast<unsigned long long>(borrows.load()), static_cast<unsigned long long>(from_a.load()),
    static_cast<unsigned long long>(from_b.load()),
    static_cast<unsigned long long>(pub1.dropped() + pub2.dropped()));

  ::shm_unlink(name.c_str());
}

// Publisher restart under unique-name segments + signpost: the last
// publisher out unlinks its unique segment; a restart mints a fresh unique segment and rotates
// the fixed-name signpost (its epoch bumps). A subscriber still mapped on the old segment notices
// the rotation once its takes stall and re-attaches, without the caller re-creating it.
TEST(MultiPublisher, RestartReplacesSegmentAndSubscriberReattaches)
{
  const std::uint64_t fp = 0x5217A57u;
  const std::string name = flux::segment_name(uniq("/flux_restart"), fp);
  ::shm_unlink(name.c_str());
  std::vector<std::byte> buf(128);

  std::optional<flux::Channel> pub1;
  pub1.emplace(flux::open_publisher_segment(name, 128, 4, fp));
  std::memset(buf.data(), 1, buf.size());
  ASSERT_EQ(publish_id(*pub1, buf.data(), buf.size(), 1), flux::Published::Ok);

  flux::Channel sub = flux::Channel::open(name, fp);  // attaches while pub1 is alive
  {
    flux::FrameView v = sub.peek();
    ASSERT_TRUE(v);
    EXPECT_EQ(frame_id(v), 1u);
  }

  const std::uint32_t epoch1 = flux::signpost_epoch(name);
  EXPECT_NE(epoch1, 0u) << "a live current segment must advertise a nonzero signpost epoch";
  pub1.reset();  // last publisher out: unlinks its unique segment, leaves the signpost in place

  std::optional<flux::Channel> pub2;
  pub2.emplace(flux::open_publisher_segment(name, 128, 4, fp));  // fresh unique segment
  EXPECT_GT(flux::signpost_epoch(name), epoch1) << "a restart rotates the signpost epoch";
  std::memset(buf.data(), 2, buf.size());
  ASSERT_EQ(publish_id(*pub2, buf.data(), buf.size(), 2), flux::Published::Ok);

  // The subscriber keeps polling; once its takes stall on the dead mapping it re-attaches.
  bool recovered = false;
  for (int i = 0; i < 200 && !recovered; ++i) {
    flux::FrameView v = sub.peek();
    if (v && frame_id(v) == 2u) {
      EXPECT_EQ(static_cast<const std::uint8_t *>(v.data())[0], 2u);
      recovered = true;
    }
  }
  EXPECT_TRUE(recovered) << "subscriber did not re-attach to the replacement segment";

  pub2.reset();
  ::shm_unlink(name.c_str());
}

// A live borrow is never stomped by a restart. The subscriber holds a FrameView across the
// publisher's exit: unlinking the name does not disturb an existing mapping (POSIX keeps the
// object alive while it is referenced), and the restarted publisher writes into a different
// object entirely -- so the held bytes stay intact. Once released, the subscriber re-attaches.
TEST(MultiPublisher, ReplacementDoesNotStompLiveBorrow)
{
  const std::uint64_t fp = 0x5717A99u;
  const std::string name = flux::segment_name(uniq("/flux_reinit_live_borrow"), fp);
  ::shm_unlink(name.c_str());
  std::vector<std::byte> buf(64);

  std::optional<flux::Channel> pub1;
  pub1.emplace(flux::open_publisher_segment(name, 64, 1, fp));
  ASSERT_EQ(publish_id(*pub1, buf.data(), buf.size(), 0xAA), flux::Published::Ok);

  flux::Channel sub = flux::Channel::open(name, fp);
  flux::FrameView v = sub.peek();  // HOLD the borrow across the restart
  ASSERT_TRUE(v);
  EXPECT_EQ(static_cast<const std::uint8_t *>(v.data())[0], 0xAAu);
  EXPECT_EQ(sub.slot_refcount(0), 1u);

  pub1.reset();  // last publisher out: unlinks the name; our mapping stays valid

  std::optional<flux::Channel> pub2;
  pub2.emplace(flux::open_publisher_segment(name, 64, 1, fp));  // a fresh unique segment
  ASSERT_EQ(publish_id(*pub2, buf.data(), buf.size(), 0xBB), flux::Published::Ok);

  // The held view is untouched: the new publisher wrote into a different segment.
  EXPECT_EQ(frame_id(v), 0xAAu);
  EXPECT_EQ(sub.slot_refcount(0), 1u);

  v.release();  // no view outstanding -> the subscriber may now swap mappings

  bool recovered = false;
  for (int i = 0; i < 200 && !recovered; ++i) {
    flux::FrameView v2 = sub.peek();
    if (v2 && frame_id(v2) == 0xBBu) {
      recovered = true;
    }
  }
  EXPECT_TRUE(recovered) << "subscriber did not re-attach after releasing its borrow";

  pub2.reset();
  ::shm_unlink(name.c_str());
}

// A config change restarts the publisher, and the signpost can still point at the crashed
// group's segment. sweep_dead() is best-effort -- it skips entirely while another sweeper holds
// /flux.sweep -- so holding that lock here pins the corpse in place deterministically. The
// restarted publisher must build a new segment rather than fail on the dead one's config.
//
TEST(MultiPublisher, ConfigChangeOverACorpseCreatesInsteadOfFailing)
{
  const std::uint64_t fp = 0xC0F1Cu;
  const std::string name = flux::signpost_name(uniq("/flux_cfgcorpse"), fp);
  ::shm_unlink(name.c_str());

  int up[2];  // child->parent: one byte once the segment is advertised
  ASSERT_EQ(::pipe(up), 0);
  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::close(up[0]);
    flux::Channel pub = flux::Channel::create(name, /*slot_size=*/128, /*slot_count=*/4, fp);
    char ready = 1;
    ssize_t ignored = ::write(up[1], &ready, 1);
    (void)ignored;
    for (;;) ::pause();  // killed below; no destructor runs, so the segment is left behind
  }
  ::close(up[1]);
  char ready = 0;
  ASSERT_EQ(::read(up[0], &ready, 1), 1);
  ::close(up[0]);

  const std::uint32_t epoch1 = flux::signpost_epoch(name);
  ASSERT_NE(epoch1, 0u);

  ASSERT_EQ(::kill(pid, SIGKILL), 0);
  int status = 0;
  ASSERT_EQ(::waitpid(pid, &status, 0), pid);

  // Pin the corpse: with the cleanup lock held, our own sweep_dead() finds it taken and skips.
  int lock_fd = ::shm_open("/flux.sweep", O_CREAT | O_RDWR, 0600);
  ASSERT_GE(lock_fd, 0);
  ASSERT_TRUE(flux::Segment::lock_write(lock_fd));

  std::optional<flux::Channel> pub2;
  ASSERT_NO_THROW(
    pub2.emplace(flux::open_publisher_segment(name, /*slot_size=*/256, /*slot_count=*/8, fp)));
  EXPECT_EQ(pub2->slot_count(), 8u) << "joined the corpse instead of building the new config";
  EXPECT_GT(flux::signpost_epoch(name), epoch1) << "the signpost still advertises the corpse";

  ::close(lock_fd);
  pub2.reset();
  ::shm_unlink(name.c_str());
}

// The other half of the same branch: a config mismatch against a publisher that is actually
// running is not a race, and swallowing it would let two configs believe they share a segment.
TEST(MultiPublisher, ConfigChangeAgainstALivePublisherStillThrows)
{
  const std::uint64_t fp = 0xC0F11Eu;
  const std::string name = flux::signpost_name(uniq("/flux_cfglive"), fp);
  ::shm_unlink(name.c_str());

  flux::Channel pub1 = flux::Channel::create(name, /*slot_size=*/128, /*slot_count=*/4, fp);
  EXPECT_THROW(
    flux::open_publisher_segment(name, /*slot_size=*/256, /*slot_count=*/8, fp),
    flux::SegmentMismatch);

  ::shm_unlink(name.c_str());
}
