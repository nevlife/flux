#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/owner.hpp"
#include "support/frame_id.hpp"

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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

// A subscriber that dies holding a borrow leaks its slot; the publisher, once starved, probes
// the dead owner over its OFD lock and reclaims the slot. slot_count=1
// means a single held slot starves the publisher, exercising the reclaim path directly. The
// child dies via _exit (no destructors), so the borrow really is abandoned, not released.
TEST(CrashReclaim, DeadSubscriberSlotIsReclaimed)
{
  const std::uint64_t fp = 0xC0FFEEu;
  const std::string name = flux::segment_name(uniq("/flux_crash_reclaim_test"), fp);
  ::shm_unlink(name.c_str());

  flux::Channel pub = flux::Channel::create(name, /*slot_size=*/64, /*slot_count=*/1, fp);
  std::vector<std::byte> buf(64, std::byte{1});
  ASSERT_EQ(publish_id(pub, buf.data(), buf.size(), 1), flux::Published::Ok);

  int up[2], down[2];  // child->parent: OwnerId; parent->child: "die now"
  ASSERT_EQ(::pipe(up), 0);
  ASSERT_EQ(::pipe(down), 0);

  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    // Child: borrow the only slot, report its identity, wait, then die still holding it.
    flux::Channel sub = flux::Channel::open(name, fp);
    flux::FrameView v = sub.peek();
    flux::OwnerId id = v ? flux::OwnerFile::self() : flux::OwnerId{};
    [[maybe_unused]] ssize_t w = ::write(up[1], &id, sizeof(id));
    char go = 0;
    [[maybe_unused]] ssize_t r = ::read(down[0], &go, 1);
    _exit(0);  // no destructor / no atexit: refcount + holder entry stay held
  }

  ::close(up[1]);
  ::close(down[0]);
  flux::OwnerId child_id{};
  ASSERT_EQ(::read(up[0], &child_id, sizeof(child_id)), static_cast<ssize_t>(sizeof(child_id)));
  ASSERT_NE(child_id.pid, 0u);  // the child's take() succeeded

  // Child is alive and holds the only slot: the publisher probes it, finds it alive, and
  // cannot reclaim -- the frame is dropped.
  EXPECT_NE(publish_id(pub, buf.data(), buf.size(), 2), flux::Published::Ok);
  EXPECT_GE(pub.dropped(), 1u);

  // Kill the child; the kernel releases its OFD lock.
  ASSERT_EQ(::write(down[1], "x", 1), 1);
  int status = 0;
  ASSERT_EQ(::waitpid(pid, &status, 0), pid);

  // The holder is now dead: the publisher reclaims the slot and the publish succeeds.
  EXPECT_EQ(publish_id(pub, buf.data(), buf.size(), 3), flux::Published::Ok);

  ::close(up[0]);
  ::close(down[1]);
  ::shm_unlink(flux::owner_file_name(child_id).c_str());  // dead child's owner file
}

// A publisher that crashes mid-write (claimed a slot via loan, seq odd, but never committed)
// leaves the slot stuck claimed. A peer publisher, once starved, probes the dead writer's OFD
// lock and reverts the claim. slot_count=1 so the single stuck slot starves
// the survivor. The child dies via _exit holding the loan (no destructor -> no abort).
TEST(CrashReclaim, DeadWriterStuckSlotIsRecovered)
{
  const std::uint64_t fp = 0xC0FFE3u;
  const std::string name = flux::segment_name(uniq("/flux_stuck_write_test"), fp);
  ::shm_unlink(name.c_str());

  flux::Channel pubA = flux::Channel::create(name, /*slot_size=*/64, /*slot_count=*/1, fp);

  int up[2];
  ASSERT_EQ(::pipe(up), 0);
  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    // Child co-publisher: claim the only slot via loan (stamps its writer identity, seq odd),
    // report its identity, then die still holding the claim.
    flux::Channel pubB = flux::Channel::create(name, 64, 1, fp);
    flux::WriteSlot ws = pubB.loan();
    flux::OwnerId id = ws ? flux::OwnerFile::self() : flux::OwnerId{};
    [[maybe_unused]] ssize_t w = ::write(up[1], &id, sizeof(id));
    _exit(0);  // crash mid-write: no commit / no abort -> seq stays odd
  }
  ::close(up[1]);
  flux::OwnerId child_id{};
  ASSERT_EQ(::read(up[0], &child_id, sizeof(child_id)), static_cast<ssize_t>(sizeof(child_id)));
  ASSERT_NE(child_id.pid, 0u);  // the child claimed the slot
  ::close(up[0]);
  int status = 0;
  ASSERT_EQ(::waitpid(pid, &status, 0), pid);

  // The only slot is stuck claimed (seq odd) by the now-dead writer; refcount is 0 (a loan does
  // not borrow). Without recovery every publish would drop forever. The survivor probes the dead
  // writer, reverts the claim, and publishes.
  EXPECT_EQ(pubA.slot_refcount(0), 0u);
  std::vector<std::byte> buf(64, std::byte{0xAB});
  EXPECT_EQ(publish_id(pubA, buf.data(), buf.size(), 1), flux::Published::Ok);

  ::shm_unlink(flux::owner_file_name(child_id).c_str());  // dead child's owner file
  ::shm_unlink(name.c_str());
}

// Concurrent reclaimers must not double-subtract a dead holder's count.
// With multiple publishers, several can be starved at once and all run reclaim over the same dead
// holder table. If two subtract the same entry's count from the slot refcount, the u32 underflows
// past 0 and wraps to a huge value -> the slot reads permanently borrowed -> every publish drops.
// Each round leaks several dead holders on a single slot, then two publisher threads reclaim at a
// barrier; the slot refcount must never exceed the holder count, and the slot must stay reusable.
TEST(CrashReclaim, ConcurrentReclaimersDoNotUnderflow)
{
  const std::uint64_t fp = 0xC0FFE2u;
  const std::string name = flux::segment_name(uniq("/flux_reclaim_race_test"), fp);
  ::shm_unlink(name.c_str());

  constexpr std::uint32_t kSlotSize = 64;
  const std::uint32_t kHolders = 6;  // dead borrowers per round (< kMaxHolders)
  unsigned rounds = 40;
  if (const char * e = std::getenv("FLUX_RECLAIM_STRESS_ROUNDS")) rounds = std::atoi(e);

  flux::Channel pubA = flux::Channel::create(name, kSlotSize, 1, fp);  // reclaimer 1
  flux::Channel pubB = flux::Channel::create(name, kSlotSize, 1, fp);  // reclaimer 2 (co-publisher)
  std::vector<std::byte> buf(kSlotSize, std::byte{1});
  const auto pub_once = [&](flux::Channel & p) { publish_id(p, buf.data(), buf.size(), 1); };

  for (unsigned r = 0; r < rounds; ++r) {
    // Fresh, free slot with a committed frame at the start of the round.
    for (int i = 0; i < 500 && pubA.slot_refcount(0) != 0; ++i)
      pub_once(pubA);  // drains via reclaim
    ASSERT_EQ(pubA.slot_refcount(0), 0u) << "round " << r << ": slot not drained";
    ASSERT_EQ(publish_id(pubA, buf.data(), buf.size(), 1), flux::Published::Ok);

    // Fork kHolders children; each borrows the slot and dies still holding it.
    int up[2], down[2];
    ASSERT_EQ(::pipe(up), 0);
    ASSERT_EQ(::pipe(down), 0);
    std::vector<pid_t> pids;
    for (std::uint32_t k = 0; k < kHolders; ++k) {
      pid_t pid = ::fork();
      ASSERT_GE(pid, 0);
      if (pid == 0) {
        flux::Channel sub = flux::Channel::open(name, fp);
        flux::FrameView v = sub.peek();
        flux::OwnerId id = v ? flux::OwnerFile::self() : flux::OwnerId{};
        [[maybe_unused]] ssize_t w = ::write(up[1], &id, sizeof(id));
        char go = 0;
        [[maybe_unused]] ssize_t rr = ::read(down[0], &go, 1);
        _exit(0);  // die holding the borrow: refcount + holder entry stay
      }
      pids.push_back(pid);
    }
    ::close(up[1]);
    std::vector<flux::OwnerId> ids;
    for (std::uint32_t k = 0; k < kHolders; ++k) {
      flux::OwnerId id{};
      ASSERT_EQ(::read(up[0], &id, sizeof(id)), static_cast<ssize_t>(sizeof(id)));
      ASSERT_NE(id.pid, 0u);  // borrow established
      ids.push_back(id);
    }
    for (std::uint32_t k = 0; k < kHolders; ++k) {
      [[maybe_unused]] ssize_t w = ::write(down[1], "x", 1);
    }
    for (pid_t pid : pids) {
      int st = 0;
      ::waitpid(pid, &st, 0);
    }
    ::close(up[0]);
    ::close(down[0]);
    ::close(down[1]);

    // Two publishers reclaim the dead holders concurrently, released together at a spin barrier.
    std::atomic<bool> start{false};
    const auto racer = [&](flux::Channel & p) {
      // Its own buffer: publish_id stamps the tag into the source before publishing, so the two
      // reclaimers sharing one would be a write-write race on it rather than on the segment.
      std::vector<std::byte> src(kSlotSize, std::byte{1});
      while (!start.load(std::memory_order_acquire)) {
      }
      // starved -> reclaim_dead on the same entries
      for (int i = 0; i < 8; ++i) publish_id(p, src.data(), src.size(), 1);
    };
    std::thread t1([&] { racer(pubA); });
    std::thread t2([&] { racer(pubB); });
    start.store(true, std::memory_order_release);
    t1.join();
    t2.join();

    // The dead holders were reclaimed cleanly: a double-subtract would underflow the u32 far past
    // kHolders. (rc is 0 when fully reclaimed; the bound is the underflow detector.)
    const std::uint32_t rc = pubA.slot_refcount(0);
    ASSERT_LE(rc, kHolders) << "round " << r << ": refcount " << rc
                            << " exceeds holders -> reclaim double-subtract underflow";

    for (const auto & id : ids) ::shm_unlink(flux::owner_file_name(id).c_str());
  }

  // The slot must remain reusable, not permanently pinned by an underflowed refcount.
  for (int i = 0; i < 500 && pubA.slot_refcount(0) != 0; ++i) pub_once(pubA);
  EXPECT_EQ(pubA.slot_refcount(0), 0u);
  EXPECT_EQ(publish_id(pubA, buf.data(), buf.size(), 1), flux::Published::Ok);

  ::shm_unlink(name.c_str());
}

// A borrower that releases normally must free its holder entry, not just decrement its count.
// While it only decremented, an entry stayed pinned to a pid that had already exited, so after
// kMaxHolders distinct subscriber processes had touched a slot the table was full for good and
// every later subscriber got an empty view forever -- silently, with no publisher involved.
TEST(CrashReclaim, HolderEntryIsFreedOnNormalRelease)
{
  const std::uint64_t fp = 0xF12EEu;
  const std::string name = flux::segment_name(uniq("/flux_holder_release_test"), fp);
  ::shm_unlink(name.c_str());

  flux::Channel pub = flux::Channel::create(name, /*slot_size=*/64, /*slot_count=*/1, fp);
  std::vector<std::byte> buf(64, std::byte{7});
  ASSERT_EQ(publish_id(pub, buf.data(), buf.size(), 1), flux::Published::Ok);

  // Three times the table size, so a per-process leak cannot hide.
  const int kRounds = static_cast<int>(flux::kMaxHolders) * 3;
  int served = 0;
  for (int i = 0; i < kRounds; ++i) {
    int fds[2];
    ASSERT_EQ(::pipe(fds), 0);
    pid_t pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      ::close(fds[0]);
      char ok = 0;
      {
        flux::Channel sub = flux::Channel::open(name, fp);
        flux::FrameView v = sub.peek();
        ok = v ? 1 : 0;
      }  // released here, before exit
      [[maybe_unused]] ssize_t w = ::write(fds[1], &ok, 1);
      _exit(0);
    }
    ::close(fds[1]);
    char ok = 0;
    ASSERT_EQ(::read(fds[0], &ok, 1), 1);
    ::close(fds[0]);
    int st = 0;
    ::waitpid(pid, &st, 0);
    served += ok;
  }
  EXPECT_EQ(served, kRounds) << "holder table exhausted after " << served
                             << " subscriber processes (kMaxHolders=" << flux::kMaxHolders << ")";
  ::shm_unlink(name.c_str());
}

// The holder table is what identity-based reclaim reads, so a borrow that cannot be recorded in
// it must not be taken: an untracked refcount is one nothing can ever give back. kMaxHolders live
// processes on one slot is the boundary, and the next borrower is refused and counted rather than
// handed a view. Nothing else in the suite reaches this path.
TEST(CrashReclaim, ABorrowIsRefusedWhenTheHolderTableIsFull)
{
  const std::uint64_t fp = 0xC0FFE3u;
  const std::string name = flux::segment_name(uniq("/flux_holder_full_test"), fp);
  ::shm_unlink(name.c_str());

  constexpr std::uint32_t kSlotSize = 64;
  flux::Channel pub = flux::Channel::create(name, kSlotSize, 1, fp);
  std::vector<std::byte> buf(kSlotSize, std::byte{1});
  ASSERT_EQ(publish_id(pub, buf.data(), buf.size(), 1), flux::Published::Ok);

  // kMaxHolders children, each a distinct pid, each holding the one slot and staying alive.
  int up[2], down[2];
  ASSERT_EQ(::pipe(up), 0);
  ASSERT_EQ(::pipe(down), 0);
  std::vector<pid_t> pids;
  for (std::uint32_t k = 0; k < flux::kMaxHolders; ++k) {
    pid_t pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      flux::Channel sub = flux::Channel::open(name, fp);
      flux::FrameView v = sub.peek();
      const char ok = v ? 'y' : 'n';
      [[maybe_unused]] ssize_t w = ::write(up[1], &ok, 1);
      char go = 0;
      [[maybe_unused]] ssize_t rr = ::read(down[0], &go, 1);
      _exit(0);  // holds the borrow until released here
    }
    pids.push_back(pid);
  }
  ::close(up[1]);
  for (std::uint32_t k = 0; k < flux::kMaxHolders; ++k) {
    char ok = 0;
    ASSERT_EQ(::read(up[0], &ok, 1), 1);
    ASSERT_EQ(ok, 'y') << "child " << k << " could not borrow";
  }

  flux::Channel sub = flux::Channel::open(name, fp);
  EXPECT_EQ(sub.refused().holder_table, 0u);
  EXPECT_FALSE(sub.peek()) << "an eleventh holder was admitted to a table of kMaxHolders";
  EXPECT_EQ(sub.refused().holder_table, 1u);
  EXPECT_EQ(sub.refused().max_borrow, 0u) << "this consumer holds nothing; the table is full";
  EXPECT_EQ(pub.slot_refcount(0), flux::kMaxHolders)
    << "a refused borrow left its refcount increment behind";

  for (std::uint32_t k = 0; k < flux::kMaxHolders; ++k) {
    [[maybe_unused]] ssize_t w = ::write(down[1], "x", 1);
  }
  for (pid_t pid : pids) {
    int st = 0;
    ::waitpid(pid, &st, 0);
  }
  ::close(up[0]);
  ::close(down[0]);
  ::close(down[1]);
  ::shm_unlink(name.c_str());
}
