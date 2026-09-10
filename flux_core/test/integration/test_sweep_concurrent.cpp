#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // F_OFD_SETLK via Segment::lock_read
#endif

#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/owner.hpp"
#include "flux/segment.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// sweep_dead() runs in every participant, so several of them sweep the shared /dev/shm namespace
// at once. The other runtime tests run one sweeper in one process. These drive real concurrent
// sweepers against real
// live participants: nothing live may be unlinked, and the orphans must go.
// The reaped orphans double as the coverage gate -- if no sweeper did any work the test would
// otherwise pass having proven nothing.
//
// The R5 creation window is too narrow to hit by chance here (measured: 3 in 20000 attempts), so
// the joining processes below are a smoke check, not its coverage. That window is forced
// deterministically through the ensure() seam in test/unit/test_owner.cpp.

namespace
{

constexpr std::uint64_t kFp = 0x5177EEB;
constexpr std::uint32_t kSlotSize = 256;
constexpr std::uint32_t kSlotCount = 4;

std::string uniq(const char * tag)
{
  return std::string("/flux_test/sweepcc/") + tag + "." + std::to_string(::getpid());
}

// A crashed publisher's leftovers: named, ready, and unlocked.
void seed_orphan_segment(const std::string & name)
{
  ::shm_unlink(name.c_str());
  int fd = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  flux::SegmentLayout layout{kSlotSize, kSlotCount};
  const std::size_t bytes = layout.total_bytes();
  ASSERT_EQ(::ftruncate(fd, static_cast<off_t>(bytes)), 0);
  void * p = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  ASSERT_NE(p, MAP_FAILED);
  flux::Segment::init_header(static_cast<std::byte *>(p), layout, kSlotSize, kSlotCount, kFp);
  ::munmap(p, bytes);
  ::close(fd);  // no lock held -> a corpse
}

void seed_orphan_owner(const std::string & name)
{
  ::shm_unlink(name.c_str());
  int fd = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  ::close(fd);  // never locked -> a corpse, like a crashed participant's file
}

bool name_exists(const std::string & name)
{
  flux::SegmentId id;
  return flux::stat_segment(name, id);
}

// Runs sweep_dead() a bounded number of times and exits. Bounded rather than timed so the test
// cannot outlive a wedged child.
[[noreturn]] void sweeper_child(int rounds)
{
  for (int i = 0; i < rounds; ++i) flux::sweep_dead();
  _exit(0);
}

// Joins while the sweepers run: takes its owner file, then checks it is still reachable by name.
// A sweeper that unlinked it inside ensure()'s O_CREAT..lock window would leave this process alive
// but unprobeable, and every later reclaim would steal its borrows.
[[noreturn]] void participant_child()
{
  try {
    flux::OwnerFile::ensure();
  } catch (...) {
    _exit(3);
  }
  const flux::OwnerId me = flux::OwnerFile::self();
  int fd = ::shm_open(flux::owner_file_name(me).c_str(), O_RDONLY, 0600);
  if (fd < 0) _exit(4);  // alive but nameless
  ::close(fd);
  if (flux::OwnerFile::probe(me) != flux::Liveness::Alive) _exit(5);
  _exit(0);
}

}  // namespace

TEST(SweepConcurrent, SweepersSpareLiveObjectsAndReapOrphans)
{
  const std::string topic = uniq("live");
  const std::string signpost = flux::signpost_name(topic, kFp);
  const std::string orphan_seg = flux::signpost_name(uniq("orphan"), kFp) + ".424242.99";
  const std::string orphan_owner = flux::owner_file_name(flux::OwnerId{424243, 0, 98});

  seed_orphan_segment(orphan_seg);
  seed_orphan_owner(orphan_owner);
  ASSERT_TRUE(name_exists(orphan_seg));
  ASSERT_TRUE(name_exists(orphan_owner));

  // A live publisher (segment read lock) and this process's own owner file (write lock).
  flux::Channel pub = flux::Channel::create(signpost, kSlotSize, kSlotCount, kFp);
  flux::OwnerFile::ensure();
  const flux::OwnerId me = flux::OwnerFile::self();
  const std::string live_seg = pub.wait_handle()->seg.name();
  ASSERT_TRUE(name_exists(live_seg));
  ASSERT_TRUE(name_exists(signpost));
  ASSERT_TRUE(name_exists(flux::owner_file_name(me)));

  constexpr int kSweepers = 4;
  constexpr int kParticipants = 8;
  std::vector<pid_t> kids;
  for (int i = 0; i < kSweepers; ++i) {
    pid_t p = ::fork();
    ASSERT_GE(p, 0);
    if (p == 0) sweeper_child(40);
    kids.push_back(p);
  }
  for (int i = 0; i < kParticipants; ++i) {
    pid_t p = ::fork();
    ASSERT_GE(p, 0);
    if (p == 0) participant_child();
    kids.push_back(p);
  }

  int joined_ok = 0;
  for (std::size_t i = 0; i < kids.size(); ++i) {
    int st = 0;
    ASSERT_EQ(::waitpid(kids[i], &st, 0), kids[i]);
    ASSERT_TRUE(WIFEXITED(st)) << "child " << i << " did not exit normally";
    if (i >= kSweepers) {
      EXPECT_EQ(WEXITSTATUS(st), 0)
        << "a process joining under concurrent sweeps was left unprobeable "
           "(3=ensure threw, 4=owner file nameless, 5=probed Dead)";
      if (WEXITSTATUS(st) == 0) ++joined_ok;
    } else {
      ASSERT_EQ(WEXITSTATUS(st), 0) << "sweeper " << i << " failed";
    }
  }
  EXPECT_EQ(joined_ok, kParticipants);

  // Everything with a live holder survived.
  EXPECT_TRUE(name_exists(live_seg)) << "sweep unlinked a segment with a live publisher";
  EXPECT_TRUE(name_exists(signpost)) << "sweep unlinked a signpost";
  EXPECT_TRUE(name_exists(flux::owner_file_name(me))) << "sweep unlinked a live owner file";
  EXPECT_EQ(flux::OwnerFile::probe(me), flux::Liveness::Alive);
  const char byte = 'x';
  EXPECT_EQ(pub.publish(&byte, 1), flux::Published::Ok)
    << "the live channel stopped working after the sweeps";

  // The corpses are gone.
  EXPECT_FALSE(name_exists(orphan_seg)) << "concurrent sweepers left an orphan segment";
  EXPECT_FALSE(name_exists(orphan_owner)) << "concurrent sweepers left an orphan owner file";

  ::shm_unlink(signpost.c_str());  // pub unlinks its own segment when it goes out of scope
  ::shm_unlink(orphan_seg.c_str());
  ::shm_unlink(orphan_owner.c_str());
}

// The cleanup lock serializes sweepers. With one held, a second
// sweep_dead() must return without touching anything rather than racing the holder.
TEST(SweepConcurrent, CleanupLockMakesASecondSweeperSkip)
{
  const std::string orphan = flux::signpost_name(uniq("locked"), kFp) + ".424244.97";
  seed_orphan_segment(orphan);
  ASSERT_TRUE(name_exists(orphan));

  int lock_fd = ::shm_open("/flux.sweep", O_CREAT | O_RDWR, 0600);
  ASSERT_GE(lock_fd, 0);
  ASSERT_TRUE(flux::Segment::lock_write(lock_fd));

  flux::sweep_dead();  // must find the cleanup lock held and do nothing
  EXPECT_TRUE(name_exists(orphan)) << "a sweeper ran while another held the cleanup lock";

  ::close(lock_fd);  // releases it
  flux::sweep_dead();
  EXPECT_FALSE(name_exists(orphan)) << "sweep did not reclaim the orphan once unblocked";

  ::shm_unlink(orphan.c_str());
}
