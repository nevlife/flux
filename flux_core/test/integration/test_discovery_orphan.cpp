#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // F_OFD_SETLK via Segment::lock_read
#endif

#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/segment.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

// The bootstrap test seam lives in discovery.cpp under FLUX_TESTING (set by BUILD_TESTING). It
// fires once in the create path, right after the unique segment's shm_open and before its lock --
// the R2 window a sweeper can unlink the name in.
namespace flux
{
namespace detail
{
extern void (*g_bootstrap_after_open)(const std::string & name, int fd);
extern void (*g_signpost_after_open)(const std::string & name, int fd);
}  // namespace detail
}  // namespace flux

namespace
{

constexpr std::uint64_t kFp = 0xdeadbeefdeadbeefULL;
constexpr std::uint32_t kSlotSize = 256;
constexpr std::uint32_t kSlotCount = 4;
const std::string kName = "/flux.test.discovery.orphan.dead." + std::to_string(::getpid());

void reset_seam()
{
  flux::detail::g_bootstrap_after_open = nullptr;
  flux::detail::g_signpost_after_open = nullptr;
}

// The seam callback the R2 scenario installs: unlink the just-created unique segment so the
// creator's fd is now on a nameless object, then disarm so the bootstrap retry runs untouched.
void unlink_and_disarm(const std::string & name, int)
{
  ::shm_unlink(name.c_str());
  flux::detail::g_bootstrap_after_open = nullptr;
}

// Same, for the signpost's own O_CREAT..lock window.
void unlink_signpost_and_disarm(const std::string & name, int)
{
  ::shm_unlink(name.c_str());
  flux::detail::g_signpost_after_open = nullptr;
}

}  // namespace

// Normal bootstrap: a publisher creates the signpost + a unique current
// segment, and a subscriber resolving the fixed signpost name reaches the same object.
TEST(DiscoveryOrphan, NormalBootstrapBindsTheSignpost)
{
  reset_seam();
  ::shm_unlink(kName.c_str());

  flux::Segment seg = flux::open_publisher_segment(kName, kSlotSize, kSlotCount, kFp);
  EXPECT_NE(flux::signpost_epoch(kName), 0u) << "signpost advertises no current segment";
  {
    flux::Segment sub = flux::open_subscriber_segment(kName, kFp);
    EXPECT_EQ(sub.id(), seg.id()) << "subscriber resolved a different object than the publisher's";
  }

  seg = flux::Segment{};  // last out -> unlinks the unique segment
  ::shm_unlink(kName.c_str());
}

// R2: a sweeper unlinks the unique segment in the O_CREAT..lock window.
// The seam reproduces that; the creator's post-lock name_still_bound recheck must send it back to
// a fresh create so the published stream stays reachable through the signpost.
TEST(DiscoveryOrphan, CreatePathRetriesWhenUniqueSegmentUnlinkedBeforeLock)
{
  reset_seam();
  ::shm_unlink(kName.c_str());

  flux::detail::g_bootstrap_after_open = &unlink_and_disarm;
  flux::Segment seg = flux::open_publisher_segment(kName, kSlotSize, kSlotCount, kFp);
  EXPECT_EQ(flux::detail::g_bootstrap_after_open, nullptr) << "seam never fired: R2 window not hit";

  EXPECT_NE(flux::signpost_epoch(kName), 0u);
  {
    flux::Segment sub = flux::open_subscriber_segment(kName, kFp);
    EXPECT_EQ(sub.id(), seg.id()) << "creator did not recover to a name-reachable segment";
  }

  seg = flux::Segment{};
  reset_seam();
  ::shm_unlink(kName.c_str());
}

// The signpost's own O_CREAT..lock window. Before the lock the object is
// unlocked with magic still 0 -- exactly what sweep_dead() unlinks, and every publisher bootstrap
// runs a sweep. Bootstrapping the unbound fd would advertise this topic on a nameless orphan that
// no subscriber can resolve, while the creator reports success. The post-lock recheck must send it
// back to a fresh object instead.
TEST(DiscoveryOrphan, SignpostBootstrapRetriesWhenNameUnlinkedBeforeLock)
{
  reset_seam();
  ::shm_unlink(kName.c_str());

  flux::detail::g_signpost_after_open = &unlink_signpost_and_disarm;
  flux::Segment seg = flux::open_publisher_segment(kName, kSlotSize, kSlotCount, kFp);
  EXPECT_EQ(flux::detail::g_signpost_after_open, nullptr) << "seam never fired: window not hit";

  EXPECT_NE(flux::signpost_epoch(kName), 0u) << "publisher advertised on a nameless orphan";
  {
    flux::Segment sub = flux::open_subscriber_segment(kName, kFp);
    EXPECT_EQ(sub.id(), seg.id()) << "subscriber cannot resolve the publisher's segment";
  }

  seg = flux::Segment{};
  reset_seam();
  ::shm_unlink(kName.c_str());
}

// Sweep reclaims a crashed publisher's orphan segment (no live read-lock holder) but never a live
// one (a held read lock). Without sweep, unique names leak one segment per crash.
TEST(DiscoveryOrphan, SweepReclaimsDeadSegmentNotLiveOne)
{
  reset_seam();
  const std::string orphan = kName + ".111.222";  // a crashed publisher's unique segment
  const std::string live = kName + ".333.444";
  ::shm_unlink(orphan.c_str());
  ::shm_unlink(live.c_str());

  // Orphan: a named, ready segment with no lock held -- exactly what a crash leaves behind.
  {
    int fd = ::shm_open(orphan.c_str(), O_CREAT | O_RDWR, 0600);
    ASSERT_GE(fd, 0);
    flux::SegmentLayout layout{kSlotSize, kSlotCount};
    const std::size_t bytes = layout.total_bytes();
    ASSERT_EQ(::ftruncate(fd, static_cast<off_t>(bytes)), 0);
    void * p = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ASSERT_NE(p, MAP_FAILED);
    flux::Segment::init_header(static_cast<std::byte *>(p), layout, kSlotSize, kSlotCount, kFp);
    ::munmap(p, bytes);
    ::close(fd);  // no lock, name bound -> a crashed orphan
  }
  // Live: hold a read lock for the whole test (a live publisher's liveness lock).
  int live_fd = ::shm_open(live.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(live_fd, 0);
  ASSERT_EQ(::ftruncate(live_fd, 64), 0);
  ASSERT_TRUE(flux::Segment::lock_read(live_fd));

  flux::sweep_dead();

  flux::SegmentId id;
  EXPECT_FALSE(flux::stat_segment(orphan, id)) << "sweep did not reclaim the dead orphan";
  EXPECT_TRUE(flux::stat_segment(live, id)) << "sweep unlinked a segment with a live holder";

  ::close(live_fd);
  ::shm_unlink(live.c_str());
  ::shm_unlink(orphan.c_str());
}

// A crashed publisher group leaves the unique segment named, ready, and unlocked. An attach must
// not adopt that dead stream: no frame will ever come, and the mapping
// would pin the memory until a new publisher rotates the signpost.
TEST(DiscoveryOrphan, SubscriberRefusesASegmentWithNoLivePublisher)
{
  reset_seam();
  const std::string name = "/flux.test.discovery.orphan.nopub." + std::to_string(::getpid());
  ::shm_unlink(name.c_str());

  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    flux::Segment seg = flux::open_publisher_segment(name, kSlotSize, kSlotCount, kFp);
    (void)seg;
    ::_exit(0);  // crash-like: skip destructors, so the unique name stays bound
  }
  int status = 0;
  ASSERT_EQ(::waitpid(pid, &status, 0), pid);
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);

  try {
    flux::Segment sub = flux::open_subscriber_segment(name, kFp);
    FAIL() << "adopted a segment with no live publisher";
  } catch (const flux::SegmentMismatch &) {
    FAIL() << "the refusal must be transient (retryable), not a mismatch";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("no live publisher"), std::string::npos) << e.what();
  }

  flux::sweep_dead();  // reclaim the corpse; the signpost persists by design
  ::shm_unlink(name.c_str());
}

// The advertised current can die without a successor. A starved
// subscriber must flag the dead stream so its owner can drop the mapping instead of pinning it.
TEST(DiscoveryOrphan, TakeFlagsOrphanWhenThePublisherGroupDies)
{
  reset_seam();
  const std::string name = "/flux.test.discovery.orphan.flag." + std::to_string(::getpid());
  ::shm_unlink(name.c_str());

  auto pub =
    std::make_unique<flux::Channel>(flux::Channel::create(name, kSlotSize, kSlotCount, kFp));
  flux::Channel sub = flux::Channel::open(name, kFp);
  EXPECT_FALSE(sub.orphaned());

  pub.reset();  // last publisher out: the unique name is unlinked, the signpost stays

  for (int i = 0; i < 64 && !sub.orphaned(); ++i) (void)sub.take();
  EXPECT_TRUE(sub.orphaned());

  ::shm_unlink(name.c_str());
}
