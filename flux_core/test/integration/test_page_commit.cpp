#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/memory.hpp"
#include "support/frame_id.hpp"

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

// Page pre-commit and mlock. flux maps a sparse tmpfs segment on purpose, so a
// large slot_size costs almost no idle RAM; the price is a fault per page on the path that first
// touches it. These tests are about moving those faults to attach and about the refusal being
// visible when the kernel says no.
//
// What is measured is /proc/self/smaps, not a timing: residency is the property, and a latency
// number would be a proxy for it that a loaded machine can make say anything.

namespace
{
using flux::test::publish_id;

std::string uniq(const std::string & base)
{
  return base + "." + std::to_string(::getpid());
}

struct Totals
{
  std::size_t size_kb = 0;
  std::size_t rss_kb = 0;
  std::size_t locked_kb = 0;
};

bool is_header(const std::string & line)
{
  // "<start>-<end> perms offset dev inode path". A detail line is "Key: value kB", and the header
  // has a colon of its own in the device field, so the two are told apart by the address range.
  const std::size_t dash = line.find('-');
  if (dash == std::string::npos || dash == 0) return false;
  for (std::size_t i = 0; i < dash; ++i) {
    if (std::isxdigit(static_cast<unsigned char>(line[i])) == 0) return false;
  }
  return true;
}

// Summed over every mapping in this process backed by a file whose name contains `needle`.
// Summed rather than listed because one channel is not one mapping: a subscriber's
// protect_payload() splits its mapping in two VMAs, and a subscriber also holds the signpost,
// whose shm name is the prefix of the segment's. Resident bytes are the property under test and
// they add up across all of them.
Totals resident_of(const std::string & needle)
{
  Totals t;
  std::ifstream f("/proc/self/smaps");
  std::string line;
  bool in_ours = false;
  while (std::getline(f, line)) {
    if (is_header(line)) {
      in_ours = line.find(needle) != std::string::npos;
      continue;
    }
    if (!in_ours) continue;
    std::istringstream is(line);
    std::string key;
    std::size_t kb = 0;
    is >> key >> kb;
    if (key == "Size:") t.size_kb += kb;
    if (key == "Rss:") t.rss_kb += kb;
    if (key == "Locked:") t.locked_kb += kb;
  }
  return t;
}

constexpr std::uint32_t kSlotSize = 1u << 20;  // 1 MiB, so sparseness is visible in whole pages
constexpr std::uint32_t kSlots = 8;
constexpr std::uint64_t kFp = 0xC0FFEEu;
// What one committed mapping of this segment is worth. The control plane adds a little on top,
// which is why the assertions below are one-sided.
constexpr std::size_t kPayloadKb = (static_cast<std::size_t>(kSlotSize) / 1024) * kSlots;

}  // namespace

// The default is off, and off has to mean sparse. If the segment were resident anyway the rest of
// these tests would pass without the feature existing.
TEST(PageCommit, TheDefaultLeavesTheSegmentSparse)
{
  const std::string name = flux::segment_name(uniq("/flux_pc/default"), kFp);
  flux::Channel pub = flux::Channel::create(name, kSlotSize, kSlots, kFp);

  EXPECT_FALSE(pub.pages_committed());
  EXPECT_FALSE(pub.pages_locked());

  const Totals t = resident_of(name);
  ASSERT_GE(t.size_kb, kPayloadKb) << "the segment mapping was not found in smaps";
  EXPECT_LT(t.rss_kb, kPayloadKb / 2) << "size=" << t.size_kb << "kB rss=" << t.rss_kb << "kB";
}

TEST(PageCommit, PrecommitMakesThePublisherMappingResident)
{
  const std::string name = flux::segment_name(uniq("/flux_pc/pub"), kFp);
  flux::MemoryPolicy mem;
  mem.precommit = true;
  flux::Channel pub = flux::Channel::create(name, kSlotSize, kSlots, kFp, {}, mem);

  EXPECT_TRUE(pub.pages_committed());
  EXPECT_FALSE(pub.pages_locked()) << "a commit is not a lock";

  const Totals t = resident_of(name);
  EXPECT_GE(t.rss_kb, kPayloadKb) << "size=" << t.size_kb << "kB rss=" << t.rss_kb << "kB";
}

// The subscriber's mapping is its own, and its payload is PROT_READ by then
// (Segment::protect_payload). Committing it therefore cannot use MADV_POPULATE_WRITE for the
// payload half, and a policy that quietly skipped that half would leave the faults it promised to
// remove. Publisher uncommitted, subscriber committed: exactly one of the two is resident.
TEST(PageCommit, ASubscriberCommitsItsOwnMappingPastTheReadOnlyPayload)
{
  const std::string name = flux::segment_name(uniq("/flux_pc/sub"), kFp);
  flux::Channel pub = flux::Channel::create(name, kSlotSize, kSlots, kFp);

  flux::MemoryPolicy mem;
  mem.precommit = true;
  flux::Channel sub = flux::Channel::open(name, kFp, {}, mem);

  ASSERT_TRUE(sub.payload_readonly()) << "this test is about committing past the PROT_READ half";
  EXPECT_TRUE(sub.pages_committed());
  EXPECT_FALSE(pub.pages_committed());

  // One mapping's worth is resident, not two: the subscriber's, payload half included. Below
  // kPayloadKb means the read-only half was skipped; at two mappings' worth the publisher would
  // have been committed too, which is not what was asked for.
  const Totals t = resident_of(name);
  EXPECT_GE(t.rss_kb, kPayloadKb) << "the subscriber's read-only payload was not committed";
  EXPECT_LT(t.rss_kb, kPayloadKb + kPayloadKb / 2) << "the publisher's mapping was committed too";
}

// Frames still cross a committed channel. A commit is about residency, not about the protocol,
// and MADV_POPULATE_WRITE on a shared mapping must not disturb what is in it.
TEST(PageCommit, ACommittedChannelStillCarriesFrames)
{
  const std::string name = flux::segment_name(uniq("/flux_pc/carry"), kFp);
  flux::MemoryPolicy mem;
  mem.precommit = true;
  flux::Channel pub = flux::Channel::create(name, 4096, 4, kFp, {}, mem);
  flux::Channel sub = flux::Channel::open(name, kFp, {}, mem);

  std::vector<std::byte> buf(4096);
  ASSERT_EQ(publish_id(pub, buf.data(), buf.size(), 0x77), flux::Published::Ok);

  flux::FrameView v = sub.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(flux::test::frame_id(v), 0x77u);
}

// mlock subsumes the commit -- it populates what it locks -- so a locked mapping reports both.
TEST(PageCommit, LockReportsItselfAndCountsAsCommitted)
{
  const std::string name = flux::segment_name(uniq("/flux_pc/lock"), kFp);
  flux::MemoryPolicy mem;
  mem.lock = true;

  std::optional<flux::Channel> pub;
  try {
    pub.emplace(flux::Channel::create(name, kSlotSize, kSlots, kFp, {}, mem));
  } catch (const std::system_error &) {
    GTEST_SKIP() << "RLIMIT_MEMLOCK on this host does not cover the segment";
  }

  EXPECT_TRUE(pub->pages_locked());
  EXPECT_TRUE(pub->pages_committed()) << "mlock populates, so a locked mapping is committed";

  const Totals t = resident_of(name);
  EXPECT_GE(t.rss_kb, kPayloadKb);
  EXPECT_GE(t.locked_kb, kPayloadKb) << "size=" << t.size_kb << "kB locked=" << t.locked_kb << "kB";
}

// The refusal contract applied to memory: a refusal is reported, never downgraded. A caller
// that asked for a bound and silently got none is the failure this option exists to prevent, so
// the attach fails rather than the channel coming back weaker than declared.
TEST(PageCommit, ALockRefusalThrowsInsteadOfDowngrading)
{
  struct rlimit saved;
  ASSERT_EQ(::getrlimit(RLIMIT_MEMLOCK, &saved), 0);
  if (saved.rlim_cur == RLIM_INFINITY && ::geteuid() == 0) {
    GTEST_SKIP() << "running as root with no memlock limit: the kernel refuses nothing to lower";
  }

  struct rlimit tiny = saved;
  tiny.rlim_cur = 4096;  // one page, far below the segment
  ASSERT_EQ(::setrlimit(RLIMIT_MEMLOCK, &tiny), 0);

  const std::string name = flux::segment_name(uniq("/flux_pc/refuse"), kFp);
  flux::MemoryPolicy mem;
  mem.lock = true;
  bool threw = false;
  try {
    flux::Channel pub = flux::Channel::create(name, kSlotSize, kSlots, kFp, {}, mem);
    (void)pub;
  } catch (const std::system_error &) {
    threw = true;
  }
  ASSERT_EQ(::setrlimit(RLIMIT_MEMLOCK, &saved), 0);  // before any assertion can leave it lowered

  EXPECT_TRUE(threw) << "a refused mlock must not come back as an unlocked channel";
}

// A re-attach is a different mapping and carries none of the old one's residency. A policy that
// applied only at the first attach would hold until the first publisher restart and then stop,
// which is the failure mode nothing else in this file would notice.
TEST(PageCommit, TheCommitFollowsAPublisherRestart)
{
  const std::string name = flux::segment_name(uniq("/flux_pc/restart"), kFp);
  ::shm_unlink(name.c_str());
  std::vector<std::byte> buf(4096);

  std::optional<flux::Channel> pub1;
  pub1.emplace(flux::open_publisher_segment(name, 4096, 4, kFp));
  ASSERT_EQ(publish_id(*pub1, buf.data(), buf.size(), 1), flux::Published::Ok);

  flux::MemoryPolicy mem;
  mem.precommit = true;
  flux::Channel sub = flux::Channel::open(name, kFp, {}, mem);
  ASSERT_TRUE(sub.pages_committed());
  const std::uint32_t gen1 = sub.attach_generation();

  pub1.reset();
  std::optional<flux::Channel> pub2;
  pub2.emplace(flux::open_publisher_segment(name, 4096, 4, kFp));
  ASSERT_EQ(publish_id(*pub2, buf.data(), buf.size(), 2), flux::Published::Ok);

  bool recovered = false;
  for (int i = 0; i < 200 && !recovered; ++i) {
    flux::FrameView v = sub.peek();
    recovered = v && flux::test::frame_id(v) == 2u;
  }
  ASSERT_TRUE(recovered) << "the subscriber never re-attached";
  EXPECT_GT(sub.attach_generation(), gen1);
  EXPECT_TRUE(sub.pages_committed()) << "the replacement mapping was left uncommitted";

  pub2.reset();
  ::shm_unlink(name.c_str());
}
