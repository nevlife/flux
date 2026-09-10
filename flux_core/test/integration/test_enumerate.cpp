#include "flux/discovery.hpp"
#include "flux/owner.hpp"
#include "flux/segment_layout.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// Enumeration reads the /dev/shm names and the manifests of processes still holding their owner
// lock. No registry, no daemon: what it reports is what a reader can see.

namespace
{

std::string uniq(const char * stem)
{
  return std::string(stem) + std::to_string(::getpid());
}

const flux::TopicView * find(const std::vector<flux::TopicView> & all, const std::string & signpost)
{
  const auto it = std::find_if(
    all.begin(), all.end(), [&](const flux::TopicView & t) { return t.signpost == signpost; });
  return it == all.end() ? nullptr : &*it;
}

}  // namespace

TEST(Enumerate, AnnouncedEndpointsComeBackWithTheirKeyAndLabel)
{
  const std::string key = "/" + uniq("enum/pub");
  const std::string sp = flux::signpost_name(key, 0xfeed, "0");

  flux::ManifestEntry e;
  e.signpost = sp;
  e.key = key;
  e.label = "/talker_node";
  e.publisher = true;
  flux::OwnerFile::announce(e);

  // The signpost has to exist for the scan to see the channel at all: announcing is metadata
  // about a channel, not what creates one.
  int fd = ::shm_open(sp.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  ::close(fd);

  const auto all = flux::enumerate_topics();
  const flux::TopicView * t = find(all, sp);
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(t->domain, "0");
  EXPECT_EQ(t->key, key);
  EXPECT_TRUE(t->key_exact);
  EXPECT_EQ(t->fingerprint, 0xfeedu);
  ASSERT_EQ(t->endpoints.size(), 1u);
  EXPECT_TRUE(t->endpoints[0].publisher);
  EXPECT_EQ(t->endpoints[0].label, "/talker_node");
  EXPECT_EQ(t->endpoints[0].owner.pid, static_cast<std::uint32_t>(::getpid()));

  ::shm_unlink(sp.c_str());
}

// A channel nobody holds is still a name: signposts are persistent. What it cannot give
// back is the key's punctuation, and it says so rather than inventing it.
TEST(Enumerate, AChannelWithNoLiveParticipantReportsAnInexactKey)
{
  const std::string key = "/" + uniq("enum/orphan");
  const std::string sp = flux::signpost_name(key, 0xbeef, "0");
  int fd = ::shm_open(sp.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  ::close(fd);

  const auto all = flux::enumerate_topics();
  const flux::TopicView * t = find(all, sp);
  ASSERT_NE(t, nullptr);
  EXPECT_TRUE(t->endpoints.empty());
  EXPECT_FALSE(t->key_exact);
  EXPECT_NE(t->key, key);  // the name's spelling, with every non-alnum character flattened
  // What a tool needs to resolve this channel by the name its publisher used: the two spellings
  // meet under flatten_key, and it is core's rule rather than a copy in the tool.
  EXPECT_EQ(t->key, flux::flatten_key(key));
  EXPECT_EQ(flux::flatten_key(t->key), flux::flatten_key(key));

  ::shm_unlink(sp.c_str());
}

// A crashed participant leaves its owner file behind with the lock auto-released. Its manifest
// describes endpoints nobody holds, so enumeration must not report them -- and must not unlink
// the file either, which is sweep_dead's job.
TEST(Enumerate, AnUnlockedOwnerFilesManifestIsIgnoredAndLeftAlone)
{
  const std::string key = "/" + uniq("enum/dead");
  const std::string sp = flux::signpost_name(key, 0xdead, "0");
  int sfd = ::shm_open(sp.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(sfd, 0);
  ::close(sfd);

  // A plausible owner file for a pid that is not us and holds no lock.
  const std::string ghost = "/flux.owner.999999." + std::to_string(::getpid());
  int fd = ::shm_open(ghost.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  const std::string line = sp + "\tpub\t" + key + "\t/ghost_node\n";
  ASSERT_EQ(::write(fd, line.data(), line.size()), static_cast<ssize_t>(line.size()));
  ::close(fd);

  // The manifest itself is readable -- this is about the lock, not the bytes.
  EXPECT_EQ(flux::OwnerFile::read_manifest(ghost).size(), 1u);

  const auto all = flux::enumerate_topics();
  const flux::TopicView * t = find(all, sp);
  ASSERT_NE(t, nullptr);
  EXPECT_TRUE(t->endpoints.empty());

  int still_there = ::shm_open(ghost.c_str(), O_RDONLY, 0600);
  EXPECT_GE(still_there, 0) << "enumeration unlinked what it was only asked to read";
  if (still_there >= 0) ::close(still_there);

  ::shm_unlink(ghost.c_str());
  ::shm_unlink(sp.c_str());
}

// A truncated tail is what a writer that died mid-append leaves. Drop it; keep the whole lines.
TEST(Enumerate, ReadManifestDropsAnUnterminatedTail)
{
  const std::string ghost = "/flux.owner.999998." + std::to_string(::getpid());
  int fd = ::shm_open(ghost.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  const std::string blob = "/sp.a\tpub\t/a\tnode\n/sp.b\tsub\t/b\tnode\n/sp.c\tpub\t/c";
  ASSERT_EQ(::write(fd, blob.data(), blob.size()), static_cast<ssize_t>(blob.size()));
  ::close(fd);

  const auto m = flux::OwnerFile::read_manifest(ghost);
  ASSERT_EQ(m.size(), 2u);
  EXPECT_EQ(m[0].signpost, "/sp.a");
  EXPECT_TRUE(m[0].publisher);
  EXPECT_EQ(m[1].signpost, "/sp.b");
  EXPECT_FALSE(m[1].publisher);

  ::shm_unlink(ghost.c_str());
}

// Regression: sweep_dead used to tell an owner file from a signpost by reading offset 4, on the
// grounds that an owner file was too small to hold a magic. A manifest makes it large enough, so
// a dead owner whose manifest happens to carry kSignpostMagic there would have been preserved
// forever. The name says which kind it is, and the name is the ABI.
TEST(Enumerate, SweepUnlinksADeadOwnerFileWhoseManifestLooksLikeASignpost)
{
  const std::string ghost = "/flux.owner.999997." + std::to_string(::getpid());
  int fd = ::shm_open(ghost.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(fd, 0);
  std::uint8_t bytes[16] = {};
  const std::uint32_t magic = flux::kSignpostMagic;
  std::memcpy(bytes + 4, &magic, sizeof(magic));
  ASSERT_EQ(::write(fd, bytes, sizeof(bytes)), static_cast<ssize_t>(sizeof(bytes)));
  ::close(fd);

  flux::sweep_dead();

  int gone = ::shm_open(ghost.c_str(), O_RDONLY, 0600);
  EXPECT_LT(gone, 0) << "a dead owner file survived the sweep by looking like a signpost";
  if (gone >= 0) {
    ::close(gone);
    ::shm_unlink(ghost.c_str());
  }
}
