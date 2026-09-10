#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // F_OFD_SETLK
#endif

#include "flux/owner.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <csignal>
#include <cstring>
#include <string>
#include <vector>

// The ensure() test seam lives in owner.cpp under FLUX_TESTING (set by BUILD_TESTING). It fires
// right after the owner file's shm_open and before its lock -- the window a sweeper can unlink
// the name in, the same R2 shape segments and signposts guard.
namespace flux
{
namespace detail
{
extern void (*g_owner_after_open)(const std::string & name, int fd);
extern std::uint64_t (*g_starttime_override)();
}  // namespace detail
}  // namespace flux

namespace
{

// OFD write-lock a whole shm file by name; returns the held fd (>=0) or -1. The caller
// keeps the fd open to keep the lock. Test scaffolding that mirrors OwnerFile internals.
int lock_owner_named(const std::string & name)
{
  int fd = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
  if (fd < 0) return -1;
  struct flock fl;
  std::memset(&fl, 0, sizeof(fl));
  fl.l_type = F_WRLCK;
  fl.l_whence = SEEK_SET;
  if (::fcntl(fd, F_OFD_SETLK, &fl) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

}  // namespace

TEST(Owner, SelfIdentityIsPopulated)
{
  const flux::OwnerId & id = flux::OwnerFile::self();
  EXPECT_NE(id.pid, 0u);
  EXPECT_EQ(id.pid, static_cast<std::uint32_t>(::getpid()));
  EXPECT_NE(id.starttime, 0u);  // /proc/self/stat field 22 is always nonzero for a live proc
  EXPECT_TRUE(id.valid());
}

TEST(Owner, FileNameFormat)
{
  flux::OwnerId id;
  id.pid = 4242;
  id.starttime = 99887766;
  EXPECT_EQ(flux::owner_file_name(id), "/flux.owner.4242.99887766");
}

TEST(Owner, EnsureThenSelfIsAlive)
{
  flux::OwnerFile::ensure();
  flux::OwnerFile::ensure();  // idempotent
  // Our own owner file is OFD-locked; a fresh probe open is a distinct OFD and conflicts,
  // so the process is seen Alive by anyone (including itself).
  EXPECT_EQ(flux::OwnerFile::probe(flux::OwnerFile::self()), flux::Liveness::Alive);
}

TEST(Owner, MissingOwnerFileIsDead)
{
  flux::OwnerId ghost;
  ghost.pid = 4000000;  // no such owner file exists
  ghost.starttime = 1;
  EXPECT_EQ(flux::OwnerFile::probe(ghost), flux::Liveness::Dead);
}

TEST(Owner, UnlockedIsDeadLockedIsAlive)
{
  flux::OwnerId id;
  id.pid = 4000001;
  id.starttime = 7;
  const std::string name = flux::owner_file_name(id);
  ::shm_unlink(name.c_str());  // clean slate

  // File exists but nobody holds the lock -> Dead (a crashed owner's file).
  int unlocked = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
  ASSERT_GE(unlocked, 0);
  EXPECT_EQ(flux::OwnerFile::probe(id), flux::Liveness::Dead);
  ::close(unlocked);

  // Held write lock -> Alive.
  int held = lock_owner_named(name);
  ASSERT_GE(held, 0);
  EXPECT_EQ(flux::OwnerFile::probe(id), flux::Liveness::Alive);

  ::close(held);  // release -> Dead again
  EXPECT_EQ(flux::OwnerFile::probe(id), flux::Liveness::Dead);
  ::shm_unlink(name.c_str());
}

// The core OFD guarantee: a process that dies holding its owner lock is detected Dead,
// because the kernel releases the lock on death. Managed manually (not via the process
// singleton) so the child holds a distinct owner file, not the test process's.
TEST(Owner, ForkedHolderIsDeadAfterExit)
{
  flux::OwnerId id;
  id.pid = 4000002;  // synthetic name; probe only opens by name and inspects the lock
  id.starttime = 11;
  const std::string name = flux::owner_file_name(id);
  ::shm_unlink(name.c_str());

  int up[2], down[2];  // child->parent "locked", parent->child "go die"
  ASSERT_EQ(::pipe(up), 0);
  ASSERT_EQ(::pipe(down), 0);

  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    // Child: hold the owner lock, tell the parent, wait, then _exit WITHOUT releasing.
    int fd = lock_owner_named(name);
    char b = (fd >= 0) ? 'k' : 'x';
    [[maybe_unused]] ssize_t w = ::write(up[1], &b, 1);
    char go;
    [[maybe_unused]] ssize_t r = ::read(down[0], &go, 1);
    _exit(0);  // kernel releases the OFD lock here
  }

  ::close(up[1]);
  ::close(down[0]);
  char b = 'x';
  ASSERT_EQ(::read(up[0], &b, 1), 1);
  ASSERT_EQ(b, 'k');  // child holds the lock

  EXPECT_EQ(flux::OwnerFile::probe(id), flux::Liveness::Alive);  // child alive, lock held

  [[maybe_unused]] ssize_t w = ::write(down[1], "g", 1);  // tell child to exit
  int status = 0;
  ASSERT_EQ(::waitpid(pid, &status, 0), pid);

  EXPECT_EQ(flux::OwnerFile::probe(id), flux::Liveness::Dead);  // lock auto-released on death

  ::close(up[0]);
  ::close(down[1]);
  ::shm_unlink(name.c_str());
}

// A crash (SIGKILL) skips atexit, so the owner file survives with its lock auto-released. A
// probe must report Dead AND reap the stale file -- otherwise /dev/shm accumulates one leaked
// owner file per crash, since the name is unique per (pid, starttime) and nothing overwrites it.
TEST(OwnerFile, ProbeReapsStaleFileFromCrashedOwner)
{
  flux::OwnerId id;
  id.pid = 4000003;  // synthetic name; the child locks it directly, not via the singleton
  id.starttime = 13;
  const std::string name = flux::owner_file_name(id);
  ::shm_unlink(name.c_str());  // clean slate

  int up[2];  // child->parent "locked"
  ASSERT_EQ(::pipe(up), 0);

  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    // Child: hold the owner lock, announce it, then wait to be killed -- never releases cleanly.
    int fd = lock_owner_named(name);
    char b = (fd >= 0) ? 'k' : 'x';
    [[maybe_unused]] ssize_t w = ::write(up[1], &b, 1);
    for (;;) ::pause();  // SIGKILL cannot be caught, so no atexit and no clean unlink
  }

  ::close(up[1]);
  char b = 'x';
  ASSERT_EQ(::read(up[0], &b, 1), 1);
  ASSERT_EQ(b, 'k');  // child holds the lock
  ::close(up[0]);

  EXPECT_EQ(flux::OwnerFile::probe(id), flux::Liveness::Alive);  // child alive, lock held

  ASSERT_EQ(::kill(pid, SIGKILL), 0);  // crash: no atexit runs, the file is left behind
  int status = 0;
  ASSERT_EQ(::waitpid(pid, &status, 0), pid);

  // The stale file is still present (the crash left it) with its lock auto-released: a
  // non-creating open succeeds.
  int leaked = ::shm_open(name.c_str(), O_RDWR, 0600);
  EXPECT_GE(leaked, 0) << "the crashed owner's file should still exist before the reaping probe";
  if (leaked >= 0) ::close(leaked);

  EXPECT_EQ(flux::OwnerFile::probe(id), flux::Liveness::Dead);  // dead, and reaps the file

  // After the reaping probe the file is gone: a non-creating open now fails.
  int after = ::shm_open(name.c_str(), O_RDWR, 0600);
  EXPECT_LT(after, 0) << "probe() did not reap the dead owner's stale file";
  if (after >= 0) ::close(after);

  ::shm_unlink(name.c_str());
}

// A fork child inherits the atexit handler and the cached identity but owns neither the fd nor
// the file. Without a pid guard its ordinary exit() unlinked the LIVE parent's owner file, after
// which every reclaimer probed the parent as dead and could steal its borrows.
TEST(OwnerFile, ForkChildExitDoesNotUnlinkParentOwnerFile)
{
  flux::OwnerFile::ensure();
  const flux::OwnerId me = flux::OwnerFile::self();
  ASSERT_EQ(flux::OwnerFile::probe(me), flux::Liveness::Alive);

  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::exit(0);  // never touches flux; runs the inherited atexit handler
  }
  int st = 0;
  ASSERT_EQ(::waitpid(pid, &st, 0), pid);

  EXPECT_EQ(flux::OwnerFile::probe(me), flux::Liveness::Alive)
    << "a forked child's exit removed this live process's owner file";
}

// ensure()'s own O_CREAT..lock window. An owner file is size 0, so it carries no magic and is
// exactly what sweep_dead() unlinks -- and every publisher start runs a sweep. Locking the
// unlinked inode leaves a LIVE process nameless: every probe of it reads Dead, so a publisher
// reclaims its live borrows and overwrites bytes it is still reading. The post-lock recheck must
// send it back to a fresh object instead. Run in a child: ensure() is once per process.
TEST(OwnerFile, EnsureRetriesWhenNameUnlinkedBeforeLock)
{
  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    flux::detail::g_owner_after_open = [](const std::string & name, int) {
      ::shm_unlink(name.c_str());  // what a sweeper does inside the window
      flux::detail::g_owner_after_open = nullptr;
    };
    try {
      flux::OwnerFile::ensure();
    } catch (...) {
      _exit(3);  // gave up instead of retrying onto a fresh object
    }
    if (flux::detail::g_owner_after_open != nullptr) _exit(4);  // seam never fired
    const flux::OwnerId me = flux::OwnerFile::self();
    int fd = ::shm_open(flux::owner_file_name(me).c_str(), O_RDONLY, 0600);
    if (fd < 0) _exit(5);  // alive, but its name is gone
    ::close(fd);
    if (flux::OwnerFile::probe(me) != flux::Liveness::Alive) _exit(6);
    _exit(0);
  }
  int st = 0;
  ASSERT_EQ(::waitpid(pid, &st, 0), pid);
  ASSERT_TRUE(WIFEXITED(st));
  EXPECT_EQ(WEXITSTATUS(st), 0)
    << "ensure() left this live process unprobeable (3=threw, 4=seam missed, 5=nameless, 6=Dead)";
}

// The other half of the same window: the sweeper got the lock first. ensure() must wait it out,
// not fail the borrow -- the sweeper is about to unlink the file and a fresh O_CREAT then wins.
// The seam holds the lock on the first attempt and drops it on the retry, so a build without the
// retry loop throws here.
TEST(OwnerFile, EnsureRetriesWhenTheLockIsMomentarilyHeld)
{
  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    static int block_fd = -1;
    static int fired = 0;
    flux::detail::g_owner_after_open = [](const std::string & name, int) {
      ++fired;
      if (block_fd < 0) {  // first attempt: a "sweeper" is probing this file
        block_fd = ::shm_open(name.c_str(), O_RDWR, 0600);
        if (block_fd >= 0) {
          struct flock fl;
          std::memset(&fl, 0, sizeof(fl));
          fl.l_type = F_WRLCK;
          fl.l_whence = SEEK_SET;
          if (::fcntl(block_fd, F_OFD_SETLK, &fl) != 0) {
            ::close(block_fd);
            block_fd = -1;
          }
        }
      } else {
        ::close(block_fd);  // the sweeper let go; this attempt must get the lock
        block_fd = -2;
      }
    };
    try {
      flux::OwnerFile::ensure();
    } catch (...) {
      _exit(3);  // gave up on a lock that was only momentarily held
    }
    if (fired < 2) _exit(4);  // never retried: the window was not exercised
    if (flux::OwnerFile::probe(flux::OwnerFile::self()) != flux::Liveness::Alive) _exit(5);
    _exit(0);
  }
  int st = 0;
  ASSERT_EQ(::waitpid(pid, &st, 0), pid);
  ASSERT_TRUE(WIFEXITED(st));
  EXPECT_EQ(WEXITSTATUS(st), 0)
    << "ensure() did not wait out a momentarily held lock (3=threw, 4=never retried, 5=Dead)";
}

// Dead is the one verdict a probe must never guess: it hands a live borrower's slot back to the
// publisher. Any open failure other than "the file is gone" is no evidence of death. fd
// exhaustion is the reachable case -- reclaim_dead() probes once per holder per slot.
TEST(OwnerFile, ProbeDoesNotReportDeadWhenOutOfFds)
{
  flux::OwnerFile::ensure();
  const flux::OwnerId me = flux::OwnerFile::self();
  ASSERT_EQ(flux::OwnerFile::probe(me), flux::Liveness::Alive);

  struct rlimit saved
  {
  };
  ASSERT_EQ(::getrlimit(RLIMIT_NOFILE, &saved), 0);
  struct rlimit low = saved;
  low.rlim_cur = 64;
  ASSERT_EQ(::setrlimit(RLIMIT_NOFILE, &low), 0);

  std::vector<int> hog;
  for (;;) {
    int fd = ::open("/dev/null", O_RDONLY | O_CLOEXEC);
    if (fd < 0) break;
    hog.push_back(fd);
  }
  const flux::Liveness verdict = flux::OwnerFile::probe(me);
  for (int fd : hog) ::close(fd);
  ASSERT_EQ(::setrlimit(RLIMIT_NOFILE, &saved), 0);

  EXPECT_EQ(verdict, flux::Liveness::Alive)
    << "fd exhaustion read as death: a live borrower's slot goes back to the publisher";
}

// Identity is cached per thread now, so the one event that changes it has to
// retire the cache: after fork the child is a different process and must not stamp holder entries
// with its parent's id. Doing so would make a reclaimer probe the LIVE parent's owner file for a
// borrow the child holds -- and, the other way round, leave the child untracked.
TEST(OwnerFile, ForkChildSeesItsOwnIdentityNotTheCachedParentOne)
{
  const flux::OwnerId parent = flux::OwnerFile::self();  // populate this thread's cache
  ASSERT_NE(parent.pid, 0u);

  int down[2];
  ASSERT_EQ(::pipe(down), 0);
  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::close(down[0]);
    const flux::OwnerId mine = flux::OwnerFile::self();  // must NOT be the inherited cache
    [[maybe_unused]] ssize_t w = ::write(down[1], &mine, sizeof(mine));
    _exit(0);
  }
  ::close(down[1]);
  flux::OwnerId child{};
  ASSERT_EQ(::read(down[0], &child, sizeof(child)), static_cast<ssize_t>(sizeof(child)));
  ::close(down[0]);
  int st = 0;
  ASSERT_EQ(::waitpid(pid, &st, 0), pid);

  EXPECT_EQ(child.pid, static_cast<std::uint32_t>(pid))
    << "the child reused its parent's cached identity";
  EXPECT_NE(child.pid, parent.pid);
  EXPECT_NE(child.starttime, parent.starttime) << "starttime came from the cache, not /proc";
  EXPECT_EQ(flux::OwnerFile::self().pid, parent.pid) << "the parent's cache was disturbed";
}

// Same for ensure(): its fast path is the same cache, so a child that skipped it would hold the
// parent's owner fd and never take a file of its own -- nothing would ever probe it alive.
TEST(OwnerFile, ForkChildTakesItsOwnOwnerFile)
{
  flux::OwnerFile::ensure();
  const flux::OwnerId parent = flux::OwnerFile::self();

  int down[2];
  ASSERT_EQ(::pipe(down), 0);
  pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::close(down[0]);
    char ok = 'n';
    try {
      flux::OwnerFile::ensure();
      const flux::OwnerId mine = flux::OwnerFile::self();
      // Its own file exists and probes alive, and it is not the parent's.
      int fd = ::shm_open(flux::owner_file_name(mine).c_str(), O_RDONLY, 0600);
      if (
        fd >= 0 && mine.pid != parent.pid &&
        flux::OwnerFile::probe(mine) == flux::Liveness::Alive) {
        ok = 'y';
      }
      if (fd >= 0) ::close(fd);
    } catch (...) {
    }
    [[maybe_unused]] ssize_t w = ::write(down[1], &ok, 1);
    _exit(0);
  }
  ::close(down[1]);
  char ok = 'n';
  ASSERT_EQ(::read(down[0], &ok, 1), 1);
  ::close(down[0]);
  int st = 0;
  ASSERT_EQ(::waitpid(pid, &st, 0), pid);

  EXPECT_EQ(ok, 'y') << "the child skipped ensure() and rode on its parent's owner file";
  EXPECT_EQ(flux::OwnerFile::probe(parent), flux::Liveness::Alive)
    << "the child's exit disturbed the parent's owner file";
}

// starttime is the only thing that separates a dead participant from a live one that inherited
// its pid. Reading it can fail (a sandbox or container with a restricted /proc), and it used to
// fail into 0 with nobody checking, which registered the process under a degraded identity and
// left every later probe of the dead predecessor reading Alive.
TEST(OwnerFile, EnsureRefusesAnIdentityWithNoStartTime)
{
  int down[2];
  ASSERT_EQ(::pipe(down), 0);
  pid_t pid = ::fork();  // a child: ensure() caches per process and this one must start clean
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::close(down[0]);
    flux::detail::g_starttime_override = [] { return std::uint64_t{0}; };
    char verdict = 'n';  // n = did not throw
    try {
      flux::OwnerFile::ensure();
    } catch (const std::runtime_error &) {
      verdict = 't';
    }
    [[maybe_unused]] ssize_t w = ::write(down[1], &verdict, 1);
    _exit(0);
  }
  ::close(down[1]);
  char verdict = 0;
  ASSERT_EQ(::read(down[0], &verdict, 1), 1);
  ::close(down[0]);
  int st = 0;
  ASSERT_EQ(::waitpid(pid, &st, 0), pid);
  EXPECT_EQ(verdict, 't') << "ensure() registered under a pid-only identity";
}

// The property that refusal protects: two participants that share a pid but not a start time are
// two identities, so probing the dead one does not read the live one's lock.
TEST(OwnerFile, SamePidDifferentStartTimeAreDistinctIdentities)
{
  flux::OwnerFile::ensure();
  const flux::OwnerId live = flux::OwnerFile::self();

  flux::OwnerId recycled = live;  // same pid, as a successor would have
  recycled.starttime = live.starttime + 1;

  EXPECT_NE(flux::owner_file_name(live), flux::owner_file_name(recycled));
  EXPECT_EQ(flux::OwnerFile::probe(live), flux::Liveness::Alive);
  EXPECT_EQ(flux::OwnerFile::probe(recycled), flux::Liveness::Dead)
    << "a pid match alone made the live process answer for a dead one";
}
