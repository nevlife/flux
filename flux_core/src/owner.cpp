#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // F_OFD_SETLK
#endif

#include "flux/owner.hpp"

#include <fcntl.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace flux
{

#ifdef FLUX_TESTING
namespace detail
{
extern std::uint64_t (*g_starttime_override)();  // defined below, used before that point
}
#endif

namespace
{

// Parse field 22 (starttime) of /proc/self/stat. comm (field 2) is wrapped in parens and
// may itself contain spaces or ')', so scan past the LAST ')': the remainder begins at
// field 3 (state), and starttime is the 20th whitespace-separated token after it.
std::uint64_t read_self_starttime()
{
  int fd = ::open("/proc/self/stat", O_RDONLY | O_CLOEXEC);
  if (fd < 0) return 0;
  char buf[1024];
  ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
  ::close(fd);
  if (n <= 0) return 0;
  buf[n] = '\0';

  const char * p = std::strrchr(buf, ')');
  if (p == nullptr) return 0;
  ++p;  // just past ')'; next non-space token is field 3

  int field = 3;
  while (*p != '\0') {
    while (*p == ' ') ++p;
    if (*p == '\0') break;
    if (field == 22) return std::strtoull(p, nullptr, 10);
    while (*p != ' ' && *p != '\0') ++p;  // skip this token
    ++field;
  }
  return 0;
}

bool try_write_lock(int fd)  // F_OFD_SETLK whole-file write lock; false if held/refused
{
  struct flock fl;
  std::memset(&fl, 0, sizeof(fl));
  fl.l_type = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len = 0;  // whole file
  return ::fcntl(fd, F_OFD_SETLK, &fl) == 0;
}

// Does `name` still resolve to the object `fd` is open on? Called only while holding the write
// lock, which blocks a sweeper from unlinking, so a single post-lock check closes the window.
bool name_bound_to(const std::string & name, int fd) noexcept
{
  struct stat mine
  {
  };
  if (::fstat(fd, &mine) != 0) return false;
  int probe = ::shm_open(name.c_str(), O_RDONLY, 0600);
  if (probe < 0) return false;
  struct stat now
  {
  };
  const bool ok = ::fstat(probe, &now) == 0;
  ::close(probe);
  return ok && now.st_dev == mine.st_dev && now.st_ino == mine.st_ino;
}

std::mutex g_mtx;
int g_owner_fd = -1;
std::uint32_t g_owner_pid = 0;  // pid that took g_owner_fd; a fork child inherits neither
OwnerId g_self;                 // valid when g_self.pid == getpid()
bool g_atexit_registered = false;
bool g_atfork_registered = false;

// Identity is constant for the life of a process, and every borrow needs it (holder stamping).
// Reading it through the mutex meant a getpid() syscall and a lock with no
// priority inheritance on the borrow path -- an unbounded inversion once callbacks run on threads
// at different RT priorities. So cache it per thread and invalidate on the one
// event that changes it.
//
// The invalidation is a generation, not a flag: fork gives the child its own copy of this memory,
// so bumping it in the child's atfork handler retires every thread's cache there while the
// parent's is untouched. A flag would only cover the thread that called fork.
std::atomic<std::uint32_t> g_identity_gen{1};
thread_local OwnerId t_self;
thread_local std::uint32_t t_self_gen = 0;    // generation t_self was filled at; 0 = empty
thread_local std::uint32_t t_ensure_gen = 0;  // generation ensure() last completed at

void on_fork_child() noexcept
{
  g_identity_gen.fetch_add(1, std::memory_order_relaxed);
}

// Caller holds g_mtx. Registered once, from the slow path, so the fast path stays branch-free.
void register_atfork_locked()
{
  if (g_atfork_registered) return;
  g_atfork_registered = true;
  ::pthread_atfork(nullptr, nullptr, &on_fork_child);
}

// Caller holds g_mtx. Refresh identity when the pid changed (first call, or after fork -- a
// forked child has a new pid and must not reuse the parent's cached identity or owner fd).
void refresh_identity_locked()
{
  const std::uint32_t cur = static_cast<std::uint32_t>(::getpid());
  if (g_self.pid == cur) return;
  if (g_owner_fd >= 0) {
    ::close(g_owner_fd);  // our inherited copy; the parent keeps its own fd, so its OFD lock
    g_owner_fd = -1;      // (shared via fork) stays held. We take a fresh owner file below.
    g_owner_pid = 0;
  }
  g_self.pid = cur;
#ifdef FLUX_TESTING
  g_self.starttime =
    detail::g_starttime_override ? detail::g_starttime_override() : read_self_starttime();
#else
  g_self.starttime = read_self_starttime();
#endif
}

const OwnerId & self_id()  // caller must hold g_mtx
{
  refresh_identity_locked();
  return g_self;
}

}  // namespace

#ifdef FLUX_TESTING
namespace detail
{
// Test seam: invoked right after ensure()'s shm_open, before the lock, so a test can force the
// sweep-vs-create window deterministically. Compiled out of release builds (FLUX_TESTING is set
// only under BUILD_TESTING); never assigned in production.
void (*g_owner_after_open)(const std::string & name, int fd) = nullptr;

// Test seam for read_self_starttime(): its failure needs an unreadable /proc to provoke.
std::uint64_t (*g_starttime_override)() = nullptr;
}  // namespace detail
#endif

std::string owner_file_name(const OwnerId & id)
{
  char name[64];
  std::snprintf(
    name, sizeof(name), "/flux.owner.%u.%llu", id.pid,
    static_cast<unsigned long long>(id.starttime));
  return std::string(name);
}

const OwnerId & OwnerFile::self()
{
  const std::uint32_t gen = g_identity_gen.load(std::memory_order_relaxed);
  if (t_self_gen == gen) return t_self;  // borrow path: one relaxed load and a compare
  {
    std::lock_guard<std::mutex> lk(g_mtx);
    register_atfork_locked();
    t_self = self_id();
  }
  t_self_gen = gen;
  return t_self;
}

void OwnerFile::ensure()
{
  const std::uint32_t gen = g_identity_gen.load(std::memory_order_relaxed);
  if (t_ensure_gen == gen) return;  // our owner file is taken and this is still our identity
  std::lock_guard<std::mutex> lk(g_mtx);
  register_atfork_locked();
  refresh_identity_locked();  // still the fork fd cleanup: the fast path above never skips it
  // Without starttime a recycled pid makes a dead participant and its successor one identity, so
  // a reclaimer reads the successor's lock and never reclaims the dead one's borrows.
  if (g_self.starttime == 0) {
    throw std::runtime_error(
      "flux: cannot read this process's start time from /proc/self/stat, so its identity would "
      "not survive pid reuse. flux needs a readable /proc; check the sandbox or container.");
  }
  if (g_owner_fd >= 0) {
    t_ensure_gen = gen;
    return;  // already have our owner file for this pid
  }

  const std::string name = owner_file_name(g_self);
  // Same O_CREAT..lock window that segments and signposts guard: until
  // the lock lands the file is unlocked and carries no magic, which is exactly what sweep_dead()
  // unlinks -- and every publisher start runs a sweep. Locking an unlinked inode would leave this
  // LIVE process nameless, so every later probe of it reads Dead and reclaims its borrows out from
  // under it. Recheck after the lock (which pins the binding) and start over on a fresh object if
  // the name moved. A refused lock is the same sweeper mid-probe: retry rather than fail a borrow.
  const struct timespec nap = {0, 100000};  // 100 us
  int fd = -1;
  int err = EAGAIN;
  for (int attempt = 0; attempt < 16 && fd < 0; ++attempt) {
    int f = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
    if (f < 0) {
      throw std::runtime_error(
        std::string("flux: owner shm_open '") + name + "': " + std::strerror(errno));
    }
#ifdef FLUX_TESTING
    if (detail::g_owner_after_open) detail::g_owner_after_open(name, f);
#endif
    if (!try_write_lock(f)) {
      err = errno;
      ::close(f);
      ::nanosleep(&nap, nullptr);
      continue;
    }
    if (!name_bound_to(name, f)) {
      ::close(f);  // drops the lock; the object is nameless now, so let it go
      continue;
    }
    fd = f;
  }
  if (fd < 0) {
    throw std::runtime_error(
      std::string("flux: owner OFD lock '") + name + "': " + std::strerror(err));
  }
  g_owner_fd = fd;  // held for process lifetime; kernel releases the lock on exit
  g_owner_pid = g_self.pid;
  t_ensure_gen = gen;  // only now: a throw above must leave the next call to retry

  // Clean shutdown unlinks the owner file (no stale marks). A crash skips atexit, so the file
  // survives with its lock auto-released -- exactly what a reclaimer probes as Dead. The
  // handler is registered once; after fork the child inherits it and cleans its own file.
  if (!g_atexit_registered) {
    g_atexit_registered = true;
    // Guarded by pid: a fork child inherits both the handler and g_self, but owns neither the
    // fd nor the file. Without the guard its exit unlinks the LIVE parent's owner file, after
    // which every reclaimer probes the parent as dead and steals its borrows.
    std::atexit([] {
      // exit() does not stop other threads and announce() writes this fd under g_mtx. try_lock,
      // so exit() from a thread holding it cannot deadlock; skipping only leaves the file behind.
      std::unique_lock<std::mutex> lk(g_mtx, std::try_to_lock);
      if (!lk.owns_lock()) return;
      if (g_owner_fd >= 0 && g_owner_pid == static_cast<std::uint32_t>(::getpid())) {
        ::close(g_owner_fd);  // release the OFD lock
        ::shm_unlink(owner_file_name(g_self).c_str());
        g_owner_fd = -1;
      }
    });
  }
}

namespace
{

std::string one_line(const std::string & raw)
{
  std::string out = raw;
  for (char & c : out) {
    if (c == '\t' || c == '\n' || c == '\r') c = ' ';
  }
  return out;
}

}  // namespace

void OwnerFile::announce(const ManifestEntry & entry) noexcept
{
  try {
    ensure();
  } catch (...) {
    return;  // no owner file means no manifest; the caller's channel is unaffected
  }
  std::string line = one_line(entry.signpost);
  line += '\t';
  line += entry.publisher ? "pub" : "sub";
  line += '\t';
  line += one_line(entry.key);
  line += '\t';
  line += one_line(entry.label);
  line += '\n';

  std::lock_guard<std::mutex> lk(g_mtx);
  if (g_owner_fd < 0) return;
  // Appended under the same mutex that guards the fd, so two threads opening channels at once
  // cannot interleave halves of a line. A short write leaves a partial tail, which the reader
  // drops for want of a newline.
  const off_t end = ::lseek(g_owner_fd, 0, SEEK_END);
  if (end < 0) return;
  [[maybe_unused]] ssize_t w = ::write(g_owner_fd, line.data(), line.size());
}

std::vector<ManifestEntry> OwnerFile::read_manifest(const std::string & name) noexcept
{
  std::vector<ManifestEntry> out;
  int fd = ::shm_open(name.c_str(), O_RDONLY, 0600);
  if (fd < 0) return out;
  std::string blob;
  char buf[4096];
  for (;;) {
    const ssize_t n = ::read(fd, buf, sizeof(buf));
    if (n <= 0) break;
    blob.append(buf, static_cast<std::size_t>(n));
    if (blob.size() > (1u << 20)) break;  // a manifest this large is corrupt, not informative
  }
  ::close(fd);

  std::size_t pos = 0;
  for (;;) {
    const std::size_t nl = blob.find('\n', pos);
    if (nl == std::string::npos) break;  // unterminated tail: the writer may have died mid-append
    const std::string line = blob.substr(pos, nl - pos);
    pos = nl + 1;
    std::string field[4];
    std::size_t f = 0;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= line.size() && f < 4; ++i) {
      if (i == line.size() || line[i] == '\t') {
        field[f++] = line.substr(start, i - start);
        start = i + 1;
      }
    }
    if (f < 2 || field[0].empty()) continue;  // signpost and role are the required pair
    ManifestEntry e;
    e.signpost = field[0];
    e.publisher = field[1] == "pub";
    e.key = field[2];
    e.label = field[3];
    out.push_back(std::move(e));
  }
  return out;
}

Liveness OwnerFile::probe(const OwnerId & id)
{
  const std::string name = owner_file_name(id);
  int fd = ::shm_open(name.c_str(), O_RDWR, 0600);
  if (fd < 0) {
    // Only a missing file proves no live participant. Any other failure says nothing about the
    // owner, and Dead is the one answer this must never guess: it hands a live borrower's slot
    // back to the publisher. EMFILE is the reachable case -- reclaim_dead() probes once per
    // holder per slot, so an fd-starved process fails here first.
    return errno == ENOENT ? Liveness::Dead : Liveness::Alive;
  }
  const bool acquired = try_write_lock(fd);  // success == lock was free == owner dead
  if (acquired) {
    // Dead owner: atexit unlinks only on a clean exit, so a crash leaves this file behind with
    // its lock auto-released. Reap it while we hold the lock, or /dev/shm accumulates one file
    // per crash. The name is unique per (pid, starttime) so a restart never reuses it, and a
    // later probe of the same dead id still reads a missing file as Dead.
    ::shm_unlink(name.c_str());
  }
  ::close(fd);  // drops the probe lock if it was acquired
  return acquired ? Liveness::Dead : Liveness::Alive;
}

}  // namespace flux
