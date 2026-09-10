#include "flux/discovery.hpp"

#include "flux/gpu_vmm.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace flux
{

std::string canonical_domain(const std::string & raw)
{
  if (raw.empty() || raw.size() > kMaxDomainLen) {
    throw std::invalid_argument(
      "flux: domain must be 1.." + std::to_string(kMaxDomainLen) + " characters, got '" + raw +
      "'");
  }
  for (char c : raw) {
    if (!std::isalnum(static_cast<unsigned char>(c))) {
      throw std::invalid_argument(
        "flux: domain must be alphanumeric (it names a /dev/shm entry), got '" + raw + "'");
    }
  }
  return raw;
}

std::string resolve_domain(const char * domain_env)
{
  const char * label = std::getenv("FLUX_DOMAIN");
  if (label != nullptr && *label != '\0') return canonical_domain(label);

  const char * domain = domain_env == nullptr ? nullptr : std::getenv(domain_env);
  if (domain == nullptr || *domain == '\0') return canonical_domain(kDefaultDomain);

  // Rendered from the parsed number, never from the raw text: "007" and "7" are one domain and
  // must not become two domains. Out of range is refused rather than wrapped -- wrapping lands in
  // someone else's domain, which is the failure this axis exists to prevent.
  char * end = nullptr;
  errno = 0;
  const unsigned long long value = std::strtoull(domain, &end, 10);
  const bool parsed = end != domain && *end == '\0' && errno == 0 &&
                      value <= std::numeric_limits<std::uint32_t>::max();
  if (!parsed || *domain == '-' || *domain == '+') {
    throw std::invalid_argument(
      std::string("flux: ") + domain_env + " must be a plain integer in [0, 2^32), got '" + domain +
      "' (set FLUX_DOMAIN to name the domain directly)");
  }
  return canonical_domain(std::to_string(value));
}

const std::string & process_domain()
{
  // Magic-static: the C++11 rule makes the first call the only one that resolves, and makes it
  // safe against a second thread arriving mid-initialization. A throwing initializer leaves the
  // static uninitialized, so a corrected environment resolves on the next call rather than
  // pinning the failure for the process.
  static const std::string domain = resolve_domain(kDefaultDomainEnv);
  return domain;
}

std::string flatten_key(const std::string & key)
{
  std::string out;
  out.reserve(key.size());
  for (char c : key) out += std::isalnum(static_cast<unsigned char>(c)) ? c : '.';
  return out;
}

std::string segment_name(
  const std::string & key, std::uint64_t fingerprint, const std::string & domain)
{
  // POSIX shm names allow one leading '/' and no other '/'. Map every non-alnum
  // key character (including '/') to '.', then append the full 64-bit fingerprint
  // (schema digest from flux_gen) so distinct schemas never collide on one segment.
  //
  // The layout version and the domain lead for the same reason the fingerprint trails: all three
  // are compatibility axes, and an object written under a different one is not ours to read.
  // Scoping the NAME by them means a version bump cannot leave an object that blocks the new
  // build -- there is nothing to recover from, and nothing of another version's to delete.
  // They lead because the NAME_MAX clamp below cuts the key, and an axis
  // that a long key could truncate away would let two domains share a name.
  //
  // canonical_domain runs here, not only at the resolve_domain call sites, so no caller can put an
  // unvalidated domain into a name.
  const std::string prefix =
    "/flux.v" + std::to_string(kLayoutVersion) + ".s" + canonical_domain(domain) + ".";
  char suffix[24];
  std::snprintf(suffix, sizeof(suffix), ".%016llx", static_cast<unsigned long long>(fingerprint));

  std::string body = flatten_key(key);

  constexpr std::size_t kNameMax = 255;  // NAME_MAX for /dev/shm entries
  const std::size_t fixed = prefix.size() + std::strlen(suffix);
  if (fixed + body.size() > kNameMax) body.resize(kNameMax - fixed);
  return prefix + body + suffix;
}

std::string signpost_name(
  const std::string & key, std::uint64_t fingerprint, const std::string & domain)
{
  // The fixed name names the signpost, not a segment.
  return segment_name(key, fingerprint, domain);
}

std::string unique_segment_name(const std::string & signpost, const OwnerId & creator)
{
  char suffix[40];
  int n = std::snprintf(
    suffix, sizeof(suffix), ".%u.%llu", creator.pid,
    static_cast<unsigned long long>(creator.starttime));
  std::string name = signpost;
  constexpr std::size_t kNameMax = 255;  // preserve the owner-id suffix; truncate the signpost part
  if (n > 0 && name.size() + static_cast<std::size_t>(n) > kNameMax) {
    name.resize(kNameMax - static_cast<std::size_t>(n));
  }
  name += suffix;
  return name;
}

bool stat_segment(const std::string & name, SegmentId & out) noexcept
{
  int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
  if (fd < 0) return false;  // ENOENT: the name is currently unbound
  struct stat st
  {
  };
  const bool ok = ::fstat(fd, &st) == 0;
  ::close(fd);
  if (!ok) return false;
  out.dev = static_cast<std::uint64_t>(st.st_dev);
  out.ino = static_cast<std::uint64_t>(st.st_ino);
  return true;
}

namespace
{

// A signpost name is `/flux.v<layout>.s<domain>.<body>.<fp16>`. Split it back apart, or report
// false when the name is not one (a segment name, which appends `.<pid>.<starttime>`, is not).
// This is the only parser of the naming ABI besides segment_name() that builds it, and it lives
// beside it for that reason: two parsers of one format is how the format silently forks.
bool parse_signpost(
  const char * entry, std::string & domain, std::string & body,
  std::uint64_t & fingerprint) noexcept
{
  const std::string n(entry);
  char prefix[24];
  std::snprintf(prefix, sizeof(prefix), "flux.v%u.s", kLayoutVersion);
  const std::size_t plen = std::strlen(prefix);
  if (n.size() <= plen || n.compare(0, plen, prefix) != 0) return false;

  const std::size_t domain_end = n.find('.', plen);
  if (domain_end == std::string::npos) return false;
  domain = n.substr(plen, domain_end - plen);
  if (domain.empty()) return false;

  // The fingerprint is the last dot group and is exactly 16 hex digits. A segment name ends in
  // `.<pid>.<starttime>` instead, so this is also what tells the two kinds apart.
  const std::size_t fp_dot = n.rfind('.');
  if (fp_dot == std::string::npos || fp_dot <= domain_end) return false;
  const std::string hex = n.substr(fp_dot + 1);
  if (hex.size() != 16) return false;
  for (char c : hex) {
    if (std::isxdigit(static_cast<unsigned char>(c)) == 0) return false;
  }
  fingerprint = std::strtoull(hex.c_str(), nullptr, 16);
  body = n.substr(domain_end + 1, fp_dot - domain_end - 1);
  return true;
}

// Alive means the owner file's OFD write lock is still held by somebody. Unlike OwnerFile::probe
// this never unlinks: enumeration reports the world, it does not edit it.
bool owner_is_alive(const std::string & name) noexcept
{
  int fd = ::shm_open(name.c_str(), O_RDWR, 0600);
  if (fd < 0) return false;
  const bool free_to_take = Segment::lock_write(fd);
  ::close(fd);
  return !free_to_take;
}

}  // namespace

std::vector<TopicView> enumerate_topics() noexcept
{
  std::vector<TopicView> topics;
  std::vector<std::string> owner_names;

  DIR * dir = ::opendir("/dev/shm");
  if (dir == nullptr) return topics;
  struct dirent * ent;
  while ((ent = ::readdir(dir)) != nullptr) {
    const char * n = ent->d_name;
    if (std::strncmp(n, "flux.", 5) != 0) continue;
    if (std::strcmp(n, "flux.sweep") == 0) continue;
    if (std::strncmp(n, "flux.owner.", 11) == 0) {
      owner_names.push_back(std::string("/") + n);
      continue;
    }
    TopicView t;
    if (!parse_signpost(n, t.domain, t.key, t.fingerprint)) continue;  // a segment, not a signpost
    t.signpost = std::string("/") + n;
    topics.push_back(std::move(t));
  }
  ::closedir(dir);

  for (const std::string & owner : owner_names) {
    if (!owner_is_alive(owner)) continue;  // a dead participant's rows describe nothing
    OwnerId id{};
    // `/flux.owner.<pid>.<starttime>`: recover the identity the reader reports, from the one
    // place it is written down.
    if (
      std::sscanf(
        owner.c_str(), "/flux.owner.%u.%llu", &id.pid,
        reinterpret_cast<unsigned long long *>(&id.starttime)) != 2) {
      continue;
    }
    for (const ManifestEntry & e : OwnerFile::read_manifest(owner)) {
      for (TopicView & t : topics) {
        if (t.signpost != e.signpost) continue;
        if (!e.key.empty()) {
          t.key = e.key;  // a live participant knows the key the name could only approximate
          t.key_exact = true;
        }
        EndpointView ep;
        ep.owner = id;
        ep.label = e.label;
        ep.publisher = e.publisher;
        t.endpoints.push_back(std::move(ep));
        break;
      }
    }
  }
  return topics;
}

void sweep_dead() noexcept
{
  // Cleanup lock: only one sweeper runs at a time. If another holds it,
  // skip -- this is best-effort reclamation, not a barrier.
  int lock_fd = ::shm_open("/flux.sweep", O_CREAT | O_RDWR, 0600);
  if (lock_fd < 0) return;
  if (!Segment::lock_write(lock_fd)) {
    ::close(lock_fd);
    return;
  }
  DIR * dir = ::opendir("/dev/shm");
  if (dir != nullptr) {
    struct dirent * ent;
    while ((ent = ::readdir(dir)) != nullptr) {
      const char * n = ent->d_name;
      if (std::strncmp(n, "flux.", 5) != 0) continue;   // not ours
      if (std::strcmp(n, "flux.sweep") == 0) continue;  // the cleanup lock itself
      // Owner files carry a manifest now, so their bytes at offset 4 are whatever the last
      // announcement put there -- possibly kSignpostMagic. The name says which kind this is and
      // the name is the ABI, so ask it rather than sniffing content.
      const bool is_owner = std::strncmp(n, "flux.owner.", 11) == 0;
      const std::string name = std::string("/") + n;
      int fd = ::shm_open(name.c_str(), O_RDWR, 0600);
      if (fd < 0) continue;  // vanished under us
      // The OFD write lock is exclusive: success proves no live publisher (segment read lock) and
      // no live participant (owner-file write lock) holds it, and it blocks any from taking one
      // until we unlink. Failure => alive/held => skip.
      if (!Segment::lock_write(fd)) {
        ::close(fd);
        continue;
      }
      // Preserve the signpost (persistent rendezvous); unlink segment and owner-file orphans.
      // magic sits at offset 4 in both Signpost and ControlHeader.
      if (is_owner) {
        ::shm_unlink(name.c_str());
      } else {
        std::uint32_t magic = 0;
        if (::pread(fd, &magic, sizeof(magic), 4) != static_cast<ssize_t>(sizeof(magic))) magic = 0;
        if (magic != kSignpostMagic) ::shm_unlink(name.c_str());
      }
      ::close(fd);  // releases the write lock
    }
    ::closedir(dir);
  }
  ::close(lock_fd);  // releases the cleanup lock
}

namespace
{
std::once_flag g_sweep_once;
}

// A publisher sweeps on every create, so a process that publishes anything reclaims orphans.
// One that only ever subscribes had no sweep at all, so "any participant reclaims" did not
// hold for it -- and it is the case that matters, since a crashed
// publisher group leaves a segment its subscribers keep being refused. Once per process, at the
// consumer-facing entry point rather than inside open_subscriber_segment: sweeping there would
// unlink the dead segment before the attach guard that refuses it ever runs.
void sweep_dead_once() noexcept
{
  std::call_once(g_sweep_once, [] { sweep_dead(); });
}

#ifdef FLUX_TESTING
namespace detail
{
// Test seams: invoked right after the corresponding shm_open, before any lock, so a test can
// force the unlink-vs-join race deterministically (W5/R2). Compiled out of
// release builds (FLUX_TESTING is set only under BUILD_TESTING); never assigned in production.
void (*g_bootstrap_after_open)(const std::string & name, int fd) = nullptr;
void (*g_signpost_after_open)(const std::string & name, int fd) = nullptr;
}  // namespace detail
#endif

namespace
{

[[noreturn]] void fail(const char * what, const std::string & name, int err)
{
  throw std::runtime_error(std::string("flux: ") + what + " '" + name + "': " + std::strerror(err));
}

SegmentId id_of(int fd) noexcept
{
  struct stat st
  {
  };
  if (::fstat(fd, &st) != 0) return SegmentId{};
  SegmentId id;
  id.dev = static_cast<std::uint64_t>(st.st_dev);
  id.ino = static_cast<std::uint64_t>(st.st_ino);
  return id;
}

// Does `name` still resolve to the object `fd` is open on? A last publisher leaving between our
// shm_open and our lock unlinks the name (segment.cpp reset), leaving our fd on a nameless orphan;
// adopting it would strand new subscribers (name absent) or split-brain with whoever recreates the
// name. Called only while holding the segment lock, which pins the binding -- unlink needs the
// write lock our lock blocks -- so a single post-lock check closes the window.
bool name_still_bound(const std::string & name, int fd) noexcept
{
  SegmentId now;
  return stat_segment(name, now) && now == id_of(fd);
}

// Spin budgets, in 100 us units: the bootstrap must outwait a peer's initialization, an
// attaching subscriber must not block its caller.
constexpr int kBootstrapSpins = 5000;  // ~0.5 s
constexpr int kAttachSpins = 20;       // ~2 ms

struct MappedSeg
{
  std::byte * base;
  std::size_t bytes;
  SegmentLayout layout;
  SegmentId id;
};

// True when this participant should put its payload in a device allocation rather than in the
// shm object. Only the DeviceHandle route splits them: on ShmDirect the mapping already is the
// GPU buffer and nothing changes.
bool wants_device_payload(Device device)
{
  return device == Device::Cuda && gpu::probe().route == gpu::Route::DeviceHandle;
}

// Can a participant asking for `device` read the payload this header describes? Judged before the
// mapping is adopted, and every no is permanent -- retrying never turns a host payload into a
// device one. `reject` unmaps and throws SegmentMismatch.
template <class Reject>
void check_payload_compat(const ControlHeader * ctrl, Device device, const Reject & reject)
{
  const bool device_backed =
    ctrl->storage_kind != static_cast<std::uint8_t>(StorageKind::HostInline);
  if (!device_backed) {
    // The publisher put its payload in host memory. A discrete GPU cannot read that, so a caller
    // that asked for one would get a device pointer into memory no kernel can touch.
    if (wants_device_payload(device)) reject("payload is host memory, which this GPU cannot read");
    return;
  }
  // A device-backed channel is GPU-only. A host reader is refused here rather than handed
  // a device pointer it would fault on.
  if (device != Device::Cuda)
    reject("payload lives in GPU memory and cannot be read from the host");
  const gpu::Platform p = gpu::probe(static_cast<int>(ctrl->device_id));
  if (p.route != gpu::Route::DeviceHandle) {
    reject("payload is a device allocation this host cannot import");
  }
}

// Take a share of the device region a device-backed segment's header points at. Runs after the
// mapping is adopted, because the endpoint is derived from the segment object's identity.
// A publisher that has not opened its endpoint yet fails transiently, like a segment not yet
// ready -- the caller retries.
void import_device_payload(Segment & s)
{
  const ControlHeader * ctrl = s.ctrl();
  if (ctrl->storage_kind == static_cast<std::uint8_t>(StorageKind::HostInline)) return;
  gpu::DeviceImport imp = gpu::DeviceImport::open(
    gpu::device_endpoint(s.id()), s.layout().payload_bytes(), static_cast<int>(ctrl->device_id));
  s.attach_device(imp.keepalive(), imp.base());
}

// Wait for the creator to size + mark the segment ready, map it, then validate. Does not own
// the fd. Throws on timeout/mismatch (unmapping first if already mapped).
MappedSeg map_wait_validate(
  int fd, const std::string & name, std::uint64_t fingerprint, const SegmentLayout * expect,
  int kSpins, Device device = Device::Cpu)
{
  const struct timespec nap = {0, 100000};  // 100 us

  struct stat st
  {
  };
  bool sized = false;
  for (int i = 0; i < kSpins; ++i) {
    if (::fstat(fd, &st) != 0) fail("fstat", name, errno);
    if (static_cast<std::size_t>(st.st_size) >= sizeof(ControlHeader)) {
      sized = true;
      break;
    }
    nanosleep(&nap, nullptr);
  }
  if (!sized) throw std::runtime_error("flux: segment never sized '" + name + "'");

  const std::size_t bytes = static_cast<std::size_t>(st.st_size);
  void * p = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (p == MAP_FAILED) fail("mmap", name, errno);

  auto * base = static_cast<std::byte *>(p);
  auto * ctrl = reinterpret_cast<ControlHeader *>(base);
  auto reject = [&](const char * why) -> void {
    munmap(p, bytes);
    throw SegmentMismatch(std::string("flux: ") + why + " '" + name + "'");
  };

  bool ready = false;
  for (int i = 0; i < kSpins; ++i) {
    if (ctrl->init_state.load(std::memory_order_acquire) == kInitReady) {
      ready = true;
      break;
    }
    nanosleep(&nap, nullptr);
  }
  if (!ready) {  // transient: the creator may still be initializing, so this is retryable
    munmap(p, bytes);
    throw std::runtime_error("flux: segment not ready yet '" + name + "'");
  }

  if (ctrl->magic != kMagic) reject("bad magic");
  if (ctrl->version != kLayoutVersion) reject("version mismatch");
  if (ctrl->fingerprint != fingerprint) reject("fingerprint mismatch");

  // Bound the config BEFORE deriving offsets from it. These two came from another process, and
  // every offset below is computed from them; out of range they make total_bytes() wrap, and the
  // mapping-size check that follows would then be comparing against a wrapped number.
  if (!layout_config_ok(ctrl->slot_size, ctrl->slot_count))
    reject("slot_size/slot_count out of range");
  SegmentLayout layout{ctrl->slot_size, ctrl->slot_count};
  // A device-backed segment keeps only the control plane in shm, so what has to fit is
  // the extent this storage_kind actually uses -- checking total_bytes there would reject every
  // dGPU segment as short.
  const bool device_backed =
    ctrl->storage_kind != static_cast<std::uint8_t>(StorageKind::HostInline);
  if ((device_backed ? layout.control_bytes() : layout.total_bytes()) > bytes) {
    reject("layout exceeds mapping");
  }
  if (
    expect != nullptr &&
    (expect->slot_size != ctrl->slot_size || expect->slot_count != ctrl->slot_count)) {
    reject("publisher config mismatch on shared segment");
  }
  check_payload_compat(ctrl, device, reject);
  return {base, bytes, layout, id_of(fd)};
}

// ---- signpost mechanics ----

constexpr std::size_t kSignpostBytes = 4096;  // one page; a Signpost is one cache line

void sp_unlock(int fd) noexcept
{
  struct flock fl;
  std::memset(&fl, 0, sizeof(fl));
  fl.l_type = F_UNLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len = 1;
  ::fcntl(fd, F_OFD_SETLK, &fl);
}

struct MappedSp
{
  Signpost * sp = nullptr;
  std::byte * base = nullptr;
  int fd = -1;

  MappedSp(Signpost * s, std::byte * b, int f) noexcept : sp(s), base(b), fd(f) {}
  MappedSp(MappedSp && o) noexcept : sp(o.sp), base(o.base), fd(o.fd)
  {
    o.sp = nullptr;
    o.base = nullptr;
    o.fd = -1;
  }
  MappedSp(const MappedSp &) = delete;
  MappedSp & operator=(const MappedSp &) = delete;
  MappedSp & operator=(MappedSp &&) = delete;
  // Closing the fd also drops any OFD lock still held on it, so an exception thrown while the
  // signpost write lock is held (e.g. OwnerFile::ensure) cannot block the topic's bootstrap
  // forever.
  ~MappedSp()
  {
    if (base != nullptr) {
      munmap(base, kSignpostBytes);
      ::close(fd);
    }
  }
};

// A signpost created but not yet ftruncate'd is zero bytes, and touching a mapping past the end
// of one raises SIGBUS, which no reader here can catch. signpost_map is the path that sets it.
bool signpost_object_sized(int fd) noexcept
{
  struct stat st
  {
  };
  return ::fstat(fd, &st) == 0 && static_cast<std::size_t>(st.st_size) >= kSignpostBytes;
}

// Map the fixed-name signpost, creating and bootstrapping it if absent. On return init_state is
// kInitReady. Holds no lock; the caller owns the fd + mapping.
MappedSp signpost_map(const std::string & name)
{
  const struct timespec nap = {0, 100000};  // 100 us
  for (int attempt = 0; attempt < 64; ++attempt) {
    int fd = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
    if (fd < 0) fail("signpost shm_open", name, errno);
#ifdef FLUX_TESTING
    if (detail::g_signpost_after_open) detail::g_signpost_after_open(name, fd);
#endif
    struct stat st
    {
    };
    if (::fstat(fd, &st) != 0) {
      int e = errno;
      ::close(fd);
      fail("signpost fstat", name, e);
    }
    if (static_cast<std::size_t>(st.st_size) < kSignpostBytes) {
      if (::ftruncate(fd, static_cast<off_t>(kSignpostBytes)) != 0) {
        int e = errno;
        ::close(fd);
        fail("signpost ftruncate", name, e);
      }
    }
    void * p = mmap(nullptr, kSignpostBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
      int e = errno;
      ::close(fd);
      fail("signpost mmap", name, e);
    }
    auto * sp = reinterpret_cast<Signpost *>(p);
    if (sp->init_state.load(std::memory_order_acquire) != kInitReady) {
      // Bootstrap: the write-lock winner writes magic/version and publishes ready. The lock is
      // short-lived (not a lifetime lock); it also arbitrates segment creation below.
      if (Segment::lock_write(fd)) {
        // R2, as for segments: until this lock the object was unlocked with magic still 0, which
        // is exactly what sweep_dead() unlinks. Bootstrapping an unbound fd would advertise this
        // topic on a nameless orphan no subscriber can resolve. The lock pins the binding, so one
        // post-lock check closes the window; a lost name means start over on a fresh object.
        if (!name_still_bound(name, fd)) {
          sp_unlock(fd);
          munmap(p, kSignpostBytes);
          ::close(fd);
          continue;
        }
        if (sp->init_state.load(std::memory_order_acquire) != kInitReady) {
          sp->magic = kSignpostMagic;
          sp->version = kLayoutVersion;
          sp->epoch = 0;
          sp->cur_pid = 0;
          sp->cur_starttime = 0;
          sp->seq.store(0, std::memory_order_relaxed);
          sp->init_state.store(kInitReady, std::memory_order_release);
        }
        sp_unlock(fd);
      } else {
        for (int i = 0; i < kBootstrapSpins; ++i) {
          if (sp->init_state.load(std::memory_order_acquire) == kInitReady) break;
          nanosleep(&nap, nullptr);
        }
      }
    }
    if (sp->init_state.load(std::memory_order_acquire) != kInitReady) {
      munmap(p, kSignpostBytes);
      ::close(fd);
      continue;
    }
    if (sp->magic != kSignpostMagic || sp->version != kLayoutVersion) {
      munmap(p, kSignpostBytes);
      ::close(fd);
      // The name carries the layout version, so another flux build cannot land here: this object
      // is corrupt or a foreign squatter. Say what to remove -- nothing else will clear it.
      throw SegmentMismatch(
        "flux: signpost '" + name + "' is not a flux v" + std::to_string(kLayoutVersion) +
        " signpost (corrupt or a name collision); remove /dev/shm" + name + " to recover");
    }
    return {sp, static_cast<std::byte *>(p), fd};
  }
  throw std::runtime_error("flux: signpost bootstrap failed '" + name + "'");
}

void signpost_unmap(MappedSp & m) noexcept
{
  if (m.base != nullptr) {
    munmap(m.base, kSignpostBytes);
    ::close(m.fd);
    m.base = nullptr;
    m.sp = nullptr;
    m.fd = -1;
  }
}

// Seqlock read of the advertised (owner-id, epoch). False if no stable read within the budget.
bool signpost_read(const Signpost * sp, OwnerId & id, std::uint32_t & epoch) noexcept
{
  for (int i = 0; i < 1000; ++i) {
    std::uint32_t s1 = sp->seq.load(std::memory_order_acquire);
    if (s1 & 1u) continue;  // a writer is mid-update
    std::uint32_t pid = sp->cur_pid;
    std::uint64_t starttime = sp->cur_starttime;
    std::uint32_t ep = sp->epoch;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (sp->seq.load(std::memory_order_relaxed) != s1) continue;  // changed under us
    id.pid = pid;
    id.starttime = starttime;
    epoch = ep;
    return true;
  }
  return false;
}

// Seqlock write of (owner-id, epoch). The caller holds the signpost write lock (single writer).
void signpost_write(Signpost * sp, const OwnerId & id, std::uint32_t epoch) noexcept
{
  // seq | 1, not seq + 1: a writer that crashed mid-update leaves seq odd, and + 1 would flip
  // THIS update's in-progress window to even -- a concurrent reader would validate a torn pair
  // as stable. Forcing the odd bit keeps every
  // unsealed window odd, and the seal below is strictly even again.
  const std::uint32_t odd = sp->seq.load(std::memory_order_relaxed) | 1u;
  sp->seq.store(odd, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  sp->cur_pid = id.pid;
  sp->cur_starttime = id.starttime;
  sp->epoch = epoch;
  // No fence here. The seal is a release STORE, which already orders the three writes above
  // before it. The fence above IS load-bearing -- it pins the odd marker before the fields, and
  // without it a reader can validate a torn pair.
  sp->seq.store(odd + 1, std::memory_order_release);  // even: sealed
}

}  // namespace

Segment open_publisher_segment(
  const std::string & name, std::uint32_t slot_size, std::uint32_t slot_count,
  std::uint64_t fingerprint, Device device)
{
  // invalid_argument, as the heap path throws for the same rejection (segment.cpp): these are the
  // caller's own numbers. A config another process wrote is a different thing and takes reject().
  if (!layout_config_ok(slot_size, slot_count)) {
    throw std::invalid_argument(
      "flux: slot_size must be >= 1 and slot_count in 1.." + std::to_string(kMaxSlotCount) +
      " (the slot field in ControlHeader.latest is 16-bit)");
  }
  SegmentLayout layout{slot_size, slot_count};
  // A device-backed segment holds only the control plane; its payload is a separate allocation.
  // The layout is identical either way -- only how far the shm object runs differs.
  const bool dgpu = wants_device_payload(device);
  const std::size_t bytes = dgpu ? layout.control_bytes() : layout.total_bytes();
  const int dgpu_device = dgpu ? gpu::probe().device : 0;

  sweep_dead();  // reclaim crashed orphans on participant start

  // `name` is the signpost name (the fixed rendezvous name); segments live under unique names the
  // signpost advertises. Read the signpost and join the live current, or
  // take the signpost write lock, recheck (R1), and create a fresh unique segment. A publisher
  // holds an OFD read lock on its segment for its lifetime (liveness).
  MappedSp msp = signpost_map(name);
  const struct timespec nap = {0, 100000};  // 100 us

  // Try to join the segment the signpost advertises. Returns an invalid Segment to fall through
  // to the create path; rethrows SegmentMismatch when a live publisher genuinely disagrees about
  // the config.
  auto try_join = [&](const OwnerId & cur) -> Segment {
    if (cur.pid == 0) return Segment{};
    const std::string seg = unique_segment_name(name, cur);
    int fd = ::shm_open(seg.c_str(), O_RDWR, 0600);
    if (fd < 0) return Segment{};  // segment gone -> create
    if (!Segment::lock_read(fd) || !name_still_bound(seg, fd)) {
      ::close(fd);
      return Segment{};
    }
    try {
      MappedSeg m = map_wait_validate(fd, seg, fingerprint, &layout, kBootstrapSpins, device);
      Segment s =
        Segment::adopt_shm(m.base, m.bytes, m.layout, fd, seg, m.id, /*unlink_on_last_out=*/true);
      fd = -1;  // s owns it now; on a throw below ~Segment closes it, so the catch must not
      // A joining publisher writes into the creator's ring, so it imports the creator's region
      // rather than making one of its own -- there is one payload region per segment.
      import_device_payload(s);
      return s;
    } catch (const SegmentMismatch &) {
      // A config change restarts the publisher, and the signpost can still advertise the crashed
      // group's segment: sweep_dead() skips entirely when another sweeper holds /flux.sweep, so
      // the corpse outlives our sweep. Upgrading our own read lock to write asks the same
      // question sweep does -- it succeeds only if no other OFD holds one, and a live publisher
      // holds a read lock for its lifetime. Success means nobody is on the far side of this
      // mismatch, so create a new segment instead of failing. Failure means a live publisher
      // really is running the other config, which is not a race and must still throw.
      const bool corpse = Segment::lock_write(fd);
      ::close(fd);
      if (corpse) return Segment{};
      signpost_unmap(msp);
      throw;
    } catch (...) {
      if (fd >= 0) ::close(fd);  // transient (not ready yet) -> retry/create
      return Segment{};
    }
  };

  constexpr int kBootstrapAttempts = 8;
  for (int attempt = 0; attempt < kBootstrapAttempts; ++attempt) {
    OwnerId cur{};
    std::uint32_t epoch = 0;
    // A read that fails its whole budget means a writer crashed mid-update and left the seqlock
    // odd -- no stable read will ever come. Proceed with no current: the locked recheck below
    // reads the fields directly, and our sealed signpost_write repairs the parity.
    signpost_read(msp.sp, cur, epoch);
    if (Segment joined = try_join(cur); joined.valid()) {
      signpost_unmap(msp);
      return joined;
    }

    // No live current (or the join lost a race): take the signpost write lock to create.
    if (!Segment::lock_write(msp.fd)) {
      nanosleep(&nap, nullptr);
      continue;  // another publisher is creating -> retry the read/join
    }
    // R1: recheck under the lock -- a peer may have created a live segment while we waited.
    // Under the write lock no live writer exists, but a crashed one may have left the seqlock
    // odd and the pair half-written, so read the words directly rather than through the
    // seqlock: each word is individually intact, a half-advanced epoch only raises the base
    // for the +1 below (monotone), and our sealed signpost_write repairs the parity.
    OwnerId cur2{};
    cur2.pid = msp.sp->cur_pid;
    cur2.starttime = msp.sp->cur_starttime;
    const std::uint32_t cur_epoch = msp.sp->epoch;
    if (cur2.pid != 0) {
      if (Segment joined = try_join(cur2); joined.valid()) {
        sp_unlock(msp.fd);
        signpost_unmap(msp);
        return joined;
      }
    }

    // Create a fresh unique segment. Take our owner file first (liveness + our owner-id).
    OwnerFile::ensure();
    const OwnerId self = OwnerFile::self();
    const std::string seg = unique_segment_name(name, self);
    int fd = ::shm_open(seg.c_str(), O_CREAT | O_RDWR, 0600);
    if (fd < 0) {
      sp_unlock(msp.fd);
      nanosleep(&nap, nullptr);
      continue;
    }
#ifdef FLUX_TESTING
    if (detail::g_bootstrap_after_open) detail::g_bootstrap_after_open(seg, fd);
#endif
    // R2: take the segment write lock first, then confirm the name still resolves to this fd (a
    // sweeper may have unlinked it in the O_CREAT..lock window). If not, retry with a fresh name.
    if (!Segment::lock_write(fd) || !name_still_bound(seg, fd)) {
      ::close(fd);
      sp_unlock(msp.fd);
      continue;
    }
    if (::ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
      int e = errno;
      ::close(fd);
      sp_unlock(msp.fd);
      fail("ftruncate", seg, e);
    }
    void * p = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
      int e = errno;
      ::close(fd);
      sp_unlock(msp.fd);
      fail("mmap create", seg, e);
    }
    auto * base = static_cast<std::byte *>(p);
    Segment::init_header(
      base, layout, slot_size, slot_count, fingerprint,
      dgpu ? StorageKind::CudaIpc : StorageKind::HostInline,
      static_cast<std::uint32_t>(dgpu_device));
    Segment s = Segment::adopt_shm(
      base, bytes, layout, fd, seg, id_of(fd),
      /*unlink_on_last_out=*/true);
    // Allocate and start serving BEFORE the signpost advertises this segment. Advertising first
    // would publish a segment whose endpoint nobody answers, and every subscriber that raced in
    // would fail an import that was never going to work.
    if (dgpu) {
      try {
        gpu::DeviceAlloc alloc = gpu::DeviceAlloc::create(
          gpu::device_endpoint(s.id()), layout.payload_bytes(), dgpu_device);
        s.attach_device(alloc.keepalive(), alloc.base());
      } catch (...) {
        sp_unlock(msp.fd);
        signpost_unmap(msp);
        throw;  // s unlinks the segment on the way out: nothing was ever advertised
      }
    }
    // Advertise the new segment and rotate the epoch, then downgrade to a read lock (liveness).
    signpost_write(msp.sp, self, cur_epoch + 1);
    Segment::lock_read(fd);  // downgrade so other publishers can join
    sp_unlock(msp.fd);
    signpost_unmap(msp);
    return s;
  }
  signpost_unmap(msp);
  throw std::runtime_error("flux: segment bootstrap failed '" + name + "'");
}

std::uint32_t signpost_epoch(const std::string & signpost) noexcept
{
  int fd = ::shm_open(signpost.c_str(), O_RDONLY, 0600);
  if (fd < 0) return 0;  // absent
  if (!signpost_object_sized(fd)) {
    ::close(fd);
    return 0;  // created but not yet sized: same answer as absent, and mapping it is SIGBUS
  }
  void * p = mmap(nullptr, kSignpostBytes, PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (p == MAP_FAILED) return 0;
  const auto * sp = reinterpret_cast<const Signpost *>(p);
  std::uint32_t epoch = 0;
  if (sp->init_state.load(std::memory_order_acquire) == kInitReady && sp->magic == kSignpostMagic) {
    OwnerId cur{};
    if (signpost_read(sp, cur, epoch) && cur.pid == 0) epoch = 0;  // no current segment
  }
  munmap(p, kSignpostBytes);
  return epoch;
}

SignpostView::SignpostView(const std::string & name) noexcept
{
  int fd = ::shm_open(name.c_str(), O_RDONLY, 0600);
  if (fd < 0) return;
  if (!signpost_object_sized(fd)) {
    ::close(fd);
    return;  // created but not yet sized: stay invalid rather than fault on the first read
  }
  void * p = mmap(nullptr, kSignpostBytes, PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);  // the mapping keeps the object alive; no lock is held
  if (p == MAP_FAILED) return;
  const auto * sp = reinterpret_cast<const Signpost *>(p);
  if (sp->init_state.load(std::memory_order_acquire) != kInitReady || sp->magic != kSignpostMagic) {
    munmap(p, kSignpostBytes);  // not bootstrapped yet: stay invalid, the caller falls back
    return;
  }
  base_ = p;
  sp_ = sp;
}

SignpostView::~SignpostView()
{
  if (base_ != nullptr) munmap(base_, kSignpostBytes);
}

SignpostView::SignpostView(SignpostView && o) noexcept : sp_(o.sp_), base_(o.base_)
{
  o.sp_ = nullptr;
  o.base_ = nullptr;
}

SignpostView & SignpostView::operator=(SignpostView && o) noexcept
{
  if (this != &o) {
    if (base_ != nullptr) munmap(base_, kSignpostBytes);
    sp_ = o.sp_;
    base_ = o.base_;
    o.sp_ = nullptr;
    o.base_ = nullptr;
  }
  return *this;
}

bool SignpostView::epoch(std::uint32_t & out) const noexcept
{
  if (sp_ == nullptr) return false;
  OwnerId cur{};
  std::uint32_t ep = 0;
  if (!signpost_read(sp_, cur, ep)) return false;
  out = (cur.pid == 0) ? 0u : ep;  // same rule as signpost_epoch(): 0 = no current segment
  return true;
}

bool segment_publishers_dead(const std::string & segment) noexcept
{
  int fd = ::shm_open(segment.c_str(), O_RDWR, 0600);
  if (fd < 0) return errno == ENOENT;
  const bool dead = Segment::lock_write(fd);  // publishers hold read locks: success = none alive
  ::close(fd);                                // releases the probe lock
  return dead;
}

ChannelStats read_channel_stats(const std::string & signpost) noexcept
{
  ChannelStats out;
  OwnerId cur{};
  std::uint32_t epoch = 0;
  try {
    MappedSp msp = signpost_map(signpost);
    const bool ok = signpost_read(msp.sp, cur, epoch);
    signpost_unmap(msp);
    if (!ok || cur.pid == 0) return out;
  } catch (...) {
    return out;  // no signpost, or nothing bootstrapped in it yet
  }
  out.epoch = epoch;

  const std::string seg = unique_segment_name(signpost, cur);
  int fd = ::shm_open(seg.c_str(), O_RDONLY, 0600);
  if (fd < 0) return out;  // advertised but already unlinked: the group left under us
  void * p = ::mmap(nullptr, sizeof(ControlHeader), PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (p == MAP_FAILED) return out;

  const auto * ctrl = static_cast<const ControlHeader *>(p);
  if (
    ctrl->init_state.load(std::memory_order_acquire) == kInitReady && ctrl->magic == kMagic &&
    ctrl->version == kLayoutVersion) {
    out.live = true;
    out.slot_size = ctrl->slot_size;
    out.slot_count = ctrl->slot_count;
    out.storage_kind = ctrl->storage_kind;
    out.fingerprint = ctrl->fingerprint;
    out.publish_seq = ctrl->publish_seq.load(std::memory_order_relaxed);
    out.waiters = ctrl->waiters.load(std::memory_order_relaxed);
  }
  ::munmap(p, sizeof(ControlHeader));
  return out;
}

Segment open_subscriber_segment(
  const std::string & name, std::uint64_t fingerprint, std::uint32_t * out_epoch, Device device)
{
  // `name` is the signpost name. Read the advertised current segment, then attach to it.
  MappedSp msp = signpost_map(name);
  OwnerId cur{};
  std::uint32_t epoch = 0;
  const bool ok = signpost_read(msp.sp, cur, epoch);
  signpost_unmap(msp);
  if (!ok || cur.pid == 0) {
    // Transient in the normal case (the publisher is not up yet). It is also what a skew on either
    // name-scoping axis looks like -- a peer built against another flux layout, or one running in
    // another domain, publishes under a different name and the two never meet -- so name both
    // possibilities here. A reader with no way to tell these apart re-reads correct code looking
    // for the bug.
    throw std::runtime_error(
      "flux: signpost has no current segment '" + name +
      "' (no publisher yet, or its publisher is on another flux layout than v" +
      std::to_string(kLayoutVersion) +
      ", or in another domain -- the name above carries this side's, and names are scoped by "
      "both)");
  }
  if (out_epoch != nullptr) *out_epoch = epoch;
  const std::string seg = unique_segment_name(name, cur);
  int fd = ::shm_open(seg.c_str(), O_RDWR, 0600);  // RW: subscriber writes refcount/holders
  if (fd < 0) fail("shm_open", seg, errno);
  try {
    // Short budget: a subscriber retries on its next take, and blocking its caller for a
    // second per call is far worse than reporting "not up yet" now.
    MappedSeg m = map_wait_validate(fd, seg, fingerprint, nullptr, kAttachSpins, device);
    // Do not adopt a stream with no live publisher: no frame will ever come and the mapping
    // would pin a dead segment. Transient, like "not up yet" -- a new
    // publisher rotates the signpost and the next attach follows it. The catch below releases
    // the probe lock by closing the fd.
    if (Segment::lock_write(fd)) {
      munmap(m.base, m.bytes);
      throw std::runtime_error("flux: segment has no live publisher '" + seg + "'");
    }
    ::close(fd);  // subscribers hold no liveness lock
    fd = -1;      // import_device_payload below can throw, and the catch must not close it twice
    Segment s = Segment::adopt_shm(
      m.base, m.bytes, m.layout, /*fd=*/-1, seg, m.id, /*unlink_on_last_out=*/false);
    // A subscriber never writes payload. Dropping it to PROT_READ turns an aliasing bug that
    // would surface as another consumer's torn read into a SIGSEGV at the faulty store.
    s.protect_payload();
    import_device_payload(s);
    return s;
  } catch (...) {
    if (fd >= 0) ::close(fd);
    throw;
  }
}

}  // namespace flux
