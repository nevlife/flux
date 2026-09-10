#ifndef _GNU_SOURCE
#define _GNU_SOURCE  // F_OFD_SETLK
#endif

#include "flux/segment.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

// Mechanics only: mapping, OFD locks, writing a fresh header. Which name to use, whether to
// create or join, whether to reinitialize or adopt, and when to unlink are rendezvous policy and
// live in the discovery layer (discovery.cpp).

namespace flux
{

namespace
{
// Non-blocking OFD lock on byte [0,1) of the segment fd. Returns false on conflict.
bool ofd_setlk(int fd, short type) noexcept
{
  struct flock fl;
  std::memset(&fl, 0, sizeof(fl));
  fl.l_type = type;  // F_RDLCK / F_WRLCK
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len = 1;
  return ::fcntl(fd, F_OFD_SETLK, &fl) == 0;
}
}  // namespace

bool Segment::lock_read(int fd) noexcept
{
  return ofd_setlk(fd, F_RDLCK);
}

bool Segment::lock_write(int fd) noexcept
{
  return ofd_setlk(fd, F_WRLCK);
}

void Segment::init_header(
  std::byte * base, const SegmentLayout & layout, std::uint32_t slot_size, std::uint32_t slot_count,
  std::uint64_t fingerprint, StorageKind kind, std::uint32_t device)
{
  // Zero only the control + slot-header region, never the payload. shm from ftruncate is
  // already zero, and payload is written before it is ever read (a slot is validated only
  // after publish commits it). Touching the payload here would fault in every page and
  // defeat tmpfs sparseness -- the whole point of a large slot_size at near-zero idle RAM.
  std::memset(base, 0, layout.payload_offset());

  ControlHeader * ctrl = new (base) ControlHeader;
  ctrl->magic = kMagic;
  ctrl->version = kLayoutVersion;
  ctrl->slot_size = slot_size;
  ctrl->slot_count = slot_count;
  ctrl->flags = kFlagNone;
  ctrl->storage_kind = static_cast<std::uint8_t>(kind);
  ctrl->device_id = device;
  ctrl->fingerprint = fingerprint;
  // Reserved. A segment is created once under a unique name and never reinitialized in place,
  // so this only ever distinguishes "initialized" from the zeroed 0.
  ctrl->epoch = 1;
  ctrl->latest.store(0, std::memory_order_relaxed);       // nothing published yet
  ctrl->publish_seq.store(1, std::memory_order_relaxed);  // ticket 0 reserved so latest 0 = none
  ctrl->wakeup.store(0, std::memory_order_relaxed);
  ctrl->waiters.store(0, std::memory_order_relaxed);

  for (std::uint32_t i = 0; i < slot_count; ++i) {
    SlotHeader * sl = new (base + layout.slots_offset() + i * layout.slot_stride()) SlotHeader;
    sl->seq.store(0, std::memory_order_relaxed);
    sl->refcount.store(0, std::memory_order_relaxed);
    sl->storage_kind = static_cast<std::uint8_t>(kind);
  }

  // Publish readiness LAST (release): an attacher spins on init_state and only trusts the
  // config above once it observes kInitReady.
  ctrl->init_state.store(kInitReady, std::memory_order_release);
}

Segment Segment::create_heap(
  std::uint32_t slot_size, std::uint32_t slot_count, std::uint64_t fingerprint)
{
  // Same bound as the shm path: one rule, or the heap backing quietly
  // accepts a config that would be rejected the moment the same channel is opened for real.
  if (!layout_config_ok(slot_size, slot_count)) {
    throw std::invalid_argument(
      "flux: slot_size must be >= 1 and slot_count in 1.." + std::to_string(kMaxSlotCount));
  }
  SegmentLayout layout{slot_size, slot_count};
  const std::size_t bytes = layout.total_bytes();
  auto * base = static_cast<std::byte *>(std::aligned_alloc(kCacheLine, bytes));
  if (base == nullptr) throw std::bad_alloc();
  init_header(base, layout, slot_size, slot_count, fingerprint);

  Segment s;
  s.base_ = base;
  s.bytes_ = bytes;
  s.ctrl_ = reinterpret_cast<ControlHeader *>(base);
  s.layout_ = layout;
  s.backing_ = kHeap;
  s.payload_base_ = base + layout.payload_offset();
  return s;
}

bool Segment::protect_payload() noexcept
{
  if (backing_ != kShm || base_ == nullptr) return false;
  if (device_ != nullptr) return false;  // the payload is not in this mapping to protect
  const long page = ::sysconf(_SC_PAGESIZE);
  if (page <= 0 || kPayloadAlign % static_cast<std::size_t>(page) != 0) return false;
  const std::size_t off = layout_.payload_offset();
  if (off >= bytes_) return false;
  if (::mprotect(base_ + off, bytes_ - off, PROT_READ) != 0) return false;
  payload_readonly_ = true;
  return true;
}

// MADV_POPULATE_WRITE rather than a loop that stores into every page: a store into a shared
// mapping is a store other participants can observe, and on a full tmpfs it is a SIGBUS instead of
// an error. This asks the kernel to do the same faulting and reports the refusal.
#ifndef MADV_POPULATE_READ
#define MADV_POPULATE_READ 22
#endif
#ifndef MADV_POPULATE_WRITE
#define MADV_POPULATE_WRITE 23
#endif

void Segment::commit_pages()
{
  if (base_ == nullptr || bytes_ == 0) {
    throw std::runtime_error("flux: commit_pages() on an unmapped segment");
  }
  // A subscriber's payload is PROT_READ (protect_payload), where POPULATE_WRITE is EINVAL. The
  // control plane it does write stays writable, so the mapping is populated in the two halves the
  // protection splits it into. POPULATE_READ is enough for the payload: shmem allocates a folio on
  // a read fault, so the page is resident and the subscriber never writes it anyway.
  const std::size_t split = payload_readonly_ ? layout_.payload_offset() : bytes_;
  if (split > 0 && ::madvise(base_, split, MADV_POPULATE_WRITE) != 0) {
    const int e = errno;
    if (e == EINVAL) {
      throw std::system_error(
        e, std::generic_category(),
        "flux: MADV_POPULATE_WRITE was refused; this kernel is older than 5.14, so page "
        "pre-commit is not available here");
    }
    throw std::system_error(e, std::generic_category(), "flux: madvise(MADV_POPULATE_WRITE)");
  }
  if (split < bytes_ && ::madvise(base_ + split, bytes_ - split, MADV_POPULATE_READ) != 0) {
    throw std::system_error(
      errno, std::generic_category(), "flux: madvise(MADV_POPULATE_READ) on the read-only payload");
  }
  pages_committed_ = true;
}

void Segment::lock_pages()
{
  if (base_ == nullptr || bytes_ == 0) {
    throw std::runtime_error("flux: lock_pages() on an unmapped segment");
  }
  if (::mlock(base_, bytes_) != 0) {
    const int e = errno;
    if (e == ENOMEM || e == EPERM) {
      throw std::system_error(
        e, std::generic_category(),
        "flux: mlock was refused for " + std::to_string(bytes_) +
          " bytes; RLIMIT_MEMLOCK does not cover this segment");
    }
    throw std::system_error(e, std::generic_category(), "flux: mlock");
  }
  pages_locked_ = true;
  pages_committed_ = true;  // mlock populates what it locks
}

Segment Segment::adopt_shm(
  std::byte * base, std::size_t bytes, const SegmentLayout & layout, int fd, std::string name,
  const SegmentId & id, bool unlink_on_last_out) noexcept
{
  Segment s;
  s.base_ = base;
  s.bytes_ = bytes;
  s.ctrl_ = reinterpret_cast<ControlHeader *>(base);
  s.layout_ = layout;
  s.backing_ = kShm;
  s.fd_ = fd;  // >= 0 keeps the liveness read lock held
  s.name_ = std::move(name);
  s.id_ = id;
  s.unlink_on_last_out_ = unlink_on_last_out;
  s.payload_base_ = base + layout.payload_offset();  // attach_device moves it off this mapping
  return s;
}

void Segment::attach_device(std::shared_ptr<void> region, void * payload_base) noexcept
{
  device_ = std::move(region);
  payload_base_ = static_cast<std::byte *>(payload_base);
}

void Segment::reset() noexcept
{
  if (base_ != nullptr) {
    if (backing_ == kHeap) {
      std::free(base_);
    } else if (backing_ == kShm) {
      munmap(base_, bytes_);
    }
  }
  // Publisher shutdown. Upgrading our liveness read lock to a write lock succeeds only when no
  // other publisher holds one -- i.e. we are the last publisher out. The discovery layer asks for
  // the name to be removed in that case, so a clean shutdown leaves no stale segment. A publisher
  // that opened this object just before the unlink still holds a fd to it, but it does not adopt
  // the orphan: open_publisher_segment rechecks, after it locks, that the name still resolves to
  // its fd (discovery.cpp name_still_bound). Subscribers hold no lock (fd_ < 0) and never unlink.
  // A crash skips this entirely.
  if (fd_ >= 0) {
    if (unlink_on_last_out_ && backing_ == kShm && !name_.empty() && ofd_setlk(fd_, F_WRLCK)) {
      ::shm_unlink(name_.c_str());
    }
    ::close(fd_);
  }
  base_ = nullptr;
  bytes_ = 0;
  ctrl_ = nullptr;
  backing_ = kNone;
  fd_ = -1;
  name_.clear();
  id_ = SegmentId{};
  unlink_on_last_out_ = false;
  payload_readonly_ = false;
  pages_committed_ = false;
  pages_locked_ = false;
  payload_base_ = nullptr;
  device_.reset();  // last holder unmaps the device region and stops its fd server
}

Segment::~Segment()
{
  reset();
}

Segment::Segment(Segment && o) noexcept
{
  *this = std::move(o);
}

Segment & Segment::operator=(Segment && o) noexcept
{
  if (this != &o) {
    reset();
    base_ = o.base_;
    bytes_ = o.bytes_;
    ctrl_ = o.ctrl_;
    layout_ = o.layout_;
    backing_ = o.backing_;
    fd_ = o.fd_;
    name_ = std::move(o.name_);
    id_ = o.id_;
    unlink_on_last_out_ = o.unlink_on_last_out_;
    payload_readonly_ = o.payload_readonly_;
    pages_committed_ = o.pages_committed_;
    pages_locked_ = o.pages_locked_;
    payload_base_ = o.payload_base_;
    device_ = std::move(o.device_);
    o.base_ = nullptr;
    o.bytes_ = 0;
    o.ctrl_ = nullptr;
    o.backing_ = kNone;
    o.fd_ = -1;
    o.id_ = SegmentId{};
    o.unlink_on_last_out_ = false;
    o.payload_readonly_ = false;
    o.pages_committed_ = false;
    o.pages_locked_ = false;
    o.payload_base_ = nullptr;
  }
  return *this;
}

}  // namespace flux
