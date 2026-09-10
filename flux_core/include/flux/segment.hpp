#ifndef FLUX_SEGMENT_HPP
#define FLUX_SEGMENT_HPP

#include "flux/segment_layout.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace flux
{

// Identity of the shm object a segment is mapped onto. A shm name is only a directory entry:
// unlinking and recreating it rebinds the name to a *different* object, so a participant still
// holding the old mapping must notice and re-attach. (dev, ino) is that stable identity; the
// namespace-level checks that use it live in discovery.hpp.
struct SegmentId
{
  std::uint64_t dev = 0;
  std::uint64_t ino = 0;

  bool valid() const noexcept { return ino != 0; }
  bool operator==(const SegmentId & o) const noexcept { return dev == o.dev && ino == o.ino; }
  bool operator!=(const SegmentId & o) const noexcept { return !(*this == o); }
};

// Owns the backing memory for one channel and exposes its base pointer + layout. Two backings:
// heap (process-local, for tests) and POSIX shm (cross-process). Move-only.
//
// This is mechanics only -- mapping, OFD locks, writing a fresh header. The rendezvous *policy*
// (which name, create vs join, reinit vs adopt, when to unlink) belongs to the discovery layer
// above it (discovery.hpp), which hands finished mappings to adopt_shm.
class Segment
{
public:
  Segment() = default;
  ~Segment();
  Segment(Segment && other) noexcept;
  Segment & operator=(Segment && other) noexcept;
  Segment(const Segment &) = delete;
  Segment & operator=(const Segment &) = delete;

  // Heap-backed, freshly initialized. For unit tests / single-process use.
  static Segment create_heap(
    std::uint32_t slot_size, std::uint32_t slot_count, std::uint64_t fingerprint = 0);

  // ---- mechanics for the discovery layer (it owns the rendezvous policy) ----

  // Write a fresh header over a mapping, ending by publishing kInitReady. `kind` and `device`
  // record where the payload lives; a host segment leaves them at the defaults.
  static void init_header(
    std::byte * base, const SegmentLayout & layout, std::uint32_t slot_size,
    std::uint32_t slot_count, std::uint64_t fingerprint, StorageKind kind = StorageKind::HostInline,
    std::uint32_t device = 0);

  // Non-blocking OFD locks on byte [0,1) of a segment fd -- advisory, unrelated to the mapped
  // data. A read lock marks a live publisher; the write lock is exclusive, so it succeeds only
  // when no publisher holds a read lock (the (re)initializer, and the last one out).
  static bool lock_read(int fd) noexcept;
  static bool lock_write(int fd) noexcept;

  // Take ownership of a finished shm mapping. `fd` < 0 means this participant holds no liveness
  // lock (a subscriber). `unlink_on_last_out` asks the destructor to remove the name if it can
  // still take the write lock, i.e. no other publisher is left.
  static Segment adopt_shm(
    std::byte * base, std::size_t bytes, const SegmentLayout & layout, int fd, std::string name,
    const SegmentId & id, bool unlink_on_last_out) noexcept;

  bool valid() const noexcept { return base_ != nullptr; }
  std::byte * base() const noexcept { return base_; }

  // Where slot 0's payload starts. Inside this mapping for a host segment, and in a device
  // allocation for a dGPU one -- slot `i` is `payload_base() + i * payload_stride()`
  // either way, which is what keeps one slot protocol running over both.
  std::byte * payload_base() const noexcept { return payload_base_; }
  bool device_backed() const noexcept { return device_ != nullptr; }

  // Point the payload at a device region and take a share of its lifetime. Type-erased on
  // purpose: the region is made by flux/gpu_vmm.hpp, which sits above this layer.
  void attach_device(std::shared_ptr<void> region, void * payload_base) noexcept;
  ControlHeader * ctrl() const noexcept { return ctrl_; }
  const SegmentLayout & layout() const noexcept { return layout_; }

  // The shm object this mapping is on, and the name it was reached by. Compare id() against the
  // name's current identity to detect that the segment was unlinked and recreated.
  const SegmentId & id() const noexcept { return id_; }
  const std::string & name() const noexcept { return name_; }

  // True for a publisher (holds the OFD liveness read lock). Subscribers hold none, so only
  // they may be re-attached to a replacement segment.
  bool holds_liveness_lock() const noexcept { return fd_ >= 0; }

  // Drop this mapping's payload region to PROT_READ (subscriber hardening; the headers a
  // subscriber writes stay writable). Returns false without side effects when the mapping is
  // not shm-backed, the runtime page size does not divide kPayloadAlign, or mprotect is
  // refused -- the attach stands either way, and payload_readonly() reports the outcome.
  bool protect_payload() noexcept;
  bool payload_readonly() const noexcept { return payload_readonly_; }

  // Fault in every page of this mapping, so no page fault is left on the publish or the take.
  // Safe after protect_payload(): the read-only half is populated read-only.
  // A device-backed segment's payload is not in this mapping and is not covered.
  //
  // Throws std::system_error rather than degrading: a caller that asked for a bound and silently
  // did not get one is the failure this option exists to prevent. On a kernel without
  // MADV_POPULATE_WRITE (before 5.14) the throw says so.
  void commit_pages();

  // mlock this mapping. Throws std::system_error, ENOMEM being RLIMIT_MEMLOCK too small for the
  // segment. Locking populates, so this subsumes commit_pages() for the same range.
  void lock_pages();

  bool pages_committed() const noexcept { return pages_committed_; }
  bool pages_locked() const noexcept { return pages_locked_; }

private:
  enum Backing : std::uint8_t { kNone, kHeap, kShm };

  void reset() noexcept;

  std::byte * base_ = nullptr;
  std::size_t bytes_ = 0;
  ControlHeader * ctrl_ = nullptr;
  SegmentLayout layout_{};
  Backing backing_ = kNone;
  int fd_ = -1;       // publisher: kept open holding an OFD read lock (liveness); -1 otherwise
  std::string name_;  // shm name
  SegmentId id_{};    // the shm object behind name_ at the time we mapped it
  bool unlink_on_last_out_ = false;     // publisher: remove the name if we are the last one out
  bool payload_readonly_ = false;       // payload region dropped to PROT_READ on this mapping
  bool pages_committed_ = false;        // every page of this mapping faulted in at attach
  bool pages_locked_ = false;           // this mapping is mlocked
  std::byte * payload_base_ = nullptr;  // inside base_, or into device_ when device-backed
  std::shared_ptr<void> device_;        // dGPU payload region (gpu::DeviceAlloc / DeviceImport)
};

}  // namespace flux

#endif  // FLUX_SEGMENT_HPP
