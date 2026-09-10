#ifndef FLUX_SEGMENT_LAYOUT_HPP
#define FLUX_SEGMENT_LAYOUT_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// Cross-process shared-memory layout. Two processes, possibly built by different
// compilers, read the same bytes, so every in-segment type is a fixed-width POD:
// no C bitfields (explicit masks only), no vtable/std::string/std::vector, hot
// atomics (seq vs refcount) on separate cache lines.
//
// std::atomic<integral> is used in-segment: it is layout-compatible with its
// underlying integer and, when lock-free, address-free -- valid on shared memory.
// The static_asserts at the bottom pin size/offset/alignment as the ABI contract.

namespace flux
{

inline constexpr std::uint32_t kMagic = 0x464C5558u;  // 'FLUX'
inline constexpr std::uint32_t kLayoutVersion =
  7u;  // v7: payload offset/stride page-aligned so a subscriber mapping can drop payload writes
inline constexpr std::size_t kCacheLine = 64;
// Payload region alignment: one page, so a subscriber can mprotect(PROT_READ) everything from
// payload_offset() to the end of its mapping while the headers it writes (refcount, holders,
// waiters) stay writable. Fixed rather than read from the kernel -- the layout must be identical
// across processes; a kernel whose page size does not divide this skips the protection instead.
inline constexpr std::size_t kPayloadAlign = 4096;
inline constexpr std::size_t kMaxDims = 8;
inline constexpr std::uint32_t kMaxHolders = 10;  // max concurrent holder processes per slot

// ControlHeader.init_state: bootstrap gate for a shared segment. An
// attacher waits for kInitReady before trusting the config; one initializer transitions it.
inline constexpr std::uint32_t kInitUninit = 0u;
inline constexpr std::uint32_t kInitBusy = 1u;
inline constexpr std::uint32_t kInitReady = 2u;
// Reserved. The sweep gates on the OFD write lock (a live holder or a mid-creation creator blocks
// the sweeper) plus a cleanup lock, so no in-band cleaning marker is needed.
// Kept for a future in-band refinement.
inline constexpr std::uint32_t kInitCleaning = 3u;

// ControlHeader.latest packs the newest published (ticket, slot): high bits = a monotone
// ticket from publish_seq, low kSlotBits = the slot. 0 = nothing published yet. The ticket
// makes "latest" move only forward across concurrent writers.
inline constexpr std::uint32_t kSlotBits = 16;
inline constexpr std::uint64_t kSlotMask = (std::uint64_t{1} << kSlotBits) - 1;
inline constexpr std::uint64_t pack_latest(std::uint64_t ticket, std::uint32_t slot)
{
  return (ticket << kSlotBits) | (static_cast<std::uint64_t>(slot) & kSlotMask);
}
inline constexpr std::uint64_t latest_ticket(std::uint64_t latest)
{
  return latest >> kSlotBits;
}
inline constexpr std::uint32_t latest_slot(std::uint64_t latest)
{
  return static_cast<std::uint32_t>(latest & kSlotMask);
}

// payload backing. Only HostInline is implemented; the rest reserve the GPU door.
enum class StorageKind : std::uint8_t {
  HostInline = 0,
  CudaIpc = 1,
  DmaBuf = 2,
};

// The value goes in FrameMeta, so it is wire ABI: new members append, existing ones never move.
// A consumer that does not know a value gets dtype_size() == 0 and refuses the frame rather than
// sizing a view from a dtype it cannot name.
enum class DType : std::uint8_t {
  U8 = 0,
  I8,
  U16,
  I16,
  U32,
  I32,
  U64,
  I64,
  F16,
  F32,
  F64,
  BF16,
};

// Bytes per element of a DType. FrameMeta carries itemsize too, but a consumer must size a
// typed view from the dtype and validate against this -- trusting the publisher's itemsize lets a
// mismatched pair build a view that runs past the frame.
inline constexpr std::size_t dtype_size(DType dt)
{
  switch (dt) {
    case DType::U8:
    case DType::I8:
      return 1;
    case DType::U16:
    case DType::I16:
    case DType::F16:
    case DType::BF16:
      return 2;
    case DType::U32:
    case DType::I32:
    case DType::F32:
      return 4;
    case DType::U64:
    case DType::I64:
    case DType::F64:
      return 8;
  }
  return 0;
}

// ControlHeader.flags: write-once config bits, explicit masks (never a C bitfield).
inline constexpr std::uint32_t kFlagNone = 0u;

// No schema contract. The raw path has no generator output to derive a fingerprint from
// (docs/en/raw_api.en.md 1), so any publisher on the same topic name attaches. Named rather than
// spelled 0 at the call site: a literal there does not say whether it is a decision or a hole.
inline constexpr std::uint64_t kNoSchema = 0u;

// Frame shape/dtype. POD, fixed max rank.
struct FrameMeta
{
  std::uint8_t ndim;
  DType dtype;
  std::uint8_t reserved0[2];
  std::uint32_t itemsize;  // bytes per element
  std::uint64_t nbytes;    // payload bytes in use (<= slot_size)
  std::uint64_t shape[kMaxDims];
};

// Per-slot control block. seq is publisher-owned, refcount subscriber-owned; they
// sit on distinct cache lines to avoid false sharing. Payload is not inline -- it
// lives in the payload region at a computed offset (see SegmentLayout).
struct alignas(kCacheLine) SlotHeader
{
  std::atomic<std::uint64_t> seq;  // even = stable, odd = write in progress
  // StorageKind. Write-once: the initializer stamps every slot and no publish path
  // touches it again. A commit that rewrote it left a device-backed segment advertising
  // CudaIpc in the header and HostInline in its slots.
  std::uint8_t storage_kind;
  std::uint8_t reserved0[7];
  FrameMeta meta;
  // Global publish ticket of the committed frame (from ControlHeader.publish_seq). take() walks
  // frames by ticket order. Publisher-written under the seq-odd window and read
  // under the seqlock (like meta), so it needs no separate atomic. 0 = never committed.
  std::uint64_t commit_ticket;
  // Identity of the publisher that currently holds this slot claimed (seq odd), so a peer can
  // recover a slot a publisher crashed mid-write. Stamped under the claim;
  // writer_stamp is the odd seq value it was stamped for. A peer treats writer_pid/starttime as
  // valid only when writer_stamp equals the slot's current odd seq, and reverts the claim only
  // if that writer probes dead. 0 = no recoverable identity.
  std::uint32_t writer_pid;
  std::uint32_t writer_pad;
  std::uint64_t writer_starttime;
  std::atomic<std::uint64_t> writer_stamp;
  alignas(kCacheLine) std::atomic<std::uint32_t> refcount;  // active borrows
  std::uint8_t reserved1[kCacheLine - sizeof(std::atomic<std::uint32_t>)];
};

// Identity and view count of one process borrowing the slot, or empty (owner == 0). `count` is
// that process's live view count -- the byte-lock refcount stays per-view and untouched; this
// entry is an additive identity layer a reclaimer uses to undo a dead borrower.
// Safe ordering (refcount++ before count++, count-- before refcount--) keeps count <= the
// process's true contribution, so reclaim never over-subtracts.
//
// pid and count share one atomic word so claim, increment, release and reclaim are each a single
// CAS. As separate atomics a reclaimer could read pid, probe it dead (slow: syscalls), and then
// take the count of an entry a LIVE process had recycled in the meantime -- stealing its refcount.
inline constexpr std::uint64_t pack_holder(std::uint32_t pid, std::uint32_t count)
{
  return (static_cast<std::uint64_t>(count) << 32) | pid;
}
inline constexpr std::uint32_t holder_pid(std::uint64_t owner)
{
  return static_cast<std::uint32_t>(owner);
}
inline constexpr std::uint32_t holder_count(std::uint64_t owner)
{
  return static_cast<std::uint32_t>(owner >> 32);
}

struct SlotHolder
{
  std::atomic<std::uint64_t> owner;  // pack_holder(pid, count); 0 = empty
  std::uint64_t starttime;
};

// Per-slot holder table: up to kMaxHolders distinct processes. Its own cache line(s) so a
// borrower stamping identity does not false-share the seq/refcount hot atomics in SlotHeader.
struct alignas(kCacheLine) SlotHolders
{
  SlotHolder entries[kMaxHolders];
};

// Segment-wide header. Identity/config are write-once (init_state gates them). latest,
// publish_seq and wakeup are publisher-written runtime atomics on their own cache line;
// waiters is subscriber-written and gets a separate line so parking a subscriber does not
// false-share with publishers' per-publish stores.
struct alignas(kCacheLine) ControlHeader
{
  // Config line: write-once by the initializer. init_state gates bootstrap -- an attacher
  // waits for kInitReady before trusting magic/config.
  std::atomic<std::uint32_t> init_state;  // kInitUninit / kInitBusy / kInitReady
  std::uint32_t magic;
  std::uint32_t version;
  std::uint32_t slot_size;
  std::uint32_t slot_count;
  std::uint32_t flags;
  std::uint8_t storage_kind;
  std::uint8_t reserved0[7];
  std::uint64_t fingerprint;  // schema fingerprint (field layout + dtype)
  // Reserved. It signalled a restarted publish_seq back when a segment could be reinitialized in
  // place; a segment is now created once under a unique name, so it is 1
  // once initialized and 0 in the zeroed state. Kept because dropping it would move every offset
  // below and cost a layout version.
  std::uint32_t epoch;
  // CUDA device ordinal the payload region lives on, meaningful only when storage_kind is not
  // HostInline. Taken from reserved space rather than appended, so no offset below moves and the
  // layout version stands: a host segment leaves it 0, and 0 is never read on that path.
  std::uint32_t device_id;
  std::uint8_t reserved1[16];
  // Publisher runtime line, written by every publisher (multi-writer):
  // latest packs (ticket, slot); publish_seq dispenses tickets so latest is monotone.
  alignas(kCacheLine) std::atomic<std::uint64_t> latest;  // pack_latest(ticket, slot); 0 = none
  std::atomic<std::uint64_t> publish_seq;                 // global ticket dispenser
  std::atomic<std::uint32_t> wakeup;                      // futex word, bumped per publish
  std::uint8_t reserved2[kCacheLine - 20];
  alignas(kCacheLine) std::atomic<std::int32_t> waiters;  // parked subscribers (wake gate)
  std::uint8_t reserved3[kCacheLine - 4];
};

inline constexpr std::uint32_t kSignpostMagic = 0x464C5350u;  // 'FLSP'

// Fixed-name rendezvous object. Its name is signpost_name(topic,
// fingerprint) and never changes; it points at the current unique-named segment by that
// segment's creator owner-id, plus an epoch a subscriber watches to detect a rotation.
// (owner-id, epoch) are written under the seqlock `seq` so a lock-free subscriber reads a
// coherent pair; init_state gates the signpost's own bootstrap like ControlHeader.
struct alignas(kCacheLine) Signpost
{
  std::atomic<std::uint32_t> seq;         // seqlock: even = stable, odd = updating
  std::uint32_t magic;                    // kSignpostMagic
  std::uint32_t version;                  // kLayoutVersion
  std::atomic<std::uint32_t> init_state;  // kInitUninit / kInitBusy / kInitReady
  std::uint64_t cur_starttime;            // current segment creator starttime (seqlock-protected)
  std::uint32_t epoch;                    // monotone; 0 = no current segment (seqlock-protected)
  std::uint32_t cur_pid;  // current segment creator pid; 0 = none (seqlock-protected)
  std::uint8_t reserved[kCacheLine - 32];
};

constexpr std::size_t align_up(std::size_t n, std::size_t a)
{
  return (n + a - 1) / a * a;
}

// slot_count is capped by the 16-bit slot field in ControlHeader.latest: a slot it cannot name
// could never be published into.
inline constexpr std::uint32_t kMaxSlotCount = static_cast<std::uint32_t>(kSlotMask);

// Every offset in SegmentLayout is derived from these two, and a consumer takes them from bytes
// another process wrote. Bounding them here rather than at each caller keeps create and attach on
// the same rule, and keeps the arithmetic below inside size_t -- unbounded, slot_size and
// slot_count near UINT32_MAX make total_bytes() wrap, and a wrapped total is exactly what the
// "layout exceeds mapping" check compares against.
constexpr bool layout_config_ok(std::uint32_t slot_size, std::uint32_t slot_count)
{
  return slot_size >= 1 && slot_count >= 1 && slot_count <= kMaxSlotCount;
}

// Computes byte offsets/strides for a segment of the given payload size and count.
// Single source of truth: offsets are derived, never stored redundantly.
struct SegmentLayout
{
  std::uint32_t slot_size;
  std::uint32_t slot_count;

  constexpr std::size_t slots_offset() const
  {
    return align_up(sizeof(ControlHeader), alignof(SlotHeader));
  }
  constexpr std::size_t slot_stride() const { return sizeof(SlotHeader); }
  constexpr std::size_t slots_bytes() const { return slot_stride() * slot_count; }
  // Holder tables sit between the slot headers and the payload: one SlotHolders per slot.
  constexpr std::size_t holders_offset() const
  {
    return align_up(slots_offset() + slots_bytes(), alignof(SlotHolders));
  }
  constexpr std::size_t holder_stride() const { return sizeof(SlotHolders); }
  constexpr std::size_t holders_bytes() const { return holder_stride() * slot_count; }
  constexpr std::size_t payload_stride() const { return align_up(slot_size, kPayloadAlign); }
  constexpr std::size_t payload_offset() const
  {
    return align_up(holders_offset() + holders_bytes(), kPayloadAlign);
  }
  constexpr std::size_t payload_bytes() const { return payload_stride() * slot_count; }
  constexpr std::size_t total_bytes() const { return payload_offset() + payload_bytes(); }
  // The mapping a segment needs when its payload does not live in it. A dGPU channel keeps the
  // control plane in shm and the payload in a device allocation, so its shm object stops
  // where the payload region would start -- the offsets are unchanged, only the extent is.
  constexpr std::size_t control_bytes() const { return payload_offset(); }
};

// ---- ABI contract: pinned at compile time ----
static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
static_assert(std::atomic<std::int32_t>::is_always_lock_free);
static_assert(sizeof(std::atomic<std::uint32_t>) == 4);
static_assert(sizeof(std::atomic<std::uint64_t>) == 8);

static_assert(std::is_standard_layout_v<FrameMeta>);
static_assert(std::is_standard_layout_v<SlotHeader>);
static_assert(std::is_standard_layout_v<ControlHeader>);

static_assert(sizeof(FrameMeta) == 80);
static_assert(alignof(FrameMeta) == 8);

static_assert(sizeof(SlotHeader) == 192);
static_assert(alignof(SlotHeader) == kCacheLine);
static_assert(offsetof(SlotHeader, seq) == 0);
static_assert(offsetof(SlotHeader, commit_ticket) == 96);
static_assert(offsetof(SlotHeader, writer_pid) == 104);
static_assert(offsetof(SlotHeader, writer_stamp) == 120);
static_assert(offsetof(SlotHeader, refcount) == 128);
static_assert(offsetof(SlotHeader, refcount) % kCacheLine == 0);
static_assert(
  offsetof(SlotHeader, seq) / kCacheLine !=
  offsetof(SlotHeader, refcount) / kCacheLine);  // no false sharing

static_assert(std::is_standard_layout_v<SlotHolder>);
static_assert(std::is_standard_layout_v<SlotHolders>);
static_assert(sizeof(SlotHolder) == 16);
static_assert(alignof(SlotHolder) == 8);
static_assert(offsetof(SlotHolder, owner) == 0);
static_assert(offsetof(SlotHolder, starttime) == 8);
static_assert(alignof(SlotHolders) == kCacheLine);
static_assert(
  sizeof(SlotHolders) ==
  (kMaxHolders * sizeof(SlotHolder) + kCacheLine - 1) / kCacheLine * kCacheLine);  // 10 -> 192

static_assert(sizeof(ControlHeader) == 192);
static_assert(alignof(ControlHeader) == kCacheLine);
static_assert(offsetof(ControlHeader, init_state) == 0);
static_assert(offsetof(ControlHeader, magic) == 4);
static_assert(offsetof(ControlHeader, fingerprint) == 32);
static_assert(offsetof(ControlHeader, epoch) == 40);
static_assert(offsetof(ControlHeader, device_id) == 44);
static_assert(offsetof(ControlHeader, latest) == 64);
static_assert(offsetof(ControlHeader, latest) % kCacheLine == 0);
static_assert(offsetof(ControlHeader, publish_seq) == 72);
static_assert(offsetof(ControlHeader, waiters) == 128);
static_assert(offsetof(ControlHeader, waiters) % kCacheLine == 0);
static_assert(
  offsetof(ControlHeader, latest) / kCacheLine !=
  offsetof(ControlHeader, waiters) / kCacheLine);  // no false sharing

// Payload page alignment (v7): a protection starting anywhere else would cover headers the
// subscriber writes.
static_assert(kPayloadAlign % kCacheLine == 0);
static_assert(SegmentLayout{1u, 1u}.payload_offset() % kPayloadAlign == 0);
static_assert(SegmentLayout{1u, 1u}.payload_stride() % kPayloadAlign == 0);
static_assert(SegmentLayout{65537u, 3u}.payload_stride() % kPayloadAlign == 0);

// Under layout_config_ok the widest configuration is 256 TiB of payload -- far from wrapping
// size_t, so every offset above is exact. This is what makes the mapping-size check meaningful.
static_assert(layout_config_ok(~0u, kMaxSlotCount));
static_assert(!layout_config_ok(~0u, kMaxSlotCount + 1));
static_assert(!layout_config_ok(0u, 1u));
static_assert(!layout_config_ok(1u, 0u));
static_assert(SegmentLayout{~0u, kMaxSlotCount}.total_bytes() < (std::size_t{1} << 50));
static_assert(
  SegmentLayout{~0u, kMaxSlotCount}.total_bytes() >
  SegmentLayout{~0u, kMaxSlotCount}.payload_bytes());  // no wrap

static_assert(std::is_standard_layout_v<Signpost>);
static_assert(sizeof(Signpost) == kCacheLine);
static_assert(alignof(Signpost) == kCacheLine);
static_assert(offsetof(Signpost, seq) == 0);
static_assert(offsetof(Signpost, init_state) == 12);
static_assert(offsetof(Signpost, cur_starttime) == 16);
static_assert(offsetof(Signpost, epoch) == 24);
static_assert(offsetof(Signpost, cur_pid) == 28);

}  // namespace flux

#endif  // FLUX_SEGMENT_LAYOUT_HPP
