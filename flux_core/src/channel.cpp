#include "flux/channel.hpp"

#include "flux/futex.hpp"

#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <utility>

// The Dekker handshake between publisher
// (claim seq odd, then re-read refcount) and subscriber (refcount++, then read seq)
// uses seq_cst on both announce/observe pairs, so in the single total order at least one side
// observes the other's store. The seqlock commit is release and the validating load is acquire.

namespace flux
{

namespace
{

constexpr std::int64_t kNsPerSec = 1'000'000'000;

// The seqlock's plain stores, isolated so test/tsan.supp names them and not the atomics around
// them. noinline is load bearing: an inlined leaf leaves no frame for a suppression to match,
// and every entry in that file goes dead without the build breaking.
[[gnu::noinline]] void store_frame_bytes(
  void * dst, const void * data, std::size_t nbytes, SlotHeader * sl,
  const FrameMeta & meta) noexcept
{
  std::memcpy(dst, data, nbytes);
  sl->meta = meta;
  sl->meta.nbytes = nbytes;
}

[[gnu::noinline]] void store_frame_meta(SlotHeader * sl, const FrameMeta & meta) noexcept
{
  sl->meta = meta;
}

[[gnu::noinline]] void store_commit_ticket(SlotHeader * sl, std::uint64_t ticket) noexcept
{
  sl->commit_ticket = ticket;
}

[[gnu::noinline]] void store_writer_identity(SlotHeader * sl, const OwnerId & id) noexcept
{
  sl->writer_pid = id.pid;
  sl->writer_starttime = id.starttime;
}

// Judge a stream declaration against what this host can actually serve.
// Refusing here is the point: a declaration this host cannot honour turns every release into a
// slot that is never reused, and it would do so silently.
gpu::Stream checked(const gpu::Stream & stream)
{
  if (stream.declared()) {
    gpu::require_route();
    gpu::require_fenceable(stream);
  }
  return stream;
}

// A declared stream promises a kernel can reach this channel's payload. require_route() judges
// the host; this judges the pairing, which the host alone cannot answer. On the DeviceHandle route
// only a device-backed segment keeps the promise -- a host mapping there would hand out a device
// pointer into memory no kernel can touch, which is exactly the silent failure a declaration is
// refused to prevent.
void require_reachable(const Segment & seg, const gpu::Stream & stream)
{
  if (!stream.declared() || seg.device_backed()) return;
  if (gpu::probe().route != gpu::Route::DeviceHandle) return;
  throw std::invalid_argument(
    "flux: this host reaches the GPU through the device handle path, so a channel that declares a "
    "stream needs a device-backed payload; this segment's payload is host memory "
    "");
}

// Upper bound on a single park inside take_blocking. Re-running take() is what advances the
// stall counter that triggers a re-attach, so an unbounded park on a segment whose publisher
// has restarted would sleep on a wake word nobody will ever bump again.
constexpr std::int64_t kMaxParkNs = 100'000'000;

struct timespec ns_to_timespec(std::int64_t ns) noexcept
{
  struct timespec ts;
  ts.tv_sec = static_cast<std::time_t>(ns / kNsPerSec);
  ts.tv_nsec = static_cast<long>(ns % kNsPerSec);
  return ts;
}

std::int64_t monotonic_ns() noexcept
{
  struct timespec ts
  {
  };
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * kNsPerSec + ts.tv_nsec;
}

}  // namespace

namespace
{
// A frame whose meta contradicts itself is rejected at publish, not reinterpreted later: a
// consumer sizes its view from dtype, so an itemsize that disagrees makes that view overrun the
// frame. ndim is here too because a consumer that rejects it on read never advances its cursor,
// which would wedge take() on that slot instead of skipping it.
bool meta_is_sane(const FrameMeta & m) noexcept
{
  return m.ndim <= kMaxDims && m.itemsize == dtype_size(m.dtype);
}

// The one place a FrameMeta is built on the publish side. itemsize follows from dtype and ndim
// from the shape's length, so the fields that could contradict each other are not inputs. An
// out-of-range rank or a byte count that overflows is carried as a size the commit path rejects,
// rather than reported here -- the caller already has one place to read the outcome.
FrameMeta meta_from(DType dt, const std::uint64_t * shape, std::size_t ndim) noexcept
{
  FrameMeta m{};
  m.dtype = dt;
  m.itemsize = dtype_size(dt);
  m.ndim = static_cast<std::uint8_t>(ndim > 0xFFu ? 0xFFu : ndim);
  std::uint64_t n = m.itemsize;
  for (std::size_t i = 0; i < ndim; ++i) {
    if (i < kMaxDims) m.shape[i] = shape[i];
    if (shape[i] != 0 && n > UINT64_MAX / shape[i]) {
      n = UINT64_MAX;  // rejected downstream as TooLarge
      break;
    }
    n *= shape[i];
  }
  m.nbytes = ndim == 0 ? 0 : n;
  return m;
}

FrameMeta u8_meta(std::uint64_t nbytes) noexcept
{
  return meta_from(DType::U8, &nbytes, 1);
}
}  // namespace

bool ChannelShared::fence(Seam seam) noexcept
{
  if (!stream.declared()) return true;  // no CUDA on this channel: no wait, and no clock either

  const std::int64_t t0 = monotonic_ns();
  const bool ok = stream.wait();
  const auto dt = static_cast<std::uint64_t>(monotonic_ns() - t0);

  // A failed wait blocked the caller too, so it is timed like any other. What it did not do is
  // establish completion, and fence_failed is where that shows.
  const bool commit = seam == Seam::Commit;
  auto & sum = commit ? commit_wait_ns : release_wait_ns;
  auto & count = commit ? commit_waits : release_waits;
  auto & worst = commit ? commit_wait_max_ns : release_wait_max_ns;
  sum.fetch_add(dt, std::memory_order_relaxed);
  count.fetch_add(1, std::memory_order_relaxed);
  std::uint64_t seen = worst.load(std::memory_order_relaxed);
  while (dt > seen && !worst.compare_exchange_weak(seen, dt, std::memory_order_relaxed)) {
  }

  if (!ok) fence_failed.fetch_add(1, std::memory_order_relaxed);
  return ok;
}

FrameView::~FrameView()
{
  release();
}

FrameView::FrameView(FrameView && o) noexcept
: sh_(std::move(o.sh_)), slot_(o.slot_), holder_(o.holder_), data_(o.data_), size_(o.size_)
{
  o.slot_ = nullptr;
  o.holder_ = nullptr;
  o.data_ = nullptr;
  o.size_ = 0;
}

FrameView & FrameView::operator=(FrameView && o) noexcept
{
  if (this != &o) {
    release();
    sh_ = std::move(o.sh_);
    slot_ = o.slot_;
    holder_ = o.holder_;
    data_ = o.data_;
    size_ = o.size_;
    o.slot_ = nullptr;
    o.holder_ = nullptr;
    o.data_ = nullptr;
    o.size_ = 0;
  }
  return *this;
}

const void * FrameView::device_ptr() const noexcept
{
  return (sh_ && sh_->stream.declared()) ? sh_->to_device(data_) : nullptr;
}

bool FrameView::host_addressable() const noexcept
{
  return sh_ != nullptr && !sh_->device_payload;
}

gpu::Stream FrameView::stream() const noexcept
{
  return sh_ ? sh_->stream : gpu::Stream{};
}

void FrameView::release() noexcept
{
  if (slot_ != nullptr) {
    // A failed fence leaves the borrow standing on purpose: the publisher must not regain a slot
    // a kernel may still be reading. The handle is consumed either way so a second release
    // cannot double-count, and the leak is visible through Channel::fence_failed().
    if (sh_->fence(ChannelShared::Seam::Release)) {
      sh_->end_borrow(holder_, slot_);  // sh_ outlives the Channel, so this is always safe
    } else {
      sh_->leases_leaked.fetch_add(1, std::memory_order_release);  // never returned; see Refused
    }
    sh_.reset();
    slot_ = nullptr;
    holder_ = nullptr;
    data_ = nullptr;
    size_ = 0;
  }
}

const FrameMeta & FrameView::meta() const noexcept
{
  return slot_->meta;
}

Channel::Channel(Segment segment) noexcept
: sh_(std::make_shared<ChannelShared>(std::move(segment)))
{
  position_cursor();
}

Channel::Channel(Segment segment, gpu::Stream stream, const MemoryPolicy & mem)
{
  checked(stream);
  require_reachable(segment, stream);
  sh_ = std::make_shared<ChannelShared>(std::move(segment), stream);
  sh_->register_host_payload();  // throws if the host needs registration and refuses it
  mem_ = mem;
  apply_memory_policy();  // before any frame crosses: the bound is on the hot path, not on init
  position_cursor();
}

Channel::Channel(
  Segment segment, std::string signpost_name, std::uint32_t signpost_epoch, gpu::Stream stream,
  const MemoryPolicy & mem)
: Channel(std::move(segment), stream, mem)
{
  signpost_name_ = std::move(signpost_name);
  signpost_epoch_ = signpost_epoch;
  signpost_ = SignpostView(signpost_name_);  // held for this consumer's lifetime
}

void Channel::position_cursor() noexcept
{
  const std::uint64_t l = sh_->ctrl->latest.load(std::memory_order_acquire);
  const std::uint64_t newest = (l == 0) ? 0 : latest_ticket(l);
  const std::uint64_t back = qos_.durability.replay;
  cursor_ = (newest > back) ? (newest - back) : 0;
}

SlotHolder * Channel::holder_acquire(std::uint32_t s) noexcept
{
  SlotHolders * tbl = holders(s);
  const OwnerId & me = OwnerFile::self();
  for (std::uint32_t k = 0; k < kMaxHolders; ++k) {  // reuse this process's entry for the slot
    SlotHolder & e = tbl->entries[k];
    std::uint64_t o = e.owner.load(std::memory_order_acquire);
    // Join an entry only once it has published a count. Count 0 is a claim in progress (below),
    // and that window belongs to the thread that claimed it -- reclaimers already skip it for the
    // same reason. Joining there breaks it twice over: this thread's increment is lost to the
    // claimer's store, and if this thread releases first the entry is freed under a claimer that
    // then publishes a count onto a row no longer bearing its identity. Either way the entry ends
    // up accounting for fewer borrows than the process holds, and reclaim cannot make that good.
    while (holder_pid(o) == me.pid && holder_count(o) != 0 && e.starttime == me.starttime) {
      const std::uint64_t next = pack_holder(me.pid, holder_count(o) + 1);
      if (e.owner.compare_exchange_weak(
            o, next, std::memory_order_acq_rel, std::memory_order_acquire)) {
        return &e;
      }
    }
  }
  // Otherwise claim an empty entry. Claiming with count 0 and publishing count 1 only after
  // starttime is written means a crash inside the window leaves count 0, which a reclaimer
  // subtracts harmlessly.
  for (std::uint32_t k = 0; k < kMaxHolders; ++k) {
    SlotHolder & e = tbl->entries[k];
    std::uint64_t expected = 0;
    if (e.owner.compare_exchange_strong(
          expected, pack_holder(me.pid, 0), std::memory_order_acq_rel, std::memory_order_relaxed)) {
      e.starttime = me.starttime;
      e.owner.store(pack_holder(me.pid, 1), std::memory_order_release);
      return &e;
    }
  }
  return nullptr;  // table full: kMaxHolders distinct processes already hold this slot
}

void ChannelShared::register_host_payload()
{
  // Nothing to register: no kernel was promised access, or the payload is not in this mapping.
  if (!stream.declared() || device_payload) return;
  if (gpu::probe().route != gpu::Route::ShmRegistered) return;
  // The region is [payload_base, +payload_bytes). Both ends are kPayloadAlign-aligned by
  // construction (segment_layout.hpp), which is what the driver wants -- the same alignment
  // Segment::protect_payload() already relies on.
  host_registration =
    gpu::HostRegistration::create(payload_base, layout.payload_bytes(), seg.payload_readonly());
  device_payload_base = static_cast<std::byte *>(host_registration.device_base());
}

void ChannelShared::end_borrow(SlotHolder * holder, SlotHeader * sl) noexcept
{
  if (holder != nullptr) {  // count-- before refcount--; the last release frees the entry, or
    std::uint64_t o = holder->owner.load(std::memory_order_acquire);  // it leaks until reinit
    while (holder_count(o) != 0) {
      const std::uint32_t c = holder_count(o);
      const std::uint64_t next = (c == 1) ? 0u : pack_holder(holder_pid(o), c - 1);
      if (holder->owner.compare_exchange_weak(
            o, next, std::memory_order_acq_rel, std::memory_order_acquire)) {
        break;
      }
    }
  }
  sl->refcount.fetch_sub(1, std::memory_order_release);
  outstanding.fetch_sub(1, std::memory_order_release);
}

bool Channel::reclaim_dead() noexcept
{
  bool reclaimed = false;
  const std::uint32_t n = sh_->layout.slot_count;
  for (std::uint32_t s = 0; s < n; ++s) {
    if (slot(s)->refcount.load(std::memory_order_acquire) == 0) continue;  // already free
    SlotHolders * tbl = holders(s);
    for (std::uint32_t k = 0; k < kMaxHolders; ++k) {
      SlotHolder & e = tbl->entries[k];
      // One CAS validates identity and count together. Reading pid, probing it (syscalls), then
      // taking the count separately would let a live process recycle the entry in between and
      // lose its refcount to us.
      const std::uint64_t o = e.owner.load(std::memory_order_acquire);
      const std::uint32_t c = holder_count(o);
      if (c == 0) continue;  // empty, or mid-claim with no refcount yet
      OwnerId id;
      id.pid = holder_pid(o);
      id.starttime = e.starttime;
      if (id.pid == 0) continue;
      if (OwnerFile::probe(id) != Liveness::Dead) continue;  // owner alive: leave the borrow
      std::uint64_t expected = o;
      if (!e.owner.compare_exchange_strong(
            expected, 0u, std::memory_order_acq_rel, std::memory_order_relaxed)) {
        continue;  // changed under us: another reclaimer took it, or it was recycled
      }
      slot(s)->refcount.fetch_sub(c, std::memory_order_acq_rel);
      reclaimed = true;
    }
  }
  return reclaimed;
}

void Channel::stamp_writer(SlotHeader * sl, std::uint64_t odd) noexcept
{
  if (!writer_ready_) {
    try {
      OwnerFile::ensure();  // this publisher must be probeable for a peer to recover its slots
      writer_id_ = OwnerFile::self();
    } catch (...) {
      writer_id_ = OwnerId{};  // no owner file -> writer_pid stays 0 -> not recoverable (as before)
    }
    writer_ready_ = true;
  }
  store_writer_identity(sl, writer_id_);
  sl->writer_stamp.store(odd, std::memory_order_release);  // publishes the identity for this claim
}

bool Channel::recover_stuck_writes() noexcept
{
  bool recovered = false;
  const std::uint32_t n = sh_->layout.slot_count;
  for (std::uint32_t s = 0; s < n; ++s) {
    SlotHeader * sl = slot(s);
    const std::uint64_t o = sl->seq.load(std::memory_order_acquire);
    if ((o & 1u) == 0u) continue;  // even: not claimed
    // The identity is trustworthy only if it was stamped for this exact odd generation. A writer
    // that died between winning the claim and stamping leaves writer_stamp != o -> skip (rare; the
    // slot stays claimed until an all-publishers-dead reinit).
    if (sl->writer_stamp.load(std::memory_order_acquire) != o) continue;
    OwnerId id;
    id.pid = sl->writer_pid;
    id.starttime = sl->writer_starttime;
    if (id.pid == 0) continue;                                   // no recoverable identity
    if (sl->seq.load(std::memory_order_acquire) != o) continue;  // changed under us: skip the probe
    if (OwnerFile::probe(id) != Liveness::Dead) continue;        // writer alive: leave the claim

    // Dead writer. Take the slot before touching it: re-claim its generation FORWARD with a CAS
    // (odd o -> odd o+2). The CAS is the gate -- it succeeds only if seq is still exactly o, so a
    // commit the writer landed just before dying, or a peer that already recovered this slot,
    // makes it fail and the slot is left untouched. On success the slot is ours alone: it stays
    // odd so every reader skips it, and writer_stamp still names the dead claim so no other
    // recoverer takes it either. Only now is dropping the ticket safe -- the dead writer's payload
    // may be half-written, so the frame the slot advertises is gone, and a consumer that read the
    // old ticket over the new garbage would tear. Release FORWARD to even (o + 3), never back onto
    // a used generation: a repeatable seq lets a reader's seqlock window close on a destroyed
    // generation, and lets a later recovery mistake a fresh claim for the dead one.
    std::uint64_t expected = o;
    if (!sl->seq.compare_exchange_strong(
          expected, o + 2, std::memory_order_seq_cst, std::memory_order_relaxed)) {
      continue;  // seq moved since the probe: not ours to reclaim
    }
    store_commit_ticket(sl, 0);                       // exclusive (odd): readers skip this slot
    sl->seq.store(o + 3, std::memory_order_release);  // release: free, even, no valid frame
    recovered = true;
  }
  return recovered;
}

std::uint64_t Channel::dropped() const noexcept
{
  return sh_->dropped.load(std::memory_order_relaxed);
}

std::uint32_t Channel::slot_refcount(std::uint32_t i) const noexcept
{
  return slot(i)->refcount.load(std::memory_order_acquire);
}

Published Channel::publish(const void * data, std::size_t nbytes) noexcept
{
  return publish_meta(data, u8_meta(nbytes));
}

Published Channel::publish(
  const void * data, DType dt, const std::uint64_t * shape, std::size_t ndim) noexcept
{
  return publish_meta(data, meta_from(dt, shape, ndim));
}

Published Channel::publish(
  const void * data, DType dt, std::initializer_list<std::uint64_t> shape) noexcept
{
  return publish(data, dt, shape.begin(), shape.size());
}

Published Channel::publish_meta(const void * data, const FrameMeta & meta) noexcept
{
  const std::size_t nbytes = static_cast<std::size_t>(meta.nbytes);
  const std::uint32_t n = sh_->layout.slot_count;
  // A device-backed slot is GPU memory. The memcpy below would store through a device
  // pointer, so this is a memory-safety gate, not only a policy one.
  if (sh_->device_payload) return Published::WrongDevice;
  // meta_is_sane cannot fail from here -- meta_from derives itemsize and ndim rather than taking
  // them -- but it is the invariant the consumer's view depends on, so it is checked where it is
  // relied on rather than assumed.
  if (meta.nbytes > sh_->layout.slot_size || !meta_is_sane(meta)) return Published::TooLarge;
  // The source is the caller's buffer, but the stream declared on this channel is the caller's
  // statement about where its frames come from, so the same fence applies before reading it.
  if (!sh_->fence(ChannelShared::Seam::Commit)) return Published::FenceFailed;
  // Two passes: on the first, select a free slot as usual; if none is free, reclaim slots
  // held by dead subscribers and retry once. The second pass drops.
  for (int pass = 0; pass < 2; ++pass) {
    // Scan from just past the latest slot. With multiple publishers this is only a hint --
    // the CAS claim below resolves any collision.
    const std::uint64_t l = sh_->ctrl->latest.load(std::memory_order_relaxed);
    const std::int64_t start = (l == 0) ? 0 : (static_cast<std::int64_t>(latest_slot(l)) + 1);

    for (std::uint32_t k = 0; k < n; ++k) {
      const std::uint32_t s =
        static_cast<std::uint32_t>((start + static_cast<std::int64_t>(k)) % n);
      SlotHeader * sl = slot(s);
      if (sl->refcount.load(std::memory_order_acquire) != 0) continue;  // hint: skip borrowed

      std::uint64_t even = sl->seq.load(std::memory_order_relaxed);
      if (even & 1u) continue;  // another publisher holds this slot (odd) -> skip
      // claim: CAS even -> odd. Only one writer wins, so a slot has a single writer at a
      // time and the per-slot protocol is unchanged.
      if (!sl->seq.compare_exchange_strong(
            even, even + 1, std::memory_order_seq_cst, std::memory_order_relaxed)) {
        continue;  // lost the claim to another writer -> reselect
      }

      if (sl->refcount.load(std::memory_order_seq_cst) != 0) {  // Dekker recheck
        sl->seq.store(even + 2, std::memory_order_seq_cst);     // borrowed under us: revert
        continue;                                               // reselect
      }
      stamp_writer(sl, even + 1);  // record identity (seq is odd = even+1) for crash recovery

      store_frame_bytes(payload(s), data, nbytes, sl, meta);  // write payload

      sh_->finish_commit(sl, s, even);  // stamp ticket, commit (release), latest + wake
      return Published::Ok;
    }

    if (pass == 0) {  // starved: recover dead borrowers' and dead writers' slots, then retry
      const bool a = reclaim_dead();
      const bool b = recover_stuck_writes();
      if (a || b) continue;
    }
    break;
  }

  sh_->dropped.fetch_add(1, std::memory_order_relaxed);  // all slots held by live borrowers
  return Published::Backpressure;
}

void ChannelShared::finish_commit(SlotHeader * sl, std::uint32_t s, std::uint64_t even) noexcept
{
  // Take the frame's monotone ticket and stamp it into the slot BEFORE the seqlock commit, so a
  // take() consumer reads commit_ticket under the same seqlock as the payload/meta.
  const std::uint64_t ticket = ctrl->publish_seq.fetch_add(1, std::memory_order_relaxed);
  store_commit_ticket(sl, ticket);
  sl->seq.store(even + 2, std::memory_order_release);  // commit: publishes payload+meta+ticket

  // publish latest: move `latest` to (ticket, slot) only if this commit is newer -- so
  // under concurrent writers latest never moves backward.
  const std::uint64_t mine = pack_latest(ticket, s);
  std::uint64_t cur = ctrl->latest.load(std::memory_order_relaxed);
  while (latest_ticket(cur) < ticket) {
    if (ctrl->latest.compare_exchange_weak(
          cur, mine, std::memory_order_release, std::memory_order_relaxed)) {
      break;
    }
  }

  // Wake gate: bump the generation, then wake only if a subscriber
  // announced itself. The seq_cst pair here (bump wakeup, read waiters) Dekkers with
  // wait()'s (announce waiters, read wakeup), so a subscriber racing to park is never
  // left asleep. Skipping the syscall when waiters==0 keeps the no-listener hot path free.
  ctrl->wakeup.fetch_add(1, std::memory_order_seq_cst);
  if (ctrl->waiters.load(std::memory_order_seq_cst) > 0) futex_wake(&ctrl->wakeup, INT_MAX);
}

WriteSlot Channel::loan() noexcept
{
  // The whole slot as one u8 run. A generated adapter takes this and commits the prefix it built.
  const std::uint64_t whole = sh_->layout.slot_size;
  return loan(DType::U8, &whole, 1);
}

WriteSlot Channel::loan(DType dt, std::initializer_list<std::uint64_t> shape) noexcept
{
  return loan(dt, shape.begin(), shape.size());
}

WriteSlot Channel::loan(DType dt, const std::uint64_t * shape, std::size_t ndim) noexcept
{
  // The shape is not sized against the slot here. A loan that cannot fit is still a claimed slot,
  // and commit() is where the caller already reads the outcome -- refusing in two places would
  // make the caller check twice for one mistake.
  const FrameMeta meta = meta_from(dt, shape, ndim);
  // Slot selection + claim + Dekker recheck are identical to publish(); the only
  // difference is the payload is written by the caller into the returned handle, not memcpy'd
  // here, and the commit is deferred to WriteSlot::commit. The slot is left claimed (seq odd)
  // for the loan's lifetime; it is not the latest yet, so no subscriber targets it.
  const std::uint32_t n = sh_->layout.slot_count;
  for (int pass = 0; pass < 2; ++pass) {
    const std::uint64_t l = sh_->ctrl->latest.load(std::memory_order_relaxed);
    const std::int64_t start = (l == 0) ? 0 : (static_cast<std::int64_t>(latest_slot(l)) + 1);

    for (std::uint32_t k = 0; k < n; ++k) {
      const std::uint32_t s =
        static_cast<std::uint32_t>((start + static_cast<std::int64_t>(k)) % n);
      SlotHeader * sl = slot(s);
      if (sl->refcount.load(std::memory_order_acquire) != 0) continue;

      std::uint64_t even = sl->seq.load(std::memory_order_relaxed);
      if (even & 1u) continue;
      if (!sl->seq.compare_exchange_strong(
            even, even + 1, std::memory_order_seq_cst, std::memory_order_relaxed)) {
        continue;
      }
      if (sl->refcount.load(std::memory_order_seq_cst) != 0) {  // Dekker recheck
        sl->seq.store(even + 2, std::memory_order_seq_cst);
        continue;
      }
      stamp_writer(sl, even + 1);  // record identity for crash recovery (loan holds the claim open)
      return WriteSlot{sh_, sl, s, even, payload(s), sh_->layout.slot_size, meta};
    }

    if (pass == 0) {  // starved: recover dead borrowers' and dead writers' slots, then retry
      const bool a = reclaim_dead();
      const bool b = recover_stuck_writes();
      if (a || b) continue;
    }
    break;
  }

  sh_->dropped.fetch_add(1, std::memory_order_relaxed);
  return WriteSlot{};
}

WriteSlot::~WriteSlot()
{
  abort();
}

WriteSlot::WriteSlot(WriteSlot && o) noexcept
: sh_(std::move(o.sh_)),
  slot_(o.slot_),
  index_(o.index_),
  even_(o.even_),
  data_(o.data_),
  cap_(o.cap_),
  meta_(o.meta_)
{
  o.slot_ = nullptr;
  o.data_ = nullptr;
  o.cap_ = 0;
}

WriteSlot & WriteSlot::operator=(WriteSlot && o) noexcept
{
  if (this != &o) {
    abort();
    sh_ = std::move(o.sh_);
    slot_ = o.slot_;
    index_ = o.index_;
    even_ = o.even_;
    data_ = o.data_;
    cap_ = o.cap_;
    meta_ = o.meta_;
    o.slot_ = nullptr;
    o.data_ = nullptr;
    o.cap_ = 0;
  }
  return *this;
}

void * WriteSlot::device_ptr() noexcept
{
  return (sh_ && sh_->stream.declared()) ? sh_->to_device(data_) : nullptr;
}

bool WriteSlot::host_addressable() const noexcept
{
  return sh_ != nullptr && !sh_->device_payload;
}

gpu::Stream WriteSlot::stream() const noexcept
{
  return sh_ ? sh_->stream : gpu::Stream{};
}

const char * to_string(Published p) noexcept
{
  switch (p) {
    case Published::Ok:
      return "Ok";
    case Published::Backpressure:
      return "Backpressure";
    case Published::TooLarge:
      return "TooLarge";
    case Published::WrongDevice:
      return "WrongDevice";
    case Published::FenceFailed:
      return "FenceFailed";
  }
  return "Unknown";
}

Published WriteSlot::commit(std::size_t nbytes) noexcept
{
  if (slot_ == nullptr) return Published::TooLarge;
  // Shrinking the byte count is only the same statement as shrinking the shape when there is one
  // dimension to shrink. On any other rank the frame would have to be described twice.
  if (
    meta_.ndim == 1 && meta_.itemsize != 0 && nbytes <= meta_.nbytes &&
    nbytes % meta_.itemsize == 0) {
    meta_.nbytes = nbytes;
    meta_.shape[0] = nbytes / meta_.itemsize;
    return commit();
  }
  abort();
  return Published::TooLarge;
}

Published WriteSlot::commit() noexcept
{
  if (slot_ == nullptr) return Published::TooLarge;
  if (meta_.nbytes > cap_ || !meta_is_sane(meta_)) {  // bad frame: revert rather than commit it
    abort();
    return Published::TooLarge;
  }
  if (!sh_->fence(ChannelShared::Seam::Commit)) {
    // Not abort(): releasing the claim would hand the slot to the next publisher while the
    // producing kernel may still be writing it. The claim stays, so this slot is never reused.
    slot_ = nullptr;
    sh_.reset();
    return Published::FenceFailed;
  }
  store_frame_meta(slot_, meta_);
  sh_->finish_commit(slot_, index_, even_);  // ticket, commit (release), latest + wake
  slot_ = nullptr;
  sh_.reset();
  return Published::Ok;
}

void WriteSlot::abort() noexcept
{
  if (slot_ != nullptr) {
    // data() aliases the slot, so anything the holder wrote already destroyed the frame this
    // slot still advertises. Drop the ticket so no consumer is handed that frame's meta over
    // the new bytes; a lagging consumer loses one frame (counted in lost()) instead of getting
    // a corrupt one. Then release the claim forward, never back onto a generation that can repeat.
    store_commit_ticket(slot_, 0);
    slot_->seq.store(even_ + 2, std::memory_order_seq_cst);
    slot_ = nullptr;
    sh_.reset();
  }
}

bool Channel::can_borrow() const noexcept
{
  return sh_->outstanding.load(std::memory_order_acquire) < qos_.max_borrow;
}

std::uint32_t ChannelShared::wake_seq() const noexcept
{
  return ctrl->wakeup.load(std::memory_order_acquire);
}

std::uint32_t Channel::wake_seq() const noexcept
{
  return sh_->wake_seq();
}

void ChannelShared::add_waiter() noexcept
{
  ctrl->waiters.fetch_add(1, std::memory_order_seq_cst);
}

void ChannelShared::remove_waiter() noexcept
{
  ctrl->waiters.fetch_sub(1, std::memory_order_seq_cst);
}

void Channel::add_waiter() noexcept
{
  sh_->add_waiter();
}

void Channel::remove_waiter() noexcept
{
  sh_->remove_waiter();
}

bool Channel::wait(std::uint32_t last_seq, std::int64_t timeout_ns) noexcept
{
  return sh_->wait(last_seq, timeout_ns);
}

bool ChannelShared::wait(std::uint32_t last_seq, std::int64_t timeout_ns) noexcept
{
  add_waiter();  // announce before observing (seq_cst); Dekkers with publish's wake gate
  bool woken = true;
  // Re-observe under seq_cst: if the generation already moved, do not sleep. Otherwise
  // FUTEX_WAIT re-checks the word in-kernel and only parks if it still equals last_seq.
  if (ctrl->wakeup.load(std::memory_order_seq_cst) == last_seq) {
    struct timespec ts;
    const struct timespec * pto = nullptr;
    if (timeout_ns >= 0) {
      ts = ns_to_timespec(timeout_ns);
      pto = &ts;
    }
    const long r = futex_wait(&ctrl->wakeup, last_seq, pto);
    if (r != 0 && errno == ETIMEDOUT) woken = false;
  }
  remove_waiter();
  return woken;
}

void Channel::qos(const QoS & q)
{
  q.validate();
  qos_ = q;
  position_cursor();
}

FrameView Channel::take_blocking(std::int64_t timeout_ns) noexcept
{
  const bool infinite = timeout_ns < 0;
  const std::int64_t deadline = infinite ? 0 : monotonic_ns() + timeout_ns;
  for (;;) {
    const std::uint32_t seq = wake_seq();  // sample generation, then check data
    if (FrameView v = take()) return v;
    // Only the caller can free a lease, and it cannot while parked here -- waiting would sleep
    // through every publish and never return.
    if (!can_borrow()) return FrameView{};
    std::int64_t rel = -1;
    if (!infinite) {
      rel = deadline - monotonic_ns();
      if (rel <= 0) return FrameView{};
    }
    wait(seq, (rel < 0 || rel > kMaxParkNs) ? kMaxParkNs : rel);
  }
}

bool Channel::begin_borrow() noexcept
{
  maybe_reattach();  // stalled? the segment may have been unlinked and recreated by a restart
  if (sh_->outstanding.fetch_add(1, std::memory_order_acq_rel) >= qos_.max_borrow) {
    sh_->outstanding.fetch_sub(1, std::memory_order_release);
    // A leaked lease is never returned, so once leaks alone fill the budget the caller cannot
    // recover by releasing anything. Reporting that as max_borrow points at the wrong knob.
    if (sh_->leases_leaked.load(std::memory_order_acquire) >= qos_.max_borrow) {
      ++refused_.fence;
    } else {
      ++refused_.max_borrow;
    }
    return false;
  }
  // Lock first, mark later.
  try {
    OwnerFile::ensure();
  } catch (...) {
    sh_->outstanding.fetch_sub(1, std::memory_order_release);
    ++refused_.no_owner_file;
    return false;
  }
  return true;
}

FrameView Channel::peek() noexcept
{
  if (!begin_borrow()) return FrameView{};

  constexpr int kMaxRetries = 64;
  int attempt = 0;
  for (; attempt < kMaxRetries; ++attempt) {
    if (sh_->ctrl->init_state.load(std::memory_order_acquire) != kInitReady) {
      ++refused_.not_ready;
      break;
    }
    const std::uint64_t l = sh_->ctrl->latest.load(std::memory_order_acquire);  // newest
    if (l == 0) break;  // nothing published yet
    const std::uint32_t s = latest_slot(l);
    // A publisher only ever packs a slot it selected, so this holds for any frame flux wrote.
    // Check it anyway: the value comes from another process, and unlike the take() scan (bounded
    // by slot_count) this indexes straight off it -- an out-of-range slot would put the borrow's
    // refcount++ outside the mapping. Same reason the meta bounds are rechecked below.
    if (s >= sh_->layout.slot_count) {
      ++refused_.bad_frame;
      break;
    }
    SlotHeader * sl = slot(s);

    sl->refcount.fetch_add(1, std::memory_order_seq_cst);              // borrow
    const std::uint64_t s1 = sl->seq.load(std::memory_order_seq_cst);  // check seq
    if (s1 & 1u) {                                                     // write in progress
      sl->refcount.fetch_sub(1, std::memory_order_release);
      continue;
    }
    const std::uint64_t s2 = sl->seq.load(std::memory_order_acquire);  // validate
    if (s1 != s2) {                                                    // changed under us
      sl->refcount.fetch_sub(1, std::memory_order_release);
      continue;
    }
    // Validated (refcount held, seq stable even). Guard the byte bound before handing out a
    // view: a corrupt nbytes/ndim (rogue writer, memory corruption) must not let a consumer
    // read past the slot's payload region. publish() enforces this on write; recheck on read.
    const std::size_t nbytes = sl->meta.nbytes;
    if (nbytes > sh_->layout.slot_size || sl->meta.ndim > kMaxDims) {
      sl->refcount.fetch_sub(1, std::memory_order_release);
      ++refused_.bad_frame;
      break;  // out-of-bounds meta: drop rather than hand out a view that reads past the slot
    }
    // A claim that was aborted or remotely recovered leaves the slot with no valid frame while
    // `latest` may still name it. commit_ticket == 0 is that state.
    const std::uint64_t seen = sl->commit_ticket;
    if (seen == 0) {
      sl->refcount.fetch_sub(1, std::memory_order_release);
      ++refused_.bad_frame;
      break;
    }
    // Stamp holder identity for crash cleanup AFTER refcount++ (safe ordering).
    // If the holder table is full we cannot track this borrow, so back it
    // out rather than hold an untracked refcount.
    SlotHolder * h = holder_acquire(static_cast<std::uint32_t>(s));
    if (h == nullptr) {
      sl->refcount.fetch_sub(1, std::memory_order_release);
      ++refused_.holder_table;
      break;
    }
    note_stall(seen != last_seen_ticket_);  // same ticket again => no new frame
    last_seen_ticket_ = seen;
    return FrameView{sh_, sl, h, payload(static_cast<std::uint32_t>(s)), nbytes};
  }
  if (attempt == kMaxRetries) ++refused_.contended;
  sh_->outstanding.fetch_sub(1, std::memory_order_release);  // no view returned: release the lease
  note_stall(false);
  return FrameView{};
}

void Channel::apply_memory_policy()
{
  if (mem_.none()) return;
  if (mem_.lock) {
    sh_->seg.lock_pages();  // populates what it locks, so the commit comes with it
    return;
  }
  sh_->seg.commit_pages();
}

bool Channel::reattach_if_replaced() noexcept
{
  if (signpost_name_.empty()) return false;          // publisher or heap: no signpost to watch
  if (sh_->seg.holds_liveness_lock()) return false;  // publisher: nobody can unlink under us
  if (sh_->outstanding.load(std::memory_order_acquire) != 0)
    return false;  // a view still aliases us
  // A rotation (publisher restart) bumps the signpost epoch. Same epoch => same current segment.
  // Read it off the mapping this consumer holds: signpost_epoch() reopens and remaps the object
  // (discovery.hpp says as much), which turned every probe into a syscall run even though the
  // page was already here. The reopen is the fallback for a consumer that never got a mapping.
  if (!signpost_.valid()) signpost_ = SignpostView(signpost_name_);
  std::uint32_t ep = 0;
  if (signpost_.valid()) {
    if (!signpost_.epoch(ep)) return false;  // torn read: look again on the next stall
  } else {
    ep = signpost_epoch(signpost_name_);
  }
  if (ep == signpost_epoch_) {
    // Still the advertised current, yet starved: if its publisher group is dead no rotation is
    // coming, and this mapping pins a dead stream. Flag it for the owner.
    if (!orphaned_) orphaned_ = segment_publishers_dead(sh_->seg.name());
    return false;
  }

  const std::uint64_t fp = sh_->ctrl->fingerprint;
  try {
    // A fresh block, not a mutation: a helper thread parked on the old wake word still holds a
    // shared_ptr to it, so the old mapping must stay valid until that thread lets go.
    std::uint32_t new_epoch = 0;
    Segment seg = open_subscriber_segment(signpost_name_, fp, &new_epoch, device_of(sh_->stream));
    // The declaration belongs to this consumer, not to the segment it happens to be attached to,
    // so it survives a rotation. fence_failed carries too: it counts slots that will never be
    // reused, which a fresh segment does not undo.
    const gpu::Stream st = sh_->stream;
    const std::uint64_t failed = sh_->fence_failed.load(std::memory_order_relaxed);
    sh_ = std::make_shared<ChannelShared>(std::move(seg), st);
    // The replacement is a different mapping, so it carries none of the old registration. Inside
    // the try: a refusal here leaves the re-attach unfinished rather than half-done, and the
    // caller retries on a later poll exactly as it does for a segment that is not ready yet.
    sh_->register_host_payload();
    sh_->fence_failed.store(failed, std::memory_order_relaxed);
    // The replacement carries none of the old mapping's residency either. Inside the try for the
    // same reason register_host_payload is: a refusal leaves the re-attach unfinished and the
    // caller retries, rather than running on a mapping whose declared bound does not hold.
    apply_memory_policy();
    signpost_epoch_ = new_epoch;
  } catch (...) {
    return false;  // the replacement is not ready yet; try again on a later poll
  }
  position_cursor();  // the new stream restarts its tickets
  lost_ = 0;          // and its own loss count
  orphaned_ = false;  // attached to a live current again
  ++attach_gen_;
  return true;
}

void Channel::note_stall(bool progressed) noexcept
{
  if (progressed) {
    stalled_polls_ = 0;
    return;
  }
  ++stalled_polls_;
}

bool Channel::rotation_seen() const noexcept
{
  std::uint32_t ep = 0;
  return signpost_.valid() && signpost_.epoch(ep) && ep != signpost_epoch_;
}

void Channel::maybe_reattach() noexcept
{
  if (stalled_polls_ == 0) return;  // frames are flowing: leave the delivering path bare

  // A rotation is one seqlock read of a page this Channel already holds, so every stalled take
  // can check it -- a publisher restart is picked up on the next take rather than after a run of
  // them. The syscall-heavy paths below (orphan probe, and reopening the signpost when it could
  // not be mapped) stay amortized over that run.
  if (rotation_seen()) {
    if (reattach_if_replaced()) {
      last_seen_ticket_ = 0;
      stalled_polls_ = 0;
    }
    return;
  }
  constexpr std::uint32_t kProbeEvery = 8;
  if (stalled_polls_ < kProbeEvery) return;
  stalled_polls_ = 0;
  if (reattach_if_replaced()) last_seen_ticket_ = 0;
}

bool Channel::ready() const noexcept
{
  if (sh_ == nullptr) return false;
  if (sh_->ctrl->init_state.load(std::memory_order_acquire) != kInitReady) return false;
  const std::uint64_t l = sh_->ctrl->latest.load(std::memory_order_acquire);
  return l != 0 && latest_ticket(l) > cursor_;
}

FrameView Channel::take() noexcept
{
  if (!begin_borrow()) return FrameView{};

  const std::uint32_t n = sh_->layout.slot_count;
  constexpr int kMaxRetries = 64;
  int attempt = 0;
  for (; attempt < kMaxRetries; ++attempt) {
    if (sh_->ctrl->init_state.load(std::memory_order_acquire) != kInitReady) {
      ++refused_.not_ready;
      break;
    }
    const std::uint64_t latest_now = sh_->ctrl->latest.load(std::memory_order_acquire);
    if (latest_now == 0) break;  // nothing published yet
    const std::uint64_t newest = latest_ticket(latest_now);
    const std::uint64_t floor = (newest > qos_.depth) ? (newest - qos_.depth) : 0;
    if (cursor_ < floor) {
      // Count the skip HERE, not on the delivery path: this take may still return nothing (no
      // candidate, retries exhausted, holder table full), and the frames the window dropped
      // would then vanish from lost() -- the next call restarts from the advanced cursor.
      lost_ += floor - cursor_;
      cursor_ = floor;  // depth == 1 pins it one back: newest only
    }

    // Scan for the oldest committed frame newer than the cursor: min commit_ticket > cursor_.
    // Each slot is read as a seqlock (seq even + unchanged across the read) so its ticket is
    // coherent; a slot being written (odd) or never committed (0) is skipped.
    std::uint32_t best_s = n;
    std::uint64_t best_t = ~std::uint64_t{0};
    for (std::uint32_t s = 0; s < n; ++s) {
      SlotHeader * sl = slot(s);
      const std::uint64_t sq0 = sl->seq.load(std::memory_order_acquire);
      if ((sq0 & 1u) || sq0 == 0) continue;
      const std::uint64_t t = sl->commit_ticket;
      // The fence pins the plain read, which the validating load cannot: acquire only stops
      // later accesses from moving up, so an earlier plain load may sink past it (signpost_read).
      std::atomic_thread_fence(std::memory_order_acquire);
      if (sl->seq.load(std::memory_order_relaxed) != sq0)
        continue;  // torn: not a stable generation
      if (t > cursor_ && t < best_t) {
        best_t = t;
        best_s = s;
      }
    }
    // Nothing newer than the cursor right now. Never infer a restart from the tickets: a commit
    // closes its seqlock before it advances `latest`, so this consumer's cursor can legitimately
    // sit above `latest` for a moment, and rewinding on that redelivers a frame it already had.
    // A restart is not this segment's business at all: it produces a new segment, and the signpost
    // epoch announces it to maybe_reattach().
    if (best_s == n) break;

    // Borrow the chosen slot with the standard seqlock/Dekker validation, tied to
    // best_t: if the slot was overwritten between the scan and the borrow, its ticket no longer
    // matches -> retry.
    SlotHeader * sl = slot(best_s);
    sl->refcount.fetch_add(1, std::memory_order_seq_cst);
    const std::uint64_t s1 = sl->seq.load(std::memory_order_seq_cst);
    if (s1 & 1u) {
      sl->refcount.fetch_sub(1, std::memory_order_release);
      continue;
    }
    const std::uint64_t t = sl->commit_ticket;
    std::atomic_thread_fence(std::memory_order_acquire);  // pins the plain read, as in the scan
    const std::uint64_t s2 = sl->seq.load(std::memory_order_relaxed);
    if (s1 != s2 || t != best_t || t <= cursor_) {
      sl->refcount.fetch_sub(1, std::memory_order_release);
      continue;
    }
    const std::size_t nbytes = sl->meta.nbytes;
    if (nbytes > sh_->layout.slot_size || sl->meta.ndim > kMaxDims) {
      sl->refcount.fetch_sub(1, std::memory_order_release);
      ++refused_.bad_frame;
      break;
    }
    SlotHolder * h = holder_acquire(best_s);
    if (h == nullptr) {
      sl->refcount.fetch_sub(1, std::memory_order_release);
      ++refused_.holder_table;
      break;
    }
    if (t > cursor_ + 1) lost_ += t - cursor_ - 1;  // lapped by the ring
    cursor_ = t;
    note_stall(true);  // take() only returns a frame newer than the cursor
    last_seen_ticket_ = t;
    return FrameView{sh_, sl, h, payload(best_s), nbytes};
  }
  if (attempt == kMaxRetries) ++refused_.contended;
  sh_->outstanding.fetch_sub(1, std::memory_order_release);
  note_stall(false);
  return FrameView{};
}

}  // namespace flux
