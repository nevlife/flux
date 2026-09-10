#ifndef FLUX_CHANNEL_HPP
#define FLUX_CHANNEL_HPP

#include "flux/discovery.hpp"  // rendezvous: open_publisher_segment / open_subscriber_segment
#include "flux/gpu_platform.hpp"
#include "flux/memory.hpp"
#include "flux/owner.hpp"
#include "flux/qos.hpp"
#include "flux/segment.hpp"
#include "flux/segment_layout.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <string>

namespace flux
{

class Channel;

// Outcome of a publish. Backpressure is the only transient one and the only one that bumps
// dropped(); the rest are wiring mistakes or a fault, and a caller that treats them as a rate
// never finds them. The publish path derives dtype and shape from its arguments, so a
// self-contradicting descriptor is not among the outcomes -- it cannot be built.
enum class Published : std::uint8_t {
  Ok,
  Backpressure,  // every slot is borrowed; dropped()++
  TooLarge,      // the frame does not fit the slot or kMaxDims, or this handle was already spent
  WrongDevice,   // a host publish into a device-backed channel; loan() is the path there
  FenceFailed,   // the declared stream could not be waited on; fence_failed()++
};

// One question instead of five. Backpressure is a rate: the ring was full, the frame was dropped,
// and delivery is best-effort by design. Everything else is a fault that does not clear on its
// own -- FenceFailed leaks the slot for good, and the other two describe a frame that can never
// go out as written. A caller that logs on this and ignores the rest has covered the enum.
constexpr bool faulted(Published p) noexcept
{
  return p != Published::Ok && p != Published::Backpressure;
}

// Stable spelling for a log line. Same words the Python enum prints.
const char * to_string(Published p) noexcept;

// The mapping, and the per-consumer state a handed-out view still needs after the Channel that
// issued it is gone. A FrameView/WriteSlot keeps a shared_ptr to this, so the segment stays
// mapped for as long as anything points into it.
struct ChannelShared
{
  explicit ChannelShared(Segment s, gpu::Stream st = {}) noexcept
  : seg(std::move(s)),
    base(seg.base()),
    payload_base(seg.payload_base()),
    ctrl(seg.ctrl()),
    layout(seg.layout()),
    device_payload(seg.device_backed()),
    stream(st)
  {
  }

  // Which of the two seams a fence closes. They block different threads
  // -- commit blocks whoever publishes, release blocks whoever drops the view -- so their waits
  // are accumulated apart.
  enum class Seam : std::uint8_t { Commit, Release };

  // Wait for the declared stream before a caller acts on "the GPU is done".
  // An undeclared stream succeeds without touching CUDA. False means the wait did not happen,
  // and every caller of this treats that as "still running".
  bool fence(Seam seam) noexcept;

  SlotHeader * slot(std::uint32_t i) noexcept
  {
    return reinterpret_cast<SlotHeader *>(base + layout.slots_offset() + i * layout.slot_stride());
  }
  const SlotHeader * slot(std::uint32_t i) const noexcept
  {
    return reinterpret_cast<const SlotHeader *>(
      base + layout.slots_offset() + i * layout.slot_stride());
  }
  // Slot i's payload. payload_base is inside this mapping on the host path and in a device
  // allocation on the dGPU one; the arithmetic is the same, which is the whole reason the
  // slot protocol did not have to change.
  void * payload(std::uint32_t i) noexcept { return payload_base + i * layout.payload_stride(); }
  SlotHolders * holders(std::uint32_t i) noexcept
  {
    return reinterpret_cast<SlotHolders *>(
      base + layout.holders_offset() + i * layout.holder_stride());
  }

  // A payload address as a kernel names it. Identity on ShmDirect (the host address already is
  // the device address) and on DeviceHandle (payload_base is device memory to begin with); a
  // rebase onto the registered region on ShmRegistered, where the two differ.
  // Callers hand it host_addressable() addresses only.
  const void * to_device(const void * host) const noexcept
  {
    if (device_payload_base == nullptr) return host;
    return device_payload_base + (static_cast<const std::byte *>(host) - payload_base);
  }
  void * to_device(void * host) noexcept
  {
    if (device_payload_base == nullptr) return host;
    return device_payload_base + (static_cast<std::byte *>(host) - payload_base);
  }

  // Make this mapping's payload reachable from a kernel when the host asks for registration
  // (Route::ShmRegistered). Does nothing on an undeclared stream, on a device-backed payload, or
  // on a route that needs no registration. Throws when the driver refuses -- a channel that told
  // its caller a kernel can reach the payload must not go on without that being true.
  void register_host_payload();

  // End a borrow: holder count-- (before refcount--), refcount--, and free the max_borrow lease.
  void end_borrow(SlotHolder * holder, SlotHeader * slot) noexcept;

  // The commit steps shared by publish() and WriteSlot::commit().
  void finish_commit(SlotHeader * sl, std::uint32_t s, std::uint64_t even) noexcept;

  // Wake primitives live here, not on Channel: a fallback wait layer parks a helper thread on
  // this word, and holding a shared_ptr to this block is what keeps the word mapped while it
  // does. A re-attach swaps the Channel's block for a new one rather than mutating this one.
  std::uint32_t wake_seq() const noexcept;
  std::atomic<std::uint32_t> * wake_word() noexcept { return &ctrl->wakeup; }
  void add_waiter() noexcept;
  void remove_waiter() noexcept;
  bool wait(std::uint32_t last_seq, std::int64_t timeout_ns) noexcept;

  Segment seg;
  std::byte * base = nullptr;
  std::byte * payload_base = nullptr;
  ControlHeader * ctrl = nullptr;
  SegmentLayout layout{};
  bool device_payload = false;  // payload is device memory: no host store may touch it
  // Route::ShmRegistered only: the registration that makes the payload mapping device-addressable
  // and the device address it resolved to. Held here so it outlives the Channel exactly as the
  // mapping does -- a FrameView still in a caller's hands may yet be asked for a device pointer.
  gpu::HostRegistration host_registration;
  std::byte * device_payload_base = nullptr;
  std::atomic<std::uint64_t> dropped{0};
  std::atomic<std::uint32_t> outstanding{0};  // views this consumer currently holds
  gpu::Stream stream;                         // process-local; undeclared = host-only
  std::atomic<std::uint64_t> fence_failed{0};
  // How long the seams actually block, per seam, as sum + count + worst. The
  // deferred-commit upgrade is gated on this number and nothing else reports it -- fence_failed
  // counts failures, not waits. Only a declared stream is clocked, so a Cpu channel pays nothing.
  std::atomic<std::uint64_t> commit_wait_ns{0};
  std::atomic<std::uint64_t> commit_waits{0};
  std::atomic<std::uint64_t> commit_wait_max_ns{0};
  std::atomic<std::uint64_t> release_wait_ns{0};
  std::atomic<std::uint64_t> release_waits{0};
  std::atomic<std::uint64_t> release_wait_max_ns{0};
  // Of `outstanding`, the part a failed release fence left behind. A leaked lease is never
  // returned, so once this reaches max_borrow the consumer can no longer borrow at all -- and
  // that is a different refusal from a caller simply holding its views (Refused::fence).
  std::atomic<std::uint32_t> leases_leaked{0};
};

// RAII borrow of a committed frame. Zero-copy: data() points into the segment.
// While a FrameView is alive it holds the borrow (refcount), so the publisher will
// not overwrite the slot -- this type IS the "validated holder", and its lifetime
// is the window in which active => byte-locked holds. Move-only.
class FrameView
{
public:
  FrameView() = default;
  ~FrameView();
  FrameView(FrameView && other) noexcept;
  FrameView & operator=(FrameView && other) noexcept;
  FrameView(const FrameView &) = delete;
  FrameView & operator=(const FrameView &) = delete;

  bool valid() const noexcept { return slot_ != nullptr; }
  explicit operator bool() const noexcept { return valid(); }

  // The address ordinary host loads may read, or null when the payload is not one. Null
  // rather than the device address is what makes a host-only reader -- a flux_gen adapter, or
  // anything else over flux/wire.hpp -- fail where it is constructed rather than at its first
  // load: both wire::Reader and wire::Writer latch bad() on a null base. device_ptr() is the
  // address on that channel.
  const void * data() const noexcept { return host_addressable() ? data_ : nullptr; }
  std::size_t size() const noexcept { return size_; }
  const FrameMeta & meta() const noexcept;  // requires valid()

  // The address a kernel reads. Null unless the channel declared a
  // stream: a consumer that never said it would use the GPU has no fence, and handing it a
  // usable device pointer would let it read a slot the publisher is free to overwrite.
  const void * device_ptr() const noexcept;

  // True when data() may be read by ordinary host loads. False on a channel whose payload is a
  // device allocation: there the address is GPU memory, and a host read of it faults or
  // returns nothing. A binding that builds a host-side view (numpy) must ask this first --
  // device_ptr() being non-null does not answer it, because an integrated GPU declares a stream
  // over a slot that is host memory as well.
  bool host_addressable() const noexcept;

  // The channel's declared stream. Kernels are launched on this rather than on one the caller
  // keeps, so the stream release() waits on and the stream the work went to cannot diverge.
  gpu::Stream stream() const noexcept;

  // Ends the borrow. With a declared stream the wait happens first, so the publisher never
  // regains the slot while a kernel still reads it. If that wait fails the borrow is deliberately
  // left standing (Channel::fence_failed) -- releasing on an unverified fence is the corruption
  // this exists to prevent.
  void release() noexcept;

private:
  friend class Channel;
  FrameView(
    std::shared_ptr<ChannelShared> sh, SlotHeader * slot, SlotHolder * holder, const void * data,
    std::size_t size) noexcept
  : sh_(std::move(sh)), slot_(slot), holder_(holder), data_(data), size_(size)
  {
  }

  std::shared_ptr<ChannelShared> sh_;  // keeps the mapping alive for this view's whole lifetime
  SlotHeader * slot_ = nullptr;        // the borrowed slot (its refcount is held while this lives)
  SlotHolder * holder_ = nullptr;  // this process's holder entry for the slot (crash-cleanup id)
  const void * data_ = nullptr;
  std::size_t size_ = 0;
};

// 0-copy publish handle: the write-side dual of FrameView (split into
// loan/commit). The producer writes the payload straight into data(); commit() publishes it.
// If the handle is dropped without commit the claim is reverted -- but data() aliases the
// slot, so whatever frame it held is gone either way; abort only guarantees nothing is
// published. Move-only.
class WriteSlot
{
public:
  WriteSlot() = default;
  ~WriteSlot();
  WriteSlot(WriteSlot && other) noexcept;
  WriteSlot & operator=(WriteSlot && other) noexcept;
  WriteSlot(const WriteSlot &) = delete;
  WriteSlot & operator=(const WriteSlot &) = delete;

  bool valid() const noexcept { return slot_ != nullptr; }
  explicit operator bool() const noexcept { return valid(); }

  // Write up to capacity() bytes here. Null on a device-backed channel, for the same reason as
  // FrameView::data(): a host store into a device allocation is not a slow path, it is a fault,
  // and a null base is what turns a wire::Writer built over it into ok() == false.
  void * data() noexcept { return host_addressable() ? data_ : nullptr; }
  std::size_t capacity() const noexcept { return cap_; }

  // The address a kernel writes, and the stream to launch it on.
  // Null unless the channel declared a stream, for the same reason as FrameView::device_ptr.
  void * device_ptr() noexcept;

  // True when data() may be written by ordinary host stores. False on a device-backed channel,
  // for the same reason as FrameView::host_addressable().
  bool host_addressable() const noexcept;

  gpu::Stream stream() const noexcept;

  // Publish the bytes written into data() as the dtype and shape this loan was taken for: commit
  // the seqlock, then advance latest and wake subscribers. Consumes the handle.
  //
  // With a declared stream the wait happens before the seqlock closes, so "committed" keeps
  // meaning "complete" for a GPU payload exactly as it does for a memcpy. A failed wait consumes
  // the handle without committing and without aborting: aborting would release the claim while
  // the producing kernel may still be writing, so the slot is kept claimed and never reused.
  Published commit() noexcept;

  // Publish only the first nbytes of the loan. Valid on a 1-D loan, where a shorter byte count is
  // the same statement as a shorter shape -- shape[0] is recomputed, never restated. A generated
  // adapter loans the whole slot and commits the prefix it actually built. TooLarge when the loan
  // is not 1-D, when nbytes exceeds it, or when nbytes is not a multiple of itemsize.
  Published commit(std::size_t nbytes) noexcept;

  // Revert the claim without publishing. The frame the slot held is NOT restored -- data()
  // aliased it -- but it is dropped rather than delivered with its old meta. Consumes the handle.
  void abort() noexcept;

private:
  friend class Channel;
  WriteSlot(
    std::shared_ptr<ChannelShared> sh, SlotHeader * slot, std::uint32_t index, std::uint64_t even,
    void * data, std::size_t cap, const FrameMeta & meta) noexcept
  : sh_(std::move(sh)), slot_(slot), index_(index), even_(even), data_(data), cap_(cap), meta_(meta)
  {
  }

  std::shared_ptr<ChannelShared> sh_;  // keeps the mapping alive until commit/abort
  SlotHeader * slot_ = nullptr;
  std::uint32_t index_ = 0;
  std::uint64_t even_ = 0;  // stable generation the claim advanced from (for commit/abort)
  void * data_ = nullptr;
  std::size_t cap_ = 0;
  // Stated once, at loan(). commit() has nothing left to describe, so there is no second
  // statement of the shape to disagree with this one.
  FrameMeta meta_{};
};

// A declared stream is what makes a channel a GPU channel, so the Device a segment is opened with
// is read off the stream rather than passed beside it -- two ways to say the same thing could
// disagree.
inline Device device_of(const gpu::Stream & stream) noexcept
{
  return stream.declared() ? Device::Cuda : Device::Cpu;
}

// Multi-publisher, multi-subscriber best-effort channel over a Segment. The same protocol
// runs over a heap-backed Segment (tests) or a POSIX shm one (cross-process).
// What a consumer sees is its QoS (docs/en/qos.en.md), set through qos().
//
// ONE THREAD PER Channel OBJECT. Many processes, and many Channel objects within a process, may
// share a segment -- that is what the segment protocol is for. A single object is not shared:
// cursor_, lost_, refused_, the stall counters and the re-attach state are plain members, and
// sh_ (the whole mapping) is swapped wholesale by a re-attach. Two threads on one object is not
// a benign counter race -- take() double-delivers frames it is supposed to consume, and a
// re-attach can pull the mapping out from under the other thread's slot pointer. The atomics on
// ChannelShared cover the cross-process protocol, not this. Give each thread its own Channel
// (docs/en/api.en.md, the threading rules under the executor section), which costs one mapping.
class Channel
{
public:
  // Heap-backed convenience (process-local; for tests).
  Channel(std::uint32_t slot_size, std::uint32_t slot_count)
  : Channel(Segment::create_heap(slot_size, slot_count))
  {
  }

  // Takes ownership of an already-built Segment.
  explicit Channel(Segment segment) noexcept;

  // Declares that this channel's payloads are produced or consumed on `stream`.
  // Declaring at construction rather than through a setter is what
  // rules out a window where the channel exists and a frame can cross it unfenced.
  //
  // Throws std::invalid_argument when this host cannot serve the declaration, checked against
  // gpu::probe(): a stream that can never be waited on would turn every release into a leaked
  // slot, and a route flux has not implemented would hand kernels an address that is not theirs.
  // `mem` is opt-in page residency for this mapping; the default is a no-op.
  // Applied here rather than through a setter so no frame can cross before the bound holds.
  Channel(Segment segment, gpu::Stream stream, const MemoryPolicy & mem = {});

  Channel(const Channel &) = delete;
  Channel & operator=(const Channel &) = delete;
  Channel(Channel &&) noexcept = default;

  // Publisher: create the shm segment for `name` and publish into it. `mem` is opt-in page
  // residency for this process's mapping; the default is a no-op.
  static Channel create(
    const std::string & name, std::uint32_t slot_size, std::uint32_t slot_count,
    std::uint64_t fingerprint, gpu::Stream stream = {}, const MemoryPolicy & mem = {})
  {
    return Channel(
      open_publisher_segment(name, slot_size, slot_count, fingerprint, device_of(stream)), stream,
      mem);
  }
  // Subscriber: attach to the segment the signpost `name` advertises (validates fingerprint), and
  // record the signpost + epoch so a later publisher restart (rotation) triggers a re-attach.
  static Channel open(
    const std::string & name, std::uint64_t fingerprint, gpu::Stream stream = {},
    const MemoryPolicy & mem = {})
  {
    sweep_dead_once();  // a subscriber-only process reclaims orphans too
    std::uint32_t epoch = 0;
    Segment seg = open_subscriber_segment(name, fingerprint, &epoch, device_of(stream));
    return Channel(std::move(seg), name, epoch, stream, mem);
  }

  // Publisher: copy nbytes into the next free slot scanning forward from `latest`
  // as one u8 run. Position, not age -- the two agree except right after a slot borrowed across
  // laps is released. Backpressure
  // when every slot is currently borrowed, TooLarge when nbytes exceeds slot_size (rejected,
  // never truncated). Only Backpressure bumps the drop counter.
  //
  // Always WrongDevice on a device-backed channel: the slot is GPU memory, and copying a
  // host buffer into it needs a CUDA copy primitive flux_core deliberately does not have. loan()
  // is the publish path there.
  Published publish(const void * data, std::size_t nbytes) noexcept;

  // Same, for a frame that is not a flat byte run. The byte count is the product of the shape and
  // dtype_size(dt), so it cannot disagree with the descriptor the consumer sizes its view from --
  // which is the one inconsistency a caller-supplied FrameMeta could express.
  Published publish(
    const void * data, DType dt, const std::uint64_t * shape, std::size_t ndim) noexcept;
  Published publish(
    const void * data, DType dt, std::initializer_list<std::uint64_t> shape) noexcept;

  // Publisher 0-copy (split): reserve+claim a free slot the same way publish()
  // selects one and hand back a
  // WriteSlot to write the payload into directly (no memcpy), committed via WriteSlot::commit.
  // Returns an invalid handle (and bumps the drop counter) when every slot is borrowed.
  //
  // The no-argument form loans the whole slot as u8, which is what an adapter wants: it commits
  // the prefix it built. The dtype/shape form states the frame once, here, and its commit() then
  // takes no arguments at all.
  WriteSlot loan() noexcept;
  WriteSlot loan(DType dt, const std::uint64_t * shape, std::size_t ndim) noexcept;
  WriteSlot loan(DType dt, std::initializer_list<std::uint64_t> shape) noexcept;

  // Subscriber. peek() reads the newest frame and leaves the cursor alone, so it
  // hands out the same frame until a new one is published; take() consumes the next frame in
  // publish order. Same split as DDS read/take (docs/en/qos.en.md 1). Both give an empty view when
  // max_borrow is reached or acquisition was lost under contention.
  FrameView peek() noexcept;
  FrameView take() noexcept;

  // Whether take() would find a frame newer than the cursor, in one load rather than a scan of
  // every slot. Advisory: a lapped or refused frame makes the take that follows it empty.
  bool ready() const noexcept;

  // take(), parking on the wake word until a frame arrives or `timeout_ns` elapses (negative =
  // forever). peek has no blocking form: current state is already there.
  FrameView take_blocking(std::int64_t timeout_ns = -1) noexcept;

  // Frames never delivered to this consumer since it joined the stream: lapped by the ring, or
  // dropped by qos().depth. Cumulative, like the DDS sample-lost status -- diff it to get a rate.
  //
  // "Since it joined the stream" is literal: a re-attach joins a NEW stream with its own tickets,
  // so this restarts at 0 there while refused() carries over (that one counts this consumer's
  // refusals, which a fresh segment does not undo). A monitor differencing the two must bracket
  // the diff with attach_generation() -- across a change, lost() is a restart and not a rate.
  std::uint64_t lost() const noexcept { return lost_; }

  // Borrows this consumer did not release, and commits this publisher did not make, because the
  // declared stream could not be waited on (docs/en/qos.en.md 1). Each one
  // costs a slot that is never reused, so a nonzero value is a hard fault, not a rate. Always 0
  // on a channel with no declared stream. Cumulative, like lost().
  //
  // It stops rising once the leaked leases reach max_borrow, because from there no borrow is
  // handed out to fence at all. Read it as a state, not as a rate: the consumer is finished, and
  // refused().fence counts the calls it refuses from then on.
  std::uint64_t fence_failed() const noexcept
  {
    return sh_->fence_failed.load(std::memory_order_relaxed);
  }

  // How long the two GPU seams blocked the threads that ran them. The
  // seams are kept apart because they block different threads and are answered by different
  // things: a publisher owns the gap between kernel launch and commit, while a callback consumer
  // owns nothing -- its view dies the moment the callback returns.
  //
  // All zero on a channel with no declared stream; that channel is never clocked. Cumulative,
  // like lost(), except the two max fields, which are running worsts and never decrease.
  struct FenceWait
  {
    std::uint64_t commit_ns = 0;       // total time inside the publish seam's wait
    std::uint64_t commit_count = 0;    // waits that ran there, failed ones included
    std::uint64_t commit_max_ns = 0;   // worst single wait there
    std::uint64_t release_ns = 0;      // total time inside the release seam's wait
    std::uint64_t release_count = 0;   // waits that ran there, failed ones included
    std::uint64_t release_max_ns = 0;  // worst single wait there
  };
  FenceWait fence_wait() const noexcept
  {
    const auto load = [](const std::atomic<std::uint64_t> & a) {
      return a.load(std::memory_order_relaxed);
    };
    return FenceWait{load(sh_->commit_wait_ns),     load(sh_->commit_waits),
                     load(sh_->commit_wait_max_ns), load(sh_->release_wait_ns),
                     load(sh_->release_waits),      load(sh_->release_wait_max_ns)};
  }

  // True when this channel's payload may be touched by host loads and stores. False exactly on
  // the dGPU route, where the payload is a device allocation.
  bool host_addressable() const noexcept { return !sh_->device_payload; }

  const gpu::Stream & stream() const noexcept { return sh_->stream; }
  std::uint64_t cursor() const noexcept { return cursor_; }

  // Why peek()/take() handed back an empty view when a frame was not simply absent. "Nothing
  // published yet" and "caught up" are the empty view working, and are not counted; everything
  // here is a refusal the caller cannot see and that neither lost() nor dropped() records --
  // without it a held lease and an idle stream look identical. Cumulative, like lost().
  struct Refused
  {
    std::uint64_t max_borrow = 0;     // this consumer already holds qos().max_borrow views
    std::uint64_t holder_table = 0;   // kMaxHolders processes already hold that slot
    std::uint64_t not_ready = 0;      // segment still initializing
    std::uint64_t contended = 0;      // seqlock validation lost its whole retry budget
    std::uint64_t bad_frame = 0;      // meta outside the slot, or no committed frame in it
    std::uint64_t no_owner_file = 0;  // this process could not take its owner file
    // Failed release fences leaked max_borrow leases: this consumer is permanently finished.
    // Split from max_borrow because the answers are opposites --
    // max_borrow is undone by releasing a view, and nothing undoes this one.
    std::uint64_t fence = 0;

    std::uint64_t total() const noexcept
    {
      return max_borrow + holder_table + not_ready + contended + bad_frame + no_owner_file + fence;
    }
  };
  // Cumulative for this consumer's whole life, re-attaches included -- unlike lost(), which
  // restarts with each stream. See lost() for what that means for a monitor.
  const Refused & refused() const noexcept { return refused_; }

  // False once this consumer holds max_borrow views. A take() that fails for this reason cannot
  // be unblocked by a publish, so a blocking wait must not park on it.
  bool can_borrow() const noexcept;

  // Throws std::invalid_argument on a QoS flux cannot honour. Applies at the next take, so it may
  // be set before the publisher's segment exists. A QoS deeper than the ring is not an error --
  // the ring caps it and the shortfall surfaces in lost().
  void qos(const QoS & q);
  const QoS & qos() const noexcept { return qos_; }

  // Wake generation, bumped once per successful publish. Sample this
  // before a take() that finds nothing, then pass it to wait() so a publish racing the
  // gap is not missed (no lost wakeup).
  std::uint32_t wake_seq() const noexcept;

  // Address of the in-segment wake word, for arming an external wait layer (io_uring
  // FUTEX_WAIT) on this channel. Pair with wake_seq() as the expected value.
  std::atomic<std::uint32_t> * wake_word() noexcept { return sh_->wake_word(); }

  // The block a fallback wait layer parks on. Holding it keeps the wake word mapped even if
  // this Channel re-attaches or is destroyed; compare attach_generation() to notice the swap.
  std::shared_ptr<ChannelShared> wait_handle() const noexcept { return sh_; }

  // Bumped per re-attach. A counter rather than the wake_word() address,
  // which a fresh mapping can reuse.
  std::uint32_t attach_generation() const noexcept { return attach_gen_; }

  // A starved re-attach probe found the attached segment's publisher group dead while the
  // signpost still advertises it. No rotation will ever come from a dead
  // group, so the owner should drop this Channel and re-attach lazily instead of pinning the
  // dead segment's memory.
  bool orphaned() const noexcept { return orphaned_; }

  // Register/unregister interest so publish() knows to issue the wake syscall. The bare
  // wait() brackets itself; an external io_uring wait layer calls add_waiter() once while
  // it watches this channel and remove_waiter() when it stops. Without this the publisher
  // sees waiters==0 and skips the wake, so an io_uring FUTEX_WAIT would never be kicked.
  void add_waiter() noexcept;
  void remove_waiter() noexcept;

  // Block until the wake generation moves past `last_seq`, or `timeout_ns` elapses
  // (negative = forever). Returns true if woken/advanced, false on timeout. This is the
  // bare in-segment wait; the multiplexed io_uring wait layer is Executor (flux/executor.hpp).
  bool wait(std::uint32_t last_seq, std::int64_t timeout_ns = -1) noexcept;

  std::uint32_t slot_size() const noexcept { return sh_->layout.slot_size; }
  std::uint32_t slot_count() const noexcept { return sh_->layout.slot_count; }

  // True when this consumer's mapping has the payload region dropped to PROT_READ (shm
  // subscriber attach on a page size that divides kPayloadAlign).
  bool payload_readonly() const noexcept { return sh_->seg.payload_readonly(); }

  // What the declared MemoryPolicy actually got for this mapping. Both stay false with the
  // default policy; a policy that asked and was refused throws rather than reporting false.
  bool pages_committed() const noexcept { return sh_->seg.pages_committed(); }
  bool pages_locked() const noexcept { return sh_->seg.pages_locked(); }

  std::uint64_t dropped() const noexcept;
  std::uint32_t slot_refcount(std::uint32_t i) const noexcept;  // introspection for tests

private:
  friend class FrameView;
  friend class WriteSlot;

  // Subscriber attach that also records the signpost name + epoch it attached at, so a later
  // rotation (publisher restart) can be detected and re-attached.
  Channel(
    Segment segment, std::string signpost_name, std::uint32_t signpost_epoch,
    gpu::Stream stream = {}, const MemoryPolicy & mem = {});

  // Prologue of peek()/take(): reattach probe, max_borrow lease, owner file. On success the
  // caller holds a lease and must release it on every path that returns no view.
  bool begin_borrow() noexcept;

  // Place the cursor for the stream as it stands now: durability decides how far back. Run when
  // this consumer joins the stream (construction, qos(), re-attach) rather than at the first
  // take, so "volatile" means from where this consumer joined, not from where it first got
  // around to reading.
  void position_cursor() noexcept;

  SlotHeader * slot(std::uint32_t i) noexcept { return sh_->slot(i); }
  const SlotHeader * slot(std::uint32_t i) const noexcept { return sh_->slot(i); }
  void * payload(std::uint32_t i) noexcept { return sh_->payload(i); }
  SlotHolders * holders(std::uint32_t i) noexcept { return sh_->holders(i); }

  // Register this process as a holder of slot s: claim/increment its holder entry (crash
  // cleanup). Returns the entry, or nullptr if the table is full. Called
  // after refcount++ so a crash leaves count <= the refcount contribution (safe ordering).
  SlotHolder * holder_acquire(std::uint32_t s) noexcept;

  // Subscriber re-attach after a rotation. Swaps segments only when no
  // view of ours is outstanding -- a live FrameView aliases the old mapping and must not
  // dangle. Publishers never re-attach: they hold the liveness lock.
  bool reattach_if_replaced() noexcept;
  // Run at the start of a take (no lease held there, so a swap is safe). "Stalled" means a run
  // of takes that produced no *new* frame -- either empty, or the same ticket again, which is
  // what a subscriber left on an unlinked segment sees forever.
  void maybe_reattach() noexcept;
  // Has the mapped signpost moved off the epoch this consumer attached at? One seqlock read of
  // a page this Channel holds, so a stalled take can afford it every time. False on a torn read
  // or no mapping -- the caller looks again rather than treating it as a change.
  bool rotation_seen() const noexcept;
  void note_stall(bool progressed) noexcept;

  // Reclaim slots held by dead subscribers: for each holder whose owner is dead (OFD probe),
  // subtract its count from the slot refcount and free the entry. Run by
  // the publisher when starved. Returns true if any refcount was reclaimed.
  // The single publish body. Both public forms build the descriptor from their own arguments and
  // hand it here, so slot selection and the seqlock protocol exist once.
  Published publish_meta(const void * data, const FrameMeta & meta) noexcept;

  bool reclaim_dead() noexcept;

  // Apply mem_ to the current mapping. Re-run after a re-attach: the replacement is a different
  // mapping and carries none of the old one's residency.
  void apply_memory_policy();

  // Stamp this publisher's identity into a freshly claimed slot (seq == odd), so a peer can
  // recover the slot if this publisher crashes before commit/abort.
  void stamp_writer(SlotHeader * sl, std::uint64_t odd) noexcept;

  // Recover slots left claimed (seq odd) by a crashed publisher: for each odd slot whose stamped
  // writer is dead (OFD probe), CAS the seq back to its pre-claim even generation. Run by the
  // publisher when starved (alongside reclaim_dead). Returns true if any slot was recovered.
  bool recover_stuck_writes() noexcept;

  std::shared_ptr<ChannelShared> sh_;
  QoS qos_{};                           // consumer QoS (docs/en/qos.en.md)
  MemoryPolicy mem_{};                  // page residency, re-applied on every attach
  std::uint64_t cursor_ = 0;            // last ticket take() returned
  std::uint64_t lost_ = 0;              // frames never delivered (cumulative)
  std::uint32_t stalled_polls_ = 0;     // takes with no new frame since the last probe
  std::uint32_t attach_gen_ = 0;        // bumped per re-attach; see attach_generation()
  std::string signpost_name_;           // subscriber: fixed signpost name, for re-attach probes
  SignpostView signpost_;               // held mapping of it, so rotation checks cost no syscall
  std::uint32_t signpost_epoch_ = 0;    // signpost epoch attached at; a change means a rotation
  bool orphaned_ = false;               // publisher group dead, no rotation coming; see orphaned()
  std::uint64_t last_seen_ticket_ = 0;  // ticket of the last frame take() handed out
  Refused refused_{};                   // see refused(); plain members, only touched on refusal
  OwnerId writer_id_{};                 // this publisher's identity, stamped on claim
  bool writer_ready_ = false;           // owner file ensured + writer_id_ cached
};

}  // namespace flux

#endif  // FLUX_CHANNEL_HPP
