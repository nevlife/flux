#ifndef FLUX_IO_URING_WAITER_HPP
#define FLUX_IO_URING_WAITER_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

// Merged wait over many channels' wake words with io_uring FUTEX_WAIT (6.7+): one io_uring_enter
// blocks until any armed channel is woken. Raw io_uring, no liburing.
//
// Waits are persistent: a FUTEX_WAIT stays armed until it fires, so a timed-out wait() leaves
// pending waits in place. Re-arm only the tags wait() returned. Executor drives this; merging ROS
// readiness into the ring is flux_cpp, which is why neither this file nor Executor names ROS.

namespace flux
{

// One completed FUTEX_WAIT. `res` is the CQE result: 0 = woken by FUTEX_WAKE,
// -EAGAIN = the word already moved off `expected` at submit (data pending), other
// negative = error (e.g. -EINVAL/-EOPNOTSUPP on a kernel without io_uring futex). Any
// completion means "go take() on this channel".
struct WakeEvent
{
  std::uint64_t tag;
  int res;
};

// The ring hands tags back as opaque u64s, but an executor arms events of more than one kind
// (channel wakes, a control/readiness fd). Encoding (kind, index) in the tag lets dispatch
// switch on the kind instead of reserving magic index values like ~0. The low 24 bits of the
// channel's attach generation ride along, so a completion from a wait on a replaced mapping is
// told apart from the wait armed after it.
enum class EventKind : std::uint32_t {
  channel = 0,
  control = 1,
  cancel = 2,  // the cancel op's own completion; the canceled wait completes under its own tag
};

constexpr std::uint64_t make_tag(
  EventKind kind, std::uint32_t index, std::uint32_t gen = 0) noexcept
{
  return (static_cast<std::uint64_t>(kind) << 56) |
         (static_cast<std::uint64_t>(gen & 0xFFFFFFu) << 32) | index;
}

constexpr EventKind tag_kind(std::uint64_t tag) noexcept
{
  return static_cast<EventKind>(tag >> 56);
}

constexpr std::uint32_t tag_gen(std::uint64_t tag) noexcept
{
  return static_cast<std::uint32_t>(tag >> 32) & 0xFFFFFFu;
}

constexpr std::uint32_t tag_index(std::uint64_t tag) noexcept
{
  return static_cast<std::uint32_t>(tag);
}

// Why a merged wait is or is not available here. A missing opcode or a forbidding host is what the
// parker-thread fallback is for; a ring that could not be created or run is resource exhaustion,
// and degrading to threads would hide it.
enum class IoUringSupport : std::uint8_t {
  Yes,
  NoOpcode,    // pre-6.7 kernel or UAPI headers, or no io_uring at all: fall back
  Forbidden,   // EPERM: a seccomp profile or kernel.io_uring_disabled; fall back, and say so
  RingFailed,  // the probe ring could not be created or run: the caller reports, never degrades
};

class IoUringWaiter
{
public:
  // Probe this host once before relying on a waiter; construction alone does not guarantee the
  // opcode is accepted.
  static IoUringSupport support() noexcept;
  static bool supported() noexcept { return support() == IoUringSupport::Yes; }

  explicit IoUringWaiter(unsigned entries = 64);
  ~IoUringWaiter();
  IoUringWaiter(IoUringWaiter && other) noexcept;
  IoUringWaiter & operator=(IoUringWaiter && other) noexcept;
  IoUringWaiter(const IoUringWaiter &) = delete;
  IoUringWaiter & operator=(const IoUringWaiter &) = delete;

  bool valid() const noexcept { return ring_fd_ >= 0; }

  // Arm a one-shot FUTEX_WAIT on `word`, expecting it still equals `expected`. `tag`
  // identifies the channel and comes back from wait(). If *word already != expected the
  // kernel completes it at submit time (data pending) -- still reported by wait().
  void arm(std::atomic<std::uint32_t> * word, std::uint32_t expected, std::uint64_t tag);

  // Arm a one-shot POLLIN wait on `fd`, so the same ring merges channel wakes with an
  // external readiness fd (a ROS-side eventfd). `tag` comes back from
  // wait(). Like arm(), a poll is consumed when it fires and must be re-armed.
  void arm_poll(int fd, std::uint64_t tag);

  // Cancel the pending wait armed with `tag` (ASYNC_CANCEL by user_data). Without this a wait
  // armed on a word that will never move again -- a re-attached channel's old mapping -- stays
  // pending in the kernel for the ring's lifetime. The canceled wait completes under `tag` with
  // -ECANCELED; the cancel op itself completes under make_tag(EventKind::cancel, 0).
  void cancel(std::uint64_t tag);

  // Submit newly armed waits and block until at least one completes, or `timeout_ns`
  // elapses (negative = forever). Fills `events` (cleared first) with what fired and
  // returns the count (0 on timeout). Fired waits are consumed; re-arm() for the next
  // round. Pending (not-yet-fired) waits survive a timeout and must not be re-armed.
  int wait(std::vector<WakeEvent> & events, std::int64_t timeout_ns = -1);

private:
  void reset() noexcept;

  int ring_fd_ = -1;
  void * ring_ptr_ = nullptr;  // SQ and CQ rings, one mapping (IORING_FEAT_SINGLE_MMAP)
  std::size_t ring_sz_ = 0;
  void * sqes_ = nullptr;
  std::size_t sqes_sz_ = 0;

  // Reserve one SQ entry, flushing already-armed ones first if the ring is full. Without the
  // head the producer cannot tell a full ring from an empty one and silently overwrites.
  unsigned reserve_sqe();

  unsigned * sq_khead_ = nullptr;
  unsigned * sq_ktail_ = nullptr;
  unsigned sq_ring_mask_ = 0;
  unsigned sq_entries_ = 0;
  unsigned * sq_array_ = nullptr;

  unsigned * cq_khead_ = nullptr;
  unsigned * cq_ktail_ = nullptr;
  unsigned cq_ring_mask_ = 0;
  void * cqes_ = nullptr;

  unsigned to_submit_ = 0;
};

}  // namespace flux

#endif  // FLUX_IO_URING_WAITER_HPP
