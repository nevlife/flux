#include "flux/io_uring_waiter.hpp"

#include <linux/futex.h>
#include <linux/io_uring.h>
#include <linux/time_types.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <utility>

// Raw io_uring, no liburing: two opcodes. The ring is shared with the kernel, so head/tail use
// acquire/release like any single-producer/single-consumer ring.

// IORING_OP_FUTEX_WAIT is an enum constant of 6.7+ headers that #if cannot see, so the build
// probes for it. On an older sysroot the probe reports no opcode and the thread fallback runs.
#ifndef FLUX_HAS_IO_URING_FUTEX
#define FLUX_HAS_IO_URING_FUTEX 0
#endif

namespace flux
{

namespace
{

int sys_io_uring_setup(unsigned entries, io_uring_params * p) noexcept
{
  return static_cast<int>(::syscall(__NR_io_uring_setup, entries, p));
}

int sys_io_uring_enter(
  int fd, unsigned to_submit, unsigned min_complete, unsigned flags, void * arg,
  std::size_t argsz) noexcept
{
  return static_cast<int>(
    ::syscall(__NR_io_uring_enter, fd, to_submit, min_complete, flags, arg, argsz));
}

constexpr std::int64_t kNsPerSec = 1'000'000'000;

}  // namespace

IoUringWaiter::IoUringWaiter(unsigned entries)
{
  io_uring_params p;
  std::memset(&p, 0, sizeof(p));
  const int fd = sys_io_uring_setup(entries, &p);
  if (fd < 0) {
    throw std::system_error(errno, std::generic_category(), "flux: io_uring_setup");
  }
  ring_fd_ = fd;
  // Below 5.4 (separate ring mappings) IORING_OP_FUTEX_WAIT (6.7) is missing anyway.
  if ((p.features & IORING_FEAT_SINGLE_MMAP) == 0) {
    reset();
    throw std::system_error(ENOSYS, std::generic_category(), "flux: io_uring without SINGLE_MMAP");
  }

  const std::size_t sq_ring_sz = p.sq_off.array + p.sq_entries * sizeof(unsigned);
  const std::size_t cq_ring_sz = p.cq_off.cqes + p.cq_entries * sizeof(io_uring_cqe);

  auto map = [&](std::size_t len, off_t off) -> void * {
    void * m = ::mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, fd, off);
    return m == MAP_FAILED ? nullptr : m;
  };

  ring_sz_ = std::max(sq_ring_sz, cq_ring_sz);
  ring_ptr_ = map(ring_sz_, IORING_OFF_SQ_RING);
  sqes_sz_ = p.sq_entries * sizeof(io_uring_sqe);
  sqes_ = map(sqes_sz_, IORING_OFF_SQES);

  if (ring_ptr_ == nullptr || sqes_ == nullptr) {
    const int e = errno;
    reset();
    throw std::runtime_error(std::string("flux: io_uring mmap: ") + std::strerror(e));
  }

  auto * const rb = static_cast<unsigned char *>(ring_ptr_);
  sq_khead_ = reinterpret_cast<unsigned *>(rb + p.sq_off.head);
  sq_ktail_ = reinterpret_cast<unsigned *>(rb + p.sq_off.tail);
  sq_entries_ = p.sq_entries;
  sq_ring_mask_ = *reinterpret_cast<unsigned *>(rb + p.sq_off.ring_mask);
  sq_array_ = reinterpret_cast<unsigned *>(rb + p.sq_off.array);

  cq_khead_ = reinterpret_cast<unsigned *>(rb + p.cq_off.head);
  cq_ktail_ = reinterpret_cast<unsigned *>(rb + p.cq_off.tail);
  cq_ring_mask_ = *reinterpret_cast<unsigned *>(rb + p.cq_off.ring_mask);
  cqes_ = rb + p.cq_off.cqes;
}

void IoUringWaiter::reset() noexcept
{
  if (sqes_ != nullptr) ::munmap(sqes_, sqes_sz_);
  if (ring_ptr_ != nullptr) ::munmap(ring_ptr_, ring_sz_);
  if (ring_fd_ >= 0) ::close(ring_fd_);
  ring_fd_ = -1;
  ring_ptr_ = sqes_ = cqes_ = nullptr;
  ring_sz_ = sqes_sz_ = 0;
  sq_khead_ = sq_ktail_ = sq_array_ = cq_khead_ = cq_ktail_ = nullptr;
  sq_ring_mask_ = cq_ring_mask_ = sq_entries_ = 0;
  to_submit_ = 0;
}

IoUringWaiter::~IoUringWaiter()
{
  reset();
}

IoUringWaiter::IoUringWaiter(IoUringWaiter && o) noexcept
{
  *this = std::move(o);
}

IoUringWaiter & IoUringWaiter::operator=(IoUringWaiter && o) noexcept
{
  if (this != &o) {
    reset();
    ring_fd_ = o.ring_fd_;
    ring_ptr_ = o.ring_ptr_;
    ring_sz_ = o.ring_sz_;
    sqes_ = o.sqes_;
    sqes_sz_ = o.sqes_sz_;
    sq_khead_ = o.sq_khead_;
    sq_ktail_ = o.sq_ktail_;
    sq_ring_mask_ = o.sq_ring_mask_;
    sq_entries_ = o.sq_entries_;
    sq_array_ = o.sq_array_;
    cq_khead_ = o.cq_khead_;
    cq_ktail_ = o.cq_ktail_;
    cq_ring_mask_ = o.cq_ring_mask_;
    cqes_ = o.cqes_;
    to_submit_ = o.to_submit_;
    o.ring_fd_ = -1;
    o.ring_ptr_ = o.sqes_ = o.cqes_ = nullptr;
    o.ring_sz_ = o.sqes_sz_ = 0;
    o.sq_khead_ = o.sq_ktail_ = o.sq_array_ = o.cq_khead_ = o.cq_ktail_ = nullptr;
    o.sq_entries_ = 0;
    o.to_submit_ = 0;
  }
  return *this;
}

// Returns a tail index there is room for. tail - head is what the kernel has not consumed; when
// that fills the ring, submit rather than overwrite a live SQE.
unsigned IoUringWaiter::reserve_sqe()
{
  for (int attempt = 0; attempt < 64; ++attempt) {
    const unsigned head = __atomic_load_n(sq_khead_, __ATOMIC_ACQUIRE);
    if (*sq_ktail_ - head < sq_entries_) return *sq_ktail_;
    const int r = sys_io_uring_enter(ring_fd_, to_submit_, 0, 0, nullptr, 0);
    if (r > 0) {
      const unsigned took = static_cast<unsigned>(r);
      to_submit_ = took >= to_submit_ ? 0 : to_submit_ - took;
    } else if (r < 0 && errno == EINTR) {
      continue;
    } else {
      break;
    }
  }
  throw std::runtime_error("flux: io_uring submission queue is full and cannot be flushed");
}

void IoUringWaiter::arm(
  std::atomic<std::uint32_t> * word, std::uint32_t expected, std::uint64_t tag)
{
  auto * sqes = static_cast<io_uring_sqe *>(sqes_);
  const unsigned tail = reserve_sqe();  // sole producer of the SQ tail
  const unsigned index = tail & sq_ring_mask_;
  io_uring_sqe & sqe = sqes[index];
  std::memset(&sqe, 0, sizeof(sqe));
#if FLUX_HAS_IO_URING_FUTEX
  sqe.opcode = IORING_OP_FUTEX_WAIT;
  sqe.fd = static_cast<int>(FUTEX2_SIZE_U32);         // 32-bit, non-private (cross-process)
  sqe.addr = reinterpret_cast<std::uintptr_t>(word);  // uaddr
  sqe.off = expected;                                 // addr2 = futex_val
  sqe.addr3 = FUTEX_BITSET_MATCH_ANY;                 // futex_mask: any wake matches
#else
  (void)word;
  (void)expected;
  sqe.opcode = IORING_OP_NOP;  // supported() is false here; the caller must use the fallback
#endif
  sqe.user_data = tag;
  sq_array_[index] = index;
  __atomic_store_n(sq_ktail_, tail + 1, __ATOMIC_RELEASE);
  ++to_submit_;
}

void IoUringWaiter::arm_poll(int fd, std::uint64_t tag)
{
  auto * sqes = static_cast<io_uring_sqe *>(sqes_);
  const unsigned tail = reserve_sqe();
  const unsigned index = tail & sq_ring_mask_;
  io_uring_sqe & sqe = sqes[index];
  std::memset(&sqe, 0, sizeof(sqe));
  sqe.opcode = IORING_OP_POLL_ADD;
  sqe.fd = fd;
  sqe.poll32_events = POLLIN;  // x86 LE: field is native; BE would need byte-reversal
  sqe.user_data = tag;
  sq_array_[index] = index;
  __atomic_store_n(sq_ktail_, tail + 1, __ATOMIC_RELEASE);
  ++to_submit_;
}

void IoUringWaiter::cancel(std::uint64_t tag)
{
  auto * sqes = static_cast<io_uring_sqe *>(sqes_);
  const unsigned tail = reserve_sqe();
  const unsigned index = tail & sq_ring_mask_;
  io_uring_sqe & sqe = sqes[index];
  std::memset(&sqe, 0, sizeof(sqe));
  sqe.opcode = IORING_OP_ASYNC_CANCEL;
  sqe.addr = tag;  // match the pending op by its user_data
  sqe.user_data = make_tag(EventKind::cancel, 0);
  sq_array_[index] = index;
  __atomic_store_n(sq_ktail_, tail + 1, __ATOMIC_RELEASE);
  ++to_submit_;
}

int IoUringWaiter::wait(std::vector<WakeEvent> & events, std::int64_t timeout_ns)
{
  events.clear();

  unsigned flags = IORING_ENTER_GETEVENTS;
  io_uring_getevents_arg arg;
  __kernel_timespec ts;
  void * argp = nullptr;
  std::size_t argsz = 0;
  if (timeout_ns >= 0) {
    std::memset(&arg, 0, sizeof(arg));
    ts.tv_sec = timeout_ns / kNsPerSec;
    ts.tv_nsec = timeout_ns % kNsPerSec;
    arg.ts = reinterpret_cast<std::uintptr_t>(&ts);
    flags |= IORING_ENTER_EXT_ARG;
    argp = &arg;
    argsz = sizeof(arg);
  }

  // Submit newly armed waits and block for >=1 completion. Reap regardless of the return:
  // -ETIME/-EINTR can coexist with completions already sitting in the CQ ring.
  // Only what the kernel took leaves the accounting; zeroing it regardless stranded armed SQEs.
  const int submitted = sys_io_uring_enter(ring_fd_, to_submit_, 1, flags, argp, argsz);
  if (submitted > 0) {
    const unsigned took = static_cast<unsigned>(submitted);
    to_submit_ = took >= to_submit_ ? 0 : to_submit_ - took;
  }

  const unsigned mask = cq_ring_mask_;
  unsigned head = *cq_khead_;  // sole consumer of the CQ head
  const unsigned tail = __atomic_load_n(cq_ktail_, __ATOMIC_ACQUIRE);
  auto * cqes = static_cast<io_uring_cqe *>(cqes_);
  int n = 0;
  while (head != tail) {
    const io_uring_cqe & cqe = cqes[head & mask];
    events.push_back(WakeEvent{cqe.user_data, cqe.res});
    ++head;
    ++n;
  }
  __atomic_store_n(cq_khead_, head, __ATOMIC_RELEASE);
  return n;
}

IoUringSupport IoUringWaiter::support() noexcept
{
#if !FLUX_HAS_IO_URING_FUTEX
  return IoUringSupport::NoOpcode;  // pre-6.7 UAPI headers: the opcode does not exist here
#else
  std::optional<IoUringWaiter> w;
  try {
    w.emplace(8);
  } catch (const std::system_error & e) {
    if (e.code().value() == EPERM) return IoUringSupport::Forbidden;
    if (e.code().value() == ENOSYS) return IoUringSupport::NoOpcode;
    return IoUringSupport::RingFailed;
  } catch (...) {
    return IoUringSupport::RingFailed;
  }
  std::atomic<std::uint32_t> word{0};
  // A supported kernel completes a wait on a stale value with -EAGAIN, an unsupported opcode with
  // -EINVAL/-EOPNOTSUPP. No completion at all is a probe that did not run: no word on the opcode.
  try {
    w->arm(&word, 1, 0);
    std::vector<WakeEvent> ev;
    w->wait(ev, 0);
    if (ev.size() != 1) return IoUringSupport::RingFailed;
    if (ev[0].res == -EAGAIN) return IoUringSupport::Yes;
    if (ev[0].res == -EINVAL || ev[0].res == -EOPNOTSUPP) return IoUringSupport::NoOpcode;
  } catch (...) {
  }
  return IoUringSupport::RingFailed;
#endif
}

}  // namespace flux
