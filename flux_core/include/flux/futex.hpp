#ifndef FLUX_FUTEX_HPP
#define FLUX_FUTEX_HPP

#include <atomic>
#include <cstdint>
#include <ctime>

// In-segment wake primitive. The wake word lives inside the
// shared segment (ControlHeader::wakeup); the publisher bumps it and wakes parked
// subscribers. Plain (non-private) futex ops so a wake crosses the mmap boundary between
// processes. This is only the minimal wake/wait; the multiplexed wait layer (io_uring
// merging many channels plus a ROS fd) is a flux_cpp/executor concern.

#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace flux
{

// Block while *word == expected. `timeout` is a RELATIVE deadline, or nullptr = forever.
// Returns 0 if woken; -1 with errno set otherwise (EAGAIN if *word != expected on entry,
// ETIMEDOUT on timeout, EINTR if interrupted by a signal).
inline long futex_wait(
  std::atomic<std::uint32_t> * word, std::uint32_t expected,
  const struct timespec * timeout) noexcept
{
  return ::syscall(
    SYS_futex, reinterpret_cast<std::uint32_t *>(word), FUTEX_WAIT, expected, timeout, nullptr, 0);
}

// Wake up to `count` waiters (INT_MAX = all). Returns the number woken, or -1 with errno.
inline long futex_wake(std::atomic<std::uint32_t> * word, int count) noexcept
{
  return ::syscall(
    SYS_futex, reinterpret_cast<std::uint32_t *>(word), FUTEX_WAKE, count, nullptr, nullptr, 0);
}

}  // namespace flux

#endif  // FLUX_FUTEX_HPP
