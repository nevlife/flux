#ifndef FLUX_SPIN_CONTROL_HPP
#define FLUX_SPIN_CONTROL_HPP

#include <atomic>
#include <functional>
#include <memory>

namespace flux
{

// The wake fd and the two loop flags every flux spin loop needs. Both executors need all of it,
// and before this they had it twice with one difference that mattered: flux::Executor guards the
// fd against its own destruction and flux::ros::PartitionedExecutor closed it outright.
//
// The fd is held through a shared_ptr so waker() can outlive this object. A foreign readiness
// hook may still be running on another thread when the owner goes away, and it must write to a
// still-open fd or do nothing, never to a recycled fd number.
//
// "Am I spinning" and "should I stop" are two flags, not one. Sharing one means entering the loop
// has to arm it, and that arming erases a stop() that landed while the caller was still starting
// the thread -- the caller has moved on to join(), so nobody asks again and the loop never ends.
//
class SpinControl
{
public:
  SpinControl();  // throws std::runtime_error when the eventfd cannot be created
  ~SpinControl();
  SpinControl(const SpinControl &) = delete;
  SpinControl & operator=(const SpinControl &) = delete;

  int fd() const noexcept { return ev_->fd; }

  // Consume every pending poke. EFD_NONBLOCK, so one read returns the accumulated count.
  void drain() noexcept;

  // Break the current wait without ending the loop. Safe from any thread.
  void interrupt() noexcept;

  // End the loop and break its wait.
  void stop() noexcept;

  // End the loop without touching the fd, for a caller that is not spinning and has nothing to
  // wake. Kept apart from stop() so tearing an idle owner down leaves no poke behind for a later
  // wait to return on.
  void request_stop() noexcept { stop_requested_.store(true, std::memory_order_relaxed); }

  bool stop_requested() const noexcept { return stop_requested_.load(std::memory_order_relaxed); }
  bool is_spinning() const noexcept { return spinning_.load(std::memory_order_acquire); }

  // A callable that breaks the current wait. Safe from any thread, and safe to call after this
  // object is destroyed: it holds the fd weakly.
  std::function<void()> waker() const;

  // Marks the loop entered for as long as it lives, and throws std::logic_error when one is
  // already alive. On the way out it clears the stop request and drains the fd, so the next loop
  // neither starts already-stopped nor wakes on this one's interrupt.
  class Session
  {
  public:
    explicit Session(SpinControl & ctl);
    ~Session();
    Session(const Session &) = delete;
    Session & operator=(const Session &) = delete;

  private:
    SpinControl & ctl_;
  };

private:
  struct WakeFd
  {
    explicit WakeFd(int f) noexcept : fd(f) {}
    ~WakeFd();
    WakeFd(const WakeFd &) = delete;
    WakeFd & operator=(const WakeFd &) = delete;
    int fd;
  };

  std::shared_ptr<WakeFd> ev_;
  std::atomic<bool> spinning_{false};
  std::atomic<bool> stop_requested_{false};
};

}  // namespace flux

#endif  // FLUX_SPIN_CONTROL_HPP
