#ifndef FLUX_EXECUTOR_HPP
#define FLUX_EXECUTOR_HPP

#include "flux/channel.hpp"
#include "flux/io_uring_waiter.hpp"
#include "flux/spin_control.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

namespace flux
{

// Default callbacks one dispatch() pass runs, summed over every channel it drives. The only
// thing that ends a pass: a consumer slower than its publisher finds another frame on every
// take, so delivery has no natural end. QoS depth does not bound it --
// depth decides how far the cursor may lag, not how many takes succeed. The remainder arrives
// on the next pass, which costs no wait.
inline constexpr int kMaxDrain = 64;

// What an Executor drives: one channel, plus whatever its owner does with the frames. flux_cpp's
// ros::Subscription and flux_py's Subscription both implement this, which is what lets the merged
// wait below exist once instead of once per language.
class Source
{
public:
  virtual ~Source() = default;

  // Idempotent. False while the publisher's segment is not up, so the executor retries next pass.
  virtual bool attach() = 0;

  // Run the owner's callback for at most one frame. 1 if it ran, 0 if nothing was ready. One
  // frame rather than a drain, because the executor re-decides which channel goes next between
  // callbacks and cannot do that inside a loop it does not own.
  virtual int deliver_one() = 0;

  // Null until attach() has succeeded, and again after the segment is orphaned.
  virtual Channel * channel() noexcept = 0;

  // For a caller driving a Source outside an Executor. Not virtual and not what the executor
  // uses.
  int deliver(int max = kMaxDrain)
  {
    int n = 0;
    while (n < max && deliver_one() > 0) ++n;
    return n;
  }
};

// Blocks on ONE io_uring for every registered Source's wake word, so N channels cost one syscall
// per wait rather than one thread each. Requires Linux 6.7+ for io_uring
// FUTEX_WAIT; on an older kernel it falls back to one parker thread per channel, all poking the
// same wake fd, so the waiting thread still blocks exactly once.
//
// This knows nothing about ROS. Merging foreign readiness into the same wait is what waker()
// is for: flux_cpp hands that callable to rclcpp's readiness hooks, and the ring then wakes on
// a ROS message exactly as it does on a flux publish.
class Executor
{
public:
  // `max_channels` bounds registration; the ring is sized for that many plus the wake fd, so one
  // pass arms every wait in a single syscall. `poll_tick_ns` bounds the fallback path's wait
  // while some channel still has no publisher, so a late attach is noticed promptly.
  explicit Executor(unsigned max_channels = 32, std::int64_t poll_tick_ns = 2'000'000);
  virtual ~Executor();
  Executor(const Executor &) = delete;
  Executor & operator=(const Executor &) = delete;

  // Registration is pre-spin only: the spin thread walks this list without a lock.
  //
  // `priority` picks the next callback: the highest-priority channel with a frame ready goes,
  // and the choice is remade between callbacks, so a frame landing on a control channel during a
  // logging callback is served before the next logging frame. Ties keep registration order. Not
  // preemptive -- a running callback is never displaced.
  void add(Source & src, int priority = 0);
  std::size_t size() const noexcept { return entries_.size(); }

  // Drop every registered Source, leaving this executor inert: dispatch() then delivers nothing
  // rather than reading through a pointer whose owner is going away. For an owner tearing itself
  // down (flux_py's cyclic-GC clear). Not valid while spinning.
  void clear() noexcept;

  // True when every channel is waited on through one io_uring. False means the kernel lacks the
  // opcode and the parker-thread fallback is active.
  bool uses_io_uring() const noexcept { return waiter_.has_value(); }

  // Attach late publishers, deliver ready frames highest-priority-first, re-arm the wait. Never
  // blocks. Returns callbacks run. Must run on the thread that owns those callbacks.
  // `yield` is asked between callbacks, never before the first; true ends the pass.
  int dispatch(const std::function<bool()> & yield = {});

  // True when the last dispatch() stopped on the pass budget with frames still ready. An
  // embedder driving wait_for_work()/dispatch() itself must skip the wait while this holds:
  // nothing pokes the ring for a frame that was already there. spin() reads dispatch() > 0 for
  // the same reason and needs no separate flag.
  bool has_more() const noexcept { return more_; }

  // Callbacks one dispatch() pass may run, over all channels together. Lower it to tighten the
  // blocking term a higher-priority channel sees; raise it to amortize the per-pass arming over
  // more frames. Pre-spin only.
  void set_pass_budget(int n);  // bounds a pass, not throughput: the next pass takes the rest
  int pass_budget() const noexcept { return pass_budget_; }

  // Block until a channel is woken, the wake fd is poked, or `timeout_ns` elapses (negative =
  // forever). Delivers nothing. Split from dispatch() so an embedder can run the blocking half
  // on one thread and the callback half on another; the two must never overlap, because both
  // submit to the same ring.
  //
  // Virtual for the one embedder that must wrap the blocking syscall: flux_py drops the GIL
  // around it. Nothing here touches the embedder's runtime, so an override needs only to call
  // the base.
  virtual void wait_for_work(std::int64_t timeout_ns);

  // dispatch(), then wait and dispatch again if nothing was ready. Returns callbacks run. A
  // bounded wait that wakes with nothing ready re-waits until the deadline, so the timeout means
  // what it says on both paths; only interrupt() and stop() end it early.
  int spin_once(std::int64_t timeout_ns);

  // Loop spin_once until `run` is cleared or stop() is called. `tick_ns` bounds each wait so both
  // are noticed and late publishers are picked up; it is a wait bound, not a poll interval.
  void spin(std::atomic<bool> & run, std::int64_t tick_ns = 100'000'000);
  void spin(std::int64_t tick_ns = 100'000'000);

  // End spin(). Safe from a callback or another thread. A stop() that lands before spin() starts
  // is held rather than lost, and cleared on the way out so the executor can be spun again.
  void stop() noexcept;

  // End the current wait without ending the loop. Safe from another thread. Not called `wake`:
  // flux uses that word for the in-segment futex wake generation.
  void interrupt() noexcept;

  bool is_spinning() const noexcept { return ctl_.is_spinning(); }

  // A callable that breaks this executor's current wait, for an embedder merging its own
  // readiness into it. Safe from any thread, and safe to call after this executor is destroyed:
  // it holds the wake fd weakly, so a hook still running on a foreign thread writes to a
  // still-open fd or does nothing, never to a recycled fd number.
  std::function<void()> waker() const;

protected:
  // Called once per spin() pass, after spin_once returns. flux_py delivers Ctrl-C here; a throw
  // ends the spin through the same guard a throw from a callback does.
  virtual void on_pass() {}

private:
  static constexpr std::uint32_t kNoAttach = ~0u;  // no segment reconciled yet

  struct Entry
  {
    Source * src = nullptr;
    int priority = 0;
    bool pending = false;  // armed in the kernel, not yet fired (merged path)
    // The segment block our +1 on the parked-waiter gate was placed on. Removal must hit this
    // exact block: a Source can re-attach on its own (a direct take() outside this executor), and
    // removing on the current channel would then drive a live segment's gate negative --
    // publishers see waiters == 0 and skip the wake syscall for everyone.
    std::shared_ptr<ChannelShared> counted_on;
    std::uint32_t gen = kNoAttach;  // attach generation `pending`/`counted_on` refer to

    // Reset at the top of every dispatch(); meaningless between passes.
    std::uint32_t seq = 0;  // wake generation sampled before this pass took anything
    bool probed = false;    // took at least once since this mapping was reconciled
    bool blocked = false;   // out of the running for the rest of this pass

    // Fallback path only: a thread parked on this channel's wake word. It holds the segment block
    // rather than a Channel pointer, so a re-attach or a dropped Channel cannot pull the word out
    // from under it.
    std::shared_ptr<ChannelShared> sh;
    std::thread parker;
    std::atomic<bool> pstop{false};
  };

  // Re-register the waiter gate and drop a stale arming when the channel moved to a new segment;
  // true if it moved.
  std::size_t ready_entry() const;
  bool sync_channel(Entry & e, Channel * ch, std::uint64_t tag) noexcept;
  void detach_entry(Entry & e, std::uint64_t tag) noexcept;
  bool rebind_parker(Entry & e, Channel & ch);
  void start_parker(Entry & e, Channel & ch);
  void stop_parker(Entry & e) noexcept;
  void release_waiters() noexcept;
  void rebuild_order();

  unsigned max_channels_;
  std::int64_t poll_tick_ns_;
  int pass_budget_ = kMaxDrain;
  SpinControl ctl_;  // wake fd, the spin/stop flag pair, and the reentry guard
  bool ev_pending_ = false;
  bool more_ = false;  // the last dispatch() ran out of budget, not of frames
  // Consumed by a bounded spin_once: on the fallback path the wake fd carries both frame wakes
  // and interrupts, so the fd alone cannot say "stop re-waiting". Not in SpinControl: it means
  // "this call's wait ended early", which only the bounded wait has.
  std::atomic<bool> interrupted_{false};
  std::optional<IoUringWaiter> waiter_;
  std::vector<WakeEvent> events_;
  std::vector<std::unique_ptr<Entry>> entries_;  // stable addresses: an Entry is named by its tag
  // Visit order for one pass: indices into entries_, by descending priority then registration.
  // Kept beside entries_ rather than sorting it, because a tag names a position in entries_ and
  // an armed wait outlives the add() that would have moved it.
  std::vector<std::size_t> order_;
};

}  // namespace flux

#endif  // FLUX_EXECUTOR_HPP
