#include "flux/executor.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>

namespace flux
{

Executor::Executor(unsigned max_channels, std::int64_t poll_tick_ns)
: max_channels_(max_channels == 0 ? 1u : max_channels),
  poll_tick_ns_(poll_tick_ns <= 0 ? 1'000'000LL : poll_tick_ns)
{
  // The fallback is selected by the kernel, which makes it untestable on a machine that has the
  // opcode. FLUX_DISABLE_IO_URING forces it, so the path Jetson Orin will run is exercised here.
  // On a kernel that has the opcode a refused ring setup (fd exhaustion) throws instead of
  // degrading to parker threads: a silent downgrade would hide the resource leak that caused it.
  const bool forced_off = std::getenv("FLUX_DISABLE_IO_URING") != nullptr;
  switch (forced_off ? IoUringSupport::NoOpcode : IoUringWaiter::support()) {
    case IoUringSupport::Yes:
      waiter_.emplace(max_channels_ + 1);  // +1 for the wake fd
      break;
    case IoUringSupport::NoOpcode:
      break;  // parker threads
    case IoUringSupport::RingFailed:
      throw std::runtime_error(
        "flux: io_uring is supported here but no ring could be created (fd or memlock limit); "
        "raise the limit rather than running on the fallback");
  }
}

Executor::~Executor()
{
  for (auto & e : entries_) stop_parker(*e);
  release_waiters();
}

void Executor::add(Source & src, int priority)
{
  if (ctl_.is_spinning()) {
    throw std::logic_error("flux: add() must be called before spin()");
  }
  if (entries_.size() >= max_channels_) {
    throw std::length_error(
      "flux: Executor is full (max_channels=" + std::to_string(max_channels_) +
      "); construct it with a larger max_channels");
  }
  auto e = std::make_unique<Entry>();
  e->src = &src;
  e->priority = priority;
  entries_.push_back(std::move(e));
  rebuild_order();
}

void Executor::set_pass_budget(int n)
{
  if (ctl_.is_spinning()) {
    throw std::logic_error("flux: set_pass_budget() must be called before spin()");
  }
  if (n < 1) throw std::invalid_argument("flux: pass budget must be at least 1");
  pass_budget_ = n;
}

// Stable, so equal priorities keep the order they were registered in. Rebuilt whole on each add
// rather than inserted into: add() is pre-spin and the list is tens of entries.
void Executor::rebuild_order()
{
  order_.resize(entries_.size());
  for (std::size_t i = 0; i < order_.size(); ++i) order_[i] = i;
  std::stable_sort(order_.begin(), order_.end(), [this](std::size_t a, std::size_t b) {
    return entries_[a]->priority > entries_[b]->priority;
  });
}

void Executor::clear() noexcept
{
  for (auto & e : entries_) stop_parker(*e);
  release_waiters();
  entries_.clear();
  order_.clear();
  // Only while a loop runs: the Session destructor clears this, so setting it with none alive
  // leaves it set and the next spin() returns before its first pass.
  if (ctl_.is_spinning()) {
    ctl_.request_stop();  // nothing is waiting on an inert executor, so leave the fd alone
  }
}

std::function<void()> Executor::waker() const
{
  return ctl_.waker();
}

void Executor::interrupt() noexcept
{
  interrupted_.store(true, std::memory_order_release);  // set before the poke that reveals it
  ctl_.interrupt();
}

void Executor::stop() noexcept
{
  interrupted_.store(true, std::memory_order_release);
  ctl_.stop();
}

bool Executor::sync_channel(Entry & e, Channel * ch, std::uint64_t tag) noexcept
{
  const std::uint32_t gen = ch->attach_generation();
  if (e.gen == gen) return false;
  if (waiter_) {
    // A wait armed on the old mapping's word would stay pending in the kernel forever;
    // reap it before arming the new generation. The old block's waiter
    // count comes off with it.
    if (e.pending) waiter_->cancel(tag);
    if (e.counted_on) e.counted_on->remove_waiter();
    ch->add_waiter();  // io_uring parks in the kernel, so hold the gate for the duration
    e.counted_on = ch->wait_handle();
  }
  e.gen = gen;
  e.pending = false;
  return true;
}

// The Source dropped its Channel (orphaned segment): a fresh attach starts a new generation
// history, so start the entry over. counted_on keeps the old block alive past the dropped
// Channel, so the gate is balanced instead of leaked.
void Executor::detach_entry(Entry & e, std::uint64_t tag) noexcept
{
  stop_parker(e);
  if (e.pending && waiter_) waiter_->cancel(tag);
  if (e.counted_on) {
    e.counted_on->remove_waiter();
    e.counted_on.reset();
  }
  e.pending = false;
  e.gen = kNoAttach;
}

void Executor::start_parker(Entry & e, Channel & ch)
{
  e.sh = ch.wait_handle();
  e.pstop.store(false, std::memory_order_relaxed);
  Entry * p = &e;
  const int fd = ctl_.fd();
  e.parker = std::thread([p, fd] {
    // The generation this parker has already reported, carried across iterations rather than
    // re-sampled. Re-sampling loses a publish that lands between wait() timing out and the next
    // sample: the new value becomes the baseline and that frame is never poked, so it waits for
    // the spin tick instead of for the wake. wait() returns immediately when the generation has
    // already moved, which is what carries such a publish across the gap.
    std::uint32_t seen = p->sh->wake_seq();
    while (!p->pstop.load(std::memory_order_relaxed)) {
      // Bounded so stop is noticed; only a real wake pokes the wake fd, so an idle channel costs
      // one kernel-side timeout here and nothing at all on the delivery path.
      if (p->sh->wait(seen, 50'000'000)) {
        seen = p->sh->wake_seq();
        const std::uint64_t one = 1;
        [[maybe_unused]] ssize_t r = ::write(fd, &one, sizeof(one));
      }
    }
  });
}

// Fallback path: (re)start the parker when the entry is on a mapping no parker covers yet. True
// if it moved, which is the fallback's answer to sync_channel().
bool Executor::rebind_parker(Entry & e, Channel & ch)
{
  const std::uint32_t gen = ch.attach_generation();
  if (e.gen == gen && e.parker.joinable()) return false;
  stop_parker(e);
  e.gen = gen;
  start_parker(e, ch);
  return true;
}

void Executor::stop_parker(Entry & e) noexcept
{
  if (!e.parker.joinable()) return;
  e.pstop.store(true, std::memory_order_relaxed);
  e.parker.join();  // bounded by the park timeout above
  e.sh.reset();
}

// Stop counting as a parked waiter on every channel we announced ourselves to, so a publisher
// stops paying for a wake syscall on our behalf. Idempotent.
void Executor::release_waiters() noexcept
{
  for (auto & e : entries_) {
    if (!e->counted_on) continue;
    e->counted_on->remove_waiter();
    e->counted_on.reset();
  }
}

std::size_t Executor::ready_entry() const
{
  for (const std::size_t i : order_) {
    Entry & e = *entries_[i];
    if (e.blocked) continue;
    Channel * ch = e.src->channel();
    if (ch == nullptr) continue;
    // An unprobed entry goes without asking: take() is the only thing that notices a segment
    // replaced under a mapping whose own latest has stopped moving.
    if (e.probed && !ch->ready()) continue;
    return i;
  }
  return entries_.size();
}

int Executor::dispatch(const std::function<bool()> & yield)
{
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    Entry & e = *entries_[i];
    e.probed = false;
    e.blocked = false;
    const std::uint64_t tag = make_tag(EventKind::channel, static_cast<std::uint32_t>(i));
    Channel * ch = e.src->channel();
    if (ch == nullptr) {
      detach_entry(e, tag);    // a dropped channel took our waiter count and history with it
      if (!e.src->attach()) {  // publisher segment not up yet; retry next pass
        e.blocked = true;
        continue;
      }
      ch = e.src->channel();
    }
    if (!waiter_) {
      // Fallback: a parker thread per channel sleeps in FUTEX_WAIT and pokes
      // the wake fd, so wait_for_work() blocks on that one fd instead of rescanning on a tick.
      rebind_parker(e, *ch);
      continue;
    }
    sync_channel(e, ch, tag);
    // Sample the wake generation BEFORE anything is taken, so a publish racing the gap between
    // the last take and arm() re-fires the wait immediately instead of being lost.
    e.seq = ch->wake_seq();
  }

  int dispatched = 0;
  more_ = false;
  for (int budget = pass_budget_; budget > 0; --budget) {
    // Never before the first: the frame that woke the wait is always delivered.
    if (dispatched > 0 && yield && yield()) {
      more_ = ready_entry() != entries_.size();
      break;
    }
    more_ = budget == 1;  // survives the loop only if the budget, not an empty scan, ended it
    const std::size_t pick = ready_entry();
    if (pick == entries_.size()) {
      more_ = false;
      break;
    }

    Entry & e = *entries_[pick];
    const std::uint64_t tag = make_tag(EventKind::channel, static_cast<std::uint32_t>(pick));
    e.probed = true;
    const int n = e.src->deliver_one();
    dispatched += n;
    Channel * ch = e.src->channel();
    if (ch == nullptr) {  // orphan drop inside the callback: the entry restarts next pass
      detach_entry(e, tag);
      e.blocked = true;
      continue;
    }
    // Ready but nothing taken means take() refused (max_borrow, holder table). Only the owner
    // clears that, so retrying here would spend the rest of the budget on the same refusal.
    if (n == 0 && ch->ready()) e.blocked = true;
    // deliver_one() can re-attach, staling the sampled seq and the word it belongs to.
    if (waiter_ ? sync_channel(e, ch, tag) : rebind_parker(e, *ch)) {
      e.seq = ch->wake_seq();
      e.probed = false;
    }
  }

  if (!waiter_) return dispatched;
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    Entry & e = *entries_[i];
    if (e.pending) continue;  // armed on this generation and not yet fired
    Channel * ch = e.src->channel();
    if (ch == nullptr) continue;
    waiter_->arm(
      ch->wake_word(), e.seq, make_tag(EventKind::channel, static_cast<std::uint32_t>(i)));
    e.pending = true;
  }
  return dispatched;
}

void Executor::wait_for_work(std::int64_t timeout_ns)
{
  if (waiter_) {
    if (!ev_pending_) {
      waiter_->arm_poll(ctl_.fd(), make_tag(EventKind::control, 0));
      ev_pending_ = true;
    }
    waiter_->wait(events_, timeout_ns);
    for (const WakeEvent & w : events_) {
      switch (tag_kind(w.tag)) {
        case EventKind::control:
          ev_pending_ = false;
          ctl_.drain();
          break;
        case EventKind::channel:
          if (tag_index(w.tag) < entries_.size()) {
            entries_[tag_index(w.tag)]->pending = false;  // re-armed and drained next dispatch
          }
          break;
      }
    }
    return;
  }
  // Fallback: the parker threads started by dispatch() poke the wake fd, so this is one blocking
  // poll honoring the caller's timeout, not a fixed-tick rescan. While any channel still lacks a
  // parker (publisher not up yet), bound the sleep by the poll tick so the next dispatch() retries
  // the attach promptly.
  bool all_parked = !entries_.empty();
  for (auto & e : entries_) {
    if (!e->parker.joinable()) {
      all_parked = false;
      break;
    }
  }
  std::int64_t ns = timeout_ns;
  if (!all_parked) ns = timeout_ns < 0 ? poll_tick_ns_ : std::min(timeout_ns, poll_tick_ns_);
  // Rounded up: a deadline under a millisecond must not become a zero timeout and spin the CPU.
  const int ms = ns < 0 ? -1 : static_cast<int>((ns + 999'999) / 1'000'000);
  struct pollfd p;
  p.fd = ctl_.fd();
  p.events = POLLIN;
  p.revents = 0;
  ::poll(&p, 1, ms);
  if (p.revents & POLLIN) ctl_.drain();
}

int Executor::spin_once(std::int64_t timeout_ns)
{
  // Every exit consumes the interrupt flag, so one interrupt() ends at most one call -- a flag
  // left set would end a later wait nobody asked to end.
  int ready = dispatch();
  if (ready > 0) {
    interrupted_.exchange(false, std::memory_order_acq_rel);
    return ready;  // work was already waiting: never block
  }
  if (timeout_ns < 0) {  // forever: any wake ends the wait, and spin() just calls again
    wait_for_work(timeout_ns);
    ready = dispatch();
    interrupted_.exchange(false, std::memory_order_acq_rel);
    return ready;
  }
  // A bounded wait can wake early with nothing ready: the fallback sleeps one attach tick while a
  // publisher is still absent, and a wake-fd poke can be spurious. Re-wait until the deadline so
  // "block up to timeout_ns" holds on both paths.
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed_ns = [t0] {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now() - t0)
      .count();
  };
  for (;;) {
    const std::int64_t left = timeout_ns - elapsed_ns();
    wait_for_work(left > 0 ? left : 0);
    ready = dispatch();
    const bool interrupted = interrupted_.exchange(false, std::memory_order_acq_rel);
    if (ready > 0 || interrupted) return ready;
    if (elapsed_ns() >= timeout_ns) return 0;
  }
}

void Executor::spin(std::atomic<bool> & run, std::int64_t tick_ns)
{
  SpinControl::Session session(
    ctl_);  // reentry guard; clears stop and drains the fd on the way out
  // Destroyed before the session, so the flag this call owns is cleared inside the guarded window.
  struct Leave
  {
    Executor & ex;
    ~Leave() { ex.interrupted_.store(false, std::memory_order_relaxed); }
  } leave{*this};

  while (run.load(std::memory_order_relaxed) && !ctl_.stop_requested()) {
    spin_once(tick_ns);
    on_pass();
  }
}

void Executor::spin(std::int64_t tick_ns)
{
  std::atomic<bool> always{true};
  spin(always, tick_ns);
}

}  // namespace flux
