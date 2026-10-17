#ifndef FLUX2_DETAIL_SUBSCRIBER_HPP
#define FLUX2_DETAIL_SUBSCRIBER_HPP

#include "flux2/channel.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace flux2::detail
{

// The node-free half of a subscription, shared by the C++ Subscription and the Python binding.
// Attaches once a publisher is up, and again after it restarts. A second thread inside it at the
// same time is refused with std::logic_error: a re-attach would free the mapping under it.
class Subscriber
{
public:
  // Throws on a QoS the channel would refuse.
  Subscriber(
    std::string signpost, std::uint64_t fingerprint, const QoS & qos, MemoryPolicy mem,
    gpu::Stream stream = {})
  : signpost_(std::move(signpost)),
    fingerprint_(fingerprint),
    qos_(qos),
    mem_(mem),
    stream_(std::move(stream))
  {
    // The channel is what judges a QoS, and with no publisher yet there is none to ask.
    Channel(1, 2).qos(qos_);
  }

  bool attach()
  {
    const InUse held(busy_);
    return attach_held();
  }

  // Refused once an executor serves this subscription: the two would split its frames.
  FrameView take()
  {
    refuse_if_served();
    const InUse held(busy_);
    return take_held();
  }

  // What an executor's sources call. From then on take() and take_blocking() are refused. A
  // refusal other than NOT_READY leaves this subscription out, so the executor's other sources
  // keep running, until its publisher restarts: then it is tried again.
  bool attach_for_executor()
  {
    const InUse held(busy_);
    if (left_out()) return false;
    try {
      return attach_held();
    } catch (const Error & e) {
      refuse(e);
      return false;
    }
  }
  FrameView take_for_executor()
  {
    served_.store(true, std::memory_order_relaxed);
    const InUse held(busy_);
    if (left_out()) return {};
    try {
      return take_held();
    } catch (const Error & e) {
      refuse(e);
      return {};
    }
  }
  const std::string & refusal() const noexcept { return refusal_; }
  // Each refusal once, to be logged; null when there is none new.
  const std::string * new_refusal() noexcept
  {
    if (refusal_.empty() || refusal_reported_) return nullptr;
    refusal_reported_ = true;
    return &refusal_;
  }

  // With no publisher yet it also waits for one. A negative timeout waits forever. Empty at once
  // while max_borrow views are held. `stop` is asked after each wait of at most kSlice, and true
  // ends it empty: a futex wait does not see a ROS shutdown or a signal.
  template <class Stop>
  FrameView take_blocking(std::int64_t timeout_ns, Stop stop)
  {
    refuse_if_served();
    const InUse held(busy_);
    // Also the attach poll, where a watch is not open yet.
    constexpr auto kSlice = std::chrono::milliseconds(100);
    using Clock = std::chrono::steady_clock;
    const bool unbounded = timeout_ns < 0;
    const auto now = Clock::now();
    const auto deadline = std::chrono::nanoseconds(timeout_ns) < Clock::time_point::max() - now
                            ? now + std::chrono::nanoseconds(timeout_ns)
                            : Clock::time_point::max();
    for (;;) {
      Clock::duration slice = kSlice;
      bool last = false;
      if (!unbounded) {
        const auto left = std::max(deadline - Clock::now(), Clock::duration::zero());
        last = left <= slice;
        slice = std::min(slice, left);
      }
      const auto slice_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(slice).count();
      if (attach_held()) {
        refuse_if_inherited();
        FrameView v = ch_->take_blocking(slice_ns);
        note();
        if (v) return v;
        drop_if_gone();
        if (ch_ && !ch_->can_borrow()) return {};
      } else if (watch_) {
        // The publisher that comes up wakes the watch, which the failed attach armed.
        watch_->wait(slice_ns);
      } else {
        std::this_thread::sleep_for(slice);
      }
      if (last || stop()) return {};
    }
  }
  FrameView take_blocking(std::int64_t timeout_ns)
  {
    return take_blocking(timeout_ns, [] { return false; });
  }

  // These four read copies the taking thread keeps, so any thread may call them.
  bool attached() const noexcept { return attached_.load(std::memory_order_relaxed); }
  const std::string & signpost_name() const noexcept { return signpost_; }
  const gpu::Stream & stream() const noexcept { return stream_; }
  const QoS & qos() const noexcept { return qos_; }
  // Over the subscription's life: kept across a re-attach and once the publisher is gone.
  std::uint64_t lost() const noexcept { return lost_.load(std::memory_order_relaxed); }
  // Changes on every re-attach.
  std::uint32_t attach_generation() const noexcept
  {
    return generation_.load(std::memory_order_relaxed);
  }
  Refused refused() const noexcept
  {
    return {
      refused_[0].load(std::memory_order_relaxed), refused_[1].load(std::memory_order_relaxed),
      refused_[2].load(std::memory_order_relaxed), refused_[3].load(std::memory_order_relaxed),
      refused_[4].load(std::memory_order_relaxed)};
  }
  // Over the subscription's life, as lost(), and as of the last take: releases happen outside
  // any call into the subscription.
  std::uint64_t fence_failed() const noexcept
  {
    return fence_failed_.load(std::memory_order_relaxed);
  }
  gpu::FenceWait fence_wait() const noexcept
  {
    gpu::FenceWait w{};
    const std::array<std::uint64_t *, 6> out = {&w.commit_ns,  &w.commit_waits,  &w.commit_max_ns,
                                                &w.release_ns, &w.release_waits, &w.release_max_ns};
    for (std::size_t i = 0; i < waits_.size(); ++i) {
      *out[i] = waits_[i].load(std::memory_order_relaxed);
    }
    return w;
  }
  // From any thread; the thread that takes applies it, to this channel and every one after.
  void set_fence_timeout(std::chrono::nanoseconds t) noexcept
  {
    fence_timeout_ns_.store(
      t.count() > 0 ? static_cast<std::uint64_t>(t.count()) : 0, std::memory_order_relaxed);
  }
  void set_release_mode(gpu::ReleaseMode m) noexcept
  {
    release_mode_.store(m, std::memory_order_relaxed);
  }
  gpu::ReleaseMode release_mode() const noexcept
  {
    return release_mode_.load(std::memory_order_relaxed);
  }
  // Only from the thread that takes: a re-attach replaces the channel.
  const Channel * channel() const noexcept { return ch_ ? &*ch_ : nullptr; }
  // While no channel is attached, what an executor sleeps on until a publisher comes up.
  const Watch * watch() const noexcept { return ch_ || !watch_ ? nullptr : &*watch_; }
  // A publisher's ring of fewer slots is refused, now and at every attach: frames queued in a
  // message_filters input would hold every slot and stall it for good.
  void require_slots(std::uint32_t n)
  {
    min_slots_ = n;
    if (ch_) check_slots();
  }

private:
  class InUse
  {
  public:
    explicit InUse(std::atomic<bool> & busy) : busy_(busy)
    {
      if (busy_.exchange(true, std::memory_order_acquire)) {
        throw std::logic_error(
          "flux2: two threads used one subscription at once; give each thread its own");
      }
    }
    ~InUse() { busy_.store(false, std::memory_order_release); }
    InUse(const InUse &) = delete;
    InUse & operator=(const InUse &) = delete;
    InUse(InUse &&) = delete;
    InUse & operator=(InUse &&) = delete;

  private:
    std::atomic<bool> & busy_;
  };

  void refuse_if_served() const
  {
    if (served_.load(std::memory_order_relaxed)) {
      throw std::logic_error(
        "flux2: an executor serves this subscription; take its frames in the callback");
    }
  }

  // Its frames are the parent's to borrow: a child would wait on them unbounded.
  void refuse_if_inherited() const
  {
    if (ch_->inherited()) {
      throw std::logic_error(
        "flux2: this subscription came across fork(); open a new one in the child");
    }
  }

  void refuse(const Error & e)
  {
    refusal_ = e.what();
    refusal_reported_ = false;
    refused_epoch_ = watch_ ? watch_->arm() : publisher_epoch();
    checked_ = std::chrono::steady_clock::now();
  }

  // Left out until the publisher restarts, which the watch shows in one load. Without a watch (it
  // could not be opened), the signpost is opened at most once per kRecheck to look.
  bool left_out()
  {
    constexpr auto kRecheck = std::chrono::milliseconds(100);
    if (refusal_.empty()) return false;
    if (watch_) {
      if (watch_->epoch() == refused_epoch_) return true;
    } else {
      const auto now = std::chrono::steady_clock::now();
      if (now - checked_ < kRecheck) return true;
      checked_ = now;
      if (publisher_epoch() == refused_epoch_) return true;
    }
    refusal_.clear();
    return false;
  }

  // 0 while no publisher is up.
  std::uint32_t publisher_epoch() const
  {
    flux2_channel_stats s{};
    return flux2_read_channel_stats(signpost_.c_str(), &s) == FLUX2_OK && s.live ? s.epoch : 0;
  }

  bool attach_held()
  {
    if (!ch_) {
      try {
        if (!watch_) watch_.emplace(signpost_);
        ch_.emplace(Channel::open(*watch_, fingerprint_, qos_, mem_, stream_));
      } catch (const Error & e) {
        if (e.status() == FLUX2_NOT_READY) return false;
        throw;
      }
      fence_timeout_applied_ = 0;
      release_mode_applied_ = gpu::ReleaseMode::Wait;
      check_slots();
      note();
    }
    const std::uint64_t t = fence_timeout_ns_.load(std::memory_order_relaxed);
    if (t != fence_timeout_applied_) {
      ch_->set_fence_timeout(std::chrono::nanoseconds(t));
      fence_timeout_applied_ = t;
    }
    const gpu::ReleaseMode m = release_mode_.load(std::memory_order_relaxed);
    if (m != release_mode_applied_) {
      ch_->set_release_mode(m);
      release_mode_applied_ = m;
    }
    return true;
  }

  void drop_channel()
  {
    lost_before_ += ch_->lost();
    fence_failed_before_ += ch_->fence_failed();
    const gpu::FenceWait w = ch_->fence_wait();
    wait_before_.commit_ns += w.commit_ns;
    wait_before_.commit_waits += w.commit_waits;
    wait_before_.commit_max_ns = std::max(wait_before_.commit_max_ns, w.commit_max_ns);
    wait_before_.release_ns += w.release_ns;
    wait_before_.release_waits += w.release_waits;
    wait_before_.release_max_ns = std::max(wait_before_.release_max_ns, w.release_max_ns);
    ch_.reset();
    note();
  }

  void note() noexcept
  {
    const Refused r = ch_ ? ch_->refused() : Refused{};
    attached_.store(ch_.has_value(), std::memory_order_relaxed);
    lost_.store(lost_before_ + (ch_ ? ch_->lost() : 0), std::memory_order_relaxed);
    generation_.store(ch_ ? ch_->attach_generation() : 0, std::memory_order_relaxed);
    refused_[0].store(r.max_borrow, std::memory_order_relaxed);
    refused_[1].store(r.no_reader, std::memory_order_relaxed);
    refused_[2].store(r.contended, std::memory_order_relaxed);
    refused_[3].store(r.bad_frame, std::memory_order_relaxed);
    refused_[4].store(r.fence, std::memory_order_relaxed);
    fence_failed_.store(
      fence_failed_before_ + (ch_ ? ch_->fence_failed() : 0), std::memory_order_relaxed);
    const gpu::FenceWait now = ch_ ? ch_->fence_wait() : gpu::FenceWait{};
    const gpu::FenceWait & b = wait_before_;
    const std::array<std::uint64_t, 6> sums = {
      b.commit_ns + now.commit_ns,
      b.commit_waits + now.commit_waits,
      std::max(b.commit_max_ns, now.commit_max_ns),
      b.release_ns + now.release_ns,
      b.release_waits + now.release_waits,
      std::max(b.release_max_ns, now.release_max_ns)};
    for (std::size_t i = 0; i < waits_.size(); ++i) {
      waits_[i].store(sums[i], std::memory_order_relaxed);
    }
  }

  void check_slots()
  {
    const std::uint32_t have = ch_->slot_count();
    if (have >= min_slots_) return;
    drop_channel();
    throw Error(
      FLUX2_INVALID_ARGUMENT,
      ("flux2: " + signpost_ + " has " + std::to_string(have) +
       " slots, and its message_filters queue needs " + std::to_string(min_slots_))
        .c_str());
  }

  FrameView take_held()
  {
    if (!attach_held()) return {};
    refuse_if_inherited();
    FrameView v = ch_->take();
    note();
    if (!v) drop_if_gone();
    return v;
  }

  // A publisher that died leaves this mapping behind, and a restarted one that refused this
  // subscriber leaves nothing to take. Either way the mapping goes; a refusal is thrown.
  void drop_if_gone()
  {
    if (ch_->orphaned()) {
      drop_channel();
      return;
    }
    try {
      ch_->check_attach();
    } catch (const Error &) {
      drop_channel();
      throw;
    }
  }

  std::string signpost_;
  std::uint64_t fingerprint_;
  QoS qos_;
  MemoryPolicy mem_;
  gpu::Stream stream_;
  std::optional<Watch> watch_;
  std::optional<Channel> ch_;
  std::uint32_t min_slots_ = 0;
  std::uint64_t lost_before_ = 0;
  std::string refusal_;
  bool refusal_reported_ = false;
  std::uint32_t refused_epoch_ = 0;
  std::chrono::steady_clock::time_point checked_{};
  std::atomic<bool> busy_{false};
  std::atomic<bool> served_{false};
  std::atomic<bool> attached_{false};
  std::atomic<std::uint64_t> lost_{0};
  std::atomic<std::uint32_t> generation_{0};
  std::array<std::atomic<std::uint64_t>, 5> refused_{};
  std::uint64_t fence_failed_before_ = 0;
  gpu::FenceWait wait_before_{};
  std::atomic<std::uint64_t> fence_failed_{0};
  std::array<std::atomic<std::uint64_t>, 6> waits_{};
  std::atomic<std::uint64_t> fence_timeout_ns_{0};
  std::uint64_t fence_timeout_applied_ = 0;
  std::atomic<gpu::ReleaseMode> release_mode_{gpu::ReleaseMode::Wait};
  gpu::ReleaseMode release_mode_applied_ = gpu::ReleaseMode::Wait;
};

}  // namespace flux2::detail

#endif  // FLUX2_DETAIL_SUBSCRIBER_HPP
