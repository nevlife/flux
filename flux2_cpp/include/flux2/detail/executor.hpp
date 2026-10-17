#ifndef FLUX2_DETAIL_EXECUTOR_HPP
#define FLUX2_DETAIL_EXECUTOR_HPP

#include "flux2.h"
#include "flux2/channel.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace flux2::detail
{

// Registered by address, so neither copied nor moved.
class Source
{
public:
  Source() = default;
  virtual ~Source() = default;
  Source(const Source &) = delete;
  Source & operator=(const Source &) = delete;
  Source(Source &&) = delete;
  Source & operator=(Source &&) = delete;
  // False while there is nothing to attach to yet.
  virtual bool attach() = 0;
  // False when no frame was there.
  virtual bool deliver_one() = 0;
  // Null until attached.
  virtual const Channel * channel() const = 0;
  // While channel() is null: the watch whose publisher ends the wait, armed by the last attach.
  // Without one, an unattached source is retried once a tick.
  virtual const Watch * watch() const { return nullptr; }
  // The endpoint flux2_trace_endpoint_init named, for trace events.
  virtual const void * endpoint() const { return nullptr; }
};

// Stop, interrupt and wake from any thread.
class ExecutorHandle
{
public:
  void stop() const noexcept { flux2_handle_stop(h_.get()); }
  // Ends the current wait, not the loop.
  void interrupt() const noexcept { flux2_handle_interrupt(h_.get()); }
  // Breaks the wait so a pass runs.
  void wake() const noexcept { flux2_handle_wake(h_.get()); }
  bool is_spinning() const noexcept { return flux2_handle_is_spinning(h_.get()); }

private:
  friend class Executor;
  struct Free
  {
    void operator()(flux2_handle * h) const noexcept { flux2_handle_free(h); }
  };
  explicit ExecutorHandle(flux2_handle * h) : h_(h, Free{}) {}
  std::shared_ptr<flux2_handle> h_;
};

// An exception from a source or on_pass is rethrown from the call that ran it. A moved-from
// executor throws std::logic_error from the calls that act and reads as empty.
class Executor
{
public:
  Executor() : st_(std::make_unique<State>())
  {
    require_abi();
    check(flux2_executor_new(&ex_));
  }
  ~Executor() { flux2_executor_free(ex_); }
  Executor(Executor && o) noexcept : ex_(o.ex_), st_(std::move(o.st_)) { o.ex_ = nullptr; }
  Executor & operator=(Executor &&) = delete;
  Executor(const Executor &) = delete;
  Executor & operator=(const Executor &) = delete;

  // Not owned: `src` outlives the executor. Equal priorities take turns. Not while spinning.
  void add(Source & src, std::int32_t priority = 0)
  {
    live();
    st_->entries.push_back(std::make_unique<Entry>(Entry{&src, st_.get()}));
    const flux2_source s{
      st_->entries.back().get(),
      &call<&Source::attach>,
      &call<&Source::deliver_one>,
      &channel_cb,
      nullptr,
      &watch_cb,
      src.endpoint()};
    check(flux2_executor_add(ex_, s, priority));
  }
  std::size_t size() const noexcept { return ex_ ? flux2_executor_size(ex_) : 0; }

  void set_pass_budget(std::uint32_t n) { check(flux2_executor_set_pass_budget(live(), n)); }
  std::uint32_t pass_budget() const noexcept { return ex_ ? flux2_executor_pass_budget(ex_) : 0; }
  // The last pass ran out of budget, not of frames.
  bool has_more() const noexcept { return ex_ && flux2_executor_has_more(ex_); }
  // Work the next pass would find, read without a syscall.
  bool ready() const noexcept { return ex_ && flux2_executor_ready(ex_); }

  // Runs after every pass of spin.
  void on_pass(std::function<void()> f)
  {
    live();
    st_->on_pass = std::move(f);
    check(flux2_executor_on_pass(ex_, &on_pass_cb, st_.get()));
  }

  std::size_t dispatch()
  {
    std::size_t n = 0;
    run(flux2_executor_dispatch(live(), &n));
    return n;
  }
  // `yield_now() -> bool` is asked between callbacks, never before the first. It must not throw.
  template <class F>
  std::size_t dispatch(F & yield_now)
  {
    std::size_t n = 0;
    const auto ask = [](void * ctx) noexcept -> bool { return (*static_cast<F *>(ctx))(); };
    run(flux2_executor_dispatch_yielding(live(), ask, &yield_now, &n));
    return n;
  }
  // Never overlapping a dispatch. A negative timeout waits forever.
  void wait_for_work(std::int64_t timeout_ns)
  {
    run(flux2_executor_wait_for_work(live(), timeout_ns));
  }
  void spin(std::int64_t tick_ns = FLUX2_DEFAULT_TICK_NS)
  {
    run(flux2_executor_spin(live(), tick_ns));
  }
  // Also ends when `run_flag` turns false.
  void spin(std::atomic<bool> & run_flag, std::int64_t tick_ns = FLUX2_DEFAULT_TICK_NS)
  {
    static_assert(
      sizeof(std::atomic<bool>) == sizeof(bool) && std::atomic<bool>::is_always_lock_free);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): the C side reads it atomically.
    run(flux2_executor_spin_while(live(), reinterpret_cast<bool *>(&run_flag), tick_ns));
  }
  void clear_interrupt() noexcept
  {
    if (ex_) flux2_executor_clear_interrupt(ex_);
  }

  ExecutorHandle handle() const { return ExecutorHandle(flux2_executor_handle(live())); }

private:
  struct State;
  struct Entry
  {
    Source * src;
    State * st;
  };
  struct State
  {
    std::exception_ptr error;
    std::function<void()> on_pass;
    std::vector<std::unique_ptr<Entry>> entries;
  };

  flux2_executor * live() const
  {
    if (!ex_) throw std::logic_error("flux2: this executor was moved from");
    return ex_;
  }

  // channel() cannot fail the C call, so its exception surfaces once that call returns.
  void run(Status s)
  {
    if (st_->error) std::rethrow_exception(std::exchange(st_->error, {}));
    check(s);
  }

  template <bool (Source::*F)()>
  static std::int32_t call(void * ctx) noexcept
  {
    auto * e = static_cast<Entry *>(ctx);
    try {
      return (e->src->*F)() ? 1 : 0;
    } catch (...) {
      e->st->error = std::current_exception();
      return -1;
    }
  }
  static const flux2_channel * channel_cb(void * ctx) noexcept
  {
    auto * e = static_cast<Entry *>(ctx);
    try {
      const Channel * ch = e->src->channel();
      return ch ? ch->raw() : nullptr;
    } catch (...) {
      e->st->error = std::current_exception();
      return nullptr;
    }
  }
  static const flux2_watch * watch_cb(void * ctx) noexcept
  {
    auto * e = static_cast<Entry *>(ctx);
    try {
      const Watch * w = e->src->watch();
      return w ? w->raw() : nullptr;
    } catch (...) {
      e->st->error = std::current_exception();
      return nullptr;
    }
  }
  static std::int32_t on_pass_cb(void * ctx) noexcept
  {
    auto * st = static_cast<State *>(ctx);
    try {
      st->on_pass();
      return 0;
    } catch (...) {
      st->error = std::current_exception();
      return -1;
    }
  }

  flux2_executor * ex_ = nullptr;
  std::unique_ptr<State> st_;
};

}  // namespace flux2::detail

#endif  // FLUX2_DETAIL_EXECUTOR_HPP
