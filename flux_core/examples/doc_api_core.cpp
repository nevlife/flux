// Compile-only companion to docs/en/core_api.en.md -- flux_core with no ROS. This file includes no
// rclcpp header and the target it builds into links flux_core alone, which is what keeps the
// document's claim honest. Every region between [doc:id] and [doc:/id] is the code the fence
// tagged `doc:id` in that document shows. Nothing here is meant to run.

#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/executor.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace flux::doc_core_examples
{

void render_into(void *, std::size_t)
{
}
void use(const void *, std::size_t)
{
}
void handle(const flux::FrameView &)
{
}

template <typename... Ts>
void sink(const Ts &...)
{
}

void doc_core_name()
{
  // [doc:core_name]
  std::string domain = flux::process_domain();
  std::string name = flux::signpost_name("/img", /*fingerprint=*/0, domain);
  // [doc:/core_name]
  sink(name);
}

void doc_core_publish(std::uint64_t height, std::uint64_t width)
{
  // [doc:core_publish]
  flux::Channel ch = flux::Channel::create(
    flux::signpost_name("/img", flux::kNoSchema), /*slot_size=*/16 << 20, /*slot_count=*/16,
    flux::kNoSchema);

  flux::WriteSlot w = ch.loan(flux::DType::U8, {height, width, 3});
  if (w) {
    render_into(w.data(), w.capacity());
    if (w.commit() != flux::Published::Ok) sink(ch.dropped());
  }
  // [doc:/core_publish]
}

void doc_core_memory_policy()
{
  // [doc:core_memory_policy]
  flux::MemoryPolicy mem;
  mem.precommit = true;  // fault every page in at attach
  mem.lock = true;       // and mlock; RLIMIT_MEMLOCK must cover the segment

  flux::Channel committed = flux::Channel::create(
    flux::signpost_name("/img", flux::kNoSchema), 4096, 8, flux::kNoSchema, {}, mem);
  sink(committed.pages_committed(), committed.pages_locked());
  // [doc:/core_memory_policy]
}

void doc_core_subscribe()
{
  // [doc:core_subscribe]
  flux::Channel ch = flux::Channel::open(flux::signpost_name("/img", 0), /*fingerprint=*/0);
  ch.qos(flux::QoS{});

  flux::FrameView v = ch.take();
  if (v) {
    use(v.data(), v.size());
    v.release();
  }
  sink(ch.lost(), ch.refused().total());
  // [doc:/core_subscribe]
}

void doc_core_wait(flux::Channel & ch)
{
  // [doc:core_wait]
  ch.add_waiter();
  const std::uint32_t seen = ch.wake_seq();
  if (!ch.take()) {
    ch.wait(seen, /*timeout_ns=*/-1);
  }
  ch.remove_waiter();
  // [doc:/core_wait]
}

// [doc:core_source]
class Sink : public flux::Source
{
public:
  explicit Sink(flux::Channel && ch) : ch_(std::move(ch)) {}

  bool attach() override { return true; }  // already open; a lazy one would open here

  int deliver_one() override
  {
    flux::FrameView v = ch_.take();
    if (!v) return 0;
    handle(v);
    return 1;
  }

  flux::Channel * channel() noexcept override { return &ch_; }

private:
  flux::Channel ch_;
};
// [doc:/core_source]

void doc_core_executor(Sink & a, Sink & b)
{
  // [doc:core_executor]
  flux::Executor ex(32);
  ex.add(a);
  ex.add(b, 10);           // priority: b goes first whenever it has a frame
  ex.set_pass_budget(16);  // callbacks one dispatch() may run over all channels together
  ex.spin(100'000'000);    // runs until stop()
  ex.stop();
  // [doc:/core_executor]

  // [doc:core_executor_split]
  ex.wait_for_work(ex.has_more() ? 0 : -1);  // never blocks on frames the budget left behind
  const int delivered = ex.dispatch();       // delivers only; never blocks
  // [doc:/core_executor_split]
  sink(delivered, ex.uses_io_uring(), ex.size(), ex.spin_once(0));
}

}  // namespace flux::doc_core_examples
