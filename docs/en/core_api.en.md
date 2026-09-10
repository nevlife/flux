# flux_core API (without ROS)

`flux_core` knows nothing about ROS. It calls neither rclcpp nor rclpy and builds with standalone cmake. This document is the surface for using flux outside a ROS node. Inside a ROS node, [api.md](api.en.md) is the right document, and using flux without a `.msg` is covered in [raw_api.md](raw_api.en.md).

## 1. What is different

`flux::ros::Publisher` and `flux::ros::Subscription` do three things. Do those three yourself and what remains is `flux::Channel`.

| What the ROS wrapper does | When doing it yourself |
| --- | --- |
| Resolves the topic name through the node namespace and remaps | Give the absolute name directly |
| Builds the signpost from name, fingerprint, and domain | Call `flux::signpost_name()` |
| Hooks the callback into the executor | Call `take()` directly or `wait()` |

`Channel` itself is the same object on both sides. The wrapper only holds one `Channel`, so the concurrency protocol, the copy model, and QoS all hold unchanged.

## 2. Build

```cmake
find_package(flux_core REQUIRED)
target_link_libraries(mytool PRIVATE flux::core)
```

Outside an ament workspace, build `flux_core` alone with cmake. The only dependency is pthread. `flux_cpp`, `flux_py`, and `flux_gen` are not needed.

## 3. Building the name

The rendezvous key is the name, the fingerprint, and the domain. All three must match to attach. Since the ROS wrapper is not involved, give the name directly as an absolute path.

```cpp doc:core_name
std::string domain = flux::process_domain();
std::string name = flux::signpost_name("/img", /*fingerprint=*/0, domain);
```

`process_domain()` checks `FLUX_DOMAIN` first, then `ROS_DOMAIN_ID` if that is absent. If neither is set, it is `0`. Even without ROS, attaching to a ROS node on the same host requires the same domain that node uses.

It is decided once on the first call and does not change until the process exits. This is the same as rcl latching `ROS_DOMAIN_ID` at context init. Otherwise the ROS side and the flux side of one process could sit in different partitions. Reading the environment fresh each time is `resolve_domain(var)`. That is a query and is not used to build names.

When attaching to the ROS wrapper, the topic name must be the post-remap one. If the node resolved `img` to `/robot1/img`, give `/robot1/img` here as well.

## 4. Publishing

```cpp doc:core_publish
flux::Channel ch = flux::Channel::create(
  flux::signpost_name("/img", flux::kNoSchema), /*slot_size=*/16 << 20, /*slot_count=*/16,
  flux::kNoSchema);

flux::WriteSlot w = ch.loan(flux::DType::U8, {height, width, 3});
if (w) {
  render_into(w.data(), w.capacity());
  if (w.commit() != flux::Published::Ok) sink(ch.dropped());
}
```

`create` builds the segment. If a live publisher already exists and `slot_size` or `slot_count` disagrees, it throws `flux::SegmentMismatch`. Retrying does not fix it, so do not catch it.

The meaning of `loan`, `commit`, and `publish` is the same as in the ROS wrapper. dtype and shape are stated once in `loan`, and `commit` takes no arguments. Return value interpretation is in [raw_api.md](raw_api.en.md) section 2. With a `.msg` adapter, `Builder` fills the frame for you. The adapter sits on top of `WriteSlot`, so it works unchanged without ROS.

### Page precommit

The last argument of `create` and `open` is `flux::MemoryPolicy`. It is the page residency for this process's mapping, and the default is off.

```cpp doc:core_memory_policy
flux::MemoryPolicy mem;
mem.precommit = true;  // fault in every page at attach
mem.lock = true;       // mlock as well. RLIMIT_MEMLOCK must cover the segment

flux::Channel committed = flux::Channel::create(
  flux::signpost_name("/img", flux::kNoSchema), 4096, 8, flux::kNoSchema, {}, mem);
sink(committed.pages_committed(), committed.pages_locked());
```

Details are in the `MemoryPolicy` section of [api.md](api.en.md) section 3. Three points. It is declared separately per process. It is reapplied on every reattach. A refusal is not silently downgraded. It throws `std::system_error`.

In Python, `flux.MemoryPolicy` is the same thing (section 7 below).

## 5. Consuming

```cpp doc:core_subscribe
flux::Channel ch = flux::Channel::open(flux::signpost_name("/img", 0), /*fingerprint=*/0);
ch.qos(flux::QoS{});

flux::FrameView v = ch.take();
if (v) {
  use(v.data(), v.size());
  v.release();
}
sink(ch.lost(), ch.refused().total());
```

`open` throws if there is no publisher yet. The lazy attach the wrapper used to do must be done by hand. Catch it and retry. A fingerprint or layout version mismatch is `SegmentMismatch`, and retrying that is pointless.

`qos()` is given after construction. `depth`, `durability`, and `max_borrow` mean the same as in [qos.md](qos.en.md).

`peek()` returns the newest frame without consuming it. `take()` consumes the next one in publish order. `take_blocking(timeout_ns)` waits until one arrives.

## 6. Waiting

There is no executor to hook a callback into, so waking up is done by hand.

```cpp doc:core_wait
ch.add_waiter();
const std::uint32_t seen = ch.wake_seq();
if (!ch.take()) {
  ch.wait(seen, /*timeout_ns=*/-1);
}
ch.remove_waiter();
```

Without `add_waiter()`, the publisher does not issue the wake syscall at all. Calling `wait()` without registering does not wake until the timeout.

The order of reading `wake_seq()` before `take()` matters. Reversed, a publish that lands in between is missed and the caller parks. Passing the saved value to `wait()` lets the kernel decide whether a publish happened in between.

If `orphaned()` is true, the publisher group has died and no rotation is coming either. That channel does not come back.

### Several channels at once

With several channels, `flux::Executor` waits on all of them with a single io_uring. It is a class that knows nothing about ROS, and both `flux::ros::Executor` and `flux.Executor` sit on top of it. The code that decides the wait order is kept in one place so it does not diverge per language.

What to wait on is expressed as a `flux::Source`. It is one channel plus what to do with its frame.

```cpp doc:core_source
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
```

`attach()` is idempotent and returns false if the publisher segment does not exist yet. `deliver_one()` handles exactly one frame and returns 1, or 0 if there is none. It is one at a time because the executor re-selects which channel to run next between callbacks. To pull several at once from the owner's side, use `deliver(max)`. That is not virtual and is not the path the executor uses.

```cpp doc:core_executor
flux::Executor ex(32);
ex.add(a);
ex.add(b, 10);  // priority: b goes first whenever it has a frame
ex.set_pass_budget(16);  // callbacks one dispatch() may run over all channels together
ex.spin(100'000'000);  // until stop()
ex.stop();
```

Registration happens only before spin. The second argument of `add` is the priority. Larger goes first, and ties go in registration order. The default is 0 and negative values are allowed. The selection is repeated per callback. If a frame arrives on a higher channel while a lower channel's callback is running, it runs before the lower channel's next frame. This is not preemptive priority. A callback already running is not displaced.

`set_pass_budget()` is the upper bound on the number of callbacks one `dispatch()` runs. It is a single budget for all channels together, not per channel, and the default is `flux::kMaxDrain` (64). Call it only before spin. It bounds the pass length, not the throughput. Whatever the budget cut off is picked up by the next pass without blocking. Lowering it shortens the time a high-priority channel waits behind a low one. Raising it spreads the per-pass arm cost over more frames. If `uses_io_uring()` is false, the kernel is older than Linux 6.7 and it runs on the per-channel parker thread fallback. The waiting thread still blocks only once.

When placing it on someone else's event loop, use the two halves separately. They share the ring, so they must not run concurrently.

```cpp doc:core_executor_split
ex.wait_for_work(ex.has_more() ? 0 : -1);  // do not block if the budget left a remainder
const int delivered = ex.dispatch();       // delivery only. Does not block
```

`has_more()` says whether the previous `dispatch()` stopped at the budget with frames remaining. Nobody knocks on the ring again for frames that already arrived, so blocking in this state sleeps one tick on top of work the caller already holds. `spin()` makes the same decision from the return value of `dispatch()`, so it does not use this flag.

In Python, `flux.Executor` wraps the same class (section 8 below).

## 7. Python (without ROS)

Importing only `flux` keeps ROS out. All `flux.ros` does is name resolution and the rclpy executor connection.

```python doc:core_py_publisher
pub = flux.Publisher("/img", fingerprint=FP, slot_size=16 << 20, slot_count=16)
pub.publish(arr)
```

```python doc:core_py_subscription
sub = flux.Subscription("/img", fingerprint=FP, qos=flux.QoS())
v = sub.take()
if v is not None:
    _sink(v.shape)
```

The name must be an absolute path. A relative name raises, because there is no node to resolve it.

The `memory` argument is the page precommit. It is the same as C++ `flux::MemoryPolicy`, with the same field names.

```python doc:core_py_memory_policy
mem = flux.MemoryPolicy(precommit=True, lock=True)
pub = flux.Publisher("/img", fingerprint=FP, slot_size=4096, slot_count=8, memory=mem)
sub = flux.Subscription("/img", fingerprint=FP, memory=mem)
```

`precommit` faults in every page at attach. `lock` also does `mlock` and includes `precommit`. A refusal is `OSError`. A channel is never created running without the bound it declared. `flux.ros.Publisher` and `flux.ros.Subscription` accept the same argument unchanged.

## 8. Python executor

This class wraps the `flux::Executor` of section 6 as is. The names and arguments are the same as C++, and the code that decides the wait order is the same single one.

```python doc:py_core_executor
ex = flux.Executor(max_channels=32, poll_tick_ns=2_000_000)
ex.add(sub, callback, priority=0)
ex.spin_once(timeout_ns=-1)
ex.spin(tick_ns=100_000_000)
ex.stop()
ex.interrupt()
merged = ex.uses_io_uring
busy = ex.is_spinning
```

If `uses_io_uring` is false, the kernel is older than Linux 6.7 and it runs on the per-channel thread fallback.

`priority` is the same value as in C++. It sets the visit order within a pass. Larger goes first, and ties go in registration order.

`stop()` is a request and `is_spinning` is a state. They are not merged into one flag, so a `stop()` that arrives before `spin()` is not lost. Code that stops a thread right after starting it does not fall into that window. The request is cleared by the `spin()` that observed it as it exits, so the same executor can be spun again. Calling `spin()` on an executor that is already spinning raises.

`spin_once(timeout_ns)` handles what is ready and waits if nothing is. It returns the number handled. `interrupt()` leaves the loop in place and wakes only the current wait. It ends exactly one call at a time.

To embed in another event loop, use the two halves separately. The two must not run concurrently.

```python doc:py_split_wait
ex.wait_for_work(timeout_ns=-1)
delivered = ex.dispatch()
```

## 9. Enumeration (for tools)

This reads which flux channels exist on this host right now and who is attached. `flux.enumerate_topics()` returns a list of `flux.Topic`, and each topic holds a list of `flux.Endpoint`. It is daemonless and has no registry. It is a snapshot built on the spot from the `/dev/shm` names and the manifests of processes that still hold the owner lock. `flux_cli` sits on top of this surface ([cli.md](cli.en.md)).

```python doc:py_enumerate
for topic in flux.enumerate_topics():
    name = topic.key if topic.key_exact else topic.signpost
    for ep in topic.endpoints:
        role = "pub" if ep.publisher else "sub"
        _sink(name, topic.domain, topic.fingerprint, role, ep.pid, ep.starttime, ep.label)
```

If `key_exact` is False, `key` was read back from the name, not from a live participant. The name replaces every non-alphanumeric character with `.`, so `/a/b` and `.a.b` are one name. What is visible then is the spelling of the name, not the key the peers actually agreed on. A channel with no participants at all is that case (the signpost is permanent, so only the name remains).

`flux.flatten_key(key)` is that transformation. A tool that must match a user-typed name against enumeration results compares through it. If a tool restates the rule, the format silently diverges. This is how `flux topic info` finds a channel whose publisher has died by its original name.

`label` is the value the boundary layer announced. `flux_cpp` puts in the node's fully qualified name, and `flux_py` leaves it empty because there is no node. core does not interpret this string.

Dead participants do not appear. If the OFD lock on the owner file is released, that manifest describes nothing, so it is skipped. It is only skipped, not deleted. Deleting is the job of sweep.

Each name costs one open. Use it in tools, not in loops.

To look at one channel without attaching, `flux.read_channel_stats()` returns a `flux.ChannelStats`. It takes neither a lock nor a borrow.

```python doc:py_channel_stats
domain = flux.process_domain()

stats = flux.read_channel_stats(signpost)
if stats.live:
    _sink(stats.slot_count, stats.slot_size, stats.storage_kind, stats.fingerprint)
    _sink(stats.publish_seq, stats.epoch, stats.waiters)
```

`publish_seq` is a monotone ticket issuer. Measure it twice and take the difference, and that is the exact number of frames in that interval. It is not an estimate. If `epoch` changed, the publisher group restarted and the counter is new, so a difference across that boundary is not a frame count. If `live` is False, no publisher currently has the segment up, and since the signpost is permanent that is the normal idle state of a channel.

`process_domain()` decides the domain with the same rules and the same timing convention as a node. Since the rule exists once in core, a tool does not derive the same answer twice. To reread the environment now, use `flux.resolve_domain("ROS_DOMAIN_ID")`, and to ignore the domain variables entirely, use `flux.resolve_domain(None)`.

`read_channel_stats` does not attach as a subscriber. It reads only the header, so it touches neither `max_borrow` nor the holder table. Observation does not disturb the target.
