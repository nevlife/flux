# flux user API

This is the entire surface used from a ROS 2 node.

## 1. Things to know first

**The rendezvous key is the topic name, the fingerprint, and the domain.** The `/dev/shm` segment name is derived from these three, so both sides must produce the same name to connect. If even one character differs, they do not connect and no error is raised. ROS rewrites `"img"` to `"/robot1/img"`, so use the constructor that takes a node and get the name after remapping. A topic name longer than 185 characters makes the constructor throw (`std::invalid_argument` in C++, `ValueError` in Python). The `/dev/shm` name holds at most 255 characters and flux adds up to 70 of its own, and a cut name would let two topics or two schemas meet on one channel.

**The domain is a partition within one host.** The default is `0`, and there is usually nothing to configure. If `ROS_DOMAIN_ID` is set, flux is partitioned by it as well. Two systems separated by domain do not mix even when they use the same topic name. To set it for flux alone, use `FLUX_DOMAIN`. If both are set, `FLUX_DOMAIN` wins. Both take a plain integer in [0, 2^32) and render it without leading zeros, so `007` and `7` are one domain. Any other value makes the constructor throw (`std::invalid_argument` in C++, `ValueError` in Python). A typo that silently falls back to the default domain would remove the isolation. Containers are separated by `/dev/shm` independently of this.

**The fingerprint is the schema hash.** If the two sides differ, attach is refused. When you bind a type from `.msg`, the generator fills it in. 0 means no check.

**A received view points into shared memory.** While it is alive, the publisher cannot use that slot. Use it and release it right away.

**Delivery is best-effort.** The publisher does not wait for slow subscribers.

If `slot_size`/`slot_count` disagree with a live publisher, attach throws `flux::SegmentMismatch`, a `std::runtime_error` (Python: `flux.SegmentMismatch`, a `RuntimeError`). Retrying does not fix it, so do not catch it. The same holds when a live publisher's segment cannot be opened, for example for lack of permission: attach throws a `std::runtime_error` (Python: `RuntimeError`) with the reason. Only an absent publisher is waited for quietly.

## 2. Binding a type (`.msg` -> adapter)

When a message package opts a `.msg` in, `flux_gen` emits a C++ header and a Python module. The layout and fingerprint are baked into both as constants, so the two languages cannot disagree. The layout rules are in `docs/message_shapes.md`.

```cmake
# my_pkg/CMakeLists.txt
find_package(flux_gen REQUIRED)
flux_generate_adapters(msg/Cloud.msg msg/Detection.msg)
```

```xml
<!-- my_pkg/package.xml -->
<buildtool_depend>flux_gen</buildtool_depend>
<exec_depend>flux_gen</exec_depend>   <!-- the Python adapter imports flux_gen.wire -->
```

```text
include/my_pkg/flux/cloud.hpp    ->  my_pkg::flux_msg::Cloud
my_pkg_flux/cloud.py             ->  my_pkg_flux.cloud.Cloud
```

```text
# my_pkg/msg/Cloud.msg
std_msgs/Header header
float32[] x
float32[] y
float32[] z
uint32 width
string label
```

Each type produces three things.

| | Meaning |
| --- | --- |
| `kFingerprint` / `FINGERPRINT__` | The fingerprint of this schema. Pass it as is to Publisher and Subscription |
| `View` | Reads a received frame. Zero-copy |
| `Builder` | Writes directly into a borrowed slot. Zero-copy |

These are the rules by which fields become accessors.

| `.msg` | C++ View | Python View | C++ Builder | Python Builder |
| --- | --- | --- | --- | --- |
| `uint32 width` | `v.width()` | `v.width` | `b.set__width(n)` | `b.width = n` |
| `float32[] x` | `v.x()` -> Span | `v.x` -> ndarray | `b.alloc__x(n)` | `b.alloc__x(n)` |
| `float64[9] m` | `v.m()` -> Span | `v.m` -> ndarray | `b.m()` -> Span | `b.m` -> ndarray |
| `string label` | `v.label()` | `v.label` | `b.set__label(s)` | `b.label = s` |
| `string[] tags` | `v.tags__size()` · `v.tags(i)` | `v.tags` -> list | `b.alloc__tags(n)` · `b.set__tags(i, s)` | `b.tags = [...]` |
| `Point[] pts` | `v.pts__size()` · `v.pts(i)` | `v.pts` -> sequence | `b.alloc__pts(n)[i]` | `b.alloc__pts(n)[i]` |
| `Header header` | `v.header__sec()` · `v.header__nanosec()` · `v.header__frame_id()` | `v.header__stamp` · `v.header__frame_id` | `b.set__header__stamp(s, ns)` · `b.set__header__frame_id(f)` | `b.set__header__stamp(s, ns)` · `b.header__frame_id = f` |

Nesting is flattened with a name prefix. `pose.position.x` becomes `pose_position_x`.

`alloc__*` returns memory inside the slot. Writing there is the zero-copy path. The call order is free. For a fixed-size array (`string[2]`, `Point[2]`) `alloc__x()` takes no count, because the schema fixes it.

If the `.msg` is rejected (`bool[]`, `T[<=N]`, `wstring`, no fields), generation fails. Send that message over plain ROS.

### C++

```cpp doc:adapter_cpp_pub
#include "my_pkg/flux/cloud.hpp"
using my_pkg::flux_msg::Cloud;

flux::ros::Publisher pub(*node, "cloud", Cloud::kFingerprint, 16 << 20, 16);

Cloud::Builder b = Cloud::build__(pub);   // loans a slot and puts a Builder on top of it
if (b) {
  auto xs = b.alloc__x(n);            // points into the slot
  lidar.read_into(xs.data(), xs.size());  // the data is produced in the slot
  b.set__width(static_cast<std::uint32_t>(n));
  b.set__label("front");
  if (const flux::Published p = b.commit__(); flux::faulted(p)) report(flux::to_string(p));  // does not publish if ok__() is false
}
```

`build__(pub)` makes the Builder own the slot. There is one object to keep alive. If there is no free slot, the Builder is false. `set__x()` in that state is harmless, but `commit__()` returns `Backpressure`. It does not pretend to have published. It is the same as Python's `Cloud.build__(pub)`. A span from `alloc__x(n)` comes back empty when the Builder is false or the slot has no room for `n`, so fill `xs.size()` elements, not `n`. In Python the false Builder's `alloc__x(n)` returns a throwaway array of `n` elements instead, so `b.alloc__x(n)[:] = xs` does not fail either.

```cpp doc:adapter_cpp_sub
using my_pkg::flux_msg::Cloud;

flux::ros::Subscription sub(
  *node, "cloud", Cloud::kFingerprint, flux::QoS{}, [](const flux::FrameView & f) {
    Cloud::View c(f);
    for (float x : c.x()) {
      use(x);
    }
    if (!c.ok__()) {  // the frame is malformed: discard the values read
      return;
    }
  });
```

### Python

```python doc:adapter_py_pub
from my_pkg_flux.cloud import Cloud

pub = flux.ros.Publisher(node, "cloud", fingerprint=Cloud.FINGERPRINT__)

b = Cloud.build__(pub)                  # loans a slot and returns a Builder. False if no free slot
if b:
    b.alloc__x(n)[:] = xs              # writes directly into the slot
    b.width = n
    b.label = "front"
    b.commit__()
```

```python doc:adapter_py_sub
sub = flux.ros.Subscription(node, "cloud", fingerprint=Cloud.FINGERPRINT__)

f = sub.take()
if f is not None:
    c = Cloud.View(f)
    print(c.x, c.width, c.label)      # c.x is a read-only ndarray pointing into the slot
```

The receiving side assumes the frame was written by another process and checks the descriptor. In C++, `ok__()` latches to false on a mismatch and the accessors return empty values afterwards. An array whose descriptor does not fit has `x__size() == 0`. Python raises `flux_gen.wire.WireError`.

### Converting to and from ROS message objects

If you already hold a ROS message object such as `sensor_msgs::msg::Image`, the bridge moves it in and out in one line. It comes as a separate header (`<pkg>/flux/<name>_ros.hpp`) and module (`<pkg>_flux.<name>_ros`). Only this bridge includes the rosidl message type. When it is not used, the adapter does not see rclcpp or rclpy. The bridge converts between two representations of the same type, so the examples below use `sensor_msgs/Image` throughout.

```cpp doc:adapter_cpp_bridge
#include "sensor_msgs/flux/image_ros.hpp"
using sensor_msgs::flux_msg::Image;

// sending: ROS object -> frame
Image::Builder b(w);
msg_to_frame(img, b);                 // copies every field of img into the slot
if (const flux::Published p = b.commit__(); flux::faulted(p)) report(flux::to_string(p));

// receiving: frame -> ROS object
sensor_msgs::msg::Image back = frame_to_msg(Image::View(f));
```

```python doc:adapter_py_bridge
from sensor_msgs_flux.image import Image
from sensor_msgs_flux.image_ros import frame_to_msg, msg_to_frame

b = Image.build__(pub)
msg_to_frame(img, b)
b.commit__()

back = frame_to_msg(Image.View(f))
```

Both copy every field. `frame_to_msg` adds one copy per subscriber, so the zero-copy property that is independent of the subscriber count is lost. Use it only when forwarding data received on a ROS topic into flux, or when an existing function signature requires a message object. If the data has not been produced yet, use `Builder`/`View` directly (`docs/copy_model.md`).

Stock ROS 2 tools (`rqt`, `ros2 topic`) see a flux channel through `flux_bridge`, which is this `frame_to_msg` path plus one DDS publisher per watched channel ([bridge.md](bridge.en.md)). Recording does not take this path: `flux_bag` writes the frame bytes to a bag as they are ([bag.md](bag.en.md)).

## 3. C++

`#include "flux/ros/publisher.hpp"` · `"flux/ros/subscription.hpp"` · `"flux/ros/executor.hpp"`

### Publisher

```cpp doc:publisher
flux::ros::Publisher pub(*node, "img", fingerprint, slot_size, slot_count);

pub.loan();                        // 0-copy. invalid handle if no free slot
pub.dropped();
pub.slot_size();                    // bytes of one slot. the generated adapter's build__() reads it
pub.segment_name();
```

The defaults are `fingerprint = flux::kNoSchema`, `slot_size = 16 MiB` (`Publisher::kDefaultSlotSize`), and `slot_count = 16` (`Publisher::kDefaultSlotCount`). The adapter path always passes the fingerprint, so only the last two are omitted. `slot_size` is an upper bound, not an allocation. The payload area is not touched at init, so tmpfs leaves it sparse. `slot_count` is the ring depth and the hard cap for subscriber QoS.

Publishing on the adapter path is `Builder::commit()`. The return value `flux::Published` (Python: `flux.Published`) separates backpressure from permanent errors. The meaning of each value is in section 2 of [`raw_api.en.md`](raw_api.en.md). There is usually no need to tell the five apart. `flux::faulted(p)` / `flux.faulted(p)` folds them into one question: "is this a dropped frame, or a fault that does not clear on its own".

What `segment_name()` returns is the signpost name (the fixed name derived from the topic and fingerprint). The actual segment name has an instance suffix appended and changes on every publisher restart.

The seventh argument is page precommit. The name differs by language. The C++ constructor parameter is `mem` (`flux_cpp/include/flux/ros/publisher.hpp`) and the Python keyword argument is `memory`. It is off by default, and the `MemoryPolicy` section below describes the trade-off.

### WriteSlot (what loan returns)

`loan` reserves a slot first and hands it over. Writing directly into it involves no copy (`docs/copy_model.md`). If you bound a type, `Builder` wraps this, so on the adapter path the only place `WriteSlot` is handled directly is the GPU section below.

A reserved slot leaves the ring until commit or abort. Holding it for a long time reduces the free slots and raises `dropped`. Like `FrameView`, it is move-only. The full surface and the meaning of `abort` are in section 3 of [`raw_api.en.md`](raw_api.en.md).

### Subscription

```cpp doc:subscription
flux::ros::Subscription sub(
  *node, "img", fingerprint, flux::QoS{}, [](const flux::FrameView & v) { handle(v); });
```

A subscription does not drive itself. There are two ways to read, and the presence of a callback decides which.

With a callback it is push. Hand it to `flux::ros::Executor` or `PartitionedExecutor`, and the callback runs for every frame on that spin thread. Both wake on the channel's futex, so there is no polling.

Without a callback it is pull. Read directly from a loop you already have. It has the same shape as Python's `flux.ros.Subscription`.

```cpp doc:subscription_pull
flux::ros::Subscription sub(*node, "img", fingerprint);  // no callback

flux::FrameView newest = sub.peek();              // latest state. does not consume
flux::FrameView next = sub.take();                // next frame. invalid if none
flux::FrameView blocked = sub.take_blocking(-1);  // sleeps on the futex until one arrives
```

The QoS comes before the callback, in the order of rclcpp's `create_subscription(topic, qos, callback)`. The sixth and seventh arguments are `Device` and `MemoryPolicy`. The latter is page precommit for this subscription's own mapping and is described in the `MemoryPolicy` section below. `sub.pages_committed()` and `sub.pages_locked()` report the result. Before attach, both are false.

Adding a subscription without a callback to an executor makes `add()` throw. It does not create a state where something is registered but nothing runs.

There was a `poll_period` argument that drove the subscription from a wall timer, and it was removed. Waking on a period charged every subscription a latency bound and idle wakeups, and it was the only arrangement `flux_py` could not provide, so the same node written in the two languages ended up with different executors.

```cpp doc:subscription_api
bool attached = sub.attached();
bool driven = sub.has_callback();  // false means pull-only. the executor refuses it
const flux::QoS & qos = sub.qos();
std::uint64_t lost = sub.lost();   // cumulative count of frames not received
bool can_borrow = sub.can_borrow();               // false means I am holding a view, so an empty result comes back
flux::Channel::Refused refused = sub.refused();   // same fields as "When an empty result comes back" in section 4
const std::string & domain = sub.domain();          // this process's domain. pub.domain() is the same
```

Read `attached()` and `domain()` together. If no frames arrive and `attached()` is false, there is no publisher yet. If `domain()` differs from what you expect, the publisher exists but is in a different partition. The two look the same from the outside. `take()` returns an empty result and `refused()` is all zeros. `flux domain list` shows what lives in which domain ([cli.md](cli.en.md)).

There is one domain per process. It is decided once when first needed and does not change afterwards, so a publisher and a subscription in the same process cannot see different values. This is the same as how rcl handles `ROS_DOMAIN_ID`. Changing the environment variable midway does not change the domain of a node that is already up, and this is the same convention.

### MemoryPolicy (page precommit)

`slot_size` is an upper bound, not an allocation. The payload stays as sparse tmpfs, and every page faults the first time it is touched. That fault lands on the first publish and the first take, not on init. `MemoryPolicy` moves it to attach.

```cpp doc:memory_policy
flux::MemoryPolicy mem;
mem.precommit = true;   // fault in every page at attach
mem.lock = true;        // mlock as well. RLIMIT_MEMLOCK must cover the segment

flux::ros::Publisher pub(
  *node, "img", fingerprint, slot_size, slot_count, flux::Device::Cpu, mem);
flux::ros::Subscription sub(*node, "img", fingerprint, flux::QoS{}, {}, flux::Device::Cpu, mem);

bool committed = pub.pages_committed();
bool locked = pub.pages_locked();
```

- Both are off by default. Turning them on trades away the benefit of sparseness (a large `slot_size` at almost zero idle RAM).
- Each process declares it separately. The publisher having committed says nothing about the subscriber's mapping. Faults happen per mapping.
- It is all or nothing. It covers the header, slots, holder table, and the entire payload. A partial commit gives no bound, and establishing a bound is the purpose of this option.
- It is reapplied on every reattach. When the publisher restarts, the subscriber gets a new mapping, and that mapping inherits none of the previous mapping's residency.
- Refusal throws instead of silently downgrading. If `mlock` hits `RLIMIT_MEMLOCK`, it is a `std::system_error` and attach fails. A channel running without the bound it declared is what this option is meant to prevent.
- `lock` includes `precommit`. `mlock` populates what it locks. With `precommit` alone, the pages are committed but can be swapped out again under memory pressure.
- On kernels below 5.14, `precommit` throws. `MADV_POPULATE_WRITE` does not exist there.
- The payload of a dGPU channel is not in this mapping. Only the control plane is covered.

`pages_committed()` and `pages_locked()` report what was actually obtained. With the default policy both are false. If it was requested and refused, the result is a throw, not false.

### FrameView (what the callback receives)

```cpp doc:frame_view_api
bool valid = static_cast<bool>(v);       // operator bool is explicit
const void * data = v.data();
std::size_t size = v.size();
v.release();                             // the destructor does the same thing
```

It is valid only while the callback runs. It is move-only, so it is not copied. `meta()` is not used on the adapter path. The schema is in the fingerprint. The place where it is read directly is section 3 of [`raw_api.en.md`](raw_api.en.md).

### GPU (CUDA)

Attaching `Device::Cuda` turns the same channel into the GPU path. Without it, `Device::Cpu` is the default and the sections above apply as is.

```cpp doc:gpu_publisher
flux::ros::Publisher pub(*node, "img", fingerprint, slot_size, slot_count, flux::Device::Cuda);

flux::WriteSlot w = pub.loan(flux::DType::U8, {480, 640, 3});
if (w) {
  render_into(w.device_ptr(), w.capacity(), w.stream());   // launch the kernel on this stream
  if (const flux::Published p = w.commit(); flux::faulted(p)) log("img", flux::to_string(p));  // waits for the stream, then publishes
}
pub.fence_failed();
pub.fence_wait();
```

```cpp doc:gpu_subscription
flux::ros::Subscription sub(
  *node, "img", fingerprint, flux::QoS{},
  [](const flux::FrameView & v) { use(v.device_ptr(), v.size(), v.stream()); },
  flux::Device::Cuda);

sub.fence_failed();
sub.fence_wait();
```

These are the differences from CPU.

| | CPU | CUDA |
| --- | --- | --- |
| Constructor | none | `flux::Device::Cuda` |
| Address | `data()` | `device_ptr()` |
| Kernel argument | -- | pass `stream()` along |

`device_ptr()` is `nullptr` unless the device is `Device::Cuda`. Do not pass `data()` to a kernel. The address the GPU sees differs by hardware.

`host_addressable()` is the opposite question: may `data()` be read by the CPU. On an iGPU the slot is also host memory, so it is `true`. On a dGPU the slot is VRAM, so it is `false`. Do not decide this by whether `device_ptr()` is non-`nullptr`. The iGPU is the only case where both hold. `FrameView`, `WriteSlot`, and `Channel` all have an accessor of the same name.

If `host_addressable()` is `false`, `data()` is `nullptr`. This stops a caller that did not ask right there, instead of at the fault on first access. `wire::Reader` and `wire::Writer` latch `bad()` on a null base, so the generated adapter's `View` and `Builder` get `ok__() == false` and `commit__()` refuses.

On a dGPU, four things are closed. On an iGPU the slot is also host memory, so everything is open.

| Closed | Instead |
| --- | --- |
| `publish(host array)` | `loan()` + `commit()` |
| `Device::Cpu` subscription | Refused at attach. Only GPU subscriptions attach |
| Python `.array` · `.bits` | `__cuda_array_interface__` or `__dlpack__` |
| C++ `data()` and the adapter on top of it | `device_ptr()`. The flat wire is a host layout and cannot be used |

`stream()` returns the stream declared on the channel. Use it instead of carrying your own stream variable. If the stream flux waits on differs from the stream the work was launched on, the wait is meaningless.

`commit()` and view release happen after waiting for the stream. So "published" means "safe to read", exactly as on the CPU. On an iGPU, a node subscribing with the CPU runs unchanged on the same channel, because the slot is also host memory. A dGPU does not have that property, so CPU subscriptions are refused (table above).

If the wait fails, flux releases nothing. That slot is never used again and `fence_failed()` goes up. A non-zero value is a fault, not a rate.

`fence_wait()` reports how long that wait actually took. One `Channel::FenceWait` holds the two seams separately.

| Field | Meaning |
| --- | --- |
| `commit_ns` · `commit_count` · `commit_max_ns` | Publish seam. Sum, count, maximum |
| `release_ns` · `release_count` · `release_max_ns` | Release seam. The same three |

The seams are kept separate because they block different threads. The publish side blocks the publishing thread, and that thread can fit other work between the kernel launch and `commit()` to reduce the wait by that much. The release side blocks the thread that releases the view, and there is no way to reduce it. The fence is per stream, so holding a view for a long time also waits for everything that accumulated meanwhile. Receiving, using, and releasing immediately is best, and the callback path already does that.

On a `Device::Cpu` channel everything is 0. Not even the clock is read. A failed wait also spent time, so it is counted too. The values are cumulative. Only the two `_max_ns` fields are the maximum so far and never decrease.

The constructor refuses on the spot a declaration this host cannot honor. Without a GPU it throws. It does not silently fall back to CPU. On an iGPU that requires registration, instead of refusing it registers the payload, and throws if the driver rejects that registration.

### FrameMeta

This is the descriptor carried in the slot with every frame. A frame written by a generated adapter is a byte string the engine does not interpret, so it is stamped as `u8[nbytes]`. The schema is in the fingerprint, not in the meta, so on this path there is no place that reads or writes `FrameMeta`. The meaning of the fields and how to read them are in section 2 of [`raw_api.en.md`](raw_api.en.md).

### Executor

Waits on flux subscriptions and ROS subscriptions together in a single io_uring. On Linux below 6.7 it falls back to a thread per channel. The same fallback serves a host that forbids io_uring (a container seccomp profile, `kernel.io_uring_disabled`), and that case prints one line to stderr per process saying so. `FLUX_DISABLE_IO_URING=1` chooses the fallback and silences the line. Any other failure to set up io_uring (an fd or memlock limit, memory) makes the constructor throw rather than run on the fallback. `uses_io_uring()` reports which path is in use.

The wait itself is `flux::Executor` (flux_core). The part that does not know ROS lives there. What this class adds is the bridge that forwards readiness into that wait, plus the rclcpp execution path. `flux_py` uses the same `flux::Executor` (`core_api.en.md` 6).

```cpp doc:executor
flux::ros::Executor ex;
ex.add(flux_sub);                // throws if the subscription has no callback
ex.add(control_sub, 10);         // priority: runs first when it has a frame. default 0
ex.add_ros_node(node);           // pass the whole node. rclcpp takes out subscriptions, timers, and services
ex.spin();                       // tick_ns defaults to 100 ms. change it with spin(tick_ns)
ex.stop();                       // ends spin. may be called from a callback
```

The second argument of `add` is the priority. Larger goes first, ties take turns a frame at a time starting in registration order, and negative values are allowed. The choice is made again for every callback. If a frame arrives on a higher channel while a lower channel's callback is running, it runs before the lower channel's next frame. It is not a preemptive priority. A callback that is already running is not displaced, and the bound of one pass does not change. Only flux channels are ordered. ROS entities in the same executor are run by `pump_ros()` in rclcpp order.

Registration happens only before spin. `add`/`add_ros_node`/`add_ros_callback_group` during spin throw. The spin thread walks the registration list without a lock. After spin returns, registration is possible again. ROS subscriptions created late on a node are the exception: the next pass connects them automatically (below).

`add_ros_node` registers the node with rclcpp and installs an on-new-message callback on each of that node's subscriptions that forwards readiness to this executor's eventfd. flux does the waiting and rclcpp does the taking. rclcpp already has the branches for serialized subscriptions, intra-process, and loaned messages, and taking by hand would mean rewriting all of it. In return, timers and services run as well.

Subscriptions that did not exist at call time are connected by the rescan at the start of the next spin pass. When both callbacks are installed, the pending count is replayed, so messages that arrived before the bridge was installed also wake that pass. The detection latency bound is one tick.

`tick_ns` is not a polling period. It is the wait bound for rechecking stop and for attaching a publisher that came up late. If the node has a timer with a period shorter than the tick, the wait only reaches that deadline, so a long tick does not lengthen the timer period.

The way to end is `stop()`. If you already hold your own shutdown flag, `spin(run, tick_ns)` watches that too. A context shutdown also ends it, as it ends an rclcpp executor: `rclcpp::shutdown()`, which Ctrl-C triggers after `rclcpp::init`, returns `spin()` of this executor and of `PartitionedExecutor` without an `rclcpp::on_shutdown` hook.

Three of the inherited rclcpp entry points have exactly one meaning in flux and are implemented. `spin()` is `spin(tick_ns)` with the default tick, `cancel()` is `stop()`, and `spin_once(timeout)` is `spin_once(timeout.count())`. So a caller holding this as `rclcpp::Executor &` drives this executor correctly. On Humble, `cancel()` is not virtual in rclcpp, so that call through `rclcpp::Executor &` only clears rclcpp's own flag and does not stop this executor. Stop it with `stop()` or `cancel()` on the flux type there. The rest (`spin_some`, `spin_all`, `spin_node_some`, `spin_node_all`, `spin_until_future_complete`) throw. They are contracts about a wait set and a duration budget that this executor does not have, and a plausible approximation would silently skip flux channels.

```cpp doc:executor_api
bool merged = ex.uses_io_uring();   // false means it is running on the per-channel thread fallback
std::size_t channels = ex.size();   // number of registered flux subscriptions
ex.interrupt();                     // keeps the loop, wakes only the wait
int frames = ex.dispatch();         // delivers only ready flux frames, without blocking
ex.wait_for_work(1'000'000);        // waits only, without delivering
int ros_ran = ex.pump_ros();        // runs what rclcpp reported ready. bounded by ros_budget
ex.set_ros_budget(16);              // ROS callback bound per pass. default 64. before spin only
int budget = ex.ros_budget();
ex.set_pass_budget(16);             // flux callback bound per pass. one for all channels. before spin only
int flux_budget = ex.pass_budget();
bool more = ex.has_more();          // the budget cut off a remainder. skip the next wait
bool ros_woke = ex.take_ros_ready();  // false means nothing arrived from ROS. pump_ros can be skipped
```

`has_more()` reports whether the previous `dispatch()` stopped at the budget with frames remaining. A loop that calls `wait_for_work()` and `dispatch()` directly skips the wait when this is true. Nobody knocks on the ring again for frames that were already there. `spin()` and `spin_once()` handle it themselves. `spin_once()` throws `std::logic_error` while `spin()` runs, since both would drive the same ring. Python's `flux.ros.Executor.spin_once()` raises `RuntimeError` in the same case. `spin_once(timeout)` returns on the first flux frame or ROS work, whichever arrives first, and otherwise at the timeout. Python's `spin_once()` does the same.

`pass_budget` is the same thing on the flux side. It is the bound on the number of callbacks one `dispatch()` runs, and it is one budget for all channels, not per channel. The default is `flux::kMaxDrain` (64). Lowering it reduces the time a high-priority channel waits behind a low one. Raising it spreads the per-pass arm cost over more frames.

`ros_budget` is the bound on the number of ROS callbacks one pass runs. Without it, `pump_ros()` runs until the rclcpp queue is empty, and a ROS stream faster than its callback holds that thread until the stream ends. Meanwhile, flux frames in the same executor do not run. The remainder beyond the bound is picked up by the next pass without blocking, so it bounds pass length, not throughput. The bound of one pass includes `ros_budget x ROS callback WCET`.

The three below are for putting flux on top of someone else's event loop. `dispatch()` takes out frames, `pump_ros()` runs ROS, and `wait_for_work()` does the blocking. `take_ros_ready()` reports whether a bridged ROS entity signaled since the previous call and clears it on read. When false, `pump_ros()` and the rclcpp pass it entails can be skipped. `spin()` makes that decision this way. The hook sets it directly rather than reading from the ring because an arrival during callback execution lands after that pass's wait has ended. `dispatch()` and `wait_for_work()` share the same ring, so they must not run concurrently. The caller owns that handshake. The rclpy bridge in `flux_py` is built on these three.

### message_filters synchronization

The standard ROS path for pairing several topics by time and receiving them in one callback is [`message_filters`](https://index.ros.org/p/message_filters/). A flux topic goes into that graph as a source. flux topics can be combined with each other, or mixed with ROS topics.

```cpp doc:adapter_cpp_sync
namespace mf = message_filters;
using sensor_msgs::flux_msg::Image;
using Frame = flux::ros::message_filters::StampedFrame<Image>;

// max_borrow = inputs x queue_size: the filter holds frames until a match arrives.
const auto qos = flux::QoS(4).max_borrow(16);

flux::ros::message_filters::Subscriber<Image> left(node, "cam/left", qos);
flux::ros::message_filters::Subscriber<Image> right(node, "cam/right", qos);

using Policy = mf::sync_policies::ApproximateTime<Frame, Frame>;
mf::Synchronizer<Policy> sync(Policy(10), left, right);
sync.registerCallback(std::bind(
  [](const std::shared_ptr<const Frame> & a, const std::shared_ptr<const Frame> & b) {
    Image::View l = a->view();
    Image::View r = b->view();
    use(l.width(), r.width());
  },
  std::placeholders::_1, std::placeholders::_2));

flux::ros::Executor ex;
ex.add(left);   // every input of one synchronizer must run on one thread
ex.add(right);
ex.spin();
```

What goes into the queue is a `StampedFrame`, not bytes. It holds one borrow and a stamp. The payload stays in the segment, so this path has no copy either. `view()` returns the adapter's `View`.

- The synchronization key is the message's header stamp. `Subscriber` reads only `header__sec()` and `header__nanosec()` from the frame and stores them in `StampedFrame::header.stamp`, a `std_msgs::msg::Header` whose other fields stay empty. That is where upstream's default `TimeStamp` trait reads, and where Python keeps it. A schema without a header cannot enter here. `FrameMeta` carries no time, so there is no alternative, and compilation stops right there.
- `Subscriber` is a `flux::Source`. Put it in the executor directly with `ex.add(sub)`. It can also be default-constructed and attached later with `subscribe(node, topic, qos)`, which is needed when it is declared as a node member. `subscribed()` reports whether it is attached and `unsubscribe()` detaches it. While detached, the executor receives nothing from this source, and that is all. Calling `subscribe()` again while the executor spins is allowed from the spin thread, in a callback or a timer. The executor waits on the new topic from its next wait on.
- `getTopic()` and `getSubscriber()` are upstream's accessors. `getSubscriber()` returns the inner `flux::ros::Subscription` for its counters (`lost()`, `refused()`), or `nullptr` while unsubscribed.
- `Subscriber<Image>::Message` is the type that goes into the queue, that is, `StampedFrame<Image>`. Either name works as the type argument of a `Synchronizer` policy.
- `forwarded()` is the count sent on to the filter, and `unreadable()` is the count discarded for not matching the schema. The synchronizer silently discards messages it cannot pair, so diffing `forwarded()` against the number of user callbacks is how to see that loss.
- It holds `queue_size` (the `Policy(10)` above) per input, so `max_borrow` must cover that product. Otherwise the consumer uses up its own leases and cannot take more.

**Every input of one synchronizer must be serviced on the same thread.** This is not a style rule. The synchronization policy runs the matched callback while holding its own `std::mutex`. If the inputs span two threads, one thread waits on the other through that lock, and the two are no longer isolated. `flux::ros::Executor` runs flux and ROS on one thread, so it satisfies this automatically. In `PartitionedExecutor`, assign all inputs of that synchronizer to the same callback group.

In `PartitionedExecutor`, declare that assignment and spin checks it. A synchronizer tells nobody what its inputs are, so the set has to be supplied from outside.

```cpp
ex.add(flux_in, g);
ex.add_sync_group(flux_in, ros_in);   // these two are the inputs of one synchronizer
ex.spin();                            // throws here if they are in different groups
```

- `add_sync_group` accepts flux inputs and stock `message_filters::Subscriber` together. A graph that mixes flux topics and DDS topics is the target of this check. Those are taken out by different machinery, `dispatch()` and `pump_ros()`, so only the same group puts both on one thread.
- An unassigned flux input throws. No child runs it, so the match never arrives.
- Inputs whose place cannot be determined are not judged, and `unplaced_sync_inputs()` counts them. These are filters in the middle of a chain, and subscriptions of a node not passed to `add_ros_node()`. 0 means every declared input was judged.
- Without a declaration there is no check. Same as before.

The filter path is a path with allocation. The policy's queues are `std::deque` and `std::map`, and every frame carries two `shared_ptr` control blocks. flux delivery without filters has no allocation.

### PartitionedExecutor

Attaches a child executor and one thread to each callback group, isolating the groups from each other. A group containing flux subscriptions is handled by a child `flux::ros::Executor` (one io_uring per group), and a group without them by the stock `SingleThreadedExecutor`.

The one-thread-per-group arrangement was informed by autowarefoundation/callback_isolated_executor and agnocast's `CallbackIsolatedAgnocastExecutor`. No implementation is shared.

```cpp doc:partitioned
flux::ros::PartitionedExecutor ex;
ex.add(flux_sub, group);   // assign to a group. same group = same thread
ex.add_ros_node(node);     // a child is attached to each of the node's callback groups
ex.on_thread_start(group, [] { set_up_this_thread(); });  // runs on the group thread before its first callback
ex.spin();                 // tick_ns defaults to 100 ms
ex.stop();                 // ends spin and every child
ex.interrupt();            // wakes only the parent's scan wait
```

- The group given to `add` must belong to a node registered with `add_ros_node`. Otherwise spin throws.
- The group given to `add` may have ROS entities. That group's ROS callbacks are serviced by the same child. Even when mixed with flux callbacks in one pass, a ROS entity is pushed back by at most one callback.
- The third argument of `add(sub, group, priority)` is the visiting order within that group's own pass. It does not cross groups. Groups are separate threads.
- Registration happens only before spin. `add`/`add_ros_node`/`on_thread_start` during spin throw.
- A callback group created after spin started is picked up by the tick scan and gets a child.
- Each child thread is named `flux-part-g<i>` on the OS thread, so `top -H`, `perf` and `gdb` show which group it serves. Python names its group threads the same way and its node threads `flux-part-n<i>`.
- `tick_ns` is used as given, below a millisecond included. Nothing rounds it. A shorter tick finds new groups sooner and costs more wakeups. Python's `PartitionedExecutor.spin(tick_ns)` is the same.
- `on_thread_start(group, fn)` runs `fn` on the child thread for that group before its first callback, for setup only that thread can do for itself. An exception from `fn` surfaces as an error from spin(). One per group. spin refuses a hook for a group no child handles.
- Reentrant groups are refused. There is one thread per group, so that group's callbacks run serially, and a group that declared concurrent execution is not silently serialized. If parallelism is needed, split into several mutually exclusive groups. Each gets its own thread. A Reentrant group handed to `add` is refused at that call; the node's own groups are checked at spin.
- Groups a node created through automatic registration get a child even without flux subscriptions. Manual groups with `automatically_add_to_executor_with_node()` false are serviced only when assigned with `add(sub, group)`.
- Callbacks in different groups actually run concurrently. State shared between groups is protected by the caller. PartitionedExecutor provides isolation, not mutual exclusion (6).
- Put a subscription receiving a GPU channel in its own group. The release fence blocks that thread until the consuming kernel finishes, so other callbacks in the same group are delayed by that much.

## 4. Python

```python doc:py_imports
import flux
import flux.ros
```

### Publisher

```python doc:py_publisher
pub = flux.ros.Publisher(node, "img", fingerprint=FP, slot_size=16 << 20, slot_count=16)

dropped = pub.dropped                           # all three are properties
slot_size = pub.slot_size
segment_name = pub.segment_name
```

Publishing is done by the adapter. `Cloud.build__(pub)` loans a slot and `commit()` publishes (section 2). The surface for putting in and taking out an ndarray directly is section 4 of [`raw_api.en.md`](raw_api.en.md).

### Subscription

```python doc:py_subscription
sub = flux.ros.Subscription(node, "img", callback=_sink, fingerprint=FP, qos=flux.QoS())

newest = sub.peek()                        # latest, not consumed. None if none
nxt = sub.take()                           # next one, consumed. None if none
blocking = sub.take_blocking(timeout_ns=-1)  # until one arrives. None on timeout. Ctrl-C is KeyboardInterrupt
lost = sub.lost                            # all are properties
qos = sub.qos
segment_name = sub.segment_name
domain = sub.domain                          # this process's domain. pub.domain is the same

if nxt is None and not sub.can_borrow:     # empty because I have not released a view
    held = sub.refused.max_borrow          # how many times that happened
if nxt is None and not sub.attached:       # no publisher in this domain
    where = sub.domain                      # distinguishes it from one in another domain
```

What it returns is a read-only numpy view. The data points straight into shared memory. The borrow is released when the last reference goes away, so set `v = None` or let it go out of scope.

### When an empty result comes back

`None` (an empty `FrameView` in C++) means one of two things. Either there is no frame yet, or it was refused for a reason unrelated to frames. The latter is counted by `refused`. These are things caught by neither `lost` nor `dropped`.

| `flux.Refused` field | When |
| --- | --- |
| `max_borrow` | Called again without releasing a held view. Does not clear when a publish arrives |
| `holder_table` | `kMaxHolders` (10) processes already hold one slot |
| `contended` | seqlock validation used up its retry budget (64 attempts) |
| `bad_frame` | The meta is outside the slot range, or no frame is committed in that slot |
| `no_owner_file` | This process could not acquire the owner file (fd exhaustion) |
| `fence` | A failed fence leaked all `max_borrow` leases. This consumer is finished |
| `total` | Sum of the six above |

Only `max_borrow` can be asked about in advance. If `can_borrow` is false, the next `take` returns an empty result regardless of whether a frame exists. `take_blocking` does not park either and returns immediately, so a retry loop here only burns CPU. Release the view first.

`fence` has no remedy. Leaked leases do not come back, so releasing views does not clear it and retrying is pointless. Discard that channel and open it again. That is why it is counted separately from `max_borrow`. One is fixed by the caller and the other cannot be.

The remaining five can be retried. That is normal operation.

### When diffing counters

`lost` and `refused` are both cumulative, but they diverge on reconnect (publisher restart).

| Counter | On reconnect | Why |
| --- | --- | --- |
| `lost` | Goes back to 0 | The new segment counts tickets from 1 again. The count of frames not received is a value tied to the stream, so appending the old stream's value to the new stream is meaningless |
| `refused` | Continues | Refusals are a value tied to this consumer. A segment change does not undo them |
| `dropped` | Does not continue | It is a publisher-side counter. Separate per segment |

So when computing a per-second rate, the diff of `lost` can be negative. `attach_generation()` (Python `attach_generation`, on `flux::ros::Subscription` and `flux::Channel` alike) changes on every reconnect, so use the diff as a rate only when the generation of the two samples is the same. If it differs, that interval is a restart, not a rate.

### GPU (CUDA)

Attaching `device="cuda"` turns the same channel into the GPU path. Without it, the host path applies as is.

```python doc:py_gpu_publisher
pub = flux.ros.Publisher(node, "img", fingerprint=FP, device="cuda")

loan = pub.loan((480, 640, 3), dtype="uint8")
if loan:
    render_into(cp.asarray(loan), loan.stream)   # launch the kernel on this stream
    loan.commit()                                # waits for the stream, then publishes
fence_failed = pub.fence_failed
worst_commit_ns = pub.fence_wait.commit_max_ns
```

```python doc:py_gpu_subscription
sub = flux.ros.Subscription(node, "img", fingerprint=FP, qos=flux.QoS(), device="cuda")

frame = sub.take()
if frame:
    with frame as v:
        use(cp.asarray(v), v.stream)             # scope exit: waits for the stream, then releases

fence_failed = sub.fence_failed
worst_release_ns = sub.fence_wait.release_max_ns
```

`device` accepts the strings `"cpu"` / `"cuda"` or `flux.Device.CPU` / `flux.Device.CUDA`. A declaration this host cannot honor is refused by the constructor with `ValueError`. A declaration that cannot be kept is not silently sent down to host.

There are only three differences from the host path.

| | host | CUDA |
| --- | --- | --- |
| Constructor | none | `device="cuda"` |
| Address | `.array` (numpy) | `__cuda_array_interface__`, which `cp.asarray()` picks up |
| Release | GC | `with` scope |

`loan` provides both. `.array` is the host view, and `cp.asarray(loan)` is the device view of the same bytes. In ShmDirect the two are the same address, and which one you take decides whether a kernel or the CPU produces that frame.

What `take()`/`peek()` return with `device="cuda"` is a `flux.Frame`, not a numpy array. Outside `with`, `__cuda_array_interface__` refuses.

| `flux.Frame` | |
| --- | --- |
| `with frame as v` | Binds the borrow to this scope. On exit, waits for the stream, then releases |
| `v.__cuda_array_interface__` | Read-only device view. `RuntimeError` when called outside the scope |
| `v.__dlpack__()` · `v.__dlpack_device__()` | DLPack capsule. `torch.from_dlpack(v)` picks it up. The scope rule is the same as CAI |
| `v.bits` | The same bytes as an unsigned numpy view of the same width. Interpretation is up to the caller |
| `v.stream` | The stream to launch the consuming kernel on. Exactly the one the release waits for |
| `v.shape` · `v.dtype` · `v.nbytes` | Frame shape. Readable outside the scope too |
| `v.released` | Whether it has already been released |

The scope is required because the release point is the GPU synchronization point. A numpy view has GC lifetime, so that point is nondeterministic and the stream wait would happen on an arbitrary thread. Using `take()` without `with` silently brings that nondeterminism back, so it is refused.

`publish(array)` accepts host arrays only. Device arrays are refused with `ValueError`, which points to `loan()`. flux copies host bytes into the slot, and moving device bytes is a separate CUDA copy. The way to publish from the GPU is `loan()`, and that path has no copy at all.

`fence_failed` is the number of slots that could not be released because the stream wait failed. A non-zero value is a fault, not a rate. `fence_wait` is the time the two seams actually blocked, and one `flux.FenceWait` holds `commit_ns` · `commit_count` · `commit_max_ns` · `release_ns` · `release_count` · `release_max_ns`. On a host channel both are 0.

### bfloat16

bf16 does not appear on the adapter path. ROS IDL has no such type. The path splitting because numpy has no name for it is also a matter of the raw path, so the whole surface is in section 4 of [`raw_api.en.md`](raw_api.en.md).

### QoS

```python doc:py_qos
qos = flux.QoS(depth=1, durability=flux.Volatile(), max_borrow=2)

volatile = flux.Volatile()          # only what is published after attaching
transient = flux.TransientLocal(n)  # replays first n of what remains in the ring

kind = qos.durability               # flux.Durability. both give this type
live_only = kind.is_volatile
replay = kind.replay                # n for TransientLocal, 0 for Volatile
```

An invalid combination is a `ValueError`. The table is in section 5.

### Executor

Runs ROS callbacks and flux callbacks on one thread.

```python doc:py_ros_executor
ex = flux.ros.Executor()
ex.add(sub)                  # the subscription holds the callback
ex.add_ros_node(node)             # pass the whole node. same as C++ add_ros_node
ex.spin()                         # until stop(). tick_ns defaults to 100 ms
ex.spin_once(timeout_ns=100_000_000)
merged = ex.uses_io_uring
ex.interrupt()                    # keeps the loop, wakes only the wait
ex.stop()
ex.close()                        # after the last stop(), before destroying the node

resolved = flux.ros.resolve(node, "img")   # absolute name with node namespace and remaps applied
```

The assembly order and names are the same as C++. Create it, hand over flux subscriptions and nodes, and spin. ROS subscriptions are not passed separately. Once created on the node, they are already serviced. On a context shutdown, `spin()` ends the way the rclpy executor it wraps ends: the default `SingleThreadedExecutor` raises `ExternalShutdownException`, which rclpy programs catch around `spin()`. `PartitionedExecutor` returns quietly, as `MultiThreadedExecutor` does.

`close()` detaches the node from the rclpy executor. If you destroy the node without calling it, the bridge thread may schedule a task against a node that no longer exists.

Without any node, it is an executor that runs flux only. It is the same as using it without `add_ros_node` in C++, and the arrangement where `rclpy.spin(node)` handles the ROS side on another thread (`split_sub` in `flux_example_executor`) is that case.

rclpy does not expose the on-new-message callback to Python. So it cannot merge into one ring the way C++ does. Instead, a separate thread waits on the flux side and hands the dispatch to the spin thread with `create_task`, and everything that touches the channel happens only on the spin thread. Tasks are a feature of the executor itself, so the same bridge runs whichever rclpy executor is passed as `rclpy_executor=`.

### PartitionedExecutor (Python)

Attaches one thread per partition unit. In the C++ version (above) one callback group is the unit, but rclpy has no `add_callback_group`. The executor's unit is the node (`add_node` uses `node.executor`). So the unit splits in two.

```python doc:py_partitioned
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
ex = flux.ros.PartitionedExecutor()
ex.add_ros_node(node)                                       # one thread per node (ROS callbacks)
ex.add(sub_a, MutuallyExclusiveCallbackGroup())        # one thread per group (flux frames)
ex.add(sub_b, MutuallyExclusiveCallbackGroup(), priority=10)
ex.spin(tick_ns=100_000_000)                                # until stop(). this thread runs no callbacks
ex.stop()
```

- The group given to `add` must have no ROS entities at all. rclpy cannot hand a group to a child executor, so that group's ROS callbacks would stay on the node thread and only flux frames would run here. The group's mutual exclusion would break silently. spin refuses it, and the tick checks again during spin.
- A group that must run ROS and flux on one thread is `flux.ros.Executor`. That is that class's job.
- Reentrant groups are refused for the same reason as C++.
- `priority` is the same as C++. When a group has several subscriptions, it decides the visiting order within that group's pass, and does not cross groups.
- Registration happens only before spin. `add`/`add_ros_node` during spin throw.
- An exception in a child thread stops every child and is rethrown from `spin()`.
- `spin()` tears its child threads down before it returns, so there is no `close()`. A stopped executor can spin again, as in C++.
- The child for a flux group runs `flux.Executor.spin` directly, without the rclpy bridge. The group has no ROS entities, so there is nothing to merge.
- A child thread's setup goes in `on_thread_start` (next section).
- With more threads, only the parts that release the GIL overlap. numpy, zlib, and decoding overlap. Pure Python bytecode is serial no matter how many threads there are.

When looking only at flux channels, the inner `flux.Executor` may be used directly. It is a surface without rclpy, so section 8 of [`core_api.en.md`](core_api.en.md) covers it. `spin_once`, `stop`, `is_spinning`, and the `wait_for_work`/`dispatch` split used when embedding into another event loop are there.

### on_thread_start (Python)

Runs a function on a child thread before its first callback, for setup only that thread can do for itself.

```python doc:py_thread_start
ex = flux.ros.PartitionedExecutor()
ex.add(sub, group)
ex.add_ros_node(node)
ex.on_thread_start(group, set_up_this_thread)
ex.on_thread_start(node, set_up_this_thread)
```

- The `unit` of `on_thread_start(unit, fn)` is either a callback group given to `add` or a node given to `add_ros_node`. In C++ it is one callback group, but here the unit splits in two.
- An exception from `fn` stops every child and is re-raised from `spin()`. The thread does not go on to run its callbacks without the setup.
- One per unit, refused on the spot. A hook for a unit this executor does not run is refused by `spin()`.

### message_filters (Python)

`import flux.ros.message_filters`. `flux.ros` itself does not import this module, so the upstream `message_filters` dependency is not pulled in when it is not used.

```python
import message_filters
import flux.ros.message_filters as fmf
from sensor_msgs_flux.image import Image

left = fmf.Subscriber(node, Image, "left", qos=flux.QoS(depth=8, max_borrow=32))
right = message_filters.Subscriber(node, sensor_msgs.msg.Image, "right")   # DDS topic

sync = message_filters.ApproximateTimeSynchronizer([left, right], 10, 0.02)
sync.registerCallback(lambda a, b: use(a.view().width, b.width))

ex = flux.ros.Executor()
ex.add(left)      # the flux side by the flux executor, the DDS side by rclpy, on the same thread
ex.add_ros_node(node)
ex.spin()
```

It has the same shape as the C++ version (section 3). Where upstream's own C++ and Python differ, each follows its upstream: the inner subscription is `.sub` here and `getSubscriber()` in C++, and a late `subscribe()`/`unsubscribe()` exists only in C++, as upstream has it only there. `getTopic()` and `header.stamp` are the same in both. Beyond that there are three differences.

- The second argument is the adapter module. It subscribes with `FINGERPRINT__` and reads with `View`. `m.view()` returns that `View`.
- The queue holds the frame objects as they are. What the Python callback receives is an object that already holds its own borrow, so there is no need to transfer ownership with `take()` as in C++.
- A schema without a header is a `TypeError` from the constructor. C++ stops at compilation where the `Subscriber` is declared.

Two values need attention.

- `depth`. The default of 1 gives only the latest one per wake. Intermediate frames vanish without reaching the filter, and it only looks like the pairs do not match. Raise it for synchronizer inputs.
- `max_borrow` and `slot_count`. A frame in the queue is a held borrow. `max_borrow` must cover `inputs x queue_size`, and the publisher's `slot_count` must have that much headroom too. Otherwise the consumer cannot take more and the publisher has no slot to write.

`forwarded` is the count sent on to the filter, and `unreadable` is the count discarded for not matching the schema. They are the same as C++ and used the same way. The synchronizer silently discards what it cannot pair, so the difference between `forwarded` and the number of user callbacks is that loss.

In `PartitionedExecutor` it diverges from C++.

```python
ex.add(left, g)
ex.add(right, g)
ex.add_sync_group(left, right)   # these two are the inputs of one synchronizer
ex.spin()                        # throws here if they do not land on one thread
```

- flux inputs only need to be in the same group. Same as C++.
- A synchronizer mixing flux inputs and DDS inputs is refused. rclpy has no `add_callback_group`, so there is no way to move a DDS input onto the flux group's thread. The place for that graph is `flux.ros.Executor` (`contracts.en.md` S-003).
- Stock ROS inputs spanning two nodes are refused too. Each node gets its own thread, so that is a split arrangement as well.
- Inputs whose place cannot be determined are counted by `unplaced_sync_inputs()`. These are filters in the middle of a chain, and `Cache`. 0 means every declared input was judged.

### Enumeration (for tools)

`flux.enumerate_topics()` and `flux.read_channel_stats()`, which read the flux channels on this host and the participants attached to them, do not use a node. They are in section 9 of [`core_api.en.md`](core_api.en.md), and the command-line tool built on top of them is [`cli.en.md`](cli.en.md).

## 5. QoS on one page

| Value | Default | C++ | Python | Meaning |
| --- | --- | --- | --- | --- |
| `depth` | 1 | `QoS(n)`, `keep_last(n)` | `depth=n` | How many frames behind the latest is acceptable. 1 means latest only |
| `durability` | volatile | `transient_local(n)`, `durability_volatile()` | `durability=flux.TransientLocal(n)` | Whether to receive what was published before attaching. `TransientLocal(n)` replays n |
| `max_borrow` | 2 | `max_borrow(n)` | `max_borrow=n` | Number of views held at the same time |

Each language builds it the way ROS does in that language. C++ chains setters like `rclcpp::QoS`: `flux::QoS(8).transient_local(4).max_borrow(4)`. Python takes keywords like rclpy's `QoSProfile`. A getter has the setter's name with no argument (`qos.depth()`), and there is no field to assign.

Delivery is always best-effort, so there is no reliability setting. Asking for reliable is a compile error in C++ and a `TypeError` in Python, not a setting silently lowered to best-effort.

Rejected: `depth == 0`, `max_borrow == 0`, and `n > depth` in `TransientLocal(n)`. In C++ the first two throw from the setter that writes them. The last one needs two fields, so it throws where the QoS is used (`Channel::qos`, the `flux::ros::Subscription` constructor), before the subscription is announced. Python `flux.QoS(...)` has every field at once and raises all three there as `ValueError`.

A QoS deeper than the publisher's `slot_count` is not an error. It is truncated to what is retained, and the frames not received are counted in `lost`.

## 6. Rules to keep

- Do not hold a view for a long time. The publisher cannot use a held slot, so `dropped` goes up. If you need it for a long time, copy it.
- Do not use one subscription from several threads at the same time. The cursor and reattach state are not thread-safe. Create a subscription per thread.
- The same applies to one publisher. The publish path holds its own identity and whether the owner file is ready in non-atomic members, so two threads calling the same `Publisher` is a race. Multiple publishers across processes are safe, independently of this. In that case the objects are separate and the shared ring handles it.
- When groups are split with `PartitionedExecutor`, callbacks run on different threads. Touching another group's subscription or publisher from a callback in a different group also falls under the two lines above. This includes reading counters. Callbacks within one group share one thread and do not have this problem.
- If a ROS timer is in one executor with flux subscriptions, that timer is delayed by up to one flux callback. The priority of `add` cannot reduce that either. Priority orders only flux channels among themselves. To reduce it further, put that timer in a group without flux using `PartitionedExecutor`. That group has no dispatch in front of it at all.
- Do not drive the same subscription from two places. Two drivers split the stream between them.
- Match the fingerprint of publisher and subscriber. With a bound type this is automatic.
- Do not hand a ROS subscription given to a flux Executor to an rclcpp executor as well.

## 7. What does not exist yet

- Reliability (lossless) and overflow policy (`docs/qos.md` 6). sub_buffer is not a separate knob. The lag bound is `depth`, and reservation is the same blocking as those two.
- Cross-host. All three GPU paths exist: direct iGPU (`ShmDirect`), iGPU requiring registration (`ShmRegistered`), and dGPU handles (`DeviceHandle`). The host decides which one is taken and the surface is the same. The GPU sections in sections 3 and 4 are that surface.
