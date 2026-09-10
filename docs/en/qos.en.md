# flux QoS

Delivery is fixed to best-effort (drop-on-full). The consumer chooses three things: `depth`, `durability`, and `max_borrow`. The first two take their names and meanings from ROS 2 rmw QoS. `max_borrow` has no counterpart (section 2). All three are implemented.

## 1. Two operations: peek / take

Before reading the QoS, distinguish two operations. It is the same distinction as DDS `read()` / `take()`.

| | Cursor | When it returns empty | Where it is used |
| --- | --- | --- | --- |
| `peek()` | Untouched | When nothing has been published yet | Read the current state on the caller's own cycle |
| `take()` | Advances | The above, plus when caught up | Process every frame without gaps |

`peek()` returns the same frame again when there is no new frame. It is a state read. `take()` returns a frame only once. `take_blocking(ns)` parks on a futex when `take()` would return empty. If the empty result is due to `max_borrow` exhaustion, it does not park and returns immediately. Only the caller can release the lease, so waiting would not free it. `peek` has no blocking variant. There is nothing to wait for.

Both also return empty in further cases, regardless of whether a frame exists. `Channel::refused()` counts these by reason. They appear in neither `lost` nor `dropped`, so this is the only place they show. The values are cumulative.

| Why | Counter | When |
| --- | --- | --- |
| `max_borrow` exhausted | `max_borrow` | Calling again without releasing a held view |
| Slot holder table full | `holder_table` | `kMaxHolders` (10) processes already hold the slot |
| Segment not ready yet | `not_ready` | `init_state != ready`. The creator is still bootstrapping |
| Lost the race to acquire the borrow | `contended` | seqlock validation keeps failing up to the retry limit (64) |
| meta outside the slot range | `bad_frame` | A corrupted frame is not handed out as a view |
| No frame in the slot | `bad_frame` | `commit_ticket == 0`. The state left by an aborted loan or a reclaimed claim. Even if `latest` still points at that slot, there is no frame to hand out |
| Cannot acquire the owner file | `no_owner_file` | This process could not open its own owner file, for example due to fd exhaustion |
| All leases leaked by fence failures | `fence` | Failed release fences have leaked `max_borrow` leases. This state is permanent |

"Nothing published yet" and "caught up" are not counted. In those two cases the empty view did its job.

Only `max_borrow` exhaustion needs a different response. Retrying does not clear it and `take_blocking` does not even park, so a loop spinning in place only burns CPU. To ask ahead of time, use `can_borrow()`. The others can simply be retried.

`fence` has no response at all. Like `max_borrow` it refuses because there is no lease, but a leaked lease cannot be returned by the caller. Abandoning the channel is the only answer.

The release side has one counter as well. `Channel::fence_failed()` is the number of times a view could not be released because the GPU stream fence failed. Decrementing the refcount without confirming the fence would let the publisher overwrite a slot that is still being read, so that slot stays locked. On `Device::Cpu` channels it is always 0. CUDA is never called, so nothing can fail.

`Channel::fence_wait()` reports the time spent at the same point. The two seams each report sum, count, and maximum (`api.en.md` GPU section). While `fence_failed()` counts failures, this one counts how long successful waits took. On `Device::Cpu` channels the clock is not even read.

This value stops at `max_borrow`. Once that many leases have leaked, no borrow goes out, so there is nothing left to fence. Read it as a state, not a ratio. A stopped value means that consumer is finished, and calls after that are counted by `refused().fence`.

The callback path (`flux::ros::Subscription`, `Executor`) delivers with `take()`. `peek()` is only for direct pull.

## 2. The three values

```text
depth        Lag bound. How many frames take() may fall behind the newest.
durability   Whether to receive frames published before attaching. Volatile() or TransientLocal(n).
max_borrow   Bound on views held at the same time. No ROS 2 counterpart.
```

| Setting | ROS 2 | Result |
| --- | --- | --- |
| `depth = 1` (default) | History KEEP_LAST(1) | Always the newest only. Intermediate frames are skipped and counted as `lost` |
| `depth = N` | History KEEP_LAST(N) | Falls behind up to N, then catches up in publish order |
| `Volatile()` (default) | Durability VOLATILE | Only frames published after this consumer joined the stream |
| `TransientLocal(n)` | Durability TRANSIENT_LOCAL | First replays n of the frames still in the ring at join time, then goes live |

On every `take()` the cursor is pulled up to `newest - depth`. With `depth = 1` the cursor is always `newest - 1`, so the newest comes out. This is not a separate mode; it is the end value of the same formula.

`depth` does not decide how many callbacks the executor runs in one pass. The next `take()` succeeds again for frames that arrived while a callback was running, so dozens can run in one pass even at `depth = 1`. `pass_budget` is what bounds that number.

The join point is attach, not the first take. If the segment already exists when the subscription is created, it attaches there; otherwise it attaches on the first attempt after the segment appears. So `Volatile()` means "after I attached", not "after I first read".

`max_borrow` (default 2) has no ROS 2 counterpart. It bounds the number of views held at the same time. A held view keeps its slot byte-locked, so the publisher cannot use that slot.

`reliability` accepts only `best_effort`. Passing `reliable` is refused.

## 3. Three depths, kept apart (easy to confuse)

```text
publish -> [ring: slot_count] -> [depth] -> take() -> [max_borrow] -> release
            where frames exist     lag bound              what is held
```

- `slot_count`. Set by the publisher at creation. Corresponds to the ROS 2 publisher History depth. It is the only place frames actually live, so it is the hard cap on the two below.
- `depth`. Set by the subscriber. Lag bound.
- `max_borrow`. Set by the subscriber. Bound on concurrent holds.

The subscriber stores nothing. The bytes exist in one copy in the publisher ring and the subscriber only moves a cursor. `depth` looks like a queue depth because the externally observed behavior is the same.

## 4. Rejected combinations

Nothing is silently changed to another value. `QoS::validate()` throws.

| Combination | Why |
| --- | --- |
| `TransientLocal(n)` with `n > depth` | Replayed frames enter the same lag window. You cannot receive more than you are allowed to fall behind |
| `depth == 0` | Newest-only is `depth = 1` |
| `max_borrow == 0` | If no view can be held, take cannot work |
| `reliability = reliable` | Not implemented (§6) |

QoS deeper than the ring is not an error. The ring belongs to a publisher that may not exist yet. It is truncated to what is stored, and the frames not received are counted in `lost`.

## 5. Usage

```cpp
flux::QoS q;                                        // depth 1, volatile
q.depth = 10;
q.durability = flux::Durability::TransientLocal(5);
ch.qos(q);
while (flux::FrameView v = ch.take()) { use(v); }
```

```python
sub = flux.Subscription("/img")                     # depth 1, volatile
sub.peek()                                          # newest right now
sub.take_blocking(-1)                               # until a new one arrives

qos = flux.QoS(depth=10, durability=flux.TransientLocal(5))
sub = flux.Subscription("/cmd", qos=qos)
while (v := sub.take()) is not None:
    handle(v)
```

`lost` is the cumulative number of frames not received since joining. It has the same meaning as the DDS sample-lost status, so differencing gives a rate.

## 6. Not implemented

| Option | Meaning | What it needs |
| --- | --- | --- |
| sub_buffer | Queue depth of frames that arrived but were not taken yet | Half of it is already done by `depth` (section 3). The other half is a per-subscriber cursor table |
| overflow policy | Whether to overwrite the oldest or block the publisher when full | A publisher policy flag |
| reliability | Delivery guarantee | Shared cursor table + backpressure |

reliability is the largest. For the publisher to wait for the slowest subscriber, a per-subscriber cursor table must live in shared memory, and the publisher must block or drop-newest when the ring fills with unconsumed frames. That is a delivery model change beyond best-effort.

reliability is not a priority. Lossless delivery is a stated non-goal, and if demand to reverse that decision is confirmed, it starts as a separate design.

The overflow policy is not a priority either. flux has no notion of "unconsumed" at all. The publisher searches from the position after `latest`, overwrites the first slot it finds that is not borrowed, and nobody records how far anyone has read. Letting it choose not to overwrite needs that record, and that record is exactly the per-subscriber cursor table that reliability requires. It is not an item that can ship as a flag first.

sub_buffer splits in two. The observed behavior "falls behind up to N, then catches up in publish order" is done by `depth` as is (section 3). Translating a ROS 2 subscription's KEEP_LAST(N) into flux terms gives that name. That half is a matter of matching the name, and it is already matched.

The remaining half is "those N are held for me", and that does not exist. `depth` is only a lag bound over the publisher ring and reserves nothing. If the publisher overwrites a frame I have not taken yet, the cursor is pulled up to `newest - depth` and the difference is counted in `lost`. Holding them would require writing, per subscriber, how far it has not taken into shared memory, and that is exactly the cursor table that overflow and reliability require.

So the three rows of this table are not three items of different size but one and the same blocker. The moment a per-subscriber cursor table exists, all three open together; without it, none of them opens. That table is a delivery model change beyond best-effort, and lossless delivery is a non-goal.
