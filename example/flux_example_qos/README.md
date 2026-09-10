# flux_example_qos

What the subscriber's QoS decides. Run the two side by side and watch the counters.

```bash
ros2 run flux_example_loan image_pub          # publisher

ros2 run flux_example_qos qos_newest          # ros2 run flux_example_qos qos_newest.py
ros2 run flux_example_qos qos_backlog         # ros2 run flux_example_qos qos_backlog.py
```

Both nodes log `seen`, `lost`, and `refused` every second.

## qos_newest, depth 1

On every wakeup, the single newest frame. Whatever passed in between is not queued and is counted as `lost`. A consumer that needs only the current state uses this.

```cpp
flux::QoS q;
q.depth = 1;
q.max_borrow = 1;
```

## qos_backlog, depth 8, transient_local(4)

On every wakeup, up to eight frames in publish order. On attach it replays four of what remains in the ring and then follows real time. The callback sleeps 50 ms, so it is slower than the publisher, and that is why `lost` actually moves.

```cpp
flux::QoS q;
q.depth = 8;
q.durability = flux::Durability::TransientLocal(4);
q.max_borrow = 4;
```

`replay` must be at most `depth`. Replayed frames pass through the same lag window, so you cannot receive more than you are allowed to fall behind.

## Three axes

| | Meaning | When exceeded |
| --- | --- | --- |
| `depth` | How far behind the newest you may fall | Frames past it are `lost` |
| `durability` | Whether the ring's backlog is replayed on attach | -- |
| `max_borrow` | How many are held at the same time | Empty frame + `refused.max_borrow` |

When `can_borrow` is false and an empty frame arrives, the stream is not idle. This subscriber is holding its own views. No publish can release that.

`refused` separates why the empty frame came. Folding a normal absence and a refusal into one value makes the latter impossible to find.

`reliability` is `BEST_EFFORT` only. Passing `RELIABLE` makes the constructor throw. flux delivery is best-effort.
