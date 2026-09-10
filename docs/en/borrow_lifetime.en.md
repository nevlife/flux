# flux borrow lifetime (view release protocol)

The view a subscriber receives is a borrowed reference to a shared-memory slot. It is not a copy. This document pins down when that borrow is taken and when it is released. The authoritative sources are this document and the code (FrameView in `flux_core`, Subscription in `flux_cpp`, the `flux_py` bindings).

## What a borrow is

- When a subscriber receives a view via `peek()`/`take()`, the refcount of that slot is incremented (borrow).
- A slot with a held borrow is byte-locked. The publisher cannot overwrite that slot (active => byte-locked).
- When the view is released, the refcount is decremented.
- The number of views one subscriber holds at the same time is limited by `max_borrow` (default 2) (`qos.en.md`). Beyond that, an empty view is returned. Releasing a held one clears it.

A view is a reference to a slot, so it is valid only while held. Copy data that must live longer.

## Two delivery paths

### take / peek (direct pull): lifetime owned by the caller

The caller receives and holds the view, so release happens when that view is dropped. `peek()` and `take()` take the borrow the same way (only the chosen slot and the cursor differ).

C++: FrameView is move-only and RAII. It releases on destruction (scope exit) or `release()`. Deterministic.

```cpp
{ flux::FrameView v = ch.take(); use(v); }            // end of block = destruction = release
flux::FrameView v = ch.take(); use(v); v.release();   // explicit release (v is an empty view afterwards)
```

Python: the numpy view has refcount (GC) lifetime. It releases when the references to the numpy object (and to slices that use it as base) reach 0 and it is collected. The host path exposes no explicit release to Python. Host bf16 frames arrive as `flux.Frame`, but the lifetime is the same. Release is a single decrement, so it happens at GC time. A scope is mandatory only on GPU channels that declared a stream (GPU section below).

```python
v = sub.take(); use(v); v = None   # (del / reassignment / scope exit) references 0 -> release
```

### Callback (executor delivery): owned by the framework during the callback

The executor creates the view, passes it to the callback, and drops its own reference when the callback returns.

C++: the callback receives `void(const FrameView &)`. FrameView is move-only and non-copyable, so it cannot be smuggled out of the callback. When the callback returns, the view is destroyed and **always released deterministically**. To keep it, copy the bytes.

```cpp
// flux::ros::Subscription callback
[](const flux::FrameView & v) { use(v); };   // return = release. Cannot be smuggled out -> copy to keep
```

Python: the callback receives a numpy view. If the callback returns without leaving a reference, it is collected and released right there. If a reference is left (for example `self.last = v`), the borrow persists until that is collected (allowed within `max_borrow`). There is no forced invalidation after the callback.

```python
def on_msg(v):
    use(v)            # if nothing is kept, released at the end of the callback
    # self.last = v   # keeping it holds the borrow (max_borrow pressure, until GC)
```

## GPU channels (`Device::Cuda`): release is the stream fence

If a stream was declared, one wait is attached at the release point. The stream is waited on before the `refcount` is decremented. Otherwise the publisher would legitimately take and overwrite a slot that the consuming kernel is still reading. So "when to release" is "when to wait on the GPU".

The C++ protocol does not change. `FrameView` is already RAII, so the release point is deterministic and the fence simply goes there. On the callback path that point is the callback return.

Python changes. `take()`/`peek()` on a subscription that declared a stream returns a `flux.Frame` instead of a numpy view. There are two conditions under which a `Frame` arrives. One is a declared stream; the other is a bf16 dtype. Only the first requires `with`. The `with` scope binds the borrow, and on scope exit it waits on the stream and then releases. Calling `__cuda_array_interface__` outside the scope is refused. Falling back to GC lifetime would have an arbitrary thread wait on the stream at an arbitrary time, and that is what this scope prevents.

```python
with sub.take() as v:
    use(cp.asarray(v), v.stream)
# scope exit: wait on the stream, then release
```

If the fence fails, the borrow is not released. Instead of treating an unconfirmed completion as complete, one slot is locked. That slot is never used again and the `max_borrow` lease stays locked with it. `Channel::fence_failed()` counts this, and once `max_borrow` leases have leaked, that consumer can no longer borrow at all and ends there (`qos.en.md` 1). If the process dies, the crash reclaim below returns the slot.

The unit of the fence is the stream, not the view. Holding a view for a long time makes its release also wait for the work queued in the meantime. So on GPU channels, receiving, using, and releasing immediately is the only optimum. Raising `max_borrow` does not deepen the pipeline.

## Released / not released

| Path · language | Released | Not released (held) |
| --- | --- | --- |
| take C++ | scope exit · `release()` · reassignment | stored in a member or container · ownership transferred by move |
| take Python | references 0 -> GC | stored in a member or list · slice `v[a:b]` · zero-copy wrapping · reference cycle |
| callback C++ | callback return (always, deterministic) | impossible. const ref and move-only, cannot be smuggled out |
| callback Python | callback return (if no reference is kept) | until GC if a reference such as `self.x = v` is kept |
| take Python (cuda) | `with` scope exit (after the stream wait, deterministic) | a `flux.Frame` taken outside the scope. Using it is refused |

There are two traps. Python GC is non-deterministic, and slices and zero-copy wrapping keep holding the view.

A copy does not hold a borrow. `np.array(v)` is independent memory. Release happens when the original `v` is dropped, and the copy stays safe afterwards.

## On crash (dying before release)

If the process dies while the subscriber holds a view, the refcount remains. When the publisher starves for slots, it identifies the dead borrower via the OFD lock and reverts that refcount (`reclaim_dead`). Nothing leaks forever. Still, on the normal path the rule is to release quickly.

## Rules (summary)

1. Callback: process inside the callback and do not smuggle it out -> automatic release. Enforced in C++; in Python, as long as no reference is kept.
2. take: keep the scope narrow. A block in C++, a short function or `del v` in Python.
3. Copy data to keep (`np.array(v)` / memcpy) -> the borrow is freed immediately and the copy is unrestricted.

A view is a borrowed reference. It is valid only while held; copy data that must outlive it.

## Determinism differences per language

- C++: deterministic through FrameView RAII. Callbacks take a const ref of a move-only type, so smuggling out is impossible and release always happens at callback end.
- Python (host): numpy GC lifetime, so release is non-deterministic if a reference remains. The protocol of invalidating the view after the callback (forcing a copy to keep) is currently not adopted. Adopting it would make the host path callback deterministic like C++. Open decision.
- Python (cuda): deterministic. The `with` scope of `flux.Frame` enforces it. The release point is the stream synchronization point, so no non-determinism could be left (GPU section above). Closing the open decision on the host path makes the two paths the same.
