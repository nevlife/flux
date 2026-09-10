# flux copy model (zero-copy vs one copy)

This is when copies happen in flux. Reading is always zero-copy; whether a copy happens is decided on the publish (write) side. Per-shape mapping is in [message_shapes.md](message_shapes.en.md), GPU in `gpu_copy.md`.

## Reading (receiving): always zero-copy

The receiver gets a view that points directly at the slot. There is no condition that adds a copy. Only the object that carries that view differs by case.

```text
C++    : view.data()          -> span aliasing the slot
Python : sub.take()           -> read-only numpy view
         (declared stream or bf16) -> flux.Frame (__dlpack__ / __cuda_array_interface__)
```

`flux.Frame` is not GPU-only. Both a subscription that declared a stream with `device="cuda"` and a frame whose dtype is bf16 arrive as `Frame`. numpy cannot name bf16, so the place that carries `__dlpack__` has to be an object. A host bf16 `Frame` does not need `with` (`borrow_lifetime.en.md`). Either way the bytes are the slot itself, so it is zero-copy.

Accessors that slice out jagged elements have the same property. The generated adapter's `items[i]` points directly at the element block (`message_shapes.en.md` 4).

## Publishing (writing): two paths

| Path | What | Copies |
| --- | --- | --- |
| loan | Receive an empty slot first and write directly into it | 0 |
| publish | memcpy data that already lives elsewhere into the slot | 1 |

```python
# loan (zero-copy)
loan = pub.loan(shape, dtype)    # claims a slot and returns a handle. None if there is no empty slot
compute_into(loan.array)         # writable numpy pointing at the slot. Throws in the three cases below
loan.commit()                    # publishes here. Discarding without commit does not publish

# publish (one copy)
arr = lib.process(...)           # lives in other memory (heap)
pub.publish(arr)                 # one memcpy into the slot
```

`loan.array` is not always numpy. It throws in three cases. After `commit()` or `abort()` the slot already belongs to someone else, on a dGPU channel whose slot is device memory there is no host address, and numpy cannot name bf16. The latter two are served instead by `loan.bits`, `__dlpack__`, and `__cuda_array_interface__`.

```cpp
// C++ has the same pair.
flux::WriteSlot w = pub.loan(flux::DType::F32, {rows, cols});
if (w) {
  compute_into(w.data(), w.capacity());
  w.commit();
}
```

## Cases that are zero-copy

| Case | Why |
| --- | --- |
| Generating a large uniform buffer directly in the slot (camera -> image, lidar -> cloud) | The data is produced inside the slot |
| Writing a computation result into the loan buffer | Same as above |
| Appending jagged elements one by one into the slot bulk with the builder | The elements are produced inside the slot |

## Cases that are always one copy

| Case | Why |
| --- | --- |
| `string` / `string[]` | The str object owns its own buffer. It must be copied to the tail and cannot be aliased |
| Already produced data (a library returns its own buffer) | Produced outside the slot. Gathered into the slot |
| Objects built one at a time and appended (the `Detection3D[]` convention) | Scattered objects are gathered into the slot |

Whether a copy happens is decided by where the data is produced. Inside the slot is zero-copy, outside is one copy. C++ versus Python has no effect.

string is one copy without exception. Fixed types have the same memory layout as the wire layout, so they are placed as is. string has variable length and its bytes are owned by an object, so it lacks that property.

## Why one copy is still fast

```text
plain ROS: serialize + kernel copy + one copy per subscriber + deserialize
flux one copy: a single memcpy (no serialization, independent of subscriber count)
flux zero-copy: nothing
```

## Current status

| | Status |
| --- | --- |
| Zero-copy read | Implemented (C++ · Python) |
| One-copy publish (`publish`) | Implemented (C++ · Python) |
| Zero-copy publish (`loan`) | Implemented (C++ · Python) |
| `.msg` -> per-field layout adapter | Implemented (`flux_gen`, `message_shapes.en.md`) |
| jagged builder (writes each element into the slot) | Implemented |

Without attaching a type, the unit the API handles is the byte string of one frame (plus shape/dtype), and the per-field layout is decided by the caller (`raw_api.en.md`). Opting in to `.msg` lets the generated `Builder`/`View` take over that layout (`api.en.md` 2).

`loan` claims the slot first and keeps it open, so that slot leaves the ring until commit. Holding it for a long time reduces free slots and increases drops.
