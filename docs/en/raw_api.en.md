# flux raw path API

This is the surface used without a `.msg`. The default path is [api.md](api.en.md). This document covers only the cases that path cannot handle. The surface for using the engine without ROS is [core_api.md](core_api.en.md).

## 1. When to use this

With a `.msg` attached, the generator places the layout and the fingerprint on both sides, and `Builder` and `View` fill and read the frame for you. None of the names in this document appear on that path. The raw path is used only when the material for that contract does not exist.

| Situation | Why `.msg` does not work |
| --- | --- |
| dtype or ndim changes per frame | The fingerprint is fixed for the lifetime of the channel |
| The schema cannot be expressed in ROS IDL | `bool[]`, `T[<=N]`, `wstring`, `bfloat16` are refused by the generator |
| The publishing side already holds a finished buffer | The data is outside the slot, so it is one copy anyway |
| dGPU payload | The flat wire is a host layout and cannot be used in a device slot |

The raw path has no type contract. Setting `fingerprint` to `flux::kNoSchema` (Python `flux.NO_SCHEMA`) attaches to anything with the same topic name. You may pick a constant of your own, but then you are defining a convention and enforcing it yourself. The number does not follow schema changes, so it cannot keep out an old-version peer.

## 2. FrameMeta and Published

`FrameMeta` is the descriptor that rides in the slot with every frame. It says how much is stored, not what is stored. The consumer reads it with `FrameView::meta()`.

```cpp doc:frame_meta
std::uint8_t ndim = m.ndim;
flux::DType dtype = m.dtype;
std::uint32_t itemsize = m.itemsize;
std::uint64_t nbytes = m.nbytes;
std::uint64_t rows = m.shape[0];
```

The publish API does not take a `FrameMeta`. The publisher states two things, `dtype` and `shape`, and the rest is derived.

| Field | Where it comes from |
| --- | --- |
| `dtype` · `shape` | The values the publisher passed to `publish` or `loan` |
| `itemsize` | `dtype_size(dtype)` |
| `ndim` | The length of `shape` |
| `nbytes` | The product of `shape` × `itemsize` |

`DType` has twelve values: `U8` `I8` `U16` `I16` `U32` `I32` `U64` `I64` `F16` `F32` `F64` `BF16`. `shape` has eight fields, and a frame cannot be built when `ndim` exceeds that.

Fields that could disagree with each other are not among the arguments, so a self-contradictory descriptor cannot be built. A frame whose `itemsize` differs from `dtype`, or whose `shape` does not match `nbytes`, cannot be represented at all.

The result of a publish is `flux::Published`.

| Value | Meaning | Counter |
| --- | --- | --- |
| `Ok` | Committed | -- |
| `Backpressure` | Every slot is borrowed. Transient | `dropped`++ |
| `TooLarge` | The payload exceeds `slot_size` or the rank exceeds 8. Committing an already used handle again also yields this value | -- |
| `WrongDevice` | A host publish was made into a device slot | -- |
| `FenceFailed` | The declared stream could not be waited on | `fence_failed`++ |

Only `Backpressure` clears by waiting. The others are wiring mistakes or faults, so they are not counted and are reported on the spot. When `dropped` rises, it means exactly one backpressure event.

Telling the five apart every time is rarely needed. There is one line to draw: was the frame dropped, or is it a fault that will not clear on its own.

```cpp
if (const flux::Published p = w.commit(); flux::faulted(p)) {
  report(flux::to_string(p));
}
```

```python
p = loan.commit()
if flux.faulted(p):
    log(f"publish refused: {p}")
```

`faulted` is false for `Ok` and `Backpressure`, and true for the other three. Every example publisher uses this shape.

Python returns the same `flux.Published`. It is not collapsed to a bool because the caller must be able to tell `Backpressure` (a frame dropped under best-effort) apart from `FenceFailed` (a fault that leaks the slot permanently). It is truthy only for `Ok`, so `if not loan.commit():` also reads as intended.

Anything that can be seen from the arguments alone is an exception in Python. Oversize, a device array, or a relative topic name raises `ValueError` before the channel is touched, because that is the place where the failing path can be named (`contracts.en.md` 3).

`DType::BF16` is a type the engine only carries and never computes on. The slot is still a plain byte range, and `dtype_size` returning 2 is all there is. C++17 has no bf16 scalar type, so the consumer interprets the bytes as its own type.

## 3. C++

### One-copy publish

```cpp doc:raw_publish
flux::ros::Publisher pub(*node, "img", flux::kNoSchema, slot_size, slot_count);

if (const flux::Published p = pub.publish(data, flux::DType::U8, {480, 640, 3});
    flux::faulted(p)) {
  report(flux::to_string(p));
}
```

`publish` does not truncate on overflow. It refuses with `TooLarge`. A rank above 8 gives the same value. If the consumer rejected at read time, the cursor would not advance and `take()` would stall on that slot.

The byte count is not an argument. It is the product of `shape`. A frame whose `shape` exceeds `nbytes` cannot be built, so a consumer never crashes while building a view from that shape.

For a flat byte string, use `publish(data, nbytes)`. It is stamped as `u8[nbytes]`. This is the form the generated adapter uses. The schema lives in the fingerprint, so the publisher has nothing to describe.

### Zero-copy publish

```cpp doc:write_slot
flux::WriteSlot w = pub.loan(flux::DType::U8, {480, 640, 3});
if (w) {
  render_into(w.data(), w.capacity());
  w.commit();
}
```

```cpp doc:write_slot_api
bool held = static_cast<bool>(w);
void * buffer = w.data();
std::size_t capacity = w.capacity();
flux::Published published = w.commit(nbytes);
w.abort();
```

A claimed slot leaves the ring until commit or abort. Holding it for a long time reduces the free slots and raises `dropped`. Like `FrameView`, it is move-only.

The slot returned by `loan()` is not an empty cell. It may still hold a frame nobody has taken yet, and `data()` points at those bytes as they are. Writing there destroys that frame, so `abort` cannot bring it back. It only skips the publish. The destroyed frame shows up in the `lost` counter of a subscriber that fell behind.

Since dtype and shape were stated once in `loan`, `commit()` takes no arguments. `commit(nbytes)` publishes only the first nbytes of a one-dimensional loan and recomputes `shape[0]`. There is no place where shape is stated twice. Passing it to a loan that is not one-dimensional, or a value larger than the loan, yields `TooLarge`.

`commit` performs the same checks as `publish` and rolls back the claim when a check fails. `dropped` does not rise, though. The slot was already received, so this is not backpressure. The previous frame that slot held disappears with it.

### Consuming

```cpp doc:raw_subscribe
flux::ros::Subscription sub(
  *node, "img", flux::kNoSchema,
  [](const flux::FrameView & v) {
    const flux::FrameMeta & m = v.meta();
    if (m.ndim == 3 && m.dtype == flux::DType::U8) {
      use(v.data(), v.size());
    }
  },
  flux::QoS{});
```

What `meta()` returns was written by another process. If it was written by the same version of flux, the product of `shape` equals `nbytes`. The publish path derives it that way. When dealing with a different implementation or a corrupted segment, check those values before sizing a view from them.

## 4. Python

`FrameMeta` is not exposed in Python. The array states `dtype` and `shape` together, so there is no place for the two to disagree.

### One-copy publish

```python doc:raw_py_publish
pub = flux.ros.Publisher(node, "img", fingerprint=FP, slot_size=16 << 20, slot_count=16)

pub.publish(arr)
```

`arr` must be C-contiguous. Exceeding the slot raises `ValueError`.

### Zero-copy publish

```python doc:raw_py_loan
loan = pub.loan((480, 640, 3), dtype="uint8")
if loan:
    loan.array[:] = frame
    loan.commit()
```

```python doc:raw_py_loan_api
loan = pub.loan(pub.slot_size, dtype="uint8")
held = loan.valid
writable = loan.host_addressable
published = loan.commit(nbytes=1024)
loan.abort()
```

`pub.loan(...)` returns a `flux.Loan`. It corresponds to C++ `flux::WriteSlot` and stays alive while holding the slot. `commit()` or `abort()` releases it. When no free slot exists it returns something falsy, so `if loan:` is the test.

`commit(nbytes=n)` publishes only the first n bytes of a one-dimensional loan. `shape[0]` is recomputed as `n / itemsize`, so shape is not restated. The generated adapter uses this. It borrows the whole slot and commits only as much as it actually filled.

### Consuming

```python doc:raw_py_view
v = sub.take()
if v is not None:
    _sink(v.shape, v.dtype, v.nbytes)
    v = None
```

The return value is a read-only numpy view. The data points directly into shared memory. The borrow is released when the last reference disappears, so set `v = None` or let it go out of scope.

Python validates the received meta itself. If the product of `shape` does not equal `nbytes`, or `itemsize` disagrees with `dtype`, it raises without building a view. The C++ consumer has no such check.

### bfloat16

Among the dtypes flux carries, bf16 is the only one numpy has no name for. So this type alone takes a different path.

```python
loan = pub.loan((4, 8), dtype="bfloat16")     # a string. Do not pass it to np.dtype()
torch.from_dlpack(loan)[:] = tensor           # or fill directly through loan.bits
loan.commit()

v = sub.take()                                # a flux.Frame, not a numpy array
t = torch.from_dlpack(v)                      # arrives as real bf16
```

The side that interprets through `ml_dtypes` uses `.bits`.

```python doc:raw_py_bits
loan = pub.loan(1024, dtype="uint16")
if loan:
    loan.bits.view(ml_dtypes.bfloat16)[:] = x
    loan.commit()
```

| | |
| --- | --- |
| `pub.loan(shape, dtype=...)` | Accepts the `"bfloat16"` string. Also accepts an `ml_dtypes.bfloat16` object (if installed) |
| `publish(array)` | Accepts an array that exposes DLPack even if its dtype is bf16. Such an array cannot be built with numpy |
| `loan.array` · numpy array from `take()` | Not available. `.array` promises numpy, so it refuses with a reason |
| `__dlpack__` | Present on both. torch and jax pick it up here |
| `.bits` | An unsigned numpy view of the same bytes (`uint16` for bf16). The `ml_dtypes` side uses `.view(ml_dtypes.bfloat16)` |
| `__cuda_array_interface__` | `RuntimeError` for bf16. The CAI typestr is numpy's and cannot spell bf16 |
| `frame.dtype` | `"bfloat16"`. Other types are the numpy typestr as is |

`take()` returning a `flux.Frame` is for the same reason as with `device="cuda"`. The place to attach `__dlpack__` is an object. A host bf16 frame, however, does not need `with`. A scope is required only when release is a stream synchronization, and releasing a host borrow is a single decrement, so it is released at GC time like a numpy view.

`ml_dtypes` is not a dependency of flux. Whoever wants it installs it and interprets `.bits`, and the DLPack path works the same without it installed.

## 5. Permanent errors and backpressure

Both sides separate the two. Only the representation differs.

| | Python | C++ |
| --- | --- | --- |
| Wrong type | `TypeError` | (not representable) |
| slot_size exceeded | `ValueError` | `TooLarge` |
| Host publish into a GPU slot | `ValueError` | `WrongDevice` |
| Stream wait failure | `Published.FenceFailed` | `FenceFailed`, `fence_failed`++ |
| Bad meta | (not representable) | (not representable) |
| No slot | `Published.Backpressure`. `loan()` returns `None` | `Backpressure`, `dropped`++ |

There is no `False` in the Python column. `publish()` and `commit()` return the same `flux.Published` as C++, and only `loan()` returns `None` when no free slot exists (section 2).

A publish that cannot succeed is a wiring mistake, not backpressure. Python raises it and C++ separates it by return value. Either way, `dropped` rises in exactly one case: there was no slot.
