# flux message shapes (.msg -> wire)

flux flattens a `.msg` into a flat wire. This document shows which region of the slot each member goes to depending on its shape, and what the adapter that reads and writes that layout looks like. `.msg` authoring rules are in [enumerations_msgs.md](enumerations_msgs.en.md), copy behavior in [copy_model.md](copy_model.en.md), core concurrency in `design.md`, segment names and fingerprint in `discovery.md`.

`flux_gen` runs four stages at build time. Each stage is the input of the next.

```text
parse      .msg -> Message (fields, constants)
flatten    Message -> leaf list (nested flattening, tier decision, rejection)
layout     leaf list -> byte offset
emit       offset -> C++ header + Python module (+ ROS message bridge)
```

The fingerprint comes from the flatten result and is embedded as a constant in both outputs. Both languages come from the same tree, so the layouts cannot diverge.

## 1. Frame structure

A frame is contiguous bytes with no pointers. It points inside itself only by offset from the frame start, never by address.

```text
frame = [ scalar block (fixed size, offsets settled at build time) ][ var region (bump-allocated per frame) ]
```

What the scalar block holds:

| Member | In the scalar block | In the var region |
| --- | --- | --- |
| Fixed scalar `int32 x` | The value itself | -- |
| Fixed array `float64[9] m` | 9x8B contiguous | -- |
| Every variable member | Descriptor 8B `{uint32 off, uint32 len}` | The actual data |

One descriptor records "where and how many". A read reads the descriptor and views that location. A write advances the cursor and writes the descriptor. Allocation order is free. The descriptor records where each member was placed.

The offset is uint32, so one frame of a generated schema is capped at 4 GiB regardless of slot_size.

## 2. What maps directly (zero-copy)

Fixed scalars, `int32 x; float64 y`:

```text
scalar: [ x(4) | pad(4) | y(8) ]        natural alignment, same as a C struct
```

Fixed array `T[N]`, `float64[9] matrix`:

```text
scalar: [ matrix[0] ... matrix[8] ]     (9x8B contiguous, read and written in place)
```

Dynamic array `T[]`, `float32[] data`, N=3:

```text
scalar: [ data.desc = {off, 3} ]
var   : [ data0 data1 data2 ]           (zero-copy)
```

Parallel same-length arrays, `float32[] x, y, z`, N=3:

```text
scalar: [ x.desc | y.desc | z.desc ]
var   : [ x0 x1 x2 ][ y0 y1 y2 ][ z0 z1 z2 ]     (zero-copy per column, each aligned on its own)
```

## 3. What maps after flattening (nested)

Nesting is only a name prefix on the wire. The members inside go to their own region according to their own shape.

Fixed nested struct, `pose` of `geometry_msgs/PoseStamped`:

```text
scalar: [ ... pose_position_x(8) pose_position_y(8) pose_position_z(8)
              pose_orientation_x(8) ... pose_orientation_w(8) ]
```

The accessor name is the path joined with `_` (`pose.position.x` -> `pose_position_x`). If two fields collapse to the same name (`a.b` and `a_b`), generation is refused. It is also refused when two paths produce the same element class name. Names the adapter uses for itself carry a doubled underscore so that a `.msg` cannot reach them ([enumerations_msgs.md](enumerations_msgs.en.md)).

Single nesting holding a variable member (hoisting), `Polygon polygon` (containing `Point32[] points`):

```text
polygon disappears and the points inside move up:
scalar: [ polygon_points.desc = {off, N} ]
var   : [ x0 y0 z0 ][ x1 y1 z1 ] ...
```

Restructuring the nesting (grouping fields into a struct or ungrouping them) changes neither the layout nor the fingerprint. The two properties must go together. A same fingerprint with a different layout would be the one mismatch the fingerprint cannot catch.

## 4. Element arrays (record, jagged)

Arrays whose element has several members are mapped one way, whether record or jagged. An array of element blocks is placed in the var region. The block size (stride) is fixed, so `items[i]` is O(1).

If every element member is fixed, it is a record column. `Point32[] points`, N=2:

```text
scalar: [ points.desc = {off, 2} ]
var   : [ x0 y0 z0 ][ x1 y1 z1 ]        (12B fixed per element, zero-copy)
```

If the element contains a variable member, it is jagged. Only one thing changes. The element block carries its own descriptor, which points further out in the var region.

`PointField[] fields` of `sensor_msgs/PointCloud2` (element = `string name; uint32 offset; uint8 datatype; uint32 count`):

```text
scalar: [ fields.desc = {off, 3} ]
var   : [ elem0: name.desc offset datatype count ]   element block 20B
        [ elem1: name.desc offset datatype count ]
        [ elem2: name.desc offset datatype count ]
        [ "x" ][ "y" ][ "z" ]                        string bytes of the elements
```

Depth goes in recursively as is. In the installed ROS jazzy messages, jagged nesting is at most 3 levels deep.

## 5. string (one copy for that member only)

string scalar, `string label = "hello"`:

```text
scalar: [ label.desc = {off, 5} ]
var   : [ "hello" ]                     (one copy, small)
```

string array, `string[] names = ["ab","cde"]`:

```text
scalar: [ names.desc = {off, 2} ]
var   : [ desc0 | desc1 ][ "ab" ][ "cde" ]
```

Numeric columns stay zero-copy and only the string bytes are copied.

## 6. Header / bare Time

`std_msgs/Header` and bare `builtin_interfaces/Time` are not flattened into fields. They are mapped in a fixed shape.

```text
Header: [ sec(int32) | nanosec(uint32) | frame_id.desc(8) ]     16B
Time  : [ sec(int32) | nanosec(uint32) ]                         8B
```

They live inside the payload. `flux_core` does not know ROS, so the frame meta (`FrameMeta`) has neither a stamp nor a frame_id. Making room for them would give the engine ROS semantics. The receiver already holds the whole frame zero-copy, so reading from the payload costs the same as reading from the meta.

A Header inside a nested struct is not special. It is flattened like an ordinary struct. There is one frame stamp, the top-level one.

## 7. What does not work (refuse -> generation error)

| Shape | Example | Why |
| --- | --- | --- |
| Bounded array `T[<=N]` | `float64[<=10] vals` | A capacity bound is meaningless on a dynamic wire |
| `bool[]` | `bool[] flags` | `std::vector<bool>` is bit-packed, so memcpy is impossible |
| `wstring` | `wstring s` | Wide string encoding |
| No fields | `std_msgs/Empty` | There is no payload to carry |
| Recursive type | `A` contains `A` | Cannot be flattened into a flat wire |
| More than 4096 leaves | `A[64] a` contains `B[64] b` contains... | Fixed nesting multiplies |

A refusal is an error. There is no automatic fallback path. The `flux_gen` CLI raises `SystemExit` (`flux_gen/flux_gen/cli.py`), and `emit_cpp`, `emit_py`, and `emit_ros` raise `ValueError`. A build that uses `flux_generate_adapters()` fails at that point. To send that message over plain ROS, the caller does not pass it to `flux_generate_adapters()`.

## 8. At a glance, the decision tree

```text
The member is...
├─ fixed scalar / T[N]                       -> scalar block (zero-copy)
├─ fixed-type T[] / fixed struct T[]          -> descriptor + var region (zero-copy)
├─ nested struct                             -> flattened (flattening / hoisting)
├─ string / string[]                         -> descriptor + var region (one copy for that part only)
├─ array of variable elements (jagged)       -> descriptor + element block array
├─ Header / bare Time                        -> fixed position (16B / 8B)
└─ T[<=N] / bool[] / wstring / no fields     -> refuse -> generation error
```

Only the last line is refused; everything else is mapped. Which messages are refused depends on the set of installed packages, so neither a ratio nor a count is written here. Count it directly on that machine with `flux_gen`.

## 9. Generated output

`flux_generate_adapters()` emits one pair of adapters and one pair of ROS bridges per message. Usage is in [api.md](api.en.md) section 2.

```text
include/<pkg>/flux/<snake>.hpp       <pkg>::flux_msg::<Msg>   -- kFingerprint, View, Builder
<pkg>_flux/<snake>.py                <pkg>_flux.<snake>.<Msg> -- FINGERPRINT__, View, Builder
include/<pkg>/flux/<snake>_ros.hpp   msg_to_frame · frame_to_msg  (ROS message object bridge)
<pkg>_flux/<snake>_ros.py            msg_to_frame · frame_to_msg
```

The basic adapters (`.hpp`/`.py`) are thin. One accessor calls the runtime ([`flux/wire.hpp`](../../flux_core/include/flux/wire.hpp), `flux_gen/flux_gen/wire.py`) once with a constant offset. There is no per-message runtime; the only thing that has to match is the offset.

The `_ros` bridge is emitted separately. `msg_to_frame` copies every field of a ROS message object into the slot and `frame_to_msg` does the reverse. Both are at least one copy, so they differ from the zero-copy path that uses `Builder`/`View` directly (`copy_model.en.md`, `api.en.md` 2). The part that includes rosidl message types is isolated here, so a package that does not use it never sees rclcpp or rclpy.

The adapter is host-payload only. `View` and `Builder` pass `FrameView::data()` and `WriteSlot::data()` straight to the `wire` runtime and do host loads and stores on top of it, so they cannot be used on a dGPU channel whose slots are device allocations. On such a channel `data()` is `nullptr`, so `View` and `Builder` become `ok__() == false` and `commit()` refuses. That is instead of silently doing a host store into GPU memory. The way to publish and receive on a dGPU is `device_ptr()` and `stream()` (`api.en.md` GPU section). On an iGPU (`ShmDirect`) the slot is also host memory, so the adapter runs as is.

The receiving side assumes another process wrote the frame and trusts none of the descriptors. It checks every range and alignment. On a mismatch, C++ latches `ok__()` to false and Python raises `WireError`. There is no path that hands out a pointer outside the frame.

In `frame_to_msg`, if the length of an array in the frame differs from the schema's `T[N]`, both languages raise an explicit error. C++ raises `std::length_error`, Python raises `ValueError`. The length is data the frame carried and N is the schema. A truncated or padded copy would be a silently wrong message.

The write side enforces the same length. Writing a `T[N]` field to a `Builder` with a length other than N raises `ValueError` in Python. In C++ it poisons the writer so `ok__()` latches to false and `commit()` refuses. A wrong frame is never produced. The check in `frame_to_msg` is the last line of defense against frames produced by a foreign (external or older) writer.
