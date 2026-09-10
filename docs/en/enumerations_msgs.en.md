# flux .msg rules (array / enum / names)

flux looks at the shape of the data (dtype, array kind) and the **top-level type name**. It does not look at field names or nested type names. These either overlap with or are unrelated to ROS 2 message conventions, so they can be used as is. The flatten and fingerprint conventions are in `message_shapes.en.md`.

## Arrays

| Shape | flux |
| --- | --- |
| `T[]` (unbounded) | Accepted. Column (zero-copy) or jagged (element block array) |
| `T[N]` (fixed) | Accepted. A count-N field in the fixed scalar block |
| `T[<=N]` (bounded) | Refused -> generation error |

The recommendation is unbounded `T[]`. A `T[]` that holds another `T[]` (nested dynamic) is jagged.

## enum (integer constants)

ROS2 has no enum. It is expressed with integer constants.

```text
uint16 ERROR_UNSAFE = 1
uint16 ERROR_GNSS   = 3
uint16 type              # the actual field
```

- A `NAME = value` line is not wire data. It enters neither the layout nor the fingerprint.
- Only the actual field (`type`) is flattened. Here it is a fixed u16.
- The generated adapter re-emits the constants as named symbols. C++ gets `Status::ERROR_GNSS` (`static constexpr`), Python gets `Status.ERROR_GNSS`.
- Constant names follow the rosidl rule: start with an uppercase letter, `UPPER_CASE`, single underscores only between letters and digits. Violations and duplicates are a ParseError.
- The constants the adapter defines for itself are spelled differently per language. C++ has `kFingerprint`, `kTypeName`, `kScalarBytes` (`static constexpr`), Python has `FINGERPRINT__`, `TYPE_NAME__`, `SCALAR_BYTES__`. Neither can be produced by the constant name rule. The C++ ones start with a lowercase `k` and the Python ones end with `__`, while the rule allows only `UPPER_CASE` starting with an uppercase letter. So a `.msg` cannot reach these names. One refusal remains. If a constant name equals the message's own class name, generation is refused.
- String constant values are the same as rosidl: strip the comment (`#`), and if both ends are a matching pair of quotes, strip them. A `#` inside quotes is part of the value. The same `.msg` must yield the same constant value on the ROS side and the flux side.

## Field names

- There are no reserved words for field names. The adapter attaches a doubled underscore to its own members and derived accessors. The field name rule cannot produce a doubled underscore, so a `.msg` cannot reach these names. This is why `size` in `std_msgs/MultiArrayDimension` passes, whether top-level or nested.
- The surfaces of the two adapters are not the same. Below is the correspondence for a field `x`.

| | C++ | Python |
| --- | --- | --- |
| Adapter's own members | `ok__()` · `size__()` · `commit__()` · `build__()` | `size__` · `commit__()` · `build__()` |
| Scalar read | `x()` | `x` property |
| Scalar write | `set__x(v)` | `x` property setter |
| Variable length | `x__size()` (string array, struct array) · `x().size()` (column) | `len()` |
| Variable length allocation | `alloc__x(n)` | `alloc__x(n)` |
| stamp read | `x__sec()` · `x__nanosec()` | `x__stamp` -> `(sec, nanosec)` |
| stamp write | `set__x__stamp(sec, nanosec)` | `set__x__stamp(sec, nanosec)` |
| frame_id | `x__frame_id()` · `set__x__frame_id(s)` | `x__frame_id` property |

Python has no `ok__`. It raises `WireError` on a mismatched descriptor instead of latching (`message_shapes.en.md` 9).

- A field name that collides with a language keyword is escaped by appending `_`. Derived accessors use that spelling too. C++ applies this to C++ keywords (`double x` -> `double_()`, `set__double_()`), and Python applies it to Python keywords plus `np` and `self` (`class x` -> `class_`). The two sets differ, so some names change on only one side. `double` changes only in C++ and `class` changes on both sides. This is the one place where the accessor spelling of the two adapters diverges.
- If the paths of two struct arrays produce the same camel-case element class name (`foo_bar` and `foo.bar` -> both `FooBarElem`), generation is refused. The error names both paths. Renaming one field resolves it.

A constant is distinguished from a default value by whether `=` comes directly after the name. Judging "a constant if there is an `=` somewhere after the type" would mistake a default like `string label "a=b"` for a constant and drop the field entirely. The wire layout and the fingerprint would change together, and both adapters would agree on the same wrong answer, so nothing would catch it.

## What the parser refuses

flux_gen does not guess at an ambiguous `.msg`. The parse result is the wire layout, so if it is silently wrong, both sides just agree on the same wrong answer and nobody catches it. So the following are errors with a file and line number.

| Input | Reason |
| --- | --- |
| `float64[abc] x` · `float64[-1] x` · `float64[1_0] x` | The array size is not a decimal integer |
| `float64[0] x` | Fixed array of length 0. For dynamic, use `T[]` |
| `float64[3 x` · `float64[ 3] x` | Bracket not closed (whitespace splits tokens) |
| `float64[70000] x` | Exceeds the fixed array cap (65536). Use `T[]` for large data |
| `int32<=4 x` | A `<=` bound is only for string/wstring |
| Two fields with the same name | Duplicate field |
| Type without a name, invalid identifier | Shape unknown |

The flatten stage has a separate cap. Nested fixed arrays multiply (`A[64]` containing `B[64]` containing `float64[64]` = 260k leaves), so exceeding 4096 total leaves is a reject. A reject is a generation error and does not fall back to plain ROS automatically (`message_shapes.en.md` 7). Real ROS jazzy messages stay in the double digits of leaves, far from this cap.

## Names, unit suffixes

- The fingerprint is the **top-level type name + flat layout**. Field names and nested type names are excluded.
- So renaming a type or moving it to another package changes the fingerprint and it no longer connects with existing peers. ROS 2 is the same. DDS matches by type name, so different names never meet in the first place.
- Changing the nesting structure (grouping fields into a struct) does not change the fingerprint. The wire bytes are the same.
- Field names are still not examined. So flux cannot catch a unit suffix mismatch (`velocity` vs `velocity_kmph`). Naming rules are for humans.

One fingerprint does what ROS 2 does with two mechanisms.

| ROS 2 | What it catches | flux |
| --- | --- | --- |
| Type name (DDS matching) | Different types connecting | Top-level type name in the fingerprint |
| Type hash (RIHS01, Iron+) | Same type with a diverged definition | Flat layout in the fingerprint |

`geometry_msgs/Point` and `geometry_msgs/Vector3` are both three `float64`. By layout alone they would share the same segment, and nothing would stop a position from being read as a direction. This was confirmed with rclpy. Publishing `Point` on a topic and subscribing with `Vector3`, ROS delivers nothing. With the type name in the fingerprint, flux makes the same decision.
