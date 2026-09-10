# flux_example_lidar

Size changes every frame. The point count of a lidar sweep differs every time.

```bash
ros2 run flux_example_lidar cloud_pub       # ros2 run flux_example_lidar cloud_pub.py
ros2 run flux_example_lidar cloud_sub       # ros2 run flux_example_lidar cloud_sub.py
```

Uses `sensor_msgs/PointCloud2`. Unlike an image the size is not a constant, and there is a nested array (`PointField[] fields`).

## Size for the worst case, publish only what was filled

The slot size is a build time constant. So it is sized with `kMaxPoints`. When the `Builder` does `commit()` only the prefix actually built is published, so a short sweep becomes a short frame.

```cpp
static constexpr std::uint32_t kSlotSize = kMaxPoints * kPointStep + 4096;

Cloud::Builder b = Cloud::build__(pub_);
b.set__width(n);
b.set__row_step(n * kPointStep);
auto data = b.alloc__data(n * kPointStep);
b.commit__();
```

Sizing the slot large is cheap. The payload region is never touched during init, so tmpfs leaves it sparse, and the unused part takes no RAM.

## Nested arrays are also inside the slot

`fields` is four `PointField` entries, each with a `string name`. What `alloc_fields(4)` returns is also memory inside the slot.

```cpp
auto fields = b.alloc__fields(4);
fields[0].set__name("x");
fields[0].set__offset(0);
fields[0].set__datatype(7);
fields[0].set__count(1);
```

The order of `alloc_*` calls is free.

## Subscription

`View` reads `width`, `point_step`, and `data` straight from the slot. That `data.size()` differs every frame is what this package shows.
