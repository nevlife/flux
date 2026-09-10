# flux_example_publish

Build an `Image` in its own buffer and put it into a slot with `publish`. There is one copy.

```bash
ros2 run flux_example_publish image_pub     # ros2 run flux_example_publish image_pub.py
ros2 run flux_example_publish image_sub     # ros2 run flux_example_publish image_sub.py
```

## How it differs from loan

[`flux_example_loan`](../flux_example_loan/) takes the slot first. Here the slot is taken after the frame is fully built.

```cpp
Image::Builder b{buf_.data(), buf_.size()};
b.set__width(kWidth);
b.set__height(kHeight);
auto data = b.alloc__data(kDataBytes);
if (!b.ok__()) {
  return;
}
pub_.publish(buf_.data(), b.size__());
```

For a frame with an adapter the schema is in the fingerprint, so the publisher has nothing to describe. It is one line, `publish(data, nbytes)`.

In exchange for one copy, the slot is not held while the frame is being built. This is the path when you already hold a finished buffer, or when building the frame takes long and you do not want to occupy a slot meanwhile.

In C++ loan is always better. In Python the break-even point is between 256 KiB and 1 MiB. Below that, creating the loan object costs more than the copy in `publish`.

## Subscription

Same as the subscriber in [`flux_example_loan`](../flux_example_loan/). The publish path differs, but the frame layout is the same when the fingerprint is the same, so it attaches to either publisher.
