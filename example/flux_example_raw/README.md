# flux_example_raw

The path when there is no `.msg`. There is no adapter, no schema, no header field, and the frame is a single `uint8` volume.

```bash
ros2 run flux_example_raw image_pub         # ros2 run flux_example_raw image_pub.py
ros2 run flux_example_raw image_sub         # ros2 run flux_example_raw image_sub.py
```

## The publisher states it directly

`loan()` takes the dtype and shape. The byte count follows from these two, so the description a consumer uses to build a view cannot diverge from the actual payload.

```cpp
flux::WriteSlot w = pub_.loan(flux::DType::U8, {kHeight, kWidth, kChannels});
auto * px = static_cast<std::uint8_t *>(w.data());
w.commit();
```

In Python the array already states its dtype and shape, so nothing is restated.

```python
loan = self.publisher.loan(self.ramp.shape, dtype="uint8")
```

The fingerprint is `flux::kNoSchema` (`flux.NO_SCHEMA`). 0 means no check, so if two nodes with different layouts attach, nothing stops them.

## What is lost

This node sends no stamp, no frame_id, and no encoding. There is nowhere to put them. All the subscriber knows is the dtype and shape.

| | Adapter path | Here |
| --- | --- | --- |
| Fields | Defined by the `.msg` | none |
| fingerprint | Schema hash | 0 |
| Mismatched peer | attach refuses | not stopped |
| What the consumer knows | The whole type | dtype · shape |

When all there is to describe is dtype and shape, as with images and tensors, this path is right. When there are fields, [`flux_example_loan`](../flux_example_loan/) is right.
