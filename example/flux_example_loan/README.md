# flux_example_loan

Borrow a slot first and build an `Image` inside it. There is no copy.

```bash
ros2 run flux_example_loan image_pub        # ros2 run flux_example_loan image_pub.py
ros2 run flux_example_loan image_sub        # ros2 run flux_example_loan image_sub.py
```

## Publish

`build(pub)` takes an empty slot and puts a `Builder` on top of it. The Builder points at that slot, so the memory returned by `alloc_data(n)` is the bytes that will be published. If the camera writes there directly there is no copy at all.

```cpp
Image::Builder b = Image::build__(pub_);
if (!b) {
  return;
}
b.set__width(kWidth);
b.set__height(kHeight);
auto data = b.alloc__data(kDataBytes);
b.commit__();
```

When there is no empty slot the Builder is false and `dropped()` increments. Delivery is best-effort, so the publisher does not wait for a slow subscriber.

Discarding the handle without commit cancels the claim. The frame that was in that slot does not come back, because `data()` was pointing at it. The only thing abort guarantees is that nothing was published.

## Subscription

`View` reads the slot as is. While the `FrameView` is alive the publisher cannot use that slot. The borrow ends when the callback returns, so copy any value you need to keep longer.

Check `v.ok__()` after reading. If the frame was torn, discard the values read.

## How it runs

A subscription does not run by itself. In both languages it is handed to the flux executor. C++ uses `flux::ros::Executor`, Python uses `flux.ros.Executor`.

The executor is created in `main`. It is not a node. This is the same place as putting the executor in `main` in rclcpp/rclpy, and the node holds only the subscription and the callback. Callback groups, in contrast, belong to the node. In rclcpp `create_callback_group` is a node method, and the group holds that node's entities.

The difference between the two arrangements is covered by [`flux_example_executor`](../flux_example_executor/).
