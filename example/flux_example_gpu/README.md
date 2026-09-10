# flux_example_gpu

A slot that kernels write and read directly. There is no staging buffer.

```bash
ros2 run flux_example_gpu image_pub_gpu     # ros2 run flux_example_gpu image_pub_gpu.py
ros2 run flux_example_gpu image_sub_gpu     # ros2 run flux_example_gpu image_sub_gpu.py
```

## The declaration is local

`Device::Cuda` makes flux create a stream to fence the two seams. The wire does not change. A host subscriber on the same topic keeps running. This declaration is not about how the bytes travel but about who touches them.

```cpp
flux::ros::Publisher pub(*this, "image_gpu", flux::kNoSchema, slot_size, slot_count,
                         flux::Device::Cuda);
```

If the host cannot honor that declaration the constructor throws. A publisher that believes it is on the GPU while quietly running the host path is what this check prevents.

## Two addresses

| | What |
| --- | --- |
| `data()` | The address plain host load/store reads. null on a dGPU |
| `device_ptr()` | The address a kernel reads. null if no stream was declared |
| `host_addressable()` | Whether `data()` may be touched |
| `stream()` | The stream to launch kernels on. Exactly the one the fence waits for |

On an integrated GPU the slot is a single mapping that both host and kernel reach. That is why the fill in this example is a host store. On a dGPU `data()` is null and the fill becomes a kernel launched on `w.stream()` writing to `w.device_ptr()`.

`device_ptr()` being non-null cannot stand in for `host_addressable()`. An integrated GPU declares a stream over a slot that is also host memory.

## What the fence prevents

commit waits for the stream before closing the seqlock. So even for a GPU payload, "committed" keeps meaning "complete". Release is the reverse. It waits before letting go of the view, so the publisher cannot reclaim a slot a consuming kernel is still reading.

If the wait fails the borrow is deliberately not released. Releasing on top of an unconfirmed fence is the corruption this mechanism exists to prevent. The cost is that one slot never comes back, so a non-zero `fence_failed()` is a fault, not a rate.

`fence_wait()` reports how long that wait actually is. On the callback path the release wait lands on the executor's spin thread. That is why a subscription receiving a GPU channel is put in its own callback group.

## Python

The frame arrives as a `flux.Frame`, not numpy. The `with` scope is mandatory. Leaving the scope is the stream synchronization, and that wait must happen where the caller decides, not when Python collects the object.

```python
def on_frame(self, frame):
    with frame as view:
        device_ptr = view.__cuda_array_interface__["data"][0]
```

With cupy available, `cp.asarray(view)` is the entire GPU-side API. Without it both nodes run the host path.
