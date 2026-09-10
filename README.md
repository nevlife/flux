# flux

An opt-in library that moves large data between C++ and Python ROS 2 nodes on the same host without copying.

It is not a new middleware. It is called per topic inside an existing `rclcpp`/`rclpy` node, and every other topic keeps flowing over DDS as usual.

> [!WARNING]
> Alpha. The API, the `.msg` layout rules, and the shared-memory segment format can all change without notice, and real-world testing is still limited.

## Build

Place it under `src/` of a colcon workspace and build there. nanobind is a submodule, so fetch submodules too, and build with symlinks.

```bash
git clone --recurse-submodules https://github.com/nevlife/flux
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
```

## Requirements

- Linux 6.7+ (io_uring futex wait; older kernels fall back to one wait thread per channel)
- ROS 2 Jazzy
- C++17
- Python 3.8+, numpy
- cmake 3.16+

## GPU support

| Path | Device | Slot payload |
| --- | --- | --- |
| `ShmDirect` | Unified-memory iGPU | The host shm mapping is the GPU memory as-is |
| `ShmRegistered` | iGPU that does not share address translation (Orin) | The same mapping is registered once per channel |
| `DeviceHandle` | dGPU | One VMM allocation is shared through an fd. Slot arithmetic is the same as on the host |

## Status

| Feature | State |
| --- | --- |
| C++ publish -> Python zero-copy receive | O |
| `.msg` -> adapter generation | O |
| loan (zero-copy) and publish (one copy) | O |
| QoS | O |
| merged executor | O |
| `PartitionedExecutor` | O |
| message_filters (`Subscriber`, sync) | O |
| io_uring futex wait | O |
| discovery | O |
| crash reclaim | O |
| multiple publishers | O |
| enumeration and the `flux` CLI | △ |
| RT: scheduling, affinity, page pre-commit, mlock | O |
| RT: chain declaration file (`rt_spec`) | △ |
| GPU `ShmDirect` | △ |
| GPU `ShmRegistered` | △ |
| GPU `DeviceHandle` | △ |
| Cross-container transport (`--ipc=host --pid=host`) | △ |
| overflow policy (overwrite / block the publisher) | X |
| rviz2, rqt, ros2 bridge | △ |

## Documents

| | |
| --- | --- |
| [`docs/en/api.en.md`](docs/en/api.en.md) | The user surface with a `.msg` attached |
| [`docs/en/raw_api.en.md`](docs/en/raw_api.en.md) | The surface without a `.msg` |
| [`docs/en/qos.en.md`](docs/en/qos.en.md) | `depth`, `durability`, `max_borrow`, and the reasons for an empty view |
| [`docs/en/message_shapes.en.md`](docs/en/message_shapes.en.md) | How a `.msg` is laid out |

## License

MIT. See [`LICENSE`](LICENSE).
