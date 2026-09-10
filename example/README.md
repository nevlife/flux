# flux examples

Each package shows one axis. The default message is `sensor_msgs/Image`; another type is used only where the axis needs it.

| Package | What | Type |
| --- | --- | --- |
| [`flux_example_msgs`](flux_example_msgs/) | Adapter generation. The rest depend on it | `Image` · `PointCloud2` |
| [`flux_example_loan`](flux_example_loan/) | Borrow a slot and write into it directly with `Builder`. Zero-copy | `Image` |
| [`flux_example_publish`](flux_example_publish/) | Build in its own buffer and put it in with `publish`. One copy | `Image` |
| [`flux_example_raw`](flux_example_raw/) | Send a raw tensor without a `.msg` | none |
| [`flux_example_executor`](flux_example_executor/) | Five executor arrangements | `Image` |
| [`flux_example_qos`](flux_example_qos/) | depth · durability · max_borrow | `Image` |
| [`flux_example_lidar`](flux_example_lidar/) | Size changes every frame | `PointCloud2` |
| [`flux_example_rt`](flux_example_rt/) | preflight and the chain declaration file | `Image` |
| [`flux_example_gpu`](flux_example_gpu/) | A slot that kernels write and read | none |

Each package has a C++ node and a Python node as a pair. `flux_example_rt` is C++ only. Python is not an RT target. `multi_sub` in `flux_example_executor` is also C++ only. rclpy has no corresponding arrangement.

## Common rules

Configuration is a class constant at the top of each file. Neither ROS parameters nor module globals are used. The executable without an extension is C++, the `.py` one is Python.

The publisher checks the `flux::Published` return value of `publish`/`commit`. `Backpressure` means one frame was dropped because a subscriber was slow, so it is skipped over. `TooLarge`, `WrongDevice`, and `FenceFailed` are wiring mistakes or faults, so they are logged. Folding these two into one `bool` makes a wiring mistake look like a rate.

The subscriber may start first. When there is no publisher it waits quietly and attaches the moment one appears.

## Topics

| Topic | Packages using it |
| --- | --- |
| `image` | `loan` · `publish` · `executor` · `qos` · `rt` |
| `image_raw` | `raw` |
| `image_gpu` | `gpu` |
| `cloud` | `lidar` |

`merged_sub` subscribes to this name over both transports, flux and ROS 2, at the same time. The transports differ, so they do not mix. The ROS 2 side publisher is not in this repository. Starting a plain ROS 2 node that publishes `sensor_msgs/msg/Image` on `image` separately makes that half run too.

## One schema

No new `.msg` is defined. `flux_example_msgs` feeds the installed `sensor_msgs/msg/Image.msg` and `PointCloud2.msg` to the generator at build time and emits the adapters. There is no copy, so the two paths cannot diverge.

```text
sensor_msgs/flux/image.hpp          ->  sensor_msgs::flux_msg::Image
sensor_msgs_flux/image.py           ->  sensor_msgs_flux.image.Image
sensor_msgs/flux/point_cloud2.hpp   ->  sensor_msgs::flux_msg::PointCloud2
sensor_msgs_flux/point_cloud2.py    ->  sensor_msgs_flux.point_cloud2.PointCloud2
```

`flux_generate_adapters()` accepts only paths relative to the package source. The installed `.msg` files are absolute paths, so `flux_example_msgs/CMakeLists.txt` calls `python3 -m flux_gen` directly.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Observe

```bash
ros2 run flux_cli flux topic hz /image      # flux publisher
ros2 topic hz /image                        # ROS publisher
```
