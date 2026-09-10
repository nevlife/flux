# flux_example_msgs

Emits the adapters the example packages use. There are no nodes.

The installed `sensor_msgs/msg/Image.msg` and `PointCloud2.msg` are fed to `flux_gen` at build time. There is no `.msg` copy, so the flux frame and the ROS message cannot diverge.

```text
sensor_msgs/flux/image.hpp          ->  sensor_msgs::flux_msg::Image
sensor_msgs_flux/image.py           ->  sensor_msgs_flux.image.Image
sensor_msgs/flux/point_cloud2.hpp   ->  sensor_msgs::flux_msg::PointCloud2
sensor_msgs_flux/point_cloud2.py    ->  sensor_msgs_flux.point_cloud2.PointCloud2
```

The C++ headers are exported with `ament_export_include_directories`, and the Python module is installed on the ament python path. A consumer does this.

```cmake
find_package(flux_example_msgs REQUIRED)
ament_target_dependencies(my_node rclcpp flux_example_msgs)
```

```cpp
#include "sensor_msgs/flux/image.hpp"
using Image = sensor_msgs::flux_msg::Image;
```

## Why the generator is called directly

`flux_generate_adapters()` accepts only paths relative to the package source. The `.msg` files here belong to `sensor_msgs` and are absolute paths, so `CMakeLists.txt` calls `python3 -m flux_gen` directly. A package with its own `.msg` can use the macro.

The generator source is also an input. When the emitter changes, the adapters must be regenerated. Otherwise a stale adapter is not a build error but a fingerprint that does not match the peer.

## Dependencies

`flux_gen` is both a build dependency and an exec dependency. The generated Python adapter imports `flux_gen.wire` at runtime.

`rosidl_generate_interfaces` is not called. `sensor_msgs` already installs the interfaces, so this package only uses them.
