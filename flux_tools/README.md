# flux_tools

Viewer plugins that subscribe to flux channels directly. Three rviz2 displays: Image, PointCloud2 and DepthCloud.

## rviz2 Image display (`flux_tools/Image`)

Opens a `flux_cpp` subscription inside the rviz2 process and, on every render tick, uploads the bytes of the frame from `take()` straight into an Ogre texture. It passes through no ROS message object, no CDR serialization, and no DDS. The one remaining copy is the GPU upload.

| Item | Content |
| --- | --- |
| Class | Inherits `rviz_common::Display` directly. `RosTopicDisplay` assumes an rclcpp subscription, so it is not used |
| Type | `sensor_msgs/Image`. The adapter is generated at build time by `flux_gen` from the installed `Image.msg` |
| QoS | `depth=1`, `max_borrow=1`. The view lives only inside `update()` and is released right after the upload |
| Encoding | `rgb8` `bgr8` `rgba8` `bgra8` `mono8` `8UC1` are uploaded to the texture as is. `mono16` `16UC1` `32FC1` (depth) are uploaded as is and mapped to gray in a fragment shader, see below. `nv12` (half-resolution UV after the Y plane, `step` is the Y row pitch) is uploaded as two textures, Y and UV, and a fragment shader produces RGB with BT.601. Anything else is shown as an error in the status panel |
| Depth range | `Normalize Range` (default on) reads the frame once in place, skipping NaN, inf and zero, and maps its minimum to black and maximum to white. Off: `Min Value` and `Max Value` in the frame's own units (millimeters for a `16UC1` camera, meters for `32FC1`). Either way the pixels are not converted on the CPU; the shader takes the raw texture |
| Topic list | Filled with live `Image` publishers whose exact channel names are known in the same domain. Persistent signposts without a publisher are omitted. Direct input also works |

### Usage

```bash
cd ~/flux_ws
colcon build --symlink-install --packages-up-to flux_tools
source install/setup.bash
rviz2
```

`Add` → `flux_tools` → `Image`. Pick or type a channel name in `Topic`. The name is resolved relative to the rviz2 node, so it follows the same remap rules as the publisher.

The status panel shows the number of frames attached, the number lost, the resolution, and the encoding. When there is no publisher it waits and attaches the moment one appears.

### Cost

- rviz2 holds the slot with a byte-lock. It is only for the duration of one upload, but if rendering is slow it becomes publisher backpressure by that much.
- No pixel conversion runs on the CPU. NV12 and the depth range map are GPU shaders. `Normalize Range` is one read pass over the slot per frame, no copy.
- Depends on rviz2 internal APIs (Ogre, `RenderPanel`). Based on jazzy.

## rviz2 PointCloud2 display (`flux_tools/PointCloud2`)

Opens a `flux_cpp` subscription inside the rviz2 process and, on every render tick, copies the points of the frame from `take()` into a `sensor_msgs::msg::PointCloud2` object and hands it to the stock point cloud renderer (`rviz_default_plugins::PointCloudCommon`). Styles, size, alpha, decay time, the position and color transformers (`XYZ`, `Intensity`, `RGB8`, `AxisColor`, `FlatColor`) and point selection are the stock ones.

| Item | Content |
| --- | --- |
| Class | Inherits `rviz_common::Display` directly, like the Image display. The property panel below `Topic` belongs to `PointCloudCommon` |
| Type | `sensor_msgs/PointCloud2`, adapter generated at build time from the installed `PointCloud2.msg`. `x`, `y`, `z` must be `FLOAT32` fields |
| QoS | `depth=1`, `max_borrow=1`. The view lives only inside the copy and is released before the transformers and the TF lookup run |
| Copy | One. Points whose `x`, `y` or `z` is NaN or inf are dropped during that copy, which is the same filter pass the stock display runs on every message it receives |
| Frame | `header.frame_id` is looked up in TF against the fixed frame, as for any stock display. Without a transform the message is dropped and the status says so |
| Topic list | Live `PointCloud2` publishers in the same domain, same rule as the Image display |

`Add` -> `flux_tools` -> `PointCloud2`, then pick the channel in `Topic`. The example publisher `ros2 run flux_example_lidar cloud_pub` (frame `lidar`) shows it with the fixed frame set to `lidar`.

### Cost

- The copy is the whole point payload, so the slot is held for one `memcpy` of the cloud, not for the render. A 131072-point `xyzi` frame is 2 MB.
- Everything after the copy is the stock renderer's cost: transformers on the CPU, then one vertex buffer upload.

## rviz2 DepthCloud display (`flux_tools/DepthCloud`)

Back-projects a depth `sensor_msgs/Image` channel into 3D points on the GPU. The depth frame goes from the slot into a texture as-is, and a vertex shader turns each pixel into a point with the `CameraInfo` intrinsics. The CPU never touches a point, so the frame rate does not depend on the point count. The `CameraInfo` comes from DDS, because it is a few hundred static bytes and every driver already publishes it there.

| Item | Content |
| --- | --- |
| Class | Inherits `rviz_common::Display` directly, like the other two |
| Type | `sensor_msgs/Image` with encoding `16UC1`, `mono16` (millimeters) or `32FC1` (meters). Anything else is an error in the status panel |
| QoS | flux: `depth=1`, `max_borrow=1`. The view lives only inside `update()` and is released right after the texture upload. `CameraInfo`: `SensorDataQoS`, which matches a best effort and a reliable publisher alike |
| Camera Info | The topic is derived from the channel name by the `image_transport` rule, so `/cam/depth/image_raw` gives `/cam/depth/camera_info`. Editing `Camera Info Topic` pins it; a name still equal to the derived one is re-derived when the channel changes |
| Projection | `p`, not `k`: on a rectified pair `p` carries the calibration that matches the image. `binning` and `roi` scale it the same way the stock display scales it |
| Geometry | One vertex per pixel holding its pixel coordinate, built once per resolution. The vertex shader reads that pixel's depth from the texture and places the point; zero, negative and non-finite pixels are the no-return values of these encodings and go behind the far plane, as do points outside `Min Range` and `Max Range`. `Max Range` 0 means no far limit |
| Color | A red to blue ramp over depth, from `Min Range` to `Max Range` (10 m past `Min Range` when `Max Range` is 0). `Point Size (Pixels)` and `Alpha` are the only render settings; there are no transformers, decay or selection |
| Frame | The depth frame's `header.frame_id`, which is the camera optical frame, looked up in TF against the fixed frame at the frame's stamp |
| Topic list | Live `Image` publishers in the same domain, same rule as the Image display |

`Add` -> `flux_tools` -> `DepthCloud`, then pick the channel in `Topic`. Check that `Camera Info Topic` names a live `CameraInfo` publisher; the status says so when it does not.

### Cost

- One texture upload per frame, the size of the depth frame (3.7 MB for 1280x720 `32FC1`), the same cost as the Image display showing that frame.
- The vertex buffer is built once per resolution and never touched again; a 1280x720 frame is 921600 vertices at 12 bytes.
- `Max Range` and `Min Range` change what is visible, not the cost.
