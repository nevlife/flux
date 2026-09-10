# flux_tools

Viewer plugins that subscribe to flux channels directly. For now there is one, the rviz2 Image display.

## rviz2 Image display (`flux_tools/Image`)

Opens a `flux_cpp` subscription inside the rviz2 process and, on every render tick, uploads the bytes of the frame from `take()` straight into an Ogre texture. It passes through no ROS message object, no CDR serialization, and no DDS. The one remaining copy is the GPU upload.

| Item | Content |
| --- | --- |
| Class | Inherits `rviz_common::Display` directly. `RosTopicDisplay` assumes an rclcpp subscription, so it is not used |
| Type | `sensor_msgs/Image`. The adapter is generated at build time by `flux_gen` from the installed `Image.msg` |
| QoS | `depth=1`, `max_borrow=1`. The view lives only inside `update()` and is released right after the upload |
| Encoding | `rgb8` `bgr8` `rgba8` `bgra8` `mono8` `8UC1` `mono16` `16UC1` are uploaded to the texture as is. `nv12` (half-resolution UV after the Y plane, `step` is the Y row pitch) is uploaded as two textures, Y and UV, and a fragment shader produces RGB with BT.601. Anything else is shown as an error in the status panel |
| Topic list | Filled with `Image` fingerprint channels in the same domain via `flux::enumerate_topics()`. Direct input also works |

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
- No 16-bit or float normalization is done. That path adds one more CPU copy, which conflicts with the purpose of this display. Use the bridge if needed. Only NV12 is converted by a GPU shader, so it is received with no CPU cost.
- Depends on rviz2 internal APIs (Ogre, `RenderPanel`). Based on jazzy.
