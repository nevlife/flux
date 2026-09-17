# flux bag

`flux_bag` records flux channels into a rosbag2 bag and plays them back onto flux. The frames do not pass through DDS in either direction. ROS topics in the same bag are handled by the stock rosbag2 recorder and player, so one bag holds both.

```bash
ros2 run flux_bag record -a                       # every flux channel and every ROS topic
ros2 run flux_bag record /cam/left /cam/right     # by name; flux and ROS names share one list
ros2 run flux_bag play rosbag2_2026_09_17-16_35_31
```

## What a bag holds

A flux channel has no ROS type on the wire, only bytes and a fingerprint. The bag stores each frame as a `flux_msgs/msg/FluxFrame` on the topic `<channel>/flux`.

```text
std_msgs/Header header      # receive time
string  flux_topic          # the channel name, "/cam/left"
uint64  fingerprint         # the channel's schema fingerprint
string  codec               # "raw"
uint8[] meta                # raw: slot_size, slot_count, dtype, ndim, shape
uint8[] data                # raw: the frame bytes as they were in the slot
```

`header.stamp` is when the recorder took the frame. The frame's own stamp, if its `.msg` has one, is inside `data` and comes back unchanged on replay. The bag topic is not the channel name so that a bridge relay publishing `sensor_msgs/Image` on `/cam/left` while a recording runs cannot land in the same bag topic.

`ros2 bag info`, `ros2 bag play`, and every other rosbag2 reader open the bag. To read a frame outside flux, deserialize the `FluxFrame` and hand `data` to the adapter's `View` (`sensor_msgs_flux.image.Image.View(np.frombuffer(m.data, np.uint8))`).

## record

```bash
ros2 run flux_bag record [-a] [TOPIC ...] [-e REGEX] [-x REGEX] [--exclude-topics TOPIC ...] [-o OUT] [--poll SEC]
```

| Item | Value |
| --- | --- |
| Selection | The same rules as `ros2 bag record`, applied to flux channel names and ROS topic names alike |
| Discovery | Every `--poll` seconds (default 1) the recorder enumerates flux channels. A live channel in this domain that matches the selection gets a thread from then on |
| Copy | flux slot -> serialized buffer, once, then the borrow is released. The writer takes it from there |
| QoS | `depth=1`, `max_borrow=2`. A frame the thread did not take in time counts in the per-channel `lost` printed at exit |
| Storage | rosbag2 defaults: mcap, no chunk compression, no write cache. `-o` names the bag as `ros2 bag record` does |
| ROS topics | `rosbag2_transport::Recorder` on the same writer, with the same options |

Measured on an Orin with one 1920x1200 bgr8 channel at 30 Hz (6.9 MB per frame): 207 MB/s written, no frame lost over 71 s.

## play

```bash
ros2 run flux_bag play BAG [-r RATE]
```

Every `FluxFrame` topic is read from the file and published on the channel named in `flux_topic`, with the recorded fingerprint and the recorded `slot_size` and `slot_count`. A subscriber written against flux receives the replay exactly as it received the live channel. Every other topic goes to `rosbag2_transport::Player` with the `FluxFrame` topics excluded, so nothing of the frames reaches DDS. Both sides pace from the bag's start time on the same clock.

| Item | Value |
| --- | --- |
| Publisher | Created on the first frame of each topic. A subscriber that was already waiting attaches on its executor's next tick, so the first frames at 30 Hz can precede it |
| Copy | file -> slot, once (`loan`, `memcpy`, `commit`) |
| Not supported | `--loop`, pause, keyboard controls. Only `-r` |

## Not this document

Watching a flux channel from a stock tool while it is live is `flux_bridge` ([bridge.md](bridge.en.md)). It costs a DDS copy per watcher and is not on the recording path.
