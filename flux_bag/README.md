# flux_bag

Records flux channels into a rosbag2 bag and plays them back onto flux, without DDS in either direction. ROS topics in the same bag go through the stock `rosbag2_transport::Recorder` and `Player`. The user document is [`docs/en/bag.en.md`](../docs/en/bag.en.md).

```bash
ros2 run flux_bag record -a                       # every flux channel and every ROS topic
ros2 run flux_bag record /cam/left /cam/right     # by name; flux and ROS names share one list
ros2 run flux_bag record -e '^/cam/' -x depth -o run1 --poll 0.5
ros2 run flux_bag play run1 -r 0.5
```

| Item | Value |
| --- | --- |
| Bag topic | `<channel>/flux`, type `flux_msgs/msg/FluxFrame`. The channel name, fingerprint and slot geometry ride inside the message |
| Selection | `-a`, names, `-e`, `-x`, `--exclude-topics`, evaluated by `rosbag2_transport::TopicFilter` for both sides |
| Discovery | `flux::enumerate_topics()` every `--poll` seconds (default 1). A channel in this domain with a live publisher and an exact key is a candidate |
| Copy | record: flux slot -> serialized buffer, once. play: file -> slot, once. `FluxFrame.header.stamp` is the receive time; the frame's own stamp is in `data` |
| QoS | `depth=1`, `max_borrow=2`. Frames the thread could not keep up with count in the per-channel `lost` printed at exit |
| Storage | rosbag2 defaults (mcap, no compression), no write cache |

| File | Content |
| --- | --- |
| `src/record.cpp` | `record`: option parsing, the shared writer, one `ChannelRecorder` thread per flux channel |
| `src/play.cpp` | `play`: `FluxPlayer` reads the `FluxFrame` topics from the file onto flux; the stock `Player` gets the rest |
| `include/flux_bag/flux_frame_cdr.hpp` | `FluxFrame` CDR encoder and decoder. `test/test_flux_frame_cdr.cpp` holds the encoder byte-equal to `rclcpp::Serialization` |
| `include/flux_bag/raw_meta.hpp` | What `meta` carries for codec `raw`: slot geometry and the frame descriptor |
