# flux bridge

`flux_bridge` republishes flux channels as DDS topics. It is the side path that lets the stock ROS 2 tools, `ros2 bag`, `rqt`, `ros2 topic`, read a channel that lives only in shared memory. It is not on the data path. A channel is relayed only while a ROS 2 subscriber for its name exists, so a channel nobody watches costs nothing.

```bash
ros2 run flux_bridge bridge
```

One node, `flux_bridge_<pid>`, started by hand. It is not forked by an application and there is no daemon. Run one per domain. The domain is the one the process resolves (`FLUX_DOMAIN`, else `ROS_DOMAIN_ID`, else `0`), the same rule the nodes use.

## What it does

Every second (`--poll`) the bridge does one pass:

1. `flux.enumerate_topics()`, kept to channels in its domain with a live publisher and an exact key.
2. For each, the installed adapter for the channel's fingerprint. A channel whose adapter is not installed is logged once and skipped.
3. `get_subscriptions_info_by_topic()` on the ROS graph, the bridge's own node excluded. One or more external subscribers means the channel is wanted.
4. A relay starts for every wanted channel that has none, and stops for every relay whose channel is no longer wanted or no longer live.

A relay is one thread: `take_blocking()` on a `flux.Subscription(depth=1, max_borrow=1)`, `frame_to_msg()` into a ROS message object, `publish()` on an rclpy publisher of the same name. The view is released before the publish. The frame path is therefore flux slot -> one copy -> CDR serialization -> DDS, and that is the cost of using a stock tool on a flux channel. The type comes from the adapter that `flux_gen` generated (`<pkg>_flux.<name>` for the frame, `<pkg>_flux.<name>_ros` for the message object), so the adapter package must be installed where the bridge runs.

| Item | Value |
| --- | --- |
| DDS QoS | `depth=1`, best effort, volatile. The same latest-frame semantics as the flux side. A tool that attaches late gets the next frame, not an old one |
| Node | `flux_bridge_<pid>`, namespace `/`. `rqt_graph` shows this node as the publisher; the flux publisher is not on the DDS graph |
| Activation latency | Up to one poll after the subscriber appears |
| Timestamp | `header.stamp` is whatever the frame carries. rosbag2 stamps each message with the time it received it from the bridge |
| Adapter lookup | Every `<pkg>_flux` package on `sys.path`, once at start. Install adapters before starting the bridge |

## The verbs

A stock verb started cold misses the first poll, and `hz` waits on a topic with no publisher. The package registers three verbs that hold a subscription which does nothing, wait until the bridge's publisher is on the graph, then run the stock verb unchanged.

```bash
ros2 topic echo_flux /cam/left --no-arr
ros2 topic hz_flux /cam/left
ros2 bag record_flux /cam/left /cam/right
ros2 bag record_flux -a
```

`echo_flux` and `hz_flux` take every argument the stock verb takes, plus `--bridge-timeout` (default 10 s). The channel name is looked up on the flux side, so it must be a live flux channel. `record_flux` takes every `ros2 bag record` argument. It keeps one subscription per flux channel that matches the selection (`-a`, topic names, `--regex`, `--exclude-regex`, `--exclude-topics`) for as long as the recording runs, so a relay stays on even if the recorder itself has not subscribed yet. No selection at all keeps every channel on.

A bag written this way is a normal bag. `ros2 bag play`, `rqt_bag`, and anything else that reads rosbag2 read it. Playback publishes on DDS, not on flux.

## Cost

Measured on an Orin with 1280x720 NV12 frames (3.4 MB) at 30 Hz from the ZED wrapper, default rmw.

| Step | Per frame |
| --- | --- |
| `frame_to_msg` | 1.5 ms |
| CDR serialization | 1.5 ms |
| DDS delivery of 3.4 MB to a local subscriber | the rest. One relay with a subscriber lands at about 20 Hz |

Relays run one thread each and the conversion holds the GIL, so several relays share one core's worth of Python. Three cameras plus a 55 MB mosaic recorded at the same time reached 7 Hz per camera in the bag. For a stream that must arrive whole at full rate, use a direct subscriber (`flux_tools`) or write the consumer against flux.

## What it does not do

- DDS -> flux. Nothing on a ROS topic is written into a flux channel.
- Services, actions, parameters.
- Channels whose adapter is not installed. They are listed by `flux topic list` and logged once by the bridge.
- Survive its own death gracefully. When the bridge exits, its publishers leave the graph and the tools see the topic disappear. Start it again.
