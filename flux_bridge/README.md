# flux_bridge

Republishes flux channels as DDS topics so the stock ROS 2 tools can read them. One node, started by hand, that relays a channel only while a ROS 2 subscriber for its name exists. The user document is [`docs/en/bridge.en.md`](../docs/en/bridge.en.md).

```bash
ros2 run flux_bridge bridge                     # the relay
ros2 topic hz_flux /cam/left                    # stock hz, after switching the relay on
ros2 topic echo_flux /cam/left --no-arr
ros2 bag record_flux /cam/left /cam/right       # stock record, relays held on while it runs
```

| File | Content |
| --- | --- |
| `flux_bridge/bridge.py` | The node. Enumerates live channels once a second, asks the ROS graph for external subscribers, starts and stops one relay thread per channel |
| `flux_bridge/adapters.py` | Fingerprint -> installed `flux_gen` adapter. The channel carries no type name, so the map is built by importing every `<pkg>_flux` package on the path |
| `flux_bridge/activate.py` | The tool side: a subscription that does nothing on a private rclpy context, held until the bridge's publisher is on the graph and then for as long as the stock verb runs |
| `flux_bridge/verb/` | `ros2 topic echo_flux`, `ros2 topic hz_flux`, `ros2 bag record_flux` |
