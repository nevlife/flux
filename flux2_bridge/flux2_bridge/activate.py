"""Switch a relay on from the tool side.

The bridge relays a channel only while a ROS 2 subscriber for its name exists, and it looks once
per poll. A stock verb started cold would therefore miss the first second and, for `hz`, wait on
a topic with no publisher. The helpers here hold a subscription that does nothing on a private
rclpy context, wait until the bridge's publisher is on the graph, and only then hand over to the
stock verb. The channel's type comes from flux2 itself: its fingerprint names an installed
message type, so no DDS discovery is needed to learn it.
"""

import argparse
import contextlib
import math
import os
import time

import flux2

# The bridge's node is named flux2_bridge_<pid> (src/bridge.cpp).
NODE_PREFIX = "flux2_bridge_"
DEFAULT_TIMEOUT = 10.0


def find_channel(name):
    """The live channel called `name` the bridge relays, else any live one by that name, or None.

    Channels of two fingerprints can share a name. Like the bridge, the first in enumeration order
    whose fingerprint names an installed message type wins."""
    live = [
        t
        for t in flux2.enumerate_topics()
        if t.key_exact and t.key == name and flux2.read_channel_stats(t.signpost).live
    ]
    typed = (t for t in live if flux2.ros_type_of(t.fingerprint) is not None)
    return next(typed, live[0] if live else None)


def _timeout(text):
    seconds = float(text)
    if not math.isfinite(seconds) or seconds < 0:
        raise argparse.ArgumentTypeError(f"{text} is not a finite number of seconds >= 0")
    return seconds


def add_arguments(parser):
    parser.add_argument(
        "--bridge-timeout",
        type=_timeout,
        default=DEFAULT_TIMEOUT,
        help="seconds to wait for the flux2 bridge to start publishing (default: %(default)s)",
    )


def _dummy_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
    )


def _bridge_publishers(node, topic):
    return [
        info
        for info in node.get_publishers_info_by_topic(topic)
        if info.node_name.startswith(NODE_PREFIX)
    ]


@contextlib.contextmanager
def private_node(name):
    import rclpy

    context = rclpy.Context()
    rclpy.init(context=context)
    node = None
    try:
        node = rclpy.create_node(name, context=context)
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown(context=context)


def wait_for_bridge(node, topic, timeout, interval=0.2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _bridge_publishers(node, topic):
            return True
        time.sleep(interval)
    return bool(_bridge_publishers(node, topic))


def run_with_bridge(args, names, action):
    """Hold a subscription on each channel in `names` (one name or a list) until `action(args)`
    returns."""
    from rclpy.expand_topic_name import expand_topic_name

    names = [names] if isinstance(names, str) else list(names or ())
    if not names:
        print("flux2: name the flux2 channel")
        return 1
    node_name = f"_flux2_cli_{os.getpid()}"
    wanted = []
    for name in names:
        # Relative to '/', as the stock verb resolves the same argument.
        try:
            resolved = expand_topic_name(name, node_name, "/")
        except ValueError as e:
            print(f"flux2: {e}")
            return 1
        channel = find_channel(resolved)
        if channel is None:
            print(f"flux2: no live channel named '{name}'")
            return 1
        ros_type = flux2.ros_type_of(channel.fingerprint)
        if ros_type is None:
            print(
                f"flux2: no installed message type for '{name}' "
                f"(fingerprint {channel.fingerprint:#018x})"
            )
            return 1
        wanted.append((channel.key, ros_type))
    from rosidl_runtime_py.utilities import get_message

    with private_node(node_name) as node:
        for key, ros_type in wanted:
            node.create_subscription(get_message(ros_type), key, lambda _msg: None, _dummy_qos())
        for key, _ in wanted:
            if not wait_for_bridge(node, key, args.bridge_timeout):
                print(
                    f"flux2: no bridge publisher on '{key}' within {args.bridge_timeout:.1f} s. "
                    "Is `ros2 run flux2_bridge bridge` running?"
                )
                return 1
        return action(args)
