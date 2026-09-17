"""Switch a relay on from the tool side.

The bridge relays a channel only while a ROS 2 subscriber for its name exists, and it looks once
per poll. A stock verb started cold would therefore miss the first second and, for `hz`, wait on
a topic with no publisher. The helpers here hold a subscription that does nothing on a private
rclpy context, wait until the bridge's publisher is on the graph, and only then hand over to the
stock verb. The channel's type comes from flux itself: the fingerprint of the channel selects the
installed adapter, so no DDS discovery is needed to learn it.
"""

import contextlib
import os
import time

import flux

from .adapters import discover
from .bridge import NODE_PREFIX

DEFAULT_TIMEOUT = 10.0


def find_channel(name, domain=None):
    """The live channel called `name` in this domain, or None."""
    domain = flux.process_domain() if domain is None else domain
    flat = flux.flatten_key(name)
    for topic in flux.enumerate_topics():
        if topic.domain != domain:
            continue
        if topic.key == name or flux.flatten_key(topic.key) == flat:
            return topic
    return None


def add_arguments(parser):
    parser.add_argument(
        "--bridge-timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help="seconds to wait for the flux bridge to start publishing (default: %(default)s)",
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


def run_with_bridge(args, topic_name, action):
    """Hold a subscription on `topic_name` until `action(args)` returns."""
    channel = find_channel(topic_name)
    if channel is None:
        print(f"flux: no live channel named '{topic_name}' in domain '{flux.process_domain()}'")
        return 1
    adapter = discover().get(channel.fingerprint)
    if adapter is None:
        print(f"flux: no adapter installed for '{topic_name}' (fingerprint {channel.fingerprint:#018x})")
        return 1
    with private_node(f"_flux_cli_{os.getpid()}") as node:
        node.create_subscription(adapter.message, channel.key, lambda _msg: None, _dummy_qos())
        if not wait_for_bridge(node, channel.key, args.bridge_timeout):
            print(
                f"flux: no bridge publisher on '{channel.key}' within {args.bridge_timeout:.1f} s. "
                "Is `ros2 run flux_bridge bridge` running in this domain?"
            )
            return 1
        return action(args)

