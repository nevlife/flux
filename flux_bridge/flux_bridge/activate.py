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
import re
import threading
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


def selector(args):
    """topic key -> bool from `ros2 bag record` arguments. Unsure means yes: a spare relay is cheap."""
    all_flag = getattr(args, "all", False) or getattr(args, "all_topics", False)
    names = set(getattr(args, "topics", None) or []) | set(getattr(args, "topics_positional", None) or [])
    include = re.compile(args.regex) if getattr(args, "regex", None) else None
    exclude = re.compile(args.exclude_regex) if getattr(args, "exclude_regex", None) else None
    excluded = set(getattr(args, "exclude_topics", None) or [])

    def wanted(key):
        if key in excluded or (exclude is not None and exclude.search(key)):
            return False
        if all_flag or key in names or (include is not None and include.search(key)):
            return True
        return not names and include is None

    return wanted


class Activator:
    """Keeps one subscription per selected live channel so every relay stays on while recording."""

    def __init__(self, wanted, interval=1.0, log=print):
        self._wanted = wanted
        self._interval = interval
        self._log = log
        self._adapters = discover()
        self._subs = {}
        self._skipped = set()
        self._running = False
        self._thread = None
        self._exit = None

    def __enter__(self):
        self._exit = contextlib.ExitStack()
        self._node = self._exit.enter_context(private_node(f"_flux_record_{os.getpid()}"))
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="flux-activator")
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._running = False
        self._thread.join()
        self._exit.close()
        return False

    def _loop(self):
        while self._running:
            self.refresh()
            time.sleep(self._interval)

    def refresh(self):
        domain = flux.process_domain()
        for topic in flux.enumerate_topics():
            key = topic.key
            if topic.domain != domain or not topic.key_exact or key in self._subs:
                continue
            if not self._wanted(key):
                continue
            adapter = self._adapters.get(topic.fingerprint)
            if adapter is None:
                if key not in self._skipped:
                    self._skipped.add(key)
                    self._log(f"flux: skipping '{key}', no adapter for fingerprint {topic.fingerprint:#018x}")
                continue
            self._subs[key] = self._node.create_subscription(
                adapter.message, key, lambda _msg: None, _dummy_qos()
            )
            self._log(f"flux: asking the bridge for '{key}' ({adapter.ros_type})")
