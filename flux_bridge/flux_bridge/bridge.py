"""The relay: one ROS 2 node that republishes flux channels on demand.

Every `poll` seconds the bridge lists the live channels in its domain and, for each one whose
adapter is installed, asks the ROS graph whether anyone other than itself subscribes to a topic
of that name. A relay starts when that becomes true and stops when it stops being true, so a
channel nobody watches costs nothing. The frame path is flux slot -> `frame_to_msg` copy -> DDS
serialization, which is the cost of using stock tools on a flux channel.
"""

import argparse
import faulthandler
import os
import signal
import sys
import threading
import time

import flux

from .adapters import discover

DEFAULT_POLL = 1.0
NODE_PREFIX = "flux_bridge_"


def plan(live, wanted, active):
    """(start, stop) given the live channel keys, the keys with a ROS subscriber, the relayed keys."""
    start = sorted(k for k in wanted if k in live and k not in active)
    stop = sorted(k for k in active if k not in wanted or k not in live)
    return start, stop


def external_subscribers(node, topic):
    """ROS 2 subscriptions on `topic` that are not the bridge's own."""
    own = (node.get_name(), node.get_namespace())
    return [
        info
        for info in node.get_subscriptions_info_by_topic(topic)
        if (info.node_name, info.node_namespace) != own
    ]


def live_channels(domain):
    """key -> Topic for every channel in `domain` with a live publisher and an exact key."""
    found = {}
    for topic in flux.enumerate_topics():
        if topic.domain != domain or not topic.key_exact:
            continue
        if flux.read_channel_stats(topic.signpost).live:
            found[topic.key] = topic
    return found


class Relay:
    """One channel relayed to one ROS 2 publisher on its own thread.

    A thread rather than `flux.ros.Executor` because relays come and go while the bridge runs, and
    the executor takes its subscriptions before spin. `take_blocking` parks on the futex, so an
    idle relay costs no CPU.
    """

    WAIT_NS = 200_000_000

    def __init__(self, node, topic, adapter, qos):
        self.key = topic.key
        self.published = 0
        self._adapter = adapter
        self._sub = flux.Subscription(
            topic.key, fingerprint=topic.fingerprint, qos=flux.QoS(depth=1, max_borrow=1)
        )
        self._pub = node.create_publisher(adapter.message, topic.key, qos)
        self._node = node
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="flux-relay")
        self._thread.start()

    def _run(self):
        view_of = self._adapter.view
        to_msg = self._adapter.frame_to_msg
        while self._running:
            frame = self._sub.take_blocking(self.WAIT_NS)
            if frame is None:
                continue
            msg = to_msg(view_of(frame))
            frame = None
            self._pub.publish(msg)
            self.published += 1

    def stop(self):
        self._running = False
        self._thread.join()
        self._node.destroy_publisher(self._pub)
        self._sub = None


class Bridge:
    def __init__(self, node, adapters, qos, log=None):
        self._node = node
        self._adapters = adapters
        self._qos = qos
        self._log = log or (lambda text: None)
        self._relays = {}
        self._unknown = set()
        self._domain = flux.process_domain()

    @property
    def active(self):
        return sorted(self._relays)

    def tick(self):
        live = live_channels(self._domain)
        bridgeable = {}
        for key, topic in live.items():
            adapter = self._adapters.get(topic.fingerprint)
            if adapter is None:
                if key not in self._unknown:
                    self._unknown.add(key)
                    self._log(f"{key}: no adapter installed for fingerprint {topic.fingerprint:#018x}")
                continue
            bridgeable[key] = (topic, adapter)
        wanted = {k for k in bridgeable if external_subscribers(self._node, k)}
        start, stop = plan(set(bridgeable), wanted, set(self._relays))
        for key in stop:
            relay = self._relays.pop(key)
            relay.stop()
            self._log(f"{key}: stopped after {relay.published} frames")
        for key in start:
            topic, adapter = bridgeable[key]
            self._relays[key] = Relay(self._node, topic, adapter, self._qos)
            self._log(f"{key}: relaying as {adapter.ros_type}")

    def close(self):
        for key in list(self._relays):
            self._relays.pop(key).stop()


def build_parser():
    parser = argparse.ArgumentParser(
        prog="ros2 run flux_bridge bridge",
        description="Republish flux channels as DDS topics while a ROS 2 subscriber wants them.",
    )
    parser.add_argument(
        "--poll",
        type=float,
        default=DEFAULT_POLL,
        help="seconds between graph checks (default: %(default)s)",
    )
    return parser


def _ros_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
    )


def main(argv=None):
    import rclpy

    args = build_parser().parse_args(argv)
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    adapters = discover()
    rclpy.init(args=sys.argv if argv is None else None)
    node = rclpy.create_node(NODE_PREFIX + str(os.getpid()))
    logger = node.get_logger()
    logger.info(
        f"{len(adapters)} adapter(s): " + ", ".join(sorted(a.type_name for a in adapters.values()))
    )
    bridge = Bridge(node, adapters, _ros_qos(), logger.info)
    try:
        while rclpy.ok():
            bridge.tick()
            time.sleep(args.poll)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
