"""The bridge executable against real flux publishers and rclpy subscribers."""

import argparse
import gc
import os
import re
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs_flux import image, imu, imu_ros

import flux
from flux import _flux
from flux_bridge.activate import add_arguments, find_channel, run_with_bridge

SMALL = {"slot_size": 4096, "slot_count": 4}
BRIDGE = os.path.join(get_package_prefix("flux_bridge"), "lib", "flux_bridge", "bridge")
BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def topic(name):
    return f"/b{os.getpid()}_{name}"


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = rclpy.create_node("flux_bridge_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def start_bridge(tmp_path):
    started = []

    def start(poll="0.05"):
        log = open(tmp_path / f"bridge{len(started)}.log", "w+")
        p = subprocess.Popen([BRIDGE, "--poll", poll], stdout=log, stderr=subprocess.STDOUT)
        started.append((p, log))
        return p, log

    yield start
    for p, log in started:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            p.wait(timeout=10)
        log.close()


@pytest.fixture
def bridge(start_bridge):
    return start_bridge()


def output(log):
    log.flush()
    log.seek(0)
    return log.read()


def eventually(done, limit=10.0, each=None):
    end = time.monotonic() + limit
    while not done() and time.monotonic() < end:
        if each is not None:
            each()
        time.sleep(0.01)
    return done()


# Holds the one subscriber place of a publisher made with max_subscribers=1.
HOLD = (
    "import sys\n"
    "from flux import _flux as f\n"
    "s = f._Subscription(sys.argv[1], sys.argv[2], '', int(sys.argv[3]), f.QoS(), f.MemoryPolicy())\n"
    "print('ready', flush=True)\n"
    "sys.stdin.read()\n"
)


def hold_the_only_place(pub, fingerprint=0):
    args = [sys.executable, "-c", HOLD, pub.signpost_name, pub.topic_name, str(fingerprint)]
    p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    assert p.stdout.readline() == "ready\n"
    return p


# A channel whose publisher is gone is still enumerated. Returning it made `ros2 topic hz_flux`
# wait on a relay that never starts.
def test_find_channel_takes_only_a_live_exact_key(node):
    pub = flux.Publisher(node, topic("gone"), **SMALL)
    assert find_channel(pub.topic_name).key == pub.topic_name
    assert find_channel(pub.topic_name + "x") is None
    del pub
    gc.collect()
    assert find_channel(topic("gone")) is None


# The bridge relays the typed one of two channels under one name; echo_flux must ask for it.
def test_find_channel_prefers_the_channel_the_bridge_relays(node):
    raw = flux.Publisher(node, topic("twins"), **SMALL)
    typed = flux.Publisher(node, topic("twins"), fingerprint=imu.FINGERPRINT__, **SMALL)
    assert find_channel(typed.topic_name).fingerprint == imu.FINGERPRINT__
    del typed
    gc.collect()
    assert find_channel(raw.topic_name).fingerprint == flux.NO_SCHEMA


def test_a_fingerprint_names_its_installed_message_type():
    assert flux.ros_type_of(imu.FINGERPRINT__) == "sensor_msgs/msg/Imu"
    assert flux.ros_type_of(0xB21D6E) is None


def test_a_watched_channel_is_relayed_and_a_bad_frame_is_skipped(node, bridge):
    proc, log = bridge
    pub = flux.Publisher(node, topic("imu"), fingerprint=imu.FINGERPRINT__, **SMALL)
    got = []
    sub = node.create_subscription(Imu, pub.topic_name, got.append, BEST_EFFORT)
    assert eventually(lambda: f"{pub.topic_name}: relaying as sensor_msgs/msg/Imu" in output(log))
    assert eventually(lambda: node.count_publishers(pub.topic_name) > 0)

    def send(z):
        m = Imu()
        m.header.frame_id = "imu"
        m.linear_acceleration.z = z
        imu_ros.msg_to_frame(m, imu.build__(pub)).commit__()
        rclpy.spin_once(node, timeout_sec=0.01)

    def relayed(z):
        return eventually(lambda: z in {m.linear_acceleration.z for m in got}, each=lambda: send(z))

    assert relayed(9.8) and relayed(1.5)
    assert got[0].header.frame_id == "imu"

    # Last, so nothing overwrites it in the relay's depth-1 queue, and kept up for longer than the
    # relay's 200 ms take wait. A frame cut short of its own descriptors is a wrong message, not
    # the end of the channel.
    pub.publish(np.zeros(8, dtype=np.uint8))
    time.sleep(0.3)
    node.destroy_subscription(sub)
    assert eventually(lambda: f"{pub.topic_name}: stopped after" in output(log))
    stopped = next(l for l in output(log).splitlines() if f"{pub.topic_name}: stopped" in l)
    assert stopped.endswith(" 1 unreadable"), stopped
    proc.send_signal(signal.SIGINT)
    assert proc.wait(timeout=10) == 0


def test_a_channel_back_under_another_type_within_one_poll_is_relayed_as_that(node, start_bridge):
    _, log = start_bridge(poll="1")
    name = topic("swap")
    pub = flux.Publisher(node, name, fingerprint=imu.FINGERPRINT__, **SMALL)
    sub = node.create_subscription(Imu, name, lambda _m: None, BEST_EFFORT)
    assert eventually(lambda: f"{name}: relaying as sensor_msgs/msg/Imu" in output(log))
    del pub
    gc.collect()
    pub = flux.Publisher(node, name, fingerprint=image.FINGERPRINT__, **SMALL)
    assert eventually(lambda: f"{name}: relaying as sensor_msgs/msg/Image" in output(log))
    node.destroy_subscription(sub)
    del pub


# A key from another process need not be a ROS name, and rclcpp throws on one that is not.
def test_a_key_that_is_not_a_ros_name_is_skipped_not_fatal(node, bridge):
    proc, log = bridge
    key = f"/b{os.getpid()} bad \x1b[31m"
    # signpost_name refuses such a key, so the peer names the object as the flattening would.
    signpost = _flux.signpost_name(re.sub(r"[^A-Za-z0-9_/]", "/", key), image.FINGERPRINT__)
    peer = _flux._Publisher(signpost, key, "", image.FINGERPRINT__, 4096, 4, flux.MemoryPolicy())
    assert eventually(lambda: "key is not a ROS topic name" in output(log))
    time.sleep(0.3)
    assert proc.poll() is None
    assert "\x1b" not in output(log)
    del peer
    proc.send_signal(signal.SIGINT)
    assert proc.wait(timeout=10) == 0


# A channel that refuses the bridge's subscription is left out until its publisher restarts; the
# others go on relaying.
def test_a_channel_that_refuses_the_relay_is_left_out_until_it_restarts(node, bridge):
    proc, log = bridge
    full = flux.Publisher(
        node, topic("full"), fingerprint=imu.FINGERPRINT__, max_subscribers=1, **SMALL
    )
    holder = hold_the_only_place(full, imu.FINGERPRINT__)
    ok = flux.Publisher(node, topic("ok"), fingerprint=imu.FINGERPRINT__, **SMALL)
    subs = [
        node.create_subscription(Imu, p.topic_name, lambda m: None, BEST_EFFORT) for p in (full, ok)
    ]
    assert eventually(lambda: f"{full.topic_name}: not relayed" in output(log))
    assert eventually(lambda: f"{ok.topic_name}: relaying as" in output(log))
    time.sleep(0.3)
    assert proc.poll() is None
    assert output(log).count(f"{full.topic_name}: not relayed") == 1
    del full
    gc.collect()
    full = flux.Publisher(
        node, topic("full"), fingerprint=imu.FINGERPRINT__, max_subscribers=2, **SMALL
    )
    assert eventually(lambda: f"{full.topic_name}: relaying as" in output(log))
    for s in subs:
        node.destroy_subscription(s)
    holder.stdin.close()
    holder.wait()


def test_a_channel_nobody_watches_is_not_relayed(node, bridge):
    _, log = bridge
    pub = flux.Publisher(node, topic("idle"), fingerprint=imu.FINGERPRINT__, **SMALL)
    time.sleep(0.5)
    assert pub.topic_name not in output(log)
    assert node.count_publishers(pub.topic_name) == 0


def test_a_channel_of_no_installed_type_is_reported_once(node, bridge):
    _, log = bridge
    pub = flux.Publisher(node, topic("unknown"), fingerprint=0xB21D6E, **SMALL)
    assert eventually(lambda: f"{pub.topic_name}: no installed message type" in output(log))
    time.sleep(0.3)
    assert output(log).count(f"{pub.topic_name}: no installed message type") == 1


def test_echo_and_hz_resolve_a_relative_name_as_the_stock_verbs_do(node, bridge):
    pub = flux.Publisher(node, topic("rel"), fingerprint=imu.FINGERPRINT__, **SMALL)
    args = argparse.Namespace(bridge_timeout=10.0)
    assert run_with_bridge(args, pub.topic_name.lstrip("/"), lambda _a: 0) == 0


@pytest.mark.parametrize("value", ["nan", "inf", "-1", "abc"])
def test_a_bridge_timeout_that_is_no_wait_is_refused(value):
    parser = argparse.ArgumentParser()
    add_arguments(parser)
    with pytest.raises(SystemExit):
        parser.parse_args(["--bridge-timeout", value])


def send_imu(node, pub, z):
    m = Imu()
    m.header.frame_id = "imu"
    m.linear_acceleration.z = z
    imu_ros.msg_to_frame(m, imu.build__(pub)).commit__()
    rclpy.spin_once(node, timeout_sec=0.01)


def mirrored(node, pub, got, z):
    return eventually(
        lambda: z in {m.linear_acceleration.z for m in got}, each=lambda: send_imu(node, pub, z)
    )


def test_a_mirrored_publisher_reaches_a_ros_subscriber_without_the_bridge(node):
    pub = flux.Publisher(
        node, topic("mirror"), fingerprint=imu.FINGERPRINT__, mirror_to_ros=True, **SMALL
    )
    assert node.count_publishers(pub.topic_name) == 1
    got = []
    sub = node.create_subscription(Imu, pub.topic_name, got.append, BEST_EFFORT)
    assert mirrored(node, pub, got, 9.8) and mirrored(node, pub, got, 1.5)
    assert got[0].header.frame_id == "imu"
    node.destroy_subscription(sub)


def test_the_bridge_leaves_a_mirrored_topic_alone(node, bridge):
    _, log = bridge
    pub = flux.Publisher(
        node, topic("mirror_br"), fingerprint=imu.FINGERPRINT__, mirror_to_ros=True, **SMALL
    )
    got = []
    sub = node.create_subscription(Imu, pub.topic_name, got.append, BEST_EFFORT)
    assert mirrored(node, pub, got, 2.5)
    time.sleep(0.3)
    assert f"{pub.topic_name}: relaying" not in output(log)
    assert node.count_publishers(pub.topic_name) == 1
    node.destroy_subscription(sub)


# The only subscriber place stays free while no ROS 2 subscriber wants the mirror.
def test_an_unwatched_mirror_holds_no_reader(node):
    pub = flux.Publisher(
        node,
        topic("mirror_idle"),
        fingerprint=imu.FINGERPRINT__,
        mirror_to_ros=True,
        max_subscribers=1,
        **SMALL,
    )
    time.sleep(0.3)
    hold = hold_the_only_place(pub, imu.FINGERPRINT__)
    hold.stdin.close()
    assert hold.wait(timeout=10) == 0


def test_dropping_a_mirrored_publisher_stops_its_thread(node):
    pub = flux.Publisher(
        node, topic("mirror_drop"), fingerprint=imu.FINGERPRINT__, mirror_to_ros=True, **SMALL
    )
    name = f"flux mirror {pub.topic_name}"
    assert any(t.name == name for t in threading.enumerate())
    del pub
    gc.collect()
    assert eventually(lambda: all(t.name != name for t in threading.enumerate()), limit=2.0)


def test_an_untyped_publisher_has_no_mirror(node):
    with pytest.raises(flux.Error):
        flux.Publisher(node, topic("mirror_raw"), mirror_to_ros=True, **SMALL)


@pytest.mark.skipif(flux.probe().route == flux.Route.NONE, reason="needs a GPU route")
def test_a_device_backed_publisher_has_no_mirror(node):
    with pytest.raises(flux.Error):
        flux.Publisher(
            node,
            topic("mirror_cuda"),
            fingerprint=imu.FINGERPRINT__,
            device="cuda",
            mirror_to_ros=True,
            **SMALL,
        )


def test_a_mirror_serves_ros_subscribers_that_come_and_go(node):
    pub = flux.Publisher(
        node, topic("mirror_churn"), fingerprint=imu.FINGERPRINT__, mirror_to_ros=True, **SMALL
    )
    for i in range(5):
        got = []
        sub = node.create_subscription(Imu, pub.topic_name, got.append, BEST_EFFORT)
        assert mirrored(node, pub, got, float(i))
        node.destroy_subscription(sub)
        # Long enough for the mirror to see no subscriber and let its flux reader go.
        time.sleep(0.3)


# A ROS 2 publisher that turns up while a relay runs ends the relay: two would deliver twice.
def test_a_relay_stops_once_another_ros_publisher_appears(node, bridge):
    _, log = bridge
    pub = flux.Publisher(node, topic("late_pub"), fingerprint=imu.FINGERPRINT__, **SMALL)
    sub = node.create_subscription(Imu, pub.topic_name, lambda _m: None, BEST_EFFORT)
    assert eventually(lambda: f"{pub.topic_name}: relaying" in output(log))
    other = node.create_publisher(Imu, pub.topic_name, BEST_EFFORT)
    assert eventually(lambda: f"{pub.topic_name}: stopped after" in output(log))
    node.destroy_publisher(other)
    node.destroy_subscription(sub)
