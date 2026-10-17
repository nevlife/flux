"""play's pause, resume and rate move the flux channels and the ROS topics on one clock, and
large frames are read ahead of the time they are due."""

import os
import statistics
import struct
import subprocess
import threading
import time

import pytest
import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_prefix
from rclpy.executors import MultiThreadedExecutor
from rclpy.serialization import serialize_message
from rosbag2_interfaces.srv import Pause, Resume, SetRate
from std_msgs.msg import Int32

import flux
from flux_msgs.msg import FluxFrame
from rosbag2_compat import topic_metadata

FINGERPRINT = 0xBA6
FRAME_TYPE = "flux_msgs/msg/FluxFrame"


def program(name):
    return os.path.join(get_package_prefix("flux_bag"), "lib", "flux_bag", name)


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = rclpy.create_node("flux_bag_control_test")
    executor = MultiThreadedExecutor()
    executor.add_node(n)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    yield n
    executor.shutdown()
    n.destroy_node()
    rclpy.shutdown()


def key(name):
    return f"/bag{os.getpid()}/{name}"


def write_bag(path, topic, n, step_ns, nbytes=16, ros_topic=None):
    """n raw uint8 FluxFrames on `topic` whose first byte is their index; with `ros_topic`, an
    Int32 of the same index at the same time."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions("cdr", "cdr"))
    name = topic + "/flux"
    writer.create_topic(topic_metadata(0, name, FRAME_TYPE))
    if ros_topic:
        writer.create_topic(topic_metadata(1, ros_topic, "std_msgs/msg/Int32"))
    f = FluxFrame()
    f.flux_topic = topic
    f.fingerprint = FINGERPRINT
    f.codec = "raw"
    f.meta = struct.pack("<IIBB", max(nbytes, 4096), 4, 0, 1) + struct.pack("<Q", nbytes)
    data = bytearray(nbytes)
    for i in range(n):
        at = 1_000_000_000 + i * step_ns
        data[0] = i
        f.data = bytes(data)
        writer.write(name, serialize_message(f), at)
        if ros_topic:
            writer.write(ros_topic, serialize_message(Int32(data=i)), at)
    del writer


class Arrivals:
    """(arrival time, index) of each frame on a flux channel and each Int32 on a ROS topic."""

    def __init__(self, node, topic, ros_topic=None):
        self.flux, self.ros = [], []
        self._sub = flux.Subscription(node, topic, fingerprint=FINGERPRINT, qos=flux.QoS(depth=16))
        self._ros_sub = None
        if ros_topic:
            self._ros_sub = node.create_subscription(
                Int32, ros_topic, lambda m: self.ros.append((time.monotonic(), m.data)), 100
            )
        self._node = node
        self._done = threading.Event()
        self._thread = threading.Thread(target=self._take)
        self._thread.start()

    def _take(self):
        while True:
            frame = self._sub.take_blocking(20_000_000)
            if frame is not None:
                self.flux.append((time.monotonic(), int(frame[0])))
                del frame
            elif self._done.is_set():
                break

    def wait_for(self, count, timeout=10.0):
        end = time.monotonic() + timeout
        while len(self.flux) < count:
            assert time.monotonic() < end, f"{len(self.flux)} of {count} frames"
            time.sleep(0.01)

    def close(self):
        self._done.set()
        self._thread.join()
        if self._ros_sub is not None:
            self._node.destroy_subscription(self._ros_sub)


def play(bag, *args):
    """play under a node name of its own, so its services are this test's alone."""
    name = f"play{os.getpid()}"
    proc = subprocess.Popen(
        [program("play"), bag, *args, "--ros-args", "-r", f"__node:={name}"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return proc, f"/{name}"


def call(node, srv_type, name, **fields):
    client = node.create_client(srv_type, name)
    assert client.wait_for_service(timeout_sec=10.0)
    response = client.call(srv_type.Request(**fields))
    node.destroy_client(client)
    return response


def in_step(arrivals, skip=(0.0, 0.0), tolerance=0.05):
    """For every index both sides delivered, outside the `skip` interval of arrival times, the ROS
    message and the frame arrive together."""
    ros = {i: t for t, i in arrivals.ros}
    pairs = [(t, ros[i]) for t, i in arrivals.flux if i in ros and not skip[0] <= t <= skip[1]]
    assert len(pairs) >= len(arrivals.flux) // 2
    return max(abs(a - b) for a, b in pairs) < tolerance


def test_pause_and_resume_hold_flux_and_ros_together(node, tmp_path):
    k, chatter = key("paused"), key("paused_chatter")
    bag = str(tmp_path / "bag")
    write_bag(bag, k, 30, 100_000_000, ros_topic=chatter)
    got = Arrivals(node, k, chatter)
    proc, name = play(bag)
    try:
        got.wait_for(8)
        call(node, Pause, name + "/pause")
        time.sleep(0.2)
        held = (len(got.flux), len(got.ros))
        time.sleep(1.0)
        assert (len(got.flux), len(got.ros)) == held
        call(node, Resume, name + "/resume")
        assert proc.wait(timeout=20) == 0
    finally:
        proc.kill()
        got.close()
    assert [i for _, i in got.flux] == list(range(30))
    gaps = [b[0] - a[0] for a, b in zip(got.flux, got.flux[1:])]
    assert max(gaps) > 1.0
    assert statistics.median(gaps) == pytest.approx(0.1, abs=0.02)
    assert in_step(got)


def test_set_rate_changes_the_pace_of_flux_frames(node, tmp_path):
    k, chatter = key("rate"), key("rate_chatter")
    bag = str(tmp_path / "bag")
    write_bag(bag, k, 40, 100_000_000, ros_topic=chatter)
    got = Arrivals(node, k, chatter)
    proc, name = play(bag)
    try:
        got.wait_for(8)
        assert call(node, SetRate, name + "/set_rate", rate=4.0).success
        at = time.monotonic()
        assert proc.wait(timeout=20) == 0
    finally:
        proc.kill()
        got.close()
    assert [i for _, i in got.flux] == list(range(40))
    before = [t for t, _ in got.flux if t < at]
    after = [t for t, _ in got.flux if t > at + 0.1]
    assert statistics.median(b - a for a, b in zip(before, before[1:])) == pytest.approx(
        0.1, abs=0.02
    )
    assert statistics.median(b - a for a, b in zip(after, after[1:])) == pytest.approx(
        0.025, abs=0.01
    )
    # The stock player sleeps out the wait it was in at the old rate, one message at most.
    assert in_step(got, skip=(at, at + 0.15))


# 8 MiB frames at 30 Hz: the loader reads them ahead, so each goes out when it is due.
def test_large_frames_play_at_their_recorded_spacing(node, tmp_path):
    k = key("large")
    bag = str(tmp_path / "bag")
    write_bag(bag, k, 30, 1_000_000_000 // 30, nbytes=8 << 20)
    got = Arrivals(node, k)
    proc, _ = play(bag)
    try:
        assert proc.wait(timeout=30) == 0
    finally:
        proc.kill()
        got.close()
    assert [i for _, i in got.flux] == list(range(30))
    t0 = got.flux[0][0]
    late = [abs(t - t0 - i / 30) for t, i in got.flux]
    assert max(late) < 0.015, late
