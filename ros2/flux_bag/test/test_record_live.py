"""record while it runs: pausing it through its service, and channels that appear after it
started."""

import gc
import os
import re
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_prefix
from rclpy.serialization import deserialize_message
from rosbag2_interfaces.srv import Pause, Resume

import flux
from flux_msgs.msg import FluxFrame


def program(name):
    return os.path.join(get_package_prefix("flux_bag"), "lib", "flux_bag", name)


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = rclpy.create_node("flux_bag_live_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def key(name):
    return f"/live{os.getpid()}/{name}"


def record(bag, *args, poll="0.1"):
    return subprocess.Popen(
        [program("record"), *args, "-o", bag, "--poll", poll],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )


def stop(proc):
    proc.send_signal(signal.SIGINT)
    _, err = proc.communicate(timeout=30)
    assert proc.returncode == 0, err
    return err


def values(bag):
    """The first byte of every frame in the bag, in order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions("cdr", "cdr"))
    out = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic.endswith("/flux"):
            out.append(bytes(deserialize_message(data, FluxFrame).data)[0])
    return out


def publish(pub, first, count, period=0.02):
    for i in range(first, first + count):
        assert pub.publish(np.full(4, i, dtype=np.uint8))
        time.sleep(period)


def call(node, srv_type, name):
    client = node.create_client(srv_type, name)
    assert client.wait_for_service(timeout_sec=10)
    future = client.call_async(srv_type.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    assert future.done()
    node.destroy_client(client)


def test_frames_published_while_paused_are_not_recorded(node, tmp_path):
    k = key("paused")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(node, k, slot_size=4096, slot_count=4)
    rec = record(bag, k)
    time.sleep(1.0)
    publish(pub, 0, 20)
    call(node, Pause, "/flux_bag_record/pause")
    publish(pub, 20, 20)
    # The last paused frame is taken before the resume.
    time.sleep(0.3)
    call(node, Resume, "/flux_bag_record/resume")
    publish(pub, 40, 20)
    err = stop(rec)
    got = values(bag)
    assert not [v for v in got if 20 <= v < 40], got
    assert len([v for v in got if v < 20]) > 10 and len([v for v in got if v >= 40]) > 10, got
    assert f"{k}: {len(got)} frames queued" in err, err


def test_a_channel_made_after_the_recorder_started_is_recorded_from_its_start(node, tmp_path):
    k = key("late")
    bag = str(tmp_path / "bag")
    rec = record(bag, k)
    time.sleep(1.0)
    pub = flux.Publisher(node, k, slot_size=4096, slot_count=4)
    publish(pub, 0, 50)
    stop(rec)
    got = values(bag)
    assert got and got[0] <= 10, got
    assert got == list(range(got[0], 50)), got


# Let go after the recorder's 10 s hold, the signpost goes with its last user; the publisher
# coming back makes it anew.
def test_a_publisher_back_after_its_channel_was_let_go_is_recorded_at_once(node, tmp_path):
    k = key("back")
    bag = str(tmp_path / "bag")
    rec = record(bag, k)
    pub = flux.Publisher(node, k, slot_size=4096, slot_count=4)
    signpost = "/dev/shm" + pub.signpost_name
    time.sleep(1.0)
    publish(pub, 0, 20)
    del pub
    gc.collect()
    end = time.monotonic() + 30
    while os.path.exists(signpost) and time.monotonic() < end:
        time.sleep(0.1)
    assert not os.path.exists(signpost)
    pub = flux.Publisher(node, k, slot_size=4096, slot_count=4)
    publish(pub, 100, 50)
    err = stop(rec)
    assert err.count(f"recording flux channel {k}") == 2, err
    again = [v for v in values(bag) if v >= 100]
    assert again and again[0] <= 110, again


FRAME = 1920 * 1080 * 3


def burst(pub, count, gap=0.005):
    """`count` full HD frames `gap` s apart, the first byte numbering them: at 5 ms 1.2 GB/s, more
    than the disk takes (about 0.7 GB/s here)."""
    frame = np.zeros(FRAME, dtype=np.uint8)
    for i in range(count):
        frame[0] = i
        assert pub.publish(frame)
        time.sleep(gap)


def test_a_burst_of_large_frames_waits_in_memory_for_the_bag(node, tmp_path):
    k = key("burst")
    bag = str(tmp_path / "bag")
    # The ZED wrapper's slot count.
    pub = flux.Publisher(node, k, slot_size=FRAME + 4096, slot_count=8)
    rec = record(bag, k)
    time.sleep(1.0)
    burst(pub, 100)
    time.sleep(0.5)
    err = stop(rec)
    assert values(bag) == list(range(100)), err
    assert f"{k}: 100 frames queued, 0 dropped with the cache full, 0 lost" in err, err


def test_frames_past_the_cache_are_dropped_and_counted(node, tmp_path):
    k = key("burst_small")
    bag = str(tmp_path / "bag")
    # The ZED wrapper's slot count.
    pub = flux.Publisher(node, k, slot_size=FRAME + 4096, slot_count=8)
    rec = record(bag, k, "--max-cache-size", str(3 * FRAME))
    time.sleep(1.0)
    # Back to back: three frames of cache fill whatever the disk takes.
    burst(pub, 100, gap=0)
    time.sleep(0.5)
    err = stop(rec)
    got = values(bag)
    line = next(x for x in err.splitlines() if f"{k}: " in x)
    queued, dropped, lost = (int(w) for w in re.findall(r"(\d+) (?:frames|dropped|lost)", line))
    assert queued == len(got) and dropped > 0 and queued + dropped + lost == 100, line
