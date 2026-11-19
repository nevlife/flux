"""record while it runs: pausing it through its service, and channels that appear after it
started."""

import ctypes
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
from verb_command import command
from flux_msgs.msg import FluxFrame


def device_fill(span, value, nbytes, stream):
    """Fills `nbytes` of the device memory `span` names with `value`, on `stream`, without torch."""
    cuda = ctypes.CDLL("libcuda.so.1")
    ctx = ctypes.c_void_p()
    assert cuda.cuInit(0) == 0
    assert cuda.cuDevicePrimaryCtxRetain(ctypes.byref(ctx), 0) == 0
    assert cuda.cuCtxPushCurrent_v2(ctx) == 0
    ptr = span.__cuda_array_interface__["data"][0]
    rc = cuda.cuMemsetD8Async(
        ctypes.c_uint64(ptr),
        ctypes.c_ubyte(value),
        ctypes.c_size_t(nbytes),
        ctypes.c_void_p(stream),
    )
    cuda.cuCtxPopCurrent_v2(ctypes.byref(ctypes.c_void_p()))
    assert rc == 0, rc


def publish_device_image(pub, value, n):
    """An Image whose `n` bytes of data are in the GPU arena, all `value`."""
    from sensor_msgs_flux.image import Image

    loan = pub.loan(dtype="uint8")
    b = Image.Builder(loan.array, loan)
    b.height, b.width, b.encoding, b.step = 1, n, "mono8", n
    device_fill(b.alloc_device__data(n), value, n, loan.stream)
    assert b.commit__() == flux.Published.OK


NO_GPU = flux.probe().route == flux.Route.NONE


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
        command("record", *args, "-o", bag, "--poll", poll),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )


def stop(proc):
    proc.send_signal(signal.SIGINT)
    _, err = proc.communicate(timeout=30)
    assert proc.returncode == 0, err
    return err


def frames(bag):
    """(recv_timestamp, FluxFrame) of every frame in the bag, in order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions("cdr", "cdr"))
    out = []
    while reader.has_next():
        topic, data, recv = reader.read_next()
        if topic.endswith("/flux"):
            out.append((recv, deserialize_message(data, FluxFrame)))
    return out


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
    # --node-name: the services are this test's alone.
    name = f"rec{os.getpid()}"
    rec = record(bag, k, "--node-name", name)
    time.sleep(1.0)
    publish(pub, 0, 20)
    call(node, Pause, f"/{name}/pause")
    publish(pub, 20, 20)
    # The last paused frame is taken before the resume.
    time.sleep(0.3)
    call(node, Resume, f"/{name}/resume")
    publish(pub, 40, 20)
    err = stop(rec)
    got = values(bag)
    assert not [v for v in got if 20 <= v < 40], got
    assert len([v for v in got if v < 20]) > 10 and len([v for v in got if v >= 40]) > 10, got
    assert f"{k}: {len(got)} frames handed to rosbag2" in err, err


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
    # rosbag2's cache (100 MiB by default) holds the burst while the disk catches up.
    rec = record(bag, k)
    time.sleep(1.0)
    burst(pub, 100)
    time.sleep(0.5)
    err = stop(rec)
    assert values(bag) == list(range(100)), err
    assert f"{k}: 100 frames handed to rosbag2, 0 lost" in err, err


# mcap chunks compressed at zstd's slowest level: a writer of about 150 MB/s here.
SLOW_WRITER = 'compression: "Zstd"\ncompressionLevel: "Slow"\nchunkSize: 4194304\n'


def channel_line(err, k):
    return next(x for x in err.splitlines() if f"{k}: " in x)


def cache_lost(err, topic):
    """What rosbag2's cache reported it lost of `topic` when the bag closed."""
    m = re.search(rf"^\s*{re.escape(topic)}: (\d+)$", err, re.M)
    return int(m.group(1)) if m else 0


# rosbag2's cache is the one buffer between the frames and the writer, as for a ROS topic: with
# it off a frame waits in its slot until its channel overwrites it, and with it full rosbag2
# drops the frame. Each frame sent is in the bag or counted lost in one of the two.
@pytest.mark.parametrize("cache", [0, 4 * FRAME], ids=["no_cache", "small_cache"])
def test_frames_the_writer_cannot_take_are_lost_and_counted_once(node, tmp_path, cache):
    k = key(f"slow{cache}")
    bag = str(tmp_path / "bag")
    config = tmp_path / "slow.yaml"
    config.write_text(SLOW_WRITER)
    pub = flux.Publisher(node, k, slot_size=FRAME + 4096, slot_count=8)
    rec = record(bag, k, "--max-cache-size", cache, "-s", "mcap", "--storage-config-file", config)
    deadline = time.monotonic() + 10
    while pub.subscriber_count == 0:
        assert time.monotonic() < deadline, "the recorder did not subscribe within 10 s"
        time.sleep(0.05)
    # About 300 MB/s, twice what the writer takes; nibbles, so zstd works at it.
    frame = np.random.default_rng(1).integers(0, 16, FRAME, dtype=np.uint8)
    for i in range(60):
        frame[0] = i
        assert pub.publish(frame)
        time.sleep(0.02)
    time.sleep(0.5)
    err = stop(rec)
    got = values(bag)
    line = channel_line(err, k)
    handed = int(re.search(r"(\d+) frames", line).group(1))
    lost = int(re.search(r"(\d+) lost", line).group(1))
    dropped = cache_lost(err, k + "/flux")
    assert lost + dropped > 0, err
    assert handed == len(got) + dropped and handed + lost == 60, err
    if cache == 0:
        assert dropped == 0, err
    assert re.search(rf"{re.escape(k)}: {handed} frames handed to rosbag2, {lost} lost in", line)


def log_seconds(line):
    """The wall time rcutils stamped on a log line."""
    return float(re.search(r"\] \[(\d+\.\d+)\]", line).group(1))


# rosbag2 counts what its cache loses but tells only when the bag closes: the recorder tells while
# recording, the first loss of each topic at once and then at most every 5 s.
def test_what_rosbag2s_cache_loses_is_logged_while_recording(node, tmp_path):
    from std_msgs.msg import Int32

    k, topic = key("lossy"), key("lossy_ros")
    bag = str(tmp_path / "bag")
    config = tmp_path / "slow.yaml"
    config.write_text(SLOW_WRITER)
    pub = flux.Publisher(node, k, slot_size=FRAME + 4096, slot_count=8)
    ros_pub = node.create_publisher(Int32, topic, 100)
    rec = record(
        bag, k, topic, "--max-cache-size", 4 * FRAME, "-s", "mcap", "--storage-config-file", config
    )
    deadline = time.monotonic() + 10
    while pub.subscriber_count == 0 or ros_pub.get_subscription_count() == 0:
        assert time.monotonic() < deadline, "the recorder did not take both within 10 s"
        time.sleep(0.05)
    frame = np.random.default_rng(1).integers(0, 16, FRAME, dtype=np.uint8)
    for i in range(60):
        frame[0] = i
        assert pub.publish(frame)
        for j in range(10):
            ros_pub.publish(Int32(data=i * 10 + j))
            time.sleep(0.002)
    time.sleep(0.5)
    stopped = time.time()
    err = stop(rec)
    node.destroy_publisher(ros_pub)
    lines = [x for x in err.splitlines() if "rosbag2's cache is full" in x]
    assert lines and all(log_seconds(x) < stopped for x in lines), err
    told = " ".join(lines)
    assert cache_lost(err, k + "/flux") > 0 and cache_lost(err, topic) > 0, err
    assert f" on {k}/flux" in told and f" on {topic}" in told, err
    # Hundreds lost in under 5 s: one line for each topic's first loss.
    assert len(lines) <= 2, err


def test_frames_with_fields_in_the_arena_are_recorded_and_play_back_as_they_were(node, tmp_path):
    from sensor_msgs_flux.image import Image

    k = key("arena")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(node, k, fingerprint=Image.FINGERPRINT__, slot_size=4096, slot_count=4)
    rec = record(bag, k)
    time.sleep(1.0)
    sizes = (16, 8192, 16, 8192, 16)
    for i, n in enumerate(sizes):
        b = Image.build__(pub)
        b.height, b.width, b.encoding, b.step = 1, n, "mono8", n
        b.alloc__data(n)[:] = i
        assert b.commit__() == flux.Published.OK
        time.sleep(0.05)
    err = stop(rec)
    del pub
    gc.collect()
    assert "0 left out with fields in the arena" in err, err
    # The bag's copy holds the 8192 bytes the arena did.
    recorded = frames(bag)
    assert [len(bytes(f.data)) >= n for _, f in recorded] == [True] * len(sizes)

    sub = flux.Subscription(node, k, fingerprint=Image.FINGERPRINT__, qos=flux.QoS(depth=16))
    play = subprocess.Popen(
        command("play", bag), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    got = []
    while True:
        frame = sub.take_blocking(0.05)
        if frame is not None:
            img = Image.View(frame.copy())
            got.append((img.width, int(np.asarray(img.data)[-1])))
        elif play.poll() is not None:
            break
    assert play.returncode == 0
    assert got == [(n, i) for i, n in enumerate(sizes)]

    # convert_flux reads the copy as an Image: the ROS bag has every frame's bytes.
    from sensor_msgs.msg import Image as RosImage

    out = str(tmp_path / "ros")
    run = subprocess.run([program("convert"), bag, "-o", out], capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=out), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ros = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == k:
            m = deserialize_message(data, RosImage)
            ros.append((m.width, m.data[-1]))
    assert ros == [(n, i) for i, n in enumerate(sizes)]


@pytest.mark.skipif(NO_GPU, reason="needs a CUDA device")
def test_frames_with_fields_in_the_gpu_arena_are_recorded_and_play_back_as_they_were(
    node, tmp_path
):
    from sensor_msgs_flux.image import Image

    k = key("gpu_arena")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(
        node, k, fingerprint=Image.FINGERPRINT__, device="cuda", slot_size=4096, slot_count=4
    )
    rec = record(bag, k)
    time.sleep(1.0)
    for v in (5, 6, 7):
        publish_device_image(pub, v, 8192)
        time.sleep(0.05)
    err = stop(rec)
    del pub
    gc.collect()
    assert "0 left out with fields in the arena" in err, err
    assert len(frames(bag)) == 3

    sub = flux.Subscription(node, k, fingerprint=Image.FINGERPRINT__, qos=flux.QoS(depth=16))
    play = subprocess.Popen(
        command("play", bag), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    got = []
    while True:
        frame = sub.take_blocking(0.05)
        if frame is not None:
            img = Image.View(frame.copy())
            data = np.asarray(img.data)
            got.append((img.width, int(data[0]), int(data[-1])))
        elif play.poll() is not None:
            break
    assert play.returncode == 0
    assert got == [(8192, v, v) for v in (5, 6, 7)]


def test_no_discovery_records_the_channels_up_at_the_start_only(node, tmp_path):
    early, late = key("nodisc_early"), key("nodisc_late")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(node, early, slot_size=4096, slot_count=4)
    rec = record(bag, early, late, "--no-discovery")
    time.sleep(1.0)
    pub_late = flux.Publisher(node, late, slot_size=4096, slot_count=4)
    time.sleep(0.5)
    publish(pub, 0, 10)
    publish(pub_late, 0, 10)
    stop(rec)
    topics = {f.flux_topic for _, f in frames(bag)}
    assert topics == {early}


def test_sim_time_records_no_frame_before_the_first_clock(node, tmp_path):
    k = key("simtime")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(node, k, slot_size=4096, slot_count=4)
    rec = record(bag, k, "--use-sim-time")
    time.sleep(1.0)
    publish(pub, 0, 10)
    stop(rec)
    assert not os.path.exists(bag) or frames(bag) == []


def test_topic_types_select_flux_channels_by_their_ros_type(node, tmp_path):
    from sensor_msgs_flux.image import Image

    image, other = key("types_image"), key("types_other")
    bag = str(tmp_path / "bag")
    pub_image = flux.Publisher(node, image, fingerprint=Image.FINGERPRINT__, slot_size=4096)
    pub_other = flux.Publisher(node, other, slot_size=4096, slot_count=4)
    # Channels are found host-wide, so the Images of other processes are left out by name.
    own = f"^(?!/live{os.getpid()}/)"
    rec = record(bag, "--topic-types", "sensor_msgs/msg/Image", "--exclude-regex", own)
    time.sleep(1.0)
    publish(pub_other, 0, 5)
    for i in range(5):
        b = Image.build__(pub_image)
        b.height, b.width, b.encoding, b.step = 1, 4, "mono8", 4
        b.alloc__data(4)[:] = i
        assert b.commit__() == flux.Published.OK
        time.sleep(0.02)
    stop(rec)
    assert {f.flux_topic for _, f in frames(bag)} == {image}


def test_snapshot_mode_writes_the_frames_when_asked(node, tmp_path):
    from rosbag2_interfaces.srv import Snapshot

    k = key("snapshot")
    bag = str(tmp_path / "bag")
    name = f"snap{os.getpid()}"
    pub = flux.Publisher(node, k, slot_size=4096, slot_count=4)
    rec = record(bag, k, "--snapshot-mode", "--max-cache-size", 1 << 20, "--node-name", name)
    time.sleep(1.0)
    publish(pub, 0, 10)
    time.sleep(0.3)
    call(node, Snapshot, f"/{name}/snapshot")
    publish(pub, 10, 10)
    stop(rec)
    got = values(bag)
    assert got == list(range(got[0], 10)) and len(got) > 5, got


# --qos-profile-overrides-path: the recorder subscribes to the ROS topic with the profile given,
# and records the flux channel next to it.
def test_a_qos_override_sets_the_recorder_subscription_of_a_ros_topic(node, tmp_path):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Int32

    k, topic = key("qos_channel"), key("qos_topic")
    bag = str(tmp_path / "bag")
    overrides = tmp_path / "qos.yaml"
    overrides.write_text(
        f"{topic}:\n  reliability: best_effort\n  durability: transient_local\n"
        "  history: keep_last\n  depth: 3\n"
    )
    pub = flux.Publisher(node, k, slot_size=4096, slot_count=4)
    ros_pub = node.create_publisher(
        Int32, topic, QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    )
    rec = record(bag, k, topic, "--qos-profile-overrides-path", str(overrides))
    deadline = time.monotonic() + 10
    while ros_pub.get_subscription_count() == 0 or pub.subscriber_count == 0:
        assert time.monotonic() < deadline, "the recorder did not take both topics within 10 s"
        time.sleep(0.05)
    (sub,) = node.get_subscriptions_info_by_topic(topic)
    for i in range(5):
        ros_pub.publish(Int32(data=i))
        assert pub.publish(np.full(4, i, dtype=np.uint8))
        time.sleep(0.02)
    time.sleep(0.2)
    stop(rec)
    node.destroy_publisher(ros_pub)
    assert sub.node_name == "rosbag2_recorder"
    assert sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert sub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert values(bag) == list(range(5))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    count = 0
    while reader.has_next():
        reader.read_next()
        count += 1
    assert count == 5
