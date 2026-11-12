"""record then play: a flux channel goes into a bag and comes back with its fingerprint, dtype,
shape and bytes."""

import ctypes
import ctypes.util
import gc
import os
import re
import signal
import struct
import subprocess
import sys
import time

import numpy as np
import pytest
import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_prefix
from rclpy.serialization import deserialize_message, serialize_message

import flux
from flux import _flux
from flux_msgs.msg import FluxFrame

FINGERPRINT = 0xBA6
FRAME_TYPE = "flux_msgs/msg/FluxFrame"


def program(name):
    return os.path.join(get_package_prefix("flux_bag"), "lib", "flux_bag", name)


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = rclpy.create_node("flux_bag_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def key(name):
    return f"/bag{os.getpid()}/{name}"


def record(bag, *args):
    return subprocess.Popen(
        [program("record"), *args, "-o", bag, "--poll", "0.1"],
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


def play_into(sub, *args):
    """Every frame `play` puts on the channel `sub` is waiting on."""
    play = subprocess.Popen(
        [program("play"), *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    got = []
    while True:
        frame = sub.take_blocking(50_000_000)
        if frame is not None:
            got.append((frame.dtype, frame.shape, frame.copy()))
        elif play.poll() is not None:
            break
    assert play.returncode == 0
    return got


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


def write_bag(path, topic, frame_list, step_ns=100_000_000):
    """A bag of hand-made raw FluxFrames on `topic`: (slot_size, slot_count, dtype, shape, data)."""
    write_bag_channels(path, {topic: frame_list}, step_ns)


def write_bag_channels(path, channels, step_ns=100_000_000):
    """write_bag for several topics, {topic: frames}, the frames of each at the same times."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions("cdr", "cdr"))
    for n, (topic, frame_list) in enumerate(channels.items()):
        name = topic + "/flux"
        writer.create_topic(
            rosbag2_py.TopicMetadata(id=n, name=name, type=FRAME_TYPE, serialization_format="cdr")
        )
        # A sixth item is the aux tail after the raw meta.
        for i, (slot_size, slot_count, dtype, shape, data, *tail) in enumerate(frame_list):
            f = FluxFrame()
            f.flux_topic = topic
            f.fingerprint = FINGERPRINT
            f.codec = "raw"
            meta = struct.pack("<IIBB", slot_size, slot_count, dtype, len(shape))
            f.meta = meta + b"".join(struct.pack("<Q", d) for d in shape) + b"".join(tail)
            f.data = data
            writer.write(name, serialize_message(f), 1_000_000_000 + i * step_ns)
    del writer


def test_a_recorded_channel_plays_back_as_it_was(node, tmp_path):
    k = key("typed")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(node, k, fingerprint=FINGERPRINT, slot_size=4096, slot_count=4)
    rec = record(bag, k)
    for i in range(100):
        assert pub.publish(np.full((2, 3), i, dtype=np.float32))
        time.sleep(0.02)
    stop(rec)
    del pub
    gc.collect()
    recorded = [int(np.frombuffer(bytes(f.data), np.float32)[0]) for _, f in frames(bag)]
    assert len(recorded) > 50

    # Waiting before play starts, so the first frame is played into an attached subscription.
    sub = flux.Subscription(node, k, fingerprint=FINGERPRINT, qos=flux.QoS(depth=16))
    got = play_into(sub, bag)
    assert [int(a[0, 0]) for _, _, a in got] == recorded
    for dtype, shape, a in got:
        assert (dtype, shape) == (np.float32, (2, 3))
        assert (a == a[0, 0]).all()


# Each frame carries the geometry of the segment it came from; the player restarts the
# publisher with it rather than drop frames that do not fit the first one.
def test_slots_changed_mid_recording_play_back_whole(node, tmp_path):
    k = key("geometry")
    bag = str(tmp_path / "bag")
    rec = record(bag, k)
    for slot_size, n in ((4096, 1000), (1 << 20, 100_000)):
        pub = flux.Publisher(node, k, slot_size=slot_size, slot_count=4)
        time.sleep(0.5)
        for i in range(20):
            assert pub.publish(np.full(n, i, dtype=np.uint8))
            time.sleep(0.02)
        del pub
        gc.collect()
    stop(rec)
    sizes = [len(f.data) for _, f in frames(bag)]
    assert {1000, 100_000} <= set(sizes)

    sub = flux.Subscription(node, k, qos=flux.QoS(depth=16))
    got = [a.size for _, _, a in play_into(sub, bag)]
    # The restarted publisher's first frames go by before the subscriber re-attaches, as live.
    small = sizes.count(1000)
    assert got[:small] == sizes[:small] and got.count(100_000) >= 10


# A channel that refuses the recorder's subscription is left out until its publisher restarts;
# the bag goes on.
def test_a_channel_that_refuses_the_recorder_is_left_out_until_it_restarts(node, tmp_path):
    bag = str(tmp_path / "bag")
    full = flux.Publisher(node, key("full"), slot_size=4096, slot_count=4, max_subscribers=1)
    holder = hold_the_only_place(full)
    ok = flux.Publisher(node, key("ok"), slot_size=4096, slot_count=4)
    rec = record(bag, full.topic_name, ok.topic_name)
    time.sleep(0.5)
    for _ in range(10):
        assert ok.publish(np.zeros(4, dtype=np.uint8))
        assert full.publish(np.zeros(4, dtype=np.uint8))
        time.sleep(0.02)
    name = full.topic_name
    del full
    gc.collect()
    full = flux.Publisher(node, name, slot_size=4096, slot_count=4, max_subscribers=2)
    time.sleep(0.5)
    for _ in range(10):
        assert full.publish(np.ones(4, dtype=np.uint8))
        time.sleep(0.02)
    err = stop(rec)
    holder.stdin.close()
    holder.wait()
    assert err.count(f"flux channel {name} not recorded") == 1, err
    recorded = [(f.flux_topic, bytes(f.data)) for _, f in frames(bag)]
    assert {t for t, _ in recorded} == {ok.topic_name, name}
    assert all(d == bytes([1] * 4) for t, d in recorded if t == name)


def test_a_publisher_back_under_another_fingerprint_is_recorded(node, tmp_path):
    k = key("retyped")
    bag = str(tmp_path / "bag")
    rec = record(bag, k)
    for fp in (FINGERPRINT, FINGERPRINT + 1):
        pub = flux.Publisher(node, k, fingerprint=fp, slot_size=4096, slot_count=4)
        time.sleep(0.5)
        for _ in range(10):
            assert pub.publish(np.zeros(4, dtype=np.uint8))
            time.sleep(0.02)
        del pub
        gc.collect()
    stop(rec)
    assert {f.fingerprint for _, f in frames(bag)} == {FINGERPRINT, FINGERPRINT + 1}


def test_a_channel_gone_for_good_takes_its_recording_thread_with_it(node, tmp_path):
    rec = record(str(tmp_path / "bag"), "-e", f"^/bag{os.getpid()}/churn")

    # The reaper starts with the first subscription and stays for the process.
    def threads():
        tasks = f"/proc/{rec.pid}/task"
        return sum(
            open(f"{tasks}/{t}/comm").read().strip() != "flux-reaper" for t in os.listdir(tasks)
        )

    time.sleep(1.0)
    before = threads()
    for i in range(10):
        pub = flux.Publisher(node, key(f"churn{i}"), slot_size=4096, slot_count=4)
        time.sleep(0.3)
        del pub
        gc.collect()
    assert threads() > before
    time.sleep(11.0)  # past the recorder's 10 s hold on a channel that is down
    assert threads() <= before
    stop(rec)


def test_a_frame_is_stamped_when_it_was_published(node, tmp_path):
    k = key("stamped")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(node, k, slot_size=4096, slot_count=4)
    rec = record(bag, k)
    time.sleep(0.5)
    for _ in range(20):
        assert pub.publish(np.zeros(4, dtype=np.uint8))
        time.sleep(0.02)
    stop(rec)
    got = frames(bag)
    assert got
    for recv, f in got:
        sent = f.header.stamp.sec * 1_000_000_000 + f.header.stamp.nanosec
        assert 0 < recv - sent < 1_000_000_000


# A key from another process need not be a ROS name, and rclcpp throws on one that is not.
def test_a_key_that_is_not_a_ros_name_is_skipped_not_fatal(node, tmp_path):
    bad = f"/bag{os.getpid()} bad \x1b[31m"
    # signpost_name refuses such a key, so the peer names the object as the flattening would.
    signpost = _flux.signpost_name(re.sub(r"[^A-Za-z0-9_/]", "/", bad), FINGERPRINT)
    peer = _flux._Publisher(signpost, bad, "", FINGERPRINT, 4096, 4, flux.MemoryPolicy())
    good = flux.Publisher(node, key("good"), slot_size=4096, slot_count=4)
    rec = record(str(tmp_path / "bag"), "-a")
    time.sleep(1.0)
    assert rec.poll() is None
    err = stop(rec)
    assert "not a ROS topic name" in err and "\x1b" not in err
    assert f"recording flux channel {good.topic_name}" in err
    del peer


def test_a_rate_too_slow_for_int64_nanoseconds_plays_slowly(node, tmp_path):
    k = key("slow")
    bag = str(tmp_path / "bag")
    write_bag(bag, k, [(4096, 4, 0, [16], bytes(16))] * 3)
    sub = flux.Subscription(node, k, fingerprint=FINGERPRINT, qos=flux.QoS(depth=16))
    play = subprocess.Popen(
        [program("play"), bag, "-r", "1e-12"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    got = 0
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        got += sub.take_blocking(50_000_000) is not None
    play.send_signal(signal.SIGINT)
    assert play.wait(timeout=10) == 0
    assert got == 1


def test_waiting_for_the_next_frame_takes_no_cpu(node, tmp_path):
    bag = str(tmp_path / "bag")
    write_bag(bag, key("idle"), [(4096, 4, 0, [16], bytes(16))] * 2, step_ns=3_000_000_000)
    play = subprocess.Popen(
        [program("play"), bag], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(1.0)

    def cpu_ticks():
        with open(f"/proc/{play.pid}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
        return int(fields[11]) + int(fields[12])

    t0 = cpu_ticks()
    time.sleep(1.0)
    used = (cpu_ticks() - t0) / os.sysconf("SC_CLK_TCK")
    assert play.wait(timeout=10) == 0
    assert used < 0.2


def test_a_frame_whose_data_does_not_fill_its_shape_is_skipped(node, tmp_path):
    k = key("shape")
    bag = str(tmp_path / "bag")
    good, bad = (4096, 4, 0, [16], bytes(16)), (4096, 4, 0, [4000], bytes(16))
    write_bag(bag, k, [good, bad, good])
    sub = flux.Subscription(node, k, fingerprint=FINGERPRINT, qos=flux.QoS(depth=16))
    assert [a.size for _, _, a in play_into(sub, bag)] == [16, 16]


# A channel whose publisher cannot be made, at the start or on a later geometry, is skipped as
# rosbag2's player skips a topic; the other channels play on.
@pytest.mark.parametrize("bad_at", [0, 1])
def test_a_channel_that_cannot_be_recreated_is_skipped(node, tmp_path, bad_at):
    bag = str(tmp_path / "bag")
    good = (4096, 4, 0, [16], bytes(16))
    rows = [good] * 2
    rows[bad_at] = (0, 0, 0, [16], bytes(16))
    ok = key(f"ok{bad_at}")
    write_bag_channels(bag, {key(f"zero{bad_at}"): rows, ok: [good] * 3})
    sub = flux.Subscription(node, ok, fingerprint=FINGERPRINT, qos=flux.QoS(depth=16))
    assert len(play_into(sub, bag)) == 3
    play = subprocess.run([program("play"), bag], capture_output=True, text=True, timeout=30)
    assert play.returncode == 0, play.stderr
    assert "Ignoring a flux channel" in play.stderr + play.stdout


def convert(bag, out):
    run = subprocess.run([program("convert"), bag, "-o", out], capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    return run.stdout


def messages(bag):
    """{topic: (type, [(recv_timestamp, serialized)])} of a bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions("cdr", "cdr"))
    out = {t.name: (t.type, []) for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, recv = reader.read_next()
        out[topic][1].append((recv, data))
    return out


# A typed channel comes out as its ROS message on the channel's own topic, at the recorded times;
# a ROS topic of the same bag is copied as it is.
def test_a_typed_channel_converts_to_its_ros_message(node, tmp_path):
    from sensor_msgs.msg import Imu
    from sensor_msgs_flux.imu import Imu as FluxImu
    from std_msgs.msg import String

    k = key("imu")
    bag, out = str(tmp_path / "bag"), str(tmp_path / "ros")
    pub = flux.Publisher(node, k, fingerprint=FluxImu.FINGERPRINT__, slot_size=4096, slot_count=4)
    rec = record(bag, k, "/bag_chatter")
    chatter = node.create_publisher(String, "/bag_chatter", 10)
    time.sleep(0.5)
    for i in range(20):
        b = FluxImu.build__(pub)
        b.header__frame_id = "imu"
        b.orientation_w = float(i)
        b.linear_acceleration_z = 9.8
        assert b.commit__() == flux.Published.OK
        chatter.publish(String(data=f"hi {i}"))
        time.sleep(0.02)
    stop(rec)
    recorded = [recv for recv, _ in frames(bag)]

    convert(bag, out)
    got = messages(out)
    assert got[k][0] == "sensor_msgs/msg/Imu"
    assert [recv for recv, _ in got[k][1]] == recorded
    imus = [deserialize_message(data, Imu) for _, data in got[k][1]]
    assert [m.orientation.w for m in imus] == [float(i) for i in range(20)][-len(imus) :]
    assert all(m.header.frame_id == "imu" and m.linear_acceleration.z == 9.8 for m in imus)
    assert got["/bag_chatter"][0] == "std_msgs/msg/String"
    assert got["/bag_chatter"][1]
    assert not any(t.endswith("/flux") for t in got)


# A frame whose type is not installed has no ROS form: it stays a FluxFrame, under its old topic.
def test_a_channel_of_no_installed_type_stays_a_flux_frame(tmp_path):
    bag, out = str(tmp_path / "bag"), str(tmp_path / "ros")
    write_bag(bag, "/raw", [(4096, 4, 1, (4,), bytes(4))] * 3)
    assert "3 kept as FluxFrame" in convert(bag, out)
    got = messages(out)
    assert got["/raw/flux"][0] == FRAME_TYPE
    assert len(got["/raw/flux"][1]) == 3


def picture(i, height, width, channels):
    """A smooth picture that moves with i, which JPEG keeps close."""
    y, x = np.mgrid[0:height, 0:width]
    planes = [128 + 60 * np.sin((x + i * 3) / 9 + c) * np.cos(y / 7) for c in range(channels)]
    return np.stack(planes, axis=-1).astype(np.uint8).reshape(height, width * channels)


def publish_images(pub, encoding, channels, count, height=48, width=64, pad=8):
    """Images with `pad` bytes past each row, as some drivers send them."""
    from sensor_msgs_flux.image import Image

    sent = []
    for i in range(count):
        rows = np.zeros((height, width * channels + pad), np.uint8)
        rows[:, : width * channels] = picture(i, height, width, channels)
        b = Image.build__(pub)
        b.set__header__stamp(100 + i, 7)
        b.header__frame_id = "cam"
        b.height, b.width, b.encoding = height, width, encoding
        b.step = rows.shape[1]
        b.alloc__data(rows.size)[:] = rows.reshape(-1)
        assert b.commit__() == flux.Published.OK
        sent.append(rows)
        time.sleep(0.02)
    return sent


# A 4096-byte slot sends the pixels to the publisher's arena: the recorder copies them into the
# frame before it compresses.
@pytest.mark.parametrize("slot", [1 << 16, 4096])
@pytest.mark.parametrize("encoding,channels", [("rgb8", 3), ("bgr8", 3), ("mono8", 1)])
def test_an_image_recorded_as_jpeg_plays_back_close_to_what_was_sent(
    node, tmp_path, encoding, channels, slot
):
    from sensor_msgs_flux.image import Image

    k = key(f"jpeg_{encoding}_{slot}")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(node, k, fingerprint=Image.FINGERPRINT__, slot_size=slot, slot_count=4)
    rec = record(bag, k, "--image-codec", "jpeg", "--jpeg-quality", "95")
    time.sleep(0.5)
    sent = publish_images(pub, encoding, channels, 20)
    stop(rec)
    del pub
    gc.collect()
    recorded = frames(bag)
    assert recorded and all(f.codec == "jpeg" for _, f in recorded)
    assert all(len(f.data) < sent[0].size / 2 for _, f in recorded)

    sub = flux.Subscription(node, k, fingerprint=Image.FINGERPRINT__, qos=flux.QoS(depth=16))
    got = play_into(sub, bag)
    assert len(got) == len(recorded)
    first = len(sent) - len(got)
    for n, (dtype, shape, data) in enumerate(got):
        img = Image.View(data)
        assert (img.encoding, img.width, img.height, img.step) == (
            encoding,
            64,
            48,
            64 * channels + 8,
        )
        assert (img.header__stamp, img.header__frame_id) == ((100 + first + n, 7), "cam")
        rows = np.asarray(img.data).reshape(48, -1).astype(int)
        want = sent[first + n].astype(int)
        assert np.abs(rows[:, : 64 * channels] - want[:, : 64 * channels]).mean() < 3
        assert not rows[:, 64 * channels :].any()


def test_an_image_jpeg_cannot_hold_is_recorded_as_it_is(node, tmp_path):
    from sensor_msgs_flux.image import Image

    k = key("jpeg_mono16")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(node, k, fingerprint=Image.FINGERPRINT__, slot_size=1 << 16, slot_count=4)
    rec = record(bag, k, "--image-codec", "jpeg")
    time.sleep(0.5)
    publish_images(pub, "mono16", 2, 10)
    stop(rec)
    recorded = frames(bag)
    assert recorded and all(f.codec == "raw" for _, f in recorded)


def test_a_jpeg_image_converts_to_a_compressed_image(node, tmp_path):
    from sensor_msgs.msg import CompressedImage
    from sensor_msgs_flux.image import Image

    k = key("jpeg_convert")
    bag, out = str(tmp_path / "bag"), str(tmp_path / "ros")
    pub = flux.Publisher(node, k, fingerprint=Image.FINGERPRINT__, slot_size=1 << 16, slot_count=4)
    rec = record(bag, k, "--image-codec", "jpeg")
    time.sleep(0.5)
    publish_images(pub, "rgb8", 3, 10)
    stop(rec)
    recorded = frames(bag)

    convert(bag, out)
    got = messages(out)
    assert got[k + "/compressed"][0] == "sensor_msgs/msg/CompressedImage"
    msgs = [deserialize_message(data, CompressedImage) for _, data in got[k + "/compressed"][1]]
    assert len(msgs) == len(recorded)
    for m, (_, f) in zip(msgs, recorded):
        assert m.format == "rgb8; jpeg compressed bgr8"
        assert m.header.frame_id == "cam"
        assert bytes(m.data[:2]) == b"\xff\xd8"
        assert bytes(m.data) == bytes(f.data)[-len(m.data) :]
    assert not any(t.endswith("/flux") for t in got)


def test_a_bag_recorded_with_zstd_reads_back_whole_and_smaller(node, tmp_path):
    k = key("zstd")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(node, k, slot_size=1 << 20, slot_count=4)
    rec = record(bag, k, "-s", "mcap", "--storage-preset-profile", "zstd_fast")
    time.sleep(0.5)
    for i in range(20):
        assert pub.publish(np.full(1 << 19, i, dtype=np.uint8))
        time.sleep(0.02)
    stop(rec)
    recorded = frames(bag)
    assert len(recorded) > 10
    assert [bytes(f.data) for _, f in recorded] == [
        bytes(np.full(1 << 19, int(f.data[0]), np.uint8)) for _, f in recorded
    ]
    size = sum(os.path.getsize(os.path.join(bag, n)) for n in os.listdir(bag))
    assert size < len(recorded) * (1 << 19) / 10


def aux_tail(aux_size, aux, version=1, aux_len=None):
    """The meta tail of a frame of a channel made with aux_size."""
    return struct.pack("<III", version, aux_size, len(aux) if aux_len is None else aux_len) + aux


def split_meta(meta):
    """(aux_size, aux) of a raw FluxFrame's meta: (0, b"") without a tail."""
    meta = bytes(meta)
    end = 10 + 8 * meta[9]
    if len(meta) == end:
        return 0, b""
    version, aux_size, aux_len = struct.unpack_from("<III", meta, end)
    assert version == 1 and len(meta) == end + 12 + aux_len
    return aux_size, meta[end + 12 :]


def play_taking(takers, *args):
    """Runs play on the bag until it exits and no taker gets a frame. A taker takes one frame and
    says whether it got one. play's output."""
    play = subprocess.Popen(
        [program("play"), *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    while True:
        got = [take() for take in takers]
        if not any(got) and play.poll() is not None:
            break
    out = play.stdout.read()
    assert play.returncode == 0, out
    return out


def taker(sub, into):
    def take():
        frame = sub.take_blocking(50_000_000)
        if frame is None:
            return False
        into.append((frame.copy(), bytes(flux.aux(frame))))
        return True

    return take


# The aux bytes of each frame and the channel's aux_size go into the bag and come back.
def test_aux_bytes_are_recorded_and_played_back(node, tmp_path):
    k = key("aux")
    bag = str(tmp_path / "bag")
    pub = flux.Publisher(
        node, k, fingerprint=FINGERPRINT, slot_size=4096, slot_count=4, aux_size=32
    )
    rec = record(bag, k)
    time.sleep(0.5)
    sent = []
    for i in range(30):
        aux = bytes([i]) * (i % 33)
        loan = pub.loan((4,), "u1")
        loan.array[:] = i
        loan.aux[: len(aux)] = np.frombuffer(aux, np.uint8)
        loan.aux_len = len(aux)
        assert loan.commit() == flux.Published.OK
        sent.append((bytes([i]) * 4, aux))
        time.sleep(0.02)
    stop(rec)
    del pub
    gc.collect()
    recorded = [(bytes(f.data), split_meta(f.meta)) for _, f in frames(bag)]
    assert len(recorded) > 20
    assert all(aux_size == 32 for _, (aux_size, _) in recorded)
    first = len(sent) - len(recorded)
    assert [(d, aux) for d, (_, aux) in recorded] == sent[first:]

    sub = flux.Subscription(node, k, fingerprint=FINGERPRINT, qos=flux.QoS(depth=16))
    got = []
    out = play_taking([taker(sub, got)], bag)
    assert "32 aux bytes" in out
    assert [(a.tobytes(), aux) for a, aux in got] == sent[first:]


# A bag recorded before the tail has meta of the codec's size only: its frames carry no aux.
def test_a_frame_without_a_tail_plays_with_no_aux(node, tmp_path):
    k = key("notail")
    bag = str(tmp_path / "bag")
    write_bag(bag, k, [(4096, 4, 0, [4], bytes(4))] * 3)
    sub = flux.Subscription(node, k, fingerprint=FINGERPRINT, qos=flux.QoS(depth=16))
    got = []
    out = play_taking([taker(sub, got)], bag)
    assert "0 aux bytes" in out
    assert [aux for _, aux in got] == [b""] * 3


# A tail of the wrong version or length is refused as bad meta is: play skips the frame, convert
# stops with exit code 1. A good one is kept as it is by convert when the type is not installed.
def test_a_frame_with_a_bad_aux_tail_is_refused(node, tmp_path):
    k = key("badtail")
    bag, out = str(tmp_path / "bag"), str(tmp_path / "ros")
    row = (4096, 4, 0, [4], bytes(4))
    bad = [
        aux_tail(8, b"xy", version=2),
        aux_tail(8, b"xy", aux_len=3),
        aux_tail(8, b"x" * 9),
        aux_tail(0, b""),
        aux_tail(8, b"xy")[:11],
    ]
    write_bag(
        bag, k, [(*row, aux_tail(8, b"a"))] + [(*row, t) for t in bad] + [(*row, aux_tail(8, b"b"))]
    )
    sub = flux.Subscription(node, k, fingerprint=FINGERPRINT, qos=flux.QoS(depth=16))
    got = []
    play_taking([taker(sub, got)], bag)
    assert [aux for _, aux in got] == [b"a", b"b"]
    run = subprocess.run([program("convert"), bag, "-o", out], capture_output=True, text=True)
    assert run.returncode == 1 and "does not parse" in run.stderr

    good = str(tmp_path / "good")
    write_bag(good, k, [(*row, aux_tail(8, b"a"))])
    convert(good, str(tmp_path / "good_ros"))
    ((_, kept),) = messages(str(tmp_path / "good_ros"))[k + "/flux"][1]
    assert split_meta(deserialize_message(kept, FluxFrame).meta) == (8, b"a")


GPU_IMAGE = 0x780CD7C35A0C29C8  # flux_wrapper's Device::Cuda Image channel
GPU_IMAGE_AUX = 316


def load_cudart():
    name = ctypes.util.find_library("cudart")
    try:
        return ctypes.CDLL(name or "libcudart.so")
    except OSError:
        return None


CUDART = load_cudart()


def gpu_image_aux(i):
    """flux_wrapper's GpuImageAux of a 64 x 64 mono8 frame."""
    encoding, frame_id = b"mono8", b"cam%d" % i
    return struct.pack(
        "<IiIIIIBBH32s256s",
        1,
        100 + i,
        7,
        64,
        64,
        64,
        0,
        len(encoding),
        len(frame_id),
        encoding,
        frame_id,
    )


def cuda_copy(dst, src, n, kind):
    assert (
        CUDART.cudaMemcpy(ctypes.c_void_p(dst), ctypes.c_void_p(src), ctypes.c_size_t(n), kind) == 0
    )


# The channel of a flux_wrapper Image publisher with Device::Cuda: the recorder's CPU subscription
# gets a host copy of each device frame; play publishes into host memory, which a CPU and a CUDA
# subscription both take, the aux header with each frame.
@pytest.mark.skipif(
    CUDART is None or flux.probe().route == flux.Route.NONE, reason="needs CUDA and a GPU route"
)
def test_a_gpu_image_channel_is_recorded_and_played_to_cpu_and_cuda(node, tmp_path):
    k = key("gpu_image")
    bag = str(tmp_path / "bag")
    n = 64 * 64
    pub = flux.Publisher(
        node,
        k,
        fingerprint=GPU_IMAGE,
        slot_size=1 << 16,
        slot_count=4,
        device="cuda",
        aux_size=GPU_IMAGE_AUX,
    )
    # On a discrete GPU the slots are device memory: the recorder reads its own host copy.
    assert pub.device_backed or flux.probe().integrated
    rec = record(bag, k)
    time.sleep(0.5)
    sent = []
    for i in range(20):
        pixels = ((np.arange(n) + i) % 251).astype(np.uint8)
        aux = gpu_image_aux(i)
        loan = pub.loan((n,), "u1")
        cuda_copy(loan.__cuda_array_interface__["data"][0], pixels.ctypes.data, n, 1)
        assert CUDART.cudaDeviceSynchronize() == 0
        loan.aux[:] = np.frombuffer(aux, np.uint8)
        loan.aux_len = len(aux)
        assert loan.commit() == flux.Published.OK
        sent.append((pixels.tobytes(), aux))
        time.sleep(0.02)
    stop(rec)
    del pub
    gc.collect()
    recorded = [(f.fingerprint, bytes(f.data), split_meta(f.meta)) for _, f in frames(bag)]
    assert len(recorded) > 10
    first = len(sent) - len(recorded)
    assert [(d, aux) for _, d, (_, aux) in recorded] == sent[first:]
    assert {(fp, aux_size) for fp, _, (aux_size, _) in recorded} == {(GPU_IMAGE, GPU_IMAGE_AUX)}

    cpu = flux.Subscription(node, k, fingerprint=GPU_IMAGE, qos=flux.QoS(depth=16))
    gpu = flux.Subscription(node, k, fingerprint=GPU_IMAGE, qos=flux.QoS(depth=16), device="cuda")
    on_cpu, on_gpu = [], []

    def take_gpu():
        frame = gpu.take_blocking(50_000_000)
        if frame is None:
            return False
        with frame:
            ptr = frame.__cuda_array_interface__["data"][0]
            assert ptr != 0
            out = np.empty(frame.nbytes, np.uint8)
            assert CUDART.cudaStreamSynchronize(ctypes.c_void_p(frame.stream)) == 0
            cuda_copy(out.ctypes.data, ptr, out.nbytes, 2)
            on_gpu.append((out.tobytes(), bytes(flux.aux(frame))))
        return True

    play_taking([taker(cpu, on_cpu), take_gpu], bag)
    assert [(a.tobytes(), aux) for a, aux in on_cpu] == sent[first:]
    assert on_gpu == sent[first:]
