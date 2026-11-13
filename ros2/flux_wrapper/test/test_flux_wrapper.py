import ctypes
import glob
import os
import struct
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

import flux_wrapper

EXPECT_FLUX = os.environ.get("FLUX_WRAPPER_EXPECT_FLUX") == "1"


def _cudart():
    for name in ["libcudart.so", *sorted(glob.glob("/usr/local/cuda*/lib64/libcudart.so*"))]:
        try:
            return ctypes.CDLL(name)
        except OSError:
            pass
    return None


CUDART = _cudart()


def _no_cuda():
    """Empty when device="cuda" can be had here, else why not."""
    if not EXPECT_FLUX:
        return "the Python ROS path has no device memory"
    import flux

    p = flux.probe()
    if p.route == flux.Route.NONE:
        return f"no flux GPU route: {p.reason}"
    if CUDART is None:
        return "no libcudart to reach device memory with"
    return ""


NO_CUDA = _no_cuda()
needs_cuda = pytest.mark.skipif(NO_CUDA != "", reason=NO_CUDA)


def to_device(obj, data):
    """Writes bytes to what obj's __cuda_array_interface__ points at."""
    ptr = obj.__cuda_array_interface__["data"][0]
    assert CUDART.cudaMemcpy(ctypes.c_void_p(ptr), data, ctypes.c_size_t(len(data)), 1) == 0


def from_device(frame):
    """The bytes of a flux.Frame on the device, once its stream is done."""
    assert CUDART.cudaStreamSynchronize(ctypes.c_void_p(frame.stream)) == 0
    out = ctypes.create_string_buffer(frame.nbytes)
    ptr = frame.__cuda_array_interface__["data"][0]
    assert CUDART.cudaMemcpy(out, ctypes.c_void_p(ptr), ctypes.c_size_t(frame.nbytes), 2) == 0
    return out.raw


@pytest.fixture
def node():
    context = rclpy.Context()
    rclpy.init(context=context)
    n = Node("wrapper", namespace=f"/flux_wrapper_py_{os.getpid()}", context=context)
    yield n
    n.destroy_node()
    rclpy.try_shutdown(context=context)


def run_until(node, sub, done, publish_once, ex=None):
    """Spins `node` while `publish_once` runs every 10 ms, until `done` is set or 10 s."""
    ex = ex or flux_wrapper.Executor()
    if sub is not None:
        ex.add(sub)
    ex.add_node(node)
    spinner = threading.Thread(target=ex.spin)
    spinner.start()
    deadline = time.monotonic() + 10
    while not done.is_set() and time.monotonic() < deadline:
        publish_once()
        time.sleep(0.01)
    ex.stop()
    spinner.join()


def header(sec, frame):
    h = Header(frame_id=frame)
    h.stamp.sec, h.stamp.nanosec = sec, 7
    return h


def test_run_switch_picks_the_path(node):
    assert flux_wrapper.use_flux() == EXPECT_FLUX
    pub = flux_wrapper.Publisher(node, Image, "switch", 1)
    assert pub.uses_flux == EXPECT_FLUX


def test_image_through_a_loan(node):
    got, done = {}, threading.Event()

    def on_image(v):
        got.update(
            sec=v.header.stamp.sec,
            frame=v.header.frame_id,
            encoding=v.encoding,
            size=(v.height, v.width, v.step),
            data=bytes(v.data),
        )
        done.set()

    sub = flux_wrapper.Subscription(node, Image, "image", on_image, 4)
    assert sub.uses_flux == EXPECT_FLUX
    pub = flux_wrapper.Publisher(node, Image, "image", 4)

    def publish_once():
        loan = pub.loan()
        assert loan
        loan.header = header(42, "camera")
        loan.height, loan.width, loan.step = 2, 3, 3
        loan.encoding = "mono8"
        loan.alloc_data(6)[:] = np.arange(1, 7, dtype=np.uint8)
        assert pub.publish(loan)

    run_until(node, sub, done, publish_once)
    assert got == {
        "sec": 42,
        "frame": "camera",
        "encoding": "mono8",
        "size": (2, 3, 3),
        "data": bytes([1, 2, 3, 4, 5, 6]),
    }


def test_point_cloud_from_a_ros_message(node):
    got, done = [], threading.Event()

    def on_cloud(v):
        assert [f.name for f in v.fields] == ["x", "y"]
        got.append(v.to_msg())
        done.set()

    sub = flux_wrapper.Subscription(node, PointCloud2, "points", on_cloud, 4)
    pub = flux_wrapper.Publisher(node, PointCloud2, "points", 4)
    cloud = PointCloud2(header=header(5, "lidar"), height=1, width=2, point_step=8, row_step=16)
    cloud.fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate("xy")
    ]
    cloud.data = bytes([9] * 16)
    cloud.is_dense = True

    run_until(node, sub, done, lambda: pub.publish(cloud))
    assert got and got[0] == cloud


def image(sec):
    m = Image(header=header(sec, "camera"), height=2, width=3, encoding="mono8", step=3)
    m.is_bigendian = 1
    m.data = bytes(range(sec, sec + 6))
    return m


def cloud(sec, n=3, name=lambda i: f"f{i}"):
    c = PointCloud2(header=header(sec, "lidar"), height=1, width=2, point_step=4 * n)
    c.fields = [
        PointField(name=name(i), offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i in range(n)
    ]
    c.row_step = 2 * c.point_step
    c.data = bytes((sec * 16 + i) % 256 for i in range(c.row_step))
    c.is_bigendian, c.is_dense = True, True
    return c


def fields(v, msg_type):
    """What the accessors of a View read, as a message without data."""
    m = msg_type(header=v.header)
    for name in flux_wrapper._SCALARS[msg_type]:
        setattr(m, name, getattr(v, name))
    if msg_type is PointCloud2:
        m.fields = v.fields
    return m


def without_data(m):
    out = type(m)()
    for name in out.get_fields_and_field_types():
        if name != "data":
            setattr(out, name, getattr(m, name))
    return out


def test_gpu_aux_is_a_frame_of_the_type_without_data():
    """The fingerprint is flux.gpu_fingerprint of the type's; the aux a frame of the type."""
    import flux
    from sensor_msgs_flux import image, point_cloud2

    assert flux_wrapper._gpu_fingerprint(Image) == flux.gpu_fingerprint(image.FINGERPRINT__)
    assert flux_wrapper._gpu_fingerprint(PointCloud2) == flux.gpu_fingerprint(
        point_cloud2.FINGERPRINT__
    )
    aux = flux_wrapper._pack(Image, Image(header=header(1, "ok"), encoding="rgb8"))
    assert image.View(aux).encoding == "rgb8"
    m = cloud(3)
    assert flux_wrapper._unpack(PointCloud2, flux_wrapper._pack(PointCloud2, m)) == without_data(m)


def gpu_round_trip(node, msg_type, topic, want, write):
    """A device="cuda" publisher's message, written by write(loan) or, with write None, published
    as the ROS message itself, read by a "cuda" and a "cpu" subscription: what each got."""
    got, done = {}, threading.Event()

    def on_device(v):
        if v.header.stamp.sec == want.header.stamp.sec:
            got["device"] = (v.on_device, v.data, from_device(v.device_data), fields(v, msg_type))
            got["device_msg"] = v.to_msg()
            if "host" in got:
                done.set()

    def on_host(v):
        if v.header.stamp.sec == want.header.stamp.sec:
            got["host"] = (v.on_device, bytes(v.data), v.to_msg())
            if "device" in got:
                done.set()

    device_sub = flux_wrapper.Subscription(node, msg_type, topic, on_device, 4, device="cuda")
    host_sub = flux_wrapper.Subscription(node, msg_type, topic, on_host, 4)
    pub = flux_wrapper.Publisher(node, msg_type, topic, 4, device="cuda")
    assert pub.uses_device

    def publish_once():
        if write is None:
            assert pub.publish(want)
            return
        loan = pub.loan()
        loan.header = want.header
        for name in flux_wrapper._SCALARS[msg_type]:
            setattr(loan, name, getattr(want, name))
        if msg_type is PointCloud2:
            loan.fields = want.fields
        assert loan
        write(loan)
        assert pub.publish(loan)

    ex = flux_wrapper.Executor()
    ex.add(device_sub)
    run_until(node, host_sub, done, publish_once, ex)
    return pub, got


def write_on_device(want):
    def write(loan):
        buf = loan.alloc_buffer(len(want.data))
        assert buf is not None
        to_device(buf, bytes(want.data))

    return write


@needs_cuda
@pytest.mark.parametrize("msg_type", [Image, PointCloud2])
def test_cuda_publisher_reaches_cuda_and_cpu_subscriptions(node, msg_type):
    want = image(1) if msg_type is Image else cloud(1)
    _, got = gpu_round_trip(node, msg_type, f"gpu_{msg_type.__name__}", want, write_on_device(want))
    on_device, data, device_bytes, got_fields = got["device"]
    assert on_device and data is None
    assert device_bytes == bytes(want.data)
    assert got_fields == without_data(want)
    assert got["device_msg"] == want
    assert got["host"] == (False, bytes(want.data), want)


def from_view(v):
    """The bytes of a View's device_data, once its stream is done."""
    assert CUDART.cudaStreamSynchronize(ctypes.c_void_p(v.stream)) == 0
    d = v.device_data
    out = ctypes.create_string_buffer(d.nbytes)
    ptr = d.__cuda_array_interface__["data"][0]
    assert CUDART.cudaMemcpy(out, ctypes.c_void_p(ptr), ctypes.c_size_t(d.nbytes), 2) == 0
    return out.raw


@needs_cuda
@pytest.mark.skipif(not EXPECT_FLUX, reason="flux path only")
def test_cuda_subscription_gets_a_cpu_publishers_data_on_the_device(node):
    """In the slot or past it in the publisher's arena, the data comes on the device, and the host
    fields with it."""
    small, big = image(3), image(4)
    big.data = bytes(i * 7 % 256 for i in range(64 * 1024))
    got, done = {}, threading.Event()

    def on_image(v):
        sec = v.header.stamp.sec
        if sec in (3, 4) and sec not in got:
            got[sec] = (v.on_device, v.stream is not None, from_view(v), bytes(v.data), v.to_msg())
            if len(got) == 2:
                done.set()

    sub = flux_wrapper.Subscription(node, Image, "cpu_to_cuda", on_image, 4, device="cuda")
    pub = flux_wrapper.Publisher(node, Image, "cpu_to_cuda", 4)
    run_until(node, sub, done, lambda: pub.publish(small) and pub.publish(big))
    for m in (small, big):
        assert got[m.header.stamp.sec] == (True, True, bytes(m.data), bytes(m.data), m)


@needs_cuda
@pytest.mark.parametrize("msg_type", [Image, PointCloud2])
def test_cuda_publish_of_a_ros_message(node, msg_type):
    """The message's data is copied into the slot, in device memory or host memory alike."""
    want = image(2) if msg_type is Image else cloud(2)
    _, got = gpu_round_trip(node, msg_type, f"gpu_message_{msg_type.__name__}", want, None)
    on_device, data, device_bytes, got_fields = got["device"]
    assert on_device and data is None
    assert device_bytes == bytes(want.data)
    assert got_fields == without_data(want)
    assert got["device_msg"] == want
    assert got["host"] == (False, bytes(want.data), want)


@needs_cuda
def test_cuda_fields_over_the_aux_fail_the_loan(node):
    pub = flux_wrapper.Publisher(node, Image, "capped", 4, device="cuda")
    loan = pub.loan()
    loan.header = header(1, "f" * 5000)
    assert not loan
    assert not pub.publish(loan)

    # Past the old caps (32 fields, 32-byte names, a 256-byte frame_id) a cloud goes out whole.
    want = cloud(1, 40, name=lambda i: "n" * 40 + str(10 + i))
    want.header.frame_id = "f" * 300
    _, got = gpu_round_trip(node, PointCloud2, "past_caps", want, write_on_device(want))
    assert got["host"] == (False, bytes(want.data), want)


class _Warnings:
    """A logger that keeps what was warned."""

    def __init__(self, logger):
        self._logger = logger
        self.lines = []

    def warning(self, text):
        self.lines.append(text)

    def __getattr__(self, name):
        return getattr(self._logger, name)


class _Errors(_Warnings):
    """A logger that keeps what was warned and what was logged as an error."""

    def error(self, text):
        self.lines.append(text)


@needs_cuda
def test_a_header_past_the_aux_is_logged_once_with_its_field(node, monkeypatch):
    logs = _Errors(node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logs)
    pub = flux_wrapper.Publisher(node, Image, "over_logged", 4, device="cuda")
    for _ in range(3):
        loan = pub.loan()
        loan.header = header(5, "f" * 5000)
        assert not pub.publish(loan)
    clouds = flux_wrapper.Publisher(node, PointCloud2, "over_logged_cloud", 4, device="cuda")
    loan = clouds.loan()
    loan.fields = cloud(1, 200, name=lambda i: "n" * 40 + str(i)).fields
    assert not clouds.publish(loan)
    want = (
        'over_logged: a message did not go out on device="cuda": header.frame_id (5000 B) did '
        "not fit the aux (4096 B, 48 B used before it); header.stamp 5.000000007"
    )
    assert sum(want in line for line in logs.lines) == 1, logs.lines
    want = (
        'over_logged_cloud: a message did not go out on device="cuda": fields (200 entries, '
        "names 8490 B)"
    )
    assert sum(want in line for line in logs.lines) == 1, logs.lines


@needs_cuda
def test_cuda_data_past_the_slot_is_logged_as_too_large(node, monkeypatch):
    """Once, and not as fields over the aux."""
    logs = _Errors(node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logs)
    pub = flux_wrapper.Publisher(
        node, Image, "data_over_logged", 4, device="cuda", max_device_data_size=4096
    )
    for _ in range(2):
        loan = pub.loan()
        assert loan.alloc_buffer(8192) is None
        assert not pub.publish(loan)
    assert sum("data_over_logged failed" in line for line in logs.lines) == 1, logs.lines
    assert not any("over the aux" in line for line in logs.lines), logs.lines


@pytest.mark.skipif(not EXPECT_FLUX, reason="flux path only")
def test_a_string_past_the_slot_is_logged_once_with_its_field(node, monkeypatch):
    """On "cpu" the message's own strings go after its fixed fields in the slot. One past the slot
    fails the message, set by hand or copied from a ROS message, and is logged once."""
    logs = _Errors(node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logs)
    pub = flux_wrapper.Publisher(node, Image, "slot_over_logged", 4)
    for _ in range(2):
        loan = pub.loan()
        loan.header = header(5, "f" * 9000)
        assert not loan
        assert not pub.publish(loan)
    m = image(5)
    m.header = header(5, "f" * 9000)
    assert not pub.publish(m)
    want = (
        "slot_over_logged: a message did not go out: header.frame_id (9000 B) did not fit the slot "
        "(8192 B, 48 B used before it); header.stamp 5.000000007"
    )
    assert sum(want in line for line in logs.lines) == 1, logs.lines
    m.header = header(5, "f" * 4096)
    assert pub.publish(m)


@pytest.mark.skipif(NO_CUDA == "", reason="device=cuda is had here")
def test_every_cuda_fallback_is_logged_with_its_topic(node, monkeypatch):
    logs = _Warnings(node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logs)
    flux_wrapper.Publisher(node, Image, "fallback_first", 1, device="cuda")
    flux_wrapper.Publisher(node, Image, "fallback_second", 1, device="cuda")
    flux_wrapper.Subscription(node, Image, "fallback_sub", lambda v: None, 1, device="cuda")
    for topic in ("fallback_first", "fallback_second", "fallback_sub"):
        assert sum(topic in line for line in logs.lines) == 1, logs.lines


@pytest.mark.skipif(NO_CUDA == "", reason="device=cuda is had here")
def test_cuda_that_cannot_be_had_is_cpu(node):
    got, done = [], threading.Event()

    def on_cloud(v):
        got.append((v.on_device, v.to_msg()))
        done.set()

    sub = flux_wrapper.Subscription(node, PointCloud2, "fallback", on_cloud, 4, device="cuda")
    pub = flux_wrapper.Publisher(node, PointCloud2, "fallback", 4, device="cuda")
    assert not pub.uses_device
    want = cloud(1)

    def publish_once():
        loan = pub.loan()
        loan.header, loan.fields = want.header, want.fields
        for name in flux_wrapper._SCALARS[PointCloud2]:
            setattr(loan, name, getattr(want, name))
        loan.alloc_buffer(len(want.data))[:] = np.frombuffer(want.data, np.uint8)
        assert pub.publish(loan)

    run_until(node, sub, done, publish_once)
    assert got and got[0] == (False, want)


@pytest.mark.skipif(not EXPECT_FLUX, reason="flux path only")
def test_gpu_frame_with_a_bad_aux_is_skipped(node):
    import flux

    delivered = []
    sub = flux_wrapper.Subscription(
        node, Image, "bad_aux", lambda v: delivered.append(v.header.frame_id), 4
    )
    fingerprint, size = flux_wrapper._gpu_fingerprint(Image), flux_wrapper._GPU_AUX_BYTES
    raw = flux.Publisher(node, "bad_aux", fingerprint=fingerprint, slot_size=4096, aux_size=size)
    good = bytearray(flux_wrapper._pack(Image, Image(header=header(1, "ok"))))
    # frame_id's descriptor (at 8) pointed past the aux.
    far = bytearray(good)
    struct.pack_into("=I", far, 8, 1 << 20)

    def send(aux):
        loan = raw.loan((0,))
        if loan is not None:
            loan.aux[: len(aux)] = np.frombuffer(bytes(aux), np.uint8)
            loan.aux_len = len(aux)
            loan.commit()

    for aux in (good[:10], far):
        before, done = sub.rejected, threading.Event()

        def publish_once(aux=aux, before=before, done=done):
            send(aux)
            if sub.rejected > before:
                done.set()

        run_until(node, sub, done, publish_once)
        assert sub.rejected > before and not delivered

    done = threading.Event()
    run_until(node, sub, done, lambda: (send(good), delivered and done.set()))
    assert delivered and delivered[0] == "ok"


def echo(type_name, topic_in, topic_out):
    """Republishes each frame of a device="cuda" topic on another, device to device, until SIGINT
    or 30 s. A C++ test runs it, so that its frames go through the Python aux layout both ways."""
    msg_type = {"Image": Image, "PointCloud2": PointCloud2}[type_name]
    rclpy.init()
    node = Node("gpu_echo")
    pub = flux_wrapper.Publisher(node, msg_type, topic_out, 4, device="cuda")

    def on_frame(v):
        loan = pub.loan()
        loan.header = v.header
        for name in flux_wrapper._SCALARS[msg_type]:
            setattr(loan, name, getattr(v, name))
        if msg_type is PointCloud2:
            loan.fields = v.fields
        frame = v.device_data
        buf = loan.alloc_buffer(frame.nbytes)
        if buf is not None and CUDART.cudaStreamSynchronize(ctypes.c_void_p(frame.stream)) == 0:
            dst = ctypes.c_void_p(buf.__cuda_array_interface__["data"][0])
            src = ctypes.c_void_p(frame.__cuda_array_interface__["data"][0])
            if CUDART.cudaMemcpy(dst, src, ctypes.c_size_t(frame.nbytes), 3) == 0:
                pub.publish(loan)

    sub = flux_wrapper.Subscription(node, msg_type, topic_in, on_frame, 4, device="cuda")
    ex = flux_wrapper.Executor()
    ex.add(sub)
    ex.add_node(node)
    stop = threading.Timer(30, ex.stop)
    stop.daemon = True
    stop.start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass  # the C++ test's SIGINT


if __name__ == "__main__":
    import sys

    echo(*sys.argv[1:])
