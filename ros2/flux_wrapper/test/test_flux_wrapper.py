import ctypes
import glob
import os
import struct
import threading
import time
import tracemalloc

from flux_wrapper_test_msgs.msg import Bar
from flux_wrapper_test_msgs.msg import Blob
from flux_wrapper_test_msgs.msg import Foo
from geometry_msgs.msg import Point
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from std_msgs.msg import String

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


def pack(msg_type, m):
    """The aux bytes a device="cuda" publisher's loan writes for m: m without data, as a frame of
    the type."""
    adapter, bridge = flux_wrapper._adapter(msg_type)
    buf = np.zeros(flux_wrapper._GPU_AUX_BYTES, np.uint8)
    b = adapter.Builder(buf)
    bridge.msg_to_frame_without_data(m, b)
    return bytes(buf[: b.size__])


@pytest.mark.skipif(flux_wrapper._flux is None, reason="a build without flux")
def test_gpu_aux_is_a_frame_of_the_type_without_data():
    """The fingerprint is flux.gpu_fingerprint of the type's; the aux a frame of the type."""
    import flux
    from sensor_msgs_flux import image, point_cloud2

    assert flux_wrapper._gpu_fingerprint(Image) == flux.gpu_fingerprint(image.FINGERPRINT__)
    assert flux_wrapper._gpu_fingerprint(PointCloud2) == flux.gpu_fingerprint(
        point_cloud2.FINGERPRINT__
    )
    aux = pack(Image, Image(header=header(1, "ok"), encoding="rgb8"))
    assert image.View(aux).encoding == "rgb8"
    m = cloud(3)
    assert flux_wrapper._unpack(PointCloud2, pack(PointCloud2, m)) == without_data(m)


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
        assert buf.__cuda_array_interface__["shape"] == (len(want.data),)
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
    big = image(1)
    big.data = bytes(8192)
    assert not pub.publish(big)
    # The C++ line: flux::to_string() of the result and the slot size.
    want = "data_over_logged failed: too large (slot 4096 bytes); later failures are not logged"
    assert sum(want in line for line in logs.lines) == 1, logs.lines
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
def test_gpu_frame_with_a_bad_aux_is_skipped(node, monkeypatch):
    import flux

    logs = _Errors(node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logs)
    delivered = []
    sub = flux_wrapper.Subscription(
        node, Image, "bad_aux", lambda v: delivered.append(v.header.frame_id), 4
    )
    fingerprint, size = flux_wrapper._gpu_fingerprint(Image), flux_wrapper._GPU_AUX_BYTES
    raw = flux.Publisher(node, "bad_aux", fingerprint=fingerprint, slot_size=4096, aux_size=size)
    good = bytearray(pack(Image, Image(header=header(1, "ok"))))
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
    want = (
        "bad_aux: skipped a GPU frame with a bad aux header (10 bytes); later ones are counted in "
        "rejected, not logged"
    )
    assert logs.lines == [want], logs.lines

    done = threading.Event()
    run_until(node, sub, done, lambda: (send(good), delivered and done.set()))
    assert delivered and delivered[0] == "ok"


def foo(sec, data):
    """A custom type with every kind of field the wrapper carries besides its root data."""
    m = Foo(header=header(sec, "foo_frame"), width=3, scale=0.5, valid=True, label="label")
    m.tags = ["a", "bc"]
    m.values = [1.5, -2.0]
    m.origin = Point(x=1.0, y=2.0, z=3.0)
    m.corners = [Point(), Point(x=4.0)]
    m.data = bytes(data)
    return m


def bar(sec):
    return Bar(header=header(sec, "bar_frame"), value=-4, name="bar", samples=[0.25, 0.5])


def test_any_type_through_a_loan_with_its_data_in_place(node):
    """A type without accessors of its own is set from its ROS message and read by its field
    names, and its root data is written and read in place, on either path."""
    got, done = {}, threading.Event()

    def on_foo(v):
        got.update(
            sec=v.header.stamp.sec,
            label=v.label,
            fields=v.without_data(),
            data=bytes(v.data),
            msg=v.to_msg(),
        )
        done.set()

    sub = flux_wrapper.Subscription(node, Foo, "foo", on_foo, 4)
    pub = flux_wrapper.Publisher(node, Foo, "foo", 4)
    assert pub.uses_flux == EXPECT_FLUX
    fields = foo(8, [1, 2, 3])

    def publish_once():
        loan = pub.loan()
        assert loan
        loan.alloc_data(4)[:] = 9
        loan.set_from(fields)  # leaves data alone: what alloc_data() wrote goes out
        assert pub.publish(loan)

    run_until(node, sub, done, publish_once)
    whole = foo(8, [9, 9, 9, 9])
    assert got == {
        "sec": 8,
        "label": "label",
        "fields": without_data(fields),
        "data": bytes([9, 9, 9, 9]),
        "msg": whole,
    }


def test_any_type_from_a_ros_message(node):
    got, done = [], threading.Event()

    def on_foo(v):
        got.append(v.to_msg())
        done.set()

    sub = flux_wrapper.Subscription(node, Foo, "foo_message", on_foo, 4)
    pub = flux_wrapper.Publisher(node, Foo, "foo_message", 4)
    want = foo(1, [4, 5, 6])
    run_until(node, sub, done, lambda: pub.publish(want))
    assert got and got[0] == want


def test_a_type_without_data_is_set_and_read_whole(node):
    """Without a root data the loan sets and the View reads every field."""
    got, done = [], threading.Event()

    def on_bar(v):
        if v.header.stamp.sec == 9:
            got.append((v.to_msg(), v.without_data(), v.name, list(v.samples)))
            done.set()

    sub = flux_wrapper.Subscription(node, Bar, "bar", on_bar, 4)
    pub = flux_wrapper.Publisher(node, Bar, "bar", 4)
    want = bar(3)

    def publish_once():
        loan = pub.loan()
        assert loan
        loan.set_from(want)
        loan.header = header(9, "late")
        assert pub.publish(loan)

    run_until(node, sub, done, publish_once)
    want.header = header(9, "late")
    assert got and got[0] == (want, want, "bar", [0.25, 0.5])


@pytest.mark.parametrize("device", ["cpu", "cuda"])
def test_a_loan_starts_from_an_empty_message(node, device):
    """A field the loan does not set is the type's default, not the last loan's, as in C++."""
    topic = f"foo_fresh_{device}"
    got, done = [], threading.Event()

    def on_foo(v):
        if v.header.stamp.sec == 2:
            got.append(v.without_data())
            done.set()

    sub = flux_wrapper.Subscription(node, Foo, topic, on_foo, 4)
    pub = flux_wrapper.Publisher(node, Foo, topic, 4, device=device)

    def publish_once():
        loan = pub.loan()
        loan.set_from(foo(1, []))
        assert pub.publish(loan)
        loan = pub.loan()
        loan.header = header(2, "only_header")
        assert pub.publish(loan)

    run_until(node, sub, done, publish_once)
    assert got and got[0] == Foo(header=header(2, "only_header"))


@pytest.mark.skipif(not EXPECT_FLUX, reason="the slots are flux's")
@pytest.mark.parametrize("device", ["cpu", "cuda"])
def test_a_loan_with_no_free_slot_is_false_at_once(node, device):
    """loan() takes the slot, on device="cuda" too, as in C++: with none free the Loan is false
    before alloc_buffer(), and filling it anyway is harmless."""
    for msg_type, msg in ((Image, image(1)), (Foo, foo(1, [1]))):
        topic = f"full_{msg_type.__name__}_{device}"
        pub = flux_wrapper.Publisher(node, msg_type, topic, 1, slot_count=2, device=device)
        held = [pub.loan() for _ in range(2)]
        assert all(held)
        loan = pub.loan()
        assert not loan
        loan.set_from(msg)
        assert loan.alloc_buffer(1) is None
        assert not loan
        assert not pub.publish(loan)
        del held


@pytest.mark.skipif(not EXPECT_FLUX, reason="the slots are flux's")
@pytest.mark.parametrize("device", ["cpu", "cuda"])
def test_a_false_loan_hands_out_no_data(node, device):
    """After a field past the slot ("cpu") or the aux ("cuda"), or data past where it goes, the
    Loan is false and the data asked of it after is None, as in C++."""
    # Past the 8192-byte slot of a "cpu" publisher and the 4096-byte aux of a "cuda" one.
    over = 9000

    def expect_no_data(loan, what):
        assert not loan, what
        assert loan.alloc_data(64) is None, what
        assert loan.alloc_buffer(64) is None, what
        assert not loan, what

    pubs = {
        t: flux_wrapper.Publisher(node, t, f"false_{t.__name__}_{device}", 1, device=device)
        for t in (Image, PointCloud2, Foo)
    }
    assert pubs[Image].uses_device == (device == "cuda" and NO_CUDA == "")
    loan = pubs[Image].loan()
    loan.header = header(1, "f" * over)
    expect_no_data(loan, "Image header")
    assert not pubs[Image].publish(loan)

    loan = pubs[PointCloud2].loan()
    loan.header = header(1, "f" * over)
    expect_no_data(loan, "PointCloud2 header")
    assert not pubs[PointCloud2].publish(loan)

    m = foo(1, [1])
    m.label = "l" * over
    loan = pubs[Foo].loan()
    loan.set_from(m)
    expect_no_data(loan, "Foo set_from")
    assert not pubs[Foo].publish(loan)

    # Data past the arena ("cpu") or the slot ("cuda") fails the Loan; smaller data after is None.
    loan = pubs[Image].loan()
    assert loan.alloc_buffer(1 << 40) is None
    expect_no_data(loan, "Image data")
    assert not pubs[Image].publish(loan)


@pytest.mark.parametrize("device", ["cpu", "cuda"])
def test_a_loan_sets_only_the_fields_cpp_sets(node, device):
    """Image and PointCloud2 have a setter per field but data, on either path; another name,
    data among them, raises the same error on every path."""
    for msg_type in (Image, PointCloud2):
        topic = f"setters_{msg_type.__name__}_{device}"
        loan = flux_wrapper.Publisher(node, msg_type, topic, 4, device=device).loan()
        names = ["widht", "data"] + (["fields"] if msg_type is Image else [])
        for name in names:
            with pytest.raises(AttributeError, match=f"{msg_type.__name__}: no setter for {name}"):
                setattr(loan, name, 3)


def test_any_type_has_the_writers_cpp_has(node):
    """set_from(), header (C++ set_header()) with a root header, and the data. Another field set
    alone raises, on either path."""
    loan = flux_wrapper.Publisher(node, Foo, "foo_writers", 4).loan()
    with pytest.raises(AttributeError):
        loan.width = 3
    blob = flux_wrapper.Publisher(node, Blob, "blob_writers", 4).loan()
    with pytest.raises(AttributeError):
        blob.header = header(1, "blob")


def test_a_standard_type_is_carried(node):
    got, done = [], threading.Event()

    def on_imu(v):
        got.append(v.to_msg())
        done.set()

    sub = flux_wrapper.Subscription(node, Imu, "imu", on_imu, 4)
    pub = flux_wrapper.Publisher(node, Imu, "imu", 4)
    want = Imu(header=header(4, "imu"))
    want.orientation.w = 1.0
    want.linear_acceleration_covariance[4] = 2.0
    run_until(node, sub, done, lambda: pub.publish(want))
    assert got and got[0] == want


def test_a_byte_data_without_a_header(node):
    """A root data of byte[] is written in place as uint8[] is. rclpy holds a byte[] as a list of
    one-byte bytes, so the View's data is a copy on the ROS path."""
    got, done = [], threading.Event()

    def on_blob(v):
        got.append((bytes(v.data), v.to_msg()))
        done.set()

    sub = flux_wrapper.Subscription(node, Blob, "blob", on_blob, 4)
    pub = flux_wrapper.Publisher(node, Blob, "blob", 4)

    def publish_once():
        loan = pub.loan()
        assert loan
        loan.set_from(Blob(seq=11))
        loan.alloc_data(3)[:] = [1, 2, 3]
        assert pub.publish(loan)

    run_until(node, sub, done, publish_once)
    want = Blob(seq=11, data=[bytes((x,)) for x in (1, 2, 3)])
    assert got and got[0] == (bytes([1, 2, 3]), want)


def test_without_data_and_set_from_do_not_copy_the_payload(node):
    """They copy or share the other fields and never the payload: no allocation the size of data,
    on either path, where to_msg() on the flux path makes one."""
    payload = 4 << 20
    big = foo(6, bytes([7]) * payload)
    pub = flux_wrapper.Publisher(node, Foo, "foo_big", 4)
    loan = pub.loan()
    tracemalloc.start()
    loan.set_from(big)
    set_from_peak = tracemalloc.get_traced_memory()[1]
    tracemalloc.stop()
    assert set_from_peak < payload // 4
    del loan

    got, done = [], threading.Event()

    def on_foo(v):
        tracemalloc.start()
        fields = v.without_data()
        fields_peak = tracemalloc.get_traced_memory()[1]
        tracemalloc.reset_peak()
        msg = v.to_msg()
        msg_peak = tracemalloc.get_traced_memory()[1]
        tracemalloc.stop()
        got.append((fields, fields_peak, msg, msg_peak))
        done.set()

    sub = flux_wrapper.Subscription(node, Foo, "foo_big", on_foo, 4)
    run_until(node, sub, done, lambda: pub.publish(big))
    fields, fields_peak, msg, msg_peak = got[0]
    assert fields == without_data(big)
    assert fields_peak < payload // 4
    assert msg == big
    if EXPECT_FLUX:  # the ROS path's to_msg() is the received message itself
        assert msg_peak >= payload


def test_cuda_on_a_type_without_data_is_cpu(node, monkeypatch):
    """device="cuda" needs a root data: without one the publisher and the subscription are "cpu",
    each saying so with its topic."""
    logs = _Warnings(node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logs)
    got, done = [], threading.Event()

    def on_bar(v):
        got.append((v.on_device, v.to_msg()))
        done.set()

    sub = flux_wrapper.Subscription(node, Bar, "bar_cuda", on_bar, 4, device="cuda")
    pub = flux_wrapper.Publisher(node, Bar, "bar_cuda", 4, device="cuda")
    assert not pub.uses_device
    want = bar(2)
    run_until(node, sub, done, lambda: pub.publish(want))
    assert got and got[0] == (False, want)
    warned = 'bar_cuda: device="cuda", but the type has no root uint8[] data'
    assert sum(warned in line for line in logs.lines) == 2, logs.lines


@needs_cuda
@pytest.mark.parametrize("pub_device", ["cpu", "cuda"])
@pytest.mark.parametrize("sub_device", ["cpu", "cuda"])
def test_cuda_any_type_in_every_device_pairing(node, pub_device, sub_device):
    """A "cuda" subscription has the data in device memory, a "cpu" one in host memory (of a
    "cuda" publisher on a discrete GPU, the copy flux makes at take)."""
    want = foo(3, [3, 1, 4, 1, 5, 9])
    topic = f"foo_pair_{pub_device}_{sub_device}"
    got, done = [], threading.Event()

    def on_foo(v):
        data = from_view(v) if v.on_device else bytes(v.data)
        got.append((v.on_device, data, v.without_data(), v.to_msg()))
        done.set()

    sub = flux_wrapper.Subscription(node, Foo, topic, on_foo, 4, device=sub_device)
    pub = flux_wrapper.Publisher(node, Foo, topic, 4, device=pub_device)
    assert pub.uses_device == (pub_device == "cuda")

    def publish_once():
        loan = pub.loan()
        loan.set_from(want)
        if pub_device == "cuda":
            buf = loan.alloc_buffer(len(want.data))
            assert buf is not None
            to_device(buf, bytes(want.data))
        else:
            loan.alloc_data(len(want.data))[:] = np.frombuffer(bytes(want.data), np.uint8)
        assert pub.publish(loan)

    run_until(node, sub, done, publish_once)
    assert got and got[0] == (sub_device == "cuda", bytes(want.data), without_data(want), want)


@pytest.mark.skipif(not EXPECT_FLUX, reason="flux path only")
def test_any_type_past_the_slot_is_logged_once_with_its_size(node, monkeypatch):
    """Of a type without accessors the log cannot name the field that did not fit, only the size."""
    logs = _Errors(node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logs)
    pub = flux_wrapper.Publisher(node, Foo, "foo_over_logged", 4)
    m = foo(5, [1])
    m.label = "l" * 9000
    for _ in range(2):
        loan = pub.loan()
        assert loan
        loan.set_from(m)
        assert not loan  # at once, as in C++
        assert not pub.publish(loan)
    assert not pub.publish(m)
    want = (
        "foo_over_logged: a message did not go out: the fields set from it did not fit the slot ("
    )
    assert sum(want in line for line in logs.lines) == 1, logs.lines
    want = "B used before them); header.stamp 5.000000007. Later ones"
    assert sum(want in line for line in logs.lines) == 1, logs.lines
    loan = pub.loan()
    loan.header = header(5, "f" * 9000)
    assert not loan
    assert not pub.publish(loan)
    if NO_CUDA:
        return
    gpu = flux_wrapper.Publisher(node, Foo, "foo_over_aux", 4, device="cuda")
    m.label = "l" * 5000
    loan = gpu.loan()
    loan.set_from(m)
    assert not loan
    assert not gpu.publish(loan)
    assert not gpu.publish(m)
    want = (
        'foo_over_aux: a message did not go out on device="cuda": the fields set from it did not '
        "fit the aux (4096 B, "
    )
    assert sum(want in line for line in logs.lines) == 1, logs.lines
    # The header set alone is named, as by C++ set_header().
    gpu = flux_wrapper.Publisher(node, Foo, "foo_header_aux", 4, device="cuda")
    loan = gpu.loan()
    loan.header = header(5, "f" * 5000)
    assert not loan
    assert not gpu.publish(loan)
    used = flux_wrapper._adapter(Foo)[0].SCALAR_BYTES__
    want = (
        'foo_header_aux: a message did not go out on device="cuda": header.frame_id (5000 B) did '
        f"not fit the aux (4096 B, {used} B used before it); header.stamp 5.000000007. Later ones "
        "are not logged"
    )
    assert sum(want in line for line in logs.lines) == 1, logs.lines


@pytest.mark.skipif(not EXPECT_FLUX, reason="flux path only")
def test_a_type_without_an_adapter_names_the_module_looked_for(node):
    with pytest.raises(TypeError, match=r"std_msgs_flux\.string_ros does not import"):
        flux_wrapper.Publisher(node, String, "no_adapter", 1)


def echo(type_name, topic_in, topic_out):
    """Republishes each frame of a device="cuda" topic on another, device to device, until SIGINT
    or 30 s. A C++ test runs it, so that its frames go through the Python aux layout both ways."""
    msg_type = {"Image": Image, "PointCloud2": PointCloud2, "Foo": Foo}[type_name]
    rclpy.init()
    node = Node("gpu_echo")
    pub = flux_wrapper.Publisher(node, msg_type, topic_out, 4, device="cuda")

    def on_frame(v):
        loan = pub.loan()
        loan.set_from(v.without_data())
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
