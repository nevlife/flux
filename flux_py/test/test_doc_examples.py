"""The examples of the user documents, between [doc:<id>] and [doc:/<id>] anchors.

scripts/check_doc_examples.py holds every docs/ fence tagged doc:<id> equal to its region.
"""

import os
import subprocess
import sys
import threading
import time

import message_filters
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage

import flux
from flux.message_filters import Subscriber

try:
    import torch
except ImportError:
    torch = None


@pytest.fixture(scope="module")
def Image(tmp_path_factory):  # noqa: N802 the generated class
    out = tmp_path_factory.mktemp("adapters")
    subprocess.run(
        [
            os.path.join(get_package_prefix("flux_gen"), "lib", "flux_gen", "flux_gen"),
            "--pkg",
            "sensor_msgs",
            "--msg",
            os.path.join(get_package_share_directory("sensor_msgs"), "msg", "Image.msg"),
            "--out-py",
            str(out),
        ],
        check=True,
    )
    sys.path.insert(0, str(out))
    from sensor_msgs_flux.image import Image

    return Image


@pytest.fixture
def node():
    context = rclpy.Context()
    rclpy.init(context=context)
    n = Node("doc", namespace=f"/doc{os.getpid()}", context=context)
    yield n
    n.destroy_node()
    rclpy.try_shutdown(context=context)


def feed(node, topic, adapter=None):
    """Publishes on `topic` from another thread, then shuts the context down, which ends a spin."""

    def run():
        pub = flux.Publisher(
            node, topic, fingerprint=adapter.FINGERPRINT__ if adapter else flux.NO_SCHEMA
        )
        for i in range(50):
            time.sleep(0.01)
            if adapter is None:
                pub.publish(np.ones((8, 8), np.uint8))
            else:
                b = adapter.build__(pub)
                b.set__header__stamp(i, 0)
                b.width = 640
                b.commit__()
        rclpy.try_shutdown(context=node.context)

    t = threading.Thread(target=run)
    t.start()
    return t


def test_publish_and_loan(node):
    # [doc:py_publish]
    pub = flux.Publisher(node, "image")

    pixels = np.zeros((480, 640, 3), np.uint8)
    p = pub.publish(pixels)
    if flux.faulted(p):
        node.get_logger().error(f"publish: {p}")
    # [doc:/py_publish]
    assert p == flux.Published.OK
    sent = flux.Subscription(node, "image", qos=flux.QoS(depth=1, replay=1)).take()
    assert sent.shape == (480, 640, 3)

    pub = flux.Publisher(node, "points", slot_size=1 << 20, slot_count=4)
    check = flux.Subscription(node, "points")
    # [doc:py_loan]
    loan = pub.loan((1024, 3), "float32")
    if loan is not None:
        loan.array[:] = 0.0
        loan.commit()
    # [doc:/py_loan]
    assert check.take().shape == (1024, 3)


def test_subscribe_with_an_executor(node, capsys):
    publisher = feed(node, "image")

    # [doc:py_subscribe]
    def on_frame(frame):
        print(frame.shape, frame.dtype)

    sub = flux.Subscription(node, "image", callback=on_frame)

    ex = flux.Executor()
    ex.add(sub)
    ex.add_node(node)
    ex.spin()
    # [doc:/py_subscribe]

    publisher.join()
    assert "(8, 8) uint8" in capsys.readouterr().out


def test_take_without_an_executor(node, capsys):
    pub = flux.Publisher(node, "image", slot_size=4096, slot_count=4)
    # [doc:py_take]
    sub = flux.Subscription(node, "image")
    # [doc:/py_take]
    pub.publish(b"a")
    pub.publish(b"b")
    # [doc:py_take_frames]
    while (frame := sub.take()) is not None:
        print("frame of", frame.nbytes, "bytes")
        del frame
    frame = sub.take_blocking(100_000_000)
    # [doc:/py_take_frames]
    assert frame is None
    assert capsys.readouterr().out.count("frame of 1 bytes") == 2


def test_qos(node):
    pub = flux.Publisher(node, "cmd", slot_size=4096, slot_count=8)
    for c in (b"a", b"b", b"c"):
        pub.publish(c)
    # [doc:py_qos]
    qos = flux.QoS(depth=4, replay=2, max_borrow=2)
    sub = flux.Subscription(node, "cmd", qos=qos)
    # [doc:/py_qos]
    assert bytes(sub.take()) == b"b"


def test_adapter(node, Image, capsys):  # noqa: N803
    # [doc:py_adapter_publish]
    pub = flux.Publisher(node, "image", fingerprint=Image.FINGERPRINT__)

    b = Image.build__(pub)
    if b:
        stamp = node.get_clock().now().to_msg()
        b.set__header__stamp(stamp.sec, stamp.nanosec)
        b.header__frame_id = "camera"
        b.height = 480
        b.width = 640
        b.encoding = "rgb8"
        b.step = 640 * 3
        b.alloc__data(480 * 640 * 3)[:] = 0
        if flux.faulted(b.commit__()):
            node.get_logger().error("image not published")
    # [doc:/py_adapter_publish]
    sub = flux.Subscription(
        node, "image", fingerprint=Image.FINGERPRINT__, qos=flux.QoS(depth=1, replay=1)
    )
    frame = sub.take()
    # [doc:py_adapter_read]
    img = Image.View(frame)
    print(img.width, img.height, img.encoding, img.data.shape)
    # [doc:/py_adapter_read]
    assert "640 480 rgb8 (921600,)" in capsys.readouterr().out


def test_ros_message_bridge(node, Image):  # noqa: N803
    from sensor_msgs_flux.image_ros import frame_to_msg, msg_to_frame

    pub = flux.Publisher(node, "image", fingerprint=Image.FINGERPRINT__)
    sub = flux.Subscription(node, "image", fingerprint=Image.FINGERPRINT__)
    msg = RosImage(width=2, height=1, encoding="mono8", data=[7, 9])
    # [doc:py_ros_bridge_publish]
    b = Image.build__(pub)
    msg_to_frame(msg, b)
    if flux.faulted(b.commit__()):
        node.get_logger().error("image not published")
    # [doc:/py_ros_bridge_publish]
    frame = sub.take()
    # [doc:py_ros_bridge_read]
    copy = frame_to_msg(Image.View(frame))
    # [doc:/py_ros_bridge_read]
    assert list(copy.data) == [7, 9] and copy.encoding == "mono8"


def test_partitioned(node, capsys):
    publisher = feed(node, "image")

    # [doc:py_partitioned]
    def on_frame(frame):
        print(threading.current_thread().name, frame.nbytes, "bytes")

    def name_thread():
        threading.current_thread().name = "camera"

    camera = MutuallyExclusiveCallbackGroup()
    sub = flux.Subscription(node, "image", callback=on_frame)

    ex = flux.PartitionedExecutor()
    ex.add(sub, camera)
    ex.add_node(node)
    ex.on_thread_start(camera, name_thread)
    ex.spin()
    # [doc:/py_partitioned]

    publisher.join()
    assert "camera 64 bytes" in capsys.readouterr().out


def test_message_filters(node, Image, capsys):  # noqa: N803
    publishers = [feed(node, "left/image", Image), feed(node, "right/image", Image)]

    # [doc:py_message_filters]
    def on_pair(left, right):
        print("pair at", left.header.stamp.sec, left.view().width, right.view().width)

    left = Subscriber(node, Image, "left/image", 10)
    right = Subscriber(node, Image, "right/image", 10)
    sync = message_filters.ApproximateTimeSynchronizer([left, right], 10, 0.01)
    sync.registerCallback(on_pair)

    ex = flux.Executor()
    ex.add(left)
    ex.add(right)
    ex.spin()
    # [doc:/py_message_filters]

    for p in publishers:
        p.join()
    assert "640 640" in capsys.readouterr().out


def test_enumerate(node, capsys):
    pub = flux.Publisher(node, "image", slot_size=4096, slot_count=4)  # noqa: F841 kept alive
    # [doc:py_enumerate]
    for t in flux.enumerate_topics():
        s = flux.read_channel_stats(t.signpost)
        print(t.key, "up" if s.live else "down", len(t.endpoints), "endpoints")
    # [doc:/py_enumerate]
    assert f"/doc{os.getpid()}/image up" in capsys.readouterr().out


@pytest.mark.skipif(
    torch is None or not torch.cuda.is_available() or flux.probe().route == flux.Route.NONE,
    reason="needs torch and a CUDA device",
)
def test_gpu(node, capsys):
    # [doc:py_gpu_publish]
    pub = flux.Publisher(node, "depth", slot_size=4 << 20, slot_count=4, device="cuda")

    loan = pub.loan((480, 640), "float32")
    if loan is not None:
        depth = torch.from_dlpack(loan)
        depth.fill_(1.5)
        loan.commit()
    # [doc:/py_gpu_publish]
    sub = flux.Subscription(node, "depth", qos=flux.QoS(depth=1, replay=1), device="cuda")
    # [doc:py_gpu_read]
    frame = sub.take()
    if frame is not None:
        with frame:
            depth = torch.from_dlpack(frame)
            print(depth.shape, float(depth.mean()))
    # [doc:/py_gpu_read]
    assert "torch.Size([480, 640]) 1.5" in capsys.readouterr().out
