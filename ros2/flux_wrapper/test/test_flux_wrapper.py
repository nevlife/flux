import os
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


@pytest.fixture
def node():
    context = rclpy.Context()
    rclpy.init(context=context)
    n = Node("wrapper", namespace=f"/flux_wrapper_py_{os.getpid()}", context=context)
    yield n
    n.destroy_node()
    rclpy.try_shutdown(context=context)


def run_until(node, sub, done, publish_once):
    """Spins `node` while `publish_once` runs every 10 ms, until `done` is set or 10 s."""
    ex = flux_wrapper.Executor()
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
