#!/usr/bin/env python3
"""Publish a lidar sweep whose point count changes every frame.

ros2 run flux_example_lidar cloud_pub.py
"""

import flux.ros
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs_flux.point_cloud2 import PointCloud2


class CloudPublisher(Node):

    TOPIC = "cloud"
    FRAME_ID = "lidar"
    POINT_STEP = 16
    MIN_POINTS = 40000
    MAX_POINTS = 131072
    SLOT_SIZE = MAX_POINTS * POINT_STEP + 4096
    SLOT_COUNT = 4
    RATE_HZ = 10.0
    FLOAT32 = 7

    def __init__(self):
        super().__init__("flux_lidar_cloud_pub")
        self.sweep = 0
        self.ramp = np.arange(self.MAX_POINTS, dtype=np.float32) * 0.001
        self.publisher = flux.ros.Publisher(
            self,
            self.TOPIC,
            fingerprint=PointCloud2.FINGERPRINT__,
            slot_size=self.SLOT_SIZE,
            slot_count=self.SLOT_COUNT,
        )
        self.timer = self.create_timer(1.0 / self.RATE_HZ, self.tick)

    def tick(self):
        b = PointCloud2.build__(self.publisher)
        if b is None:
            return
        n = self.MIN_POINTS + self.sweep % (self.MAX_POINTS - self.MIN_POINTS)
        stamp = self.get_clock().now().nanoseconds
        b.set__header__stamp(stamp // 1000000000, stamp % 1000000000)
        b.header__frame_id = self.FRAME_ID
        b.height = 1
        b.width = n
        b.is_bigendian = False
        b.point_step = self.POINT_STEP
        b.row_step = n * self.POINT_STEP
        b.is_dense = True

        fields = b.alloc__fields(4)
        for i, name in enumerate(("x", "y", "z", "intensity")):
            fields[i].name = name
            fields[i].offset = 4 * i
            fields[i].datatype = self.FLOAT32
            fields[i].count = 1

        xyzi = b.alloc__data(n * self.POINT_STEP).view(np.float32).reshape(n, 4)
        a = self.ramp[:n]
        xyzi[:, 0] = a
        xyzi[:, 1] = a * 2.0
        xyzi[:, 2] = a * 3.0
        xyzi[:, 3] = float(self.sweep % 256)

        self.report(b.commit__())
        self.sweep += 1

    def report(self, published):
        """Backpressure is a dropped frame on a best-effort transport, so it is not an error.
        Anything else does not clear on its own."""
        if flux.faulted(published):
            self.get_logger().error(f"publish refused: {published}")


def main(args=None):
    rclpy.init(args=args)
    node = CloudPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
