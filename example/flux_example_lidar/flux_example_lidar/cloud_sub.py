#!/usr/bin/env python3
"""Read a lidar sweep out of the publisher's slot.

ros2 run flux_example_lidar cloud_sub.py
"""

import flux.ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs_flux.point_cloud2 import PointCloud2


class CloudSubscriber(Node):

    TOPIC = "cloud"

    def __init__(self):
        super().__init__("flux_lidar_cloud_sub")
        self.seen = 0
        self.points = 0
        self.point_step = 0
        self.data_bytes = 0
        self.subscription = flux.ros.Subscription(
            self, self.TOPIC, callback=self.on_frame, fingerprint=PointCloud2.FINGERPRINT__
        )
        self.timer = self.create_timer(1.0, self.report)

    def on_frame(self, view):
        v = PointCloud2.View(view)
        self.points = v.width
        self.point_step = v.point_step
        self.data_bytes = v.data.size
        self.seen += 1

    def report(self):
        self.get_logger().info(
            f"seen {self.seen}  points {self.points}  point_step {self.point_step}  "
            f"bytes {self.data_bytes}  lost {self.subscription.lost}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CloudSubscriber()
    executor = flux.ros.Executor()
    executor.add_flux(node.subscription)
    executor.add_ros_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.stop()
        executor.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
