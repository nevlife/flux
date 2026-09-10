#!/usr/bin/env python3
"""Serve a flux channel and a ROS subscription from one spin loop.

ros2 run flux_example_executor merged_sub.py
"""

import flux.ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image as RosImage
from sensor_msgs_flux.image import Image


class MergedSubscriber(Node):

    TOPIC = "image"
    DEPTH = 8

    def __init__(self):
        super().__init__("flux_merged_sub")
        self.flux_seen = 0
        self.ros_seen = 0
        self.flux_width = 0
        self.ros_width = 0

        self.flux_sub = flux.ros.Subscription(
            self, self.TOPIC, callback=self.on_flux, fingerprint=Image.FINGERPRINT__
        )
        qos = QoSProfile(depth=self.DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.ros_sub = self.create_subscription(RosImage, self.TOPIC, self.on_ros, qos)
        self.timer = self.create_timer(1.0, self.report)

    def on_flux(self, view):
        v = Image.View(view)
        self.flux_width = v.width
        self.flux_seen += 1

    def on_ros(self, msg):
        self.ros_width = msg.width
        self.ros_seen += 1

    def report(self):
        self.get_logger().info(
            f"flux {self.flux_seen} (w={self.flux_width})  "
            f"ros {self.ros_seen} (w={self.ros_width})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MergedSubscriber()
    executor = flux.ros.Executor()
    executor.add_flux(node.flux_sub)
    executor.add_ros_node(node)
    node.get_logger().info(f"merged wait: io_uring={executor.uses_io_uring}")
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
