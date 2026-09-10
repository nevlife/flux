#!/usr/bin/env python3
"""Give the ROS side and the flux side an executor each, on two threads.

ros2 run flux_example_executor split_sub.py

merged_sub puts both on one thread and gets mutual exclusion for free. Here they run at the same
time, so anything both callbacks touch needs a lock. That lock is the cost of the shape.

This works because a flux subscription is not an rclpy entity: it is in no callback group, so the
two executors never contend for one. The flux executor is given the flux subscription and nothing
else: no add_ros_node, because rclpy.spin already has the node.
"""

import threading

import flux.ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image as RosImage
from sensor_msgs_flux.image import Image


class SplitSubscriber(Node):

    TOPIC = "image"
    DEPTH = 8

    def __init__(self):
        super().__init__("flux_split_sub")
        self.lock = threading.Lock()
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
        width = Image.View(view).width
        with self.lock:               # the rclpy thread reads these too
            self.flux_width = width
            self.flux_seen += 1

    def on_ros(self, msg):
        with self.lock:
            self.ros_width = msg.width
            self.ros_seen += 1

    def report(self):
        with self.lock:
            self.get_logger().info(
                f"flux {self.flux_seen} (w={self.flux_width})  "
                f"ros {self.ros_seen} (w={self.ros_width})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SplitSubscriber()

    # No add_ros_node here: rclpy.spin below has the node, and handing it to both is the mistake
    # this shape exists to avoid.
    flux_executor = flux.ros.Executor()
    flux_executor.add_flux(node.flux_sub)
    flux_thread = threading.Thread(target=flux_executor.spin, daemon=True, name="flux-spin")
    flux_thread.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        flux_executor.stop()
        flux_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
