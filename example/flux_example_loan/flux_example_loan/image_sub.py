#!/usr/bin/env python3
"""Read an Image straight out of the publisher's slot.

ros2 run flux_example_loan image_sub.py
"""

import flux.ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs_flux.image import Image


class ImageSubscriber(Node):
    TOPIC = "image"

    def __init__(self):
        super().__init__("flux_loan_image_sub")
        self.seen = 0
        self.width = 0
        self.height = 0
        self.encoding = ""
        self.data_bytes = 0
        self.subscription = flux.ros.create_subscription(
            self, self.TOPIC, callback=self.on_frame, fingerprint=Image.FINGERPRINT__
        )
        self.timer = self.create_timer(1.0, self.report)

    def on_frame(self, view):
        v = Image.View(view)
        self.width = v.width
        self.height = v.height
        self.encoding = v.encoding
        self.data_bytes = v.data.size
        self.seen += 1

    def report(self):
        self.get_logger().info(
            f"seen {self.seen}  {self.width}x{self.height} {self.encoding}  "
            f"bytes {self.data_bytes}  lost {self.subscription.lost}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    executor = flux.ros.Executor()
    executor.add(node.subscription)
    executor.add_ros_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
