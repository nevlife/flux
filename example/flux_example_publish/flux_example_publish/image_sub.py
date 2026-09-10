#!/usr/bin/env python3
"""Read an Image straight out of the publisher's slot.

ros2 run flux_example_publish image_sub.py
"""

import flux.ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs_flux.image import Image


class ImageSubscriber(Node):

    TOPIC = "image"

    def __init__(self):
        super().__init__("flux_publish_image_sub")
        self.seen = 0
        self.width = 0
        self.height = 0
        self.encoding = ""
        self.data_bytes = 0
        self.subscription = flux.ros.Subscription(
            self, self.TOPIC, callback=self.on_frame, fingerprint=Image.FINGERPRINT__
        )

    def on_frame(self, view):
        v = Image.View(view)
        self.width = v.width
        self.height = v.height
        self.encoding = v.encoding
        self.data_bytes = v.data.size
        self.seen += 1


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
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
