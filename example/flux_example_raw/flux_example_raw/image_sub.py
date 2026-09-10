#!/usr/bin/env python3
"""Read a bare uint8 volume with no .msg and no adapter.

ros2 run flux_example_raw image_sub.py
"""

import flux.ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ImageRawSubscriber(Node):

    TOPIC = "image_raw"

    def __init__(self):
        super().__init__("flux_raw_image_sub")
        self.seen = 0
        self.shape = ()
        self.data_bytes = 0
        self.subscription = flux.ros.Subscription(
            self, self.TOPIC, callback=self.on_frame, fingerprint=flux.NO_SCHEMA
        )

    def on_frame(self, view):
        # The view already carries dtype and shape, which is all a schemaless frame ever says.
        self.shape = view.shape
        self.data_bytes = view.nbytes
        self.seen += 1


def main(args=None):
    rclpy.init(args=args)
    node = ImageRawSubscriber()
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
