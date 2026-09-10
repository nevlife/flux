#!/usr/bin/env python3
"""Publish a bare uint8 volume with no .msg and no adapter.

ros2 run flux_example_raw image_pub.py
"""

import flux.ros
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ImageRawPublisher(Node):

    TOPIC = "image_raw"
    WIDTH = 1920
    HEIGHT = 1200
    CHANNELS = 3
    DATA_BYTES = WIDTH * HEIGHT * CHANNELS
    SLOT_COUNT = 8
    RATE_HZ = 30.0

    def __init__(self):
        super().__init__("flux_raw_image_pub")
        self.ramp = np.arange(self.DATA_BYTES, dtype=np.uint8).reshape(
            self.HEIGHT, self.WIDTH, self.CHANNELS
        )
        self.phase = 0
        self.publisher = flux.ros.Publisher(
            self,
            self.TOPIC,
            fingerprint=flux.NO_SCHEMA,
            slot_size=self.DATA_BYTES,
            slot_count=self.SLOT_COUNT,
        )
        self.timer = self.create_timer(1.0 / self.RATE_HZ, self.tick)

    def tick(self):
        # The array already states dtype and shape, so nothing restates them here. This is what
        # the C++ raw node spells out by hand in loan().
        loan = self.publisher.loan(self.ramp.shape, dtype="uint8")
        if loan:
            np.add(self.ramp, np.uint8(self.phase), out=loan.array)
            self.report(loan.commit())
        self.phase = (self.phase + 1) % 256

    def report(self, published):
        """Backpressure is a dropped frame on a best-effort transport, so it is not an error.
        Anything else does not clear on its own."""
        if flux.faulted(published):
            self.get_logger().error(f"publish refused: {published}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageRawPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
