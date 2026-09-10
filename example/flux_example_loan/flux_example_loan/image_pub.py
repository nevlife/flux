#!/usr/bin/env python3
"""Publish an Image into a loaned slot, with no copy.

ros2 run flux_example_loan image_pub.py
"""

import flux.ros
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs_flux.image import Image


class ImageLoanPublisher(Node):

    TOPIC = "image"
    FRAME_ID = "camera"
    ENCODING = "bgr8"
    WIDTH = 1920
    HEIGHT = 1200
    STEP = WIDTH * 3
    DATA_BYTES = STEP * HEIGHT
    SLOT_SIZE = DATA_BYTES + 1024
    SLOT_COUNT = 8
    RATE_HZ = 30.0

    def __init__(self):
        super().__init__("flux_loan_image_pub")
        self.ramp = np.arange(self.DATA_BYTES, dtype=np.uint8)
        self.phase = 0
        self.publisher = flux.ros.Publisher(
            self,
            self.TOPIC,
            fingerprint=Image.FINGERPRINT__,
            slot_size=self.SLOT_SIZE,
            slot_count=self.SLOT_COUNT,
        )
        self.timer = self.create_timer(1.0 / self.RATE_HZ, self.tick)

    def tick(self):
        b = Image.build__(self.publisher)
        if b is None:
            return
        stamp = self.get_clock().now().nanoseconds
        b.set__header__stamp(stamp // 1000000000, stamp % 1000000000)
        b.header__frame_id = self.FRAME_ID
        b.height = self.HEIGHT
        b.width = self.WIDTH
        b.encoding = self.ENCODING
        b.is_bigendian = 0
        b.step = self.STEP
        np.add(self.ramp, np.uint8(self.phase), out=b.alloc__data(self.DATA_BYTES))
        self.report(b.commit__())
        self.phase = (self.phase + 1) % 256

    def report(self, published):
        """Backpressure is a dropped frame on a best-effort transport, so it is not an error.
        Anything else does not clear on its own."""
        if flux.faulted(published):
            self.get_logger().error(f"publish refused: {published}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageLoanPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
