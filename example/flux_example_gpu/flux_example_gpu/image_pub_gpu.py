#!/usr/bin/env python3
"""Publish into a slot a kernel can write, with no staging buffer.

ros2 run flux_example_gpu image_pub_gpu.py

With cupy installed the loan is written on the device through __cuda_array_interface__; without
it the same slot is filled with host stores, which an integrated GPU allows because the mapping
is one both sides reach.
"""

import flux.ros
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    import cupy as cp
except ImportError:
    cp = None


class GpuImagePublisher(Node):

    TOPIC = "image_gpu"
    WIDTH = 1920
    HEIGHT = 1200
    CHANNELS = 3
    DATA_BYTES = WIDTH * HEIGHT * CHANNELS
    SLOT_COUNT = 4
    RATE_HZ = 30.0

    def __init__(self):
        super().__init__("flux_gpu_image_pub")
        self.phase = 0
        self.shape = (self.HEIGHT, self.WIDTH, self.CHANNELS)
        self.publisher = flux.ros.Publisher(
            self,
            self.TOPIC,
            fingerprint=flux.NO_SCHEMA,
            slot_size=self.DATA_BYTES,
            slot_count=self.SLOT_COUNT,
            device="cuda",
        )
        self.ramp = (
            cp.arange(self.DATA_BYTES, dtype=cp.uint8).reshape(self.shape)
            if cp is not None
            else np.arange(self.DATA_BYTES, dtype=np.uint8).reshape(self.shape)
        )
        self.get_logger().info(f"cupy {cp is not None}, stream {self.publisher.stream}")
        self.timer = self.create_timer(1.0 / self.RATE_HZ, self.tick)

    def tick(self):
        loan = self.publisher.loan(self.shape, dtype="uint8")
        if loan:
            if cp is not None:
                # Writes on the publisher's stream, which is the one commit() waits for.
                cp.add(self.ramp, cp.uint8(self.phase), out=cp.asarray(loan))
            else:
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
    try:
        node = GpuImagePublisher()
    except ValueError as exc:
        rclpy.logging.get_logger("flux_gpu_image_pub").error(str(exc))
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
