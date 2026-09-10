#!/usr/bin/env python3
"""CUDA subscriber for the same host publisher: no copy, no staging, no handle.

ros2 run flux_example_gpu image_sub_gpu.py

The publisher need never mention the GPU. On an integrated GPU the slot mapping already is device
memory, so device="cuda" is a local statement about who reads the bytes -- nothing on the wire
changes, and a host subscriber keeps running unmodified.

Frames arrive as flux.Frame instead of numpy. The `with` domain is required: leaving it waits on
the stream before releasing the borrow, and that wait must happen where the caller put it rather
than whenever Python collects the object.
"""

import flux.ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GpuImageSubscriber(Node):

    TOPIC = "image_gpu"

    def __init__(self):
        super().__init__("flux_gpu_image_sub")
        self.seen = 0
        self.device_ptr = 0
        self.shape = ()
        self.subscription = flux.ros.Subscription(
            self, self.TOPIC, callback=self.on_frame, fingerprint=flux.NO_SCHEMA, device="cuda"
        )
        self.timer = self.create_timer(1.0, self.report)

    def on_frame(self, frame):
        with frame as view:
            self.device_ptr = view.__cuda_array_interface__["data"][0]
            self.shape = view.shape
            self.seen += 1

    def report(self):
        self.get_logger().info(
            f"seen {self.seen}  {self.shape}  device_ptr 0x{self.device_ptr:x}  "
            f"release max {self.subscription.fence_wait.release_max_ns / 1e3:.0f} us  "
            f"fence_failed {self.subscription.fence_failed}"
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GpuImageSubscriber()
    except ValueError as exc:
        rclpy.logging.get_logger("flux_gpu_image_sub").error(str(exc))
        rclpy.try_shutdown()
        return
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
