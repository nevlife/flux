#!/usr/bin/env python3
"""Give each flux subscription its own thread, so a slow callback delays nobody else.

ros2 run flux_example_executor partitioned_sub.py
"""

import time

import flux.ros
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs_flux.image import Image


class PartitionedSubscriber(Node):

    TOPIC = "image"
    SLOW_WORK_SEC = 0.08

    def __init__(self):
        super().__init__("flux_partitioned_sub")
        self.fast_seen = 0
        self.slow_seen = 0

        # Partition tokens only. rclpy cannot move a ROS entity onto a child executor's thread,
        # so a group handed to add_flux must hold none.
        self.fast_group = MutuallyExclusiveCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()

        self.fast_sub = flux.ros.Subscription(
            self, self.TOPIC, callback=self.on_fast, fingerprint=Image.FINGERPRINT__
        )
        self.slow_sub = flux.ros.Subscription(
            self, self.TOPIC, callback=self.on_slow, fingerprint=Image.FINGERPRINT__
        )

        self.timer = self.create_timer(1.0, self.report)

    def on_fast(self, _view):
        self.fast_seen += 1

    def on_slow(self, _view):
        time.sleep(self.SLOW_WORK_SEC)
        self.slow_seen += 1

    def report(self):
        self.get_logger().info(f"fast {self.fast_seen}  slow {self.slow_seen}")


def main(args=None):
    rclpy.init(args=args)
    node = PartitionedSubscriber()
    executor = flux.ros.PartitionedExecutor()
    executor.add_ros_node(node)
    executor.add_flux(node.fast_sub, node.fast_group)
    executor.add_flux(node.slow_sub, node.slow_group)
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
