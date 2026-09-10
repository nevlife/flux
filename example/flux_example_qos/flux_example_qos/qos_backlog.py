#!/usr/bin/env python3
"""depth 8 with transient_local(4): drain a backlog, and replay what the ring still holds.

ros2 run flux_example_qos qos_backlog.py
"""

import time

import flux.ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs_flux.image import Image


class BacklogSubscriber(Node):

    TOPIC = "image"
    WORK_SEC = 0.05

    def __init__(self):
        super().__init__("flux_qos_backlog")
        self.seen = 0
        self.subscription = flux.ros.Subscription(
            self,
            self.TOPIC,
            callback=self.on_frame,
            fingerprint=Image.FINGERPRINT__,
            qos=flux.QoS(depth=8, durability=flux.TransientLocal(4), max_borrow=4),
        )
        self.timer = self.create_timer(1.0, self.report)

    def on_frame(self, _view):
        time.sleep(self.WORK_SEC)
        self.seen += 1

    def report(self):
        r = self.subscription.refused
        self.get_logger().info(
            f"seen {self.seen}  lost {self.subscription.lost}  "
            f"refused {r.total} (max_borrow {r.max_borrow})  "
            f"can_borrow {self.subscription.can_borrow}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BacklogSubscriber()
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
