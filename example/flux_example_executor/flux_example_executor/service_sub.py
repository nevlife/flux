#!/usr/bin/env python3
"""Serve flux frames and a ROS service from one spin loop.

ros2 run flux_example_executor service_sub.py
ros2 service call /frame_count std_srvs/srv/Trigger

The C++ twin makes a sharper claim: there the tick is 3 s and the request is still answered at
once, because the service's readiness is armed on the same io_uring as the flux channel. rclpy
gives Python no such hook, so here the service is served by the rclpy executor this one wraps and
the point is only that the two transports share a thread.
"""

import flux.ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs_flux.image import Image
from std_srvs.srv import Trigger


class ServiceSubscriber(Node):

    TOPIC = "image"
    SERVICE = "frame_count"

    def __init__(self):
        super().__init__("flux_service_sub")
        self.seen = 0
        self.subscription = flux.ros.Subscription(
            self, self.TOPIC, callback=self.on_frame, fingerprint=Image.FINGERPRINT__
        )
        self.service = self.create_service(Trigger, self.SERVICE, self.on_request)

    def on_frame(self, view):
        self.seen += 1

    def on_request(self, _request, response):
        response.success = True
        response.message = f"{self.seen} frames"
        self.get_logger().info(f"answered: {response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceSubscriber()
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
