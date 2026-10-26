"""clock_pub.py topic count period: after a line on stdin, `count` frames of monotonic_ns, from
another process so they keep coming whatever the subscriber's GIL does."""

import sys
import time

import numpy as np
import rclpy

import flux

rclpy.init()
node = rclpy.create_node("flux_clock_pub")
pub = flux.Publisher(node, sys.argv[1], slot_size=4096, slot_count=8)
print("ready", flush=True)
sys.stdin.readline()
for _ in range(int(sys.argv[2])):
    pub.publish(np.array([time.monotonic_ns()], np.int64))
    time.sleep(float(sys.argv[3]))
