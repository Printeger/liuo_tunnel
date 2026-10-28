#!/usr/bin/env python3
"""Independent deliberately slow best-effort A6 observer; never stalls estimator."""
import csv
import hashlib
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.serialization import serialize_message
from liuo_tunnel_msgs.msg import StaticAnchorRecord


def main():
    rclpy.init()
    node = Node('liuo_slow_static_probe')
    with (Path(os.environ['LIUO_RUN_DIR'])/'profiling/slow_static_subscriber.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['stamp','sha256','receive_wall_ns','sleep_s'])
        def receive(message):
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            delay = .1 if stamp < 51 else 0.
            writer.writerow([stamp,hashlib.sha256(serialize_message(message)).hexdigest(),time.monotonic_ns(),delay])
            stream.flush()
            if delay:
                time.sleep(delay)
        node.create_subscription(StaticAnchorRecord,'/liuo/uwb/static_edges',receive,
            QoSProfile(depth=37,reliability=ReliabilityPolicy.BEST_EFFORT))
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
            pass
        finally:
            node.destroy_node()


if __name__ == '__main__':
    main()
