#!/usr/bin/env python3
"""Independent best-effort slow consumer; never stalls the production executor."""
import csv
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from liuo_uwb_msgs.msg import UwbObservationBatch


def main():
    rclpy.init();node=Node('liuo_slow_uwb_probe')
    path=Path(os.environ['LIUO_RUN_DIR'])/'profiling/slow_subscriber.csv'
    with path.open('w') as f:
        writer=csv.writer(f);writer.writerow(['batch_seq','source_stamp','received_wall_ns','sleep_s','observations'])
        def receive(b):
            stamp=b.header.stamp.sec+b.header.stamp.nanosec*1e-9
            delay=.1 if stamp<51 else 0.
            writer.writerow([b.batch_seq,stamp,time.monotonic_ns(),delay,len(b.observations)]);f.flush()
            if delay:time.sleep(delay)
        node.create_subscription(UwbObservationBatch,node.declare_parameter('uwb_topic','/liuo/uwb/observations').value,receive,QoSProfile(depth=37,reliability=ReliabilityPolicy.BEST_EFFORT))
        try:rclpy.spin(node)
        except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
        finally:node.destroy_node()

if __name__=='__main__':main()
