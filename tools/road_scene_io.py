#!/usr/bin/env python3
"""Sensor-only bag replay and native-ring/time format validation. Reads no truth/config."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'packages/liuo_tunnel_sim/road_scene'))
from audit import read_bag
from wire import semantic_bytes


def native_fastlio_cloud(msg):
  """Already canonical PointCloud2 seconds/ring layout; never infer ring or deskew."""
  expected = {'x': (0, 7), 'y': (4, 7), 'z': (8, 7), 'intensity': (12, 7), 'ring': (16, 4), 'time': (20, 7)}
  if msg.point_step != 24 or {f.name: (f.offset, f.datatype) for f in msg.fields} != expected:
    raise ValueError('not native rolling layout')
  return msg


def replay(bag, report, duration=2., cpp=None, rate=1.):
  if report.exists():
    raise FileExistsError('preserve existing wire-check evidence')
  import rclpy
  from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
  # Only short wire smoke is buffered; full replay streams from the original bag.
  order = ['/sensors/lidar', '/sensors/imu', '/liuo/uwb/observations', '/clock']
  selected = []
  first = None
  counts = {t: 0 for t in order}
  for topic, data, t, msg in read_bag(bag):
    if topic not in counts:
      raise RuntimeError('non-sensor topic forbidden')
    if first is None:
      first = t
    if (t - first) / 1e9 > duration:
      break
    if topic == '/sensors/lidar':
      assert semantic_bytes(native_fastlio_cloud(msg)) == semantic_bytes(msg)
    selected.append((topic, t, msg))
    counts[topic] += 1
  env = os.environ.copy()
  env['ROS_LOG_DIR'] = os.environ.get('ROS_LOG_DIR', str(report.parent / 'ros'))
  Path(env['ROS_LOG_DIR']).mkdir(exist_ok=True)
  child = None
  rclpy.init()
  node = rclpy.create_node('road_scene_sensor_replay')
  qos = QoSProfile(depth=4096, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
  publishers = {}
  for topic, _, msg in selected:
    if topic not in publishers:
      publishers[topic] = node.create_publisher(type(msg), topic, qos)
  try:
    if cpp:
      command = [str(cpp), *[str(counts[t]) for t in order], str(report)]
      child = subprocess.Popen(command, env=env)
      adopted = os.environ.get('LIUO_RUN_DIR')
      if adopted:
        (Path(adopted) / 'metadata/processes/wire_checker.json').write_text(json.dumps(dict(pid=child.pid, argv=command, adopted_run_dir=adopted), indent=2))
      deadline = time.monotonic() + 10
      while not all(p.get_subscription_count() >= 1 for p in publishers.values()):
        if time.monotonic() > deadline:
          raise TimeoutError('independent checker discovery')
        rclpy.spin_once(node, timeout_sec=.02)
    # DDS graph discovery can precede reader matching; settle before frozen playback.
    time.sleep(1.)
    start = time.monotonic()
    for topic, t, msg in selected:
      due = start + (t - first) / 1e9 / rate
      while time.monotonic() < due:
        time.sleep(min(.005, due - time.monotonic()))
      publishers[topic].publish(msg)
    if child and child.wait(timeout=10) != 0:
      raise RuntimeError('C++ wire check failed')
    if not child:
      report.write_text(json.dumps(dict(published=counts, scope='sensor-only replay; no locator'), indent=2))
  finally:
    if child and child.poll() is None:
      child.terminate()
      child.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
  return counts


if __name__ == '__main__':
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('bag', type=Path)
  p.add_argument('--report', required=True, type=Path)
  p.add_argument('--seconds', type=float, default=2)
  p.add_argument('--cpp', type=Path)
  p.add_argument('--rate', type=float, default=1)
  a = p.parse_args()
  print(replay(a.bag, a.report, a.seconds, a.cpp, a.rate))
