#!/usr/bin/env python3
"""Bounded, truth-free identical-input replay for the two external LIO baselines."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

ROOT = Path(__file__).resolve().parents[2]
BASE = Path(os.environ.get('LIUO_DATA_ROOT', '/home/dev/ws_iap/liuo_observation')) / 'baselines'
sys.path.insert(0, str(ROOT / 'tools'))
import run_directory as rd
from raw_identity import hash_message


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def reader(source, topics):
    out = rosbag2_py.SequentialReader()
    out.open(rosbag2_py.StorageOptions(uri=str(source / 'export/capture/raw'), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    # Storage filtering precedes the first read, including legacy S bags that
    # also contain truth. Excluded payloads never enter this replay process.
    out.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return out


def startup(source):
    stream = reader(source, ['/sensors/imu'])
    acc, gyro = [], []
    first = None
    while stream.has_next():
        topic, data, _ = stream.read_next()
        if topic != '/sensors/imu':
            continue
        m = deserialize_message(data, Imu)
        first = stamp(m) if first is None else first
        if stamp(m) - first > 1.5:
            break
        acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        gyro.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    if len(acc) < 200:
        raise RuntimeError('insufficient declared stationary startup IMU')
    a = np.mean(acc, axis=0)
    # Gravity fixes roll/pitch only. Declare yaw zero as the local gauge.
    roll = np.arctan2(a[1], a[2])
    pitch = np.arctan2(-a[0], np.hypot(a[1], a[2]))
    rotation = Rotation.from_euler('xyz', [roll, pitch, 0.])
    return rotation, np.mean(gyro, axis=0), len(acc)


class Adapter:
    def __init__(self, rotation, bias, scan_model="snapshot"):
        if scan_model not in {"snapshot", "rolling"}:
            raise ValueError("unknown scan model")
        self.scan_model = scan_model
        self.rotation = rotation
        self.bias = bias
        self.previous = None
        self.max_ring_residual_deg = 0.

    def imu(self, msg):
        t = stamp(msg)
        if self.previous is not None:
            dt = t - self.previous
            if not 0 < dt < .02:
                raise RuntimeError('non-continuous IMU input')
            w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            self.rotation = self.rotation * Rotation.from_rotvec((w - self.bias) * dt)
        self.previous = t
        q = self.rotation.as_quat()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q.tolist()
        # Derived six-axis attitude, NOT an independent measured orientation.
        msg.orientation_covariance = [0.] * 9
        msg.orientation_covariance[0] = -1.
        return msg

    def cloud(self, msg):
        if msg.header.frame_id != 'lidar':
            raise RuntimeError('unexpected lidar frame')
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        if not np.isfinite(xyz).all():
            raise RuntimeError('nonfinite frozen cloud')
        times = point_cloud2.read_points_numpy(msg, field_names=('time',), skip_nans=False).reshape(-1)
        if self.scan_model == 'rolling':
            if msg.is_bigendian or msg.height != 1 or msg.row_step != msg.width * msg.point_step or len(msg.data) != msg.row_step:
                raise RuntimeError('invalid native rolling cloud layout')
            for name in ('x', 'y', 'z', 'intensity', 'time', 'ring'):
                fields = [f for f in msg.fields if f.name == name]
                datatype, size = (PointField.UINT16, 2) if name == 'ring' else (PointField.FLOAT32, 4)
                if len(fields) != 1 or fields[0].datatype != datatype or fields[0].count != 1 or fields[0].offset + size > msg.point_step:
                    raise RuntimeError('invalid native rolling field: ' + name)
            ring = point_cloud2.read_points(msg, field_names=('ring',), skip_nans=False)['ring']
            if not len(times) or not np.isfinite(times).all() or np.any((times < 0) | (times >= .1)) or np.any(np.diff(times) < 0) or np.any(ring >= 40):
                raise RuntimeError('invalid native rolling time or ring')
            # FAST-LIO Velodyne preprocessing with timestamp_unit=SEC consumes
            # these native float seconds and uint16 rings. No deskew here.
            return msg
        if np.any(times != 0):
            raise RuntimeError('this adapter is snapshot-only')
        elevation = np.rad2deg(np.arctan2(xyz[:, 2], np.linalg.norm(xyz[:, :2], axis=1)))
        ring = np.rint((elevation + 25.) * 39. / 50.).astype(np.int32)
        residual = np.max(np.abs(elevation - (ring * 50. / 39. - 25.)))
        self.max_ring_residual_deg = max(self.max_ring_residual_deg, float(residual))
        if residual > .05 or np.any((ring < 0) | (ring >= 40)):
            raise RuntimeError('cloud does not match declared sensor ray elevation grid')
        dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'], 'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'], 'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})
        points = np.zeros(len(xyz), dtype=dtype)
        for i, key in enumerate(('x', 'y', 'z')):
            points[key] = xyz[:, i]
        points['intensity'] = point_cloud2.read_points_numpy(msg, field_names=('intensity',)).reshape(-1)
        points['ring'] = ring
        fields = [PointField(name=k, offset=dtype.fields[k][1], datatype=PointField.UINT16 if k == 'ring' else PointField.FLOAT32, count=1) for k in dtype.names]
        return PointCloud2(header=msg.header, height=1, width=len(points), fields=fields, is_bigendian=False, point_step=24, row_step=24 * len(points), data=points.tobytes(), is_dense=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('algorithm', choices=['fastlio', 'liosam'])
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--scan-model', choices=['snapshot', 'rolling'], default='snapshot')
    parser.add_argument('--duration', type=float, default=360.)
    args = parser.parse_args()
    if not 0 < args.duration <= (392.6 if args.scan_model == 'rolling' else 540):
        parser.error('duration exceeds declared scene bound')
    if args.scan_model == 'rolling' and args.algorithm != 'fastlio':
        parser.error('rolling comparison supports FAST-LIO2 only')
    if rd._git_value('status', '--porcelain'):
        raise RuntimeError('commit changes before baseline replay')
    run = rd.resolve_run_directory(entrypoint='external_lio', scenario='frozen_' + args.scan_model)
    print(run, flush=True)
    os.environ.update(LIUO_RUN_DIR=str(run), ROS_LOG_DIR=str(run / 'runtime/ros'), ROS_DOMAIN_ID=str(100 + os.getpid() % 100), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', OMP_NUM_THREADS='4')
    children, handles = [], []
    lifecycle, failure = 'failed', None
    node = None
    poses, latency = [], []
    counts = dict(imu=0, cloud=0, clock=0)
    hashes = {key: hashlib.sha256() for key in ('imu', 'cloud')}
    semantic_hashes = {key: hashlib.sha256() for key in ('imu', 'cloud')}
    raw_hashes = {key: hashlib.sha256() for key in ('imu', 'cloud')}
    observed_hashes = {key: hashlib.sha256() for key in ('imu', 'cloud')}
    observed_counts = dict(imu=0, cloud=0)
    publish_wall_s = None
    adapter = None
    identity = dict(algorithm=args.algorithm, source=str(args.source.resolve()), duration_s=args.duration, uwb_published=False, truth_published=False, commands=[], binaries={})
    try:
        if shutil.disk_usage(run).free < 8 * 1024**3:
            raise RuntimeError('disk reserve below 8 GiB')
        config = ROOT / 'tools/baselines/config' / (args.algorithm + ('_rolling' if args.scan_model == 'rolling' else '') + '.yaml')
        snapshot = run / 'metadata/config' / config.name
        snapshot.write_bytes(config.read_bytes())
        rd.register_config_snapshot(run, snapshot)
        identity['config_sha256'] = hashlib.sha256(snapshot.read_bytes()).hexdigest()
        identity['sources'] = json.loads((ROOT / 'tools/baselines/sources.json').read_text())
        identity['patch_sha256'] = hashlib.sha256((ROOT / 'tools/baselines/patches' / (args.algorithm + '.patch')).read_bytes()).hexdigest()
        identity['capture_sha256'] = {p.name: hashlib.file_digest(p.open('rb'), 'sha256').hexdigest() for p in (args.source / 'export/capture/raw').glob('*.mcap')}
        initial, bias, n = startup(args.source)
        identity['derived_attitude'] = dict(stationary_samples=n, gyro_bias= bias.tolist(), initial_quaternion=initial.as_quat().tolist(), independent_orientation_measurement=False, imu_rpy_weight=0)
        adapter = Adapter(initial, bias, args.scan_model)
        identity['scan_model'] = args.scan_model
        identity['rate'] = 1.
        scene_artifacts = args.source / 'metadata/manifests/scene_artifacts.json'
        if scene_artifacts.exists():
            declared = json.loads(scene_artifacts.read_text())['files']
            for name, digest in identity['capture_sha256'].items():
                if declared['export/capture/raw/' + name]['sha256'] != digest:
                    raise RuntimeError('frozen scene capture hash mismatch')
        rclpy.init()
        node = Node('baseline_replay_and_recorder')
        pubs = {key: node.create_publisher(cls, topic, 2048 if key == 'imu' else 32) for key, cls, topic in [('imu', Imu, '/benchmark/imu'), ('cloud', PointCloud2, '/benchmark/points'), ('clock', Clock, '/clock')]}
        def audit(key, msg):
            observed_counts[key] += 1
            hash_message(observed_hashes[key], key, msg)
        audit_subscriptions = [node.create_subscription(cls, topic, lambda msg, key=key: audit(key, msg), 2048)
            for key, cls, topic in [('imu', Imu, '/benchmark/imu'), ('cloud', PointCloud2, '/benchmark/points')]]
        last_output = [time.monotonic()]
        latest_sensor = [0.]
        r_bl = Rotation.from_euler('z', .05)
        t_lb = -r_bl.inv().apply([.2, 0., .1])

        def record(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            pos = np.array([p.x, p.y, p.z])
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
            if args.algorithm == 'liosam':
                pos += rotation.apply(t_lb)
                rotation = rotation * r_bl.inv()
            poses.append([stamp(msg), *pos, *rotation.as_quat()])
            last_output[0] = time.monotonic()
            latency.append([stamp(msg), latest_sensor[0] - stamp(msg)])

        output_topic = '/Odometry' if args.algorithm == 'fastlio' else '/lio_sam/mapping/odometry'
        sub = node.create_subscription(Odometry, output_topic, record, qos_profile_sensor_data)
        executables = [('fast_lio', 'fastlio_mapping')] if args.algorithm == 'fastlio' else [('lio_sam', 'lio_sam_' + suffix) for suffix in ('imageProjection', 'featureExtraction', 'imuPreintegration', 'mapOptimization')]
        for package, executable in executables:
            binary = BASE / 'install' / package / 'lib' / package / executable
            command = [str(binary), '--ros-args', '--params-file', str(snapshot)]
            identity['commands'].append(command)
            identity['binaries'][executable] = hashlib.file_digest(binary.open('rb'), 'sha256').hexdigest()
            f = (run / 'runtime' / (executable + '.log')).open('w')
            handles.append(f)
            children.append(subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, cwd=run))
        rd.write_subordinate_manifest(run, 'baseline_input', identity)
        deadline = time.monotonic() + 30
        while any(pubs[k].get_subscription_count() < 2 for k in ('imu', 'cloud')) or pubs['clock'].get_subscription_count() == 0:
            if time.monotonic() > deadline or any(p.poll() is not None for p in children):
                raise RuntimeError('baseline subscribers failed to become ready')
            rclpy.spin_once(node, timeout_sec=.01)
        matched_until = time.monotonic() + 1
        while time.monotonic() < matched_until:
            rclpy.spin_once(node, timeout_sec=.01)
        lidar_topic = '/sensors/lidar' if args.scan_model == 'rolling' else '/sensors/lidar/points'
        types = {'/sensors/imu': ('imu', Imu), lidar_topic: ('cloud', PointCloud2), '/clock': ('clock', Clock)}
        stream = reader(args.source, types)
        identity['sensor_access_boundary'] = dict(
            selected_topics=list(types), startup_topics=['/sensors/imu'],
            excluded_topics=[t.name for t in stream.get_all_topics_and_types() if t.name not in types],
            storage_filter_before_first_read=True)
        identity['end_exclusive'] = args.scan_model == 'snapshot'
        rd.write_subordinate_manifest(run, 'baseline_input', identity)
        duration_ns = round(args.duration * 1e9)
        first_record, wall_start = None, time.monotonic()
        last_output[0] = wall_start
        while stream.has_next():
            topic, data, ts = stream.read_next()
            if topic not in types:
                continue
            first_record = ts if first_record is None else first_record
            elapsed = (ts - first_record) * 1e-9
            elapsed_ns = ts - first_record
            if elapsed_ns > duration_ns or (identity['end_exclusive'] and elapsed_ns == duration_ns):
                break
            target = wall_start + elapsed
            while time.monotonic() < target:
                rclpy.spin_once(node, timeout_sec=min(.002, max(0., target - time.monotonic())))
            if time.monotonic() - wall_start > args.duration + 120 or time.monotonic() - last_output[0] > 45:
                raise RuntimeError('bounded wall/no-output watchdog expired')
            if any(p.poll() is not None for p in children):
                raise RuntimeError('baseline process exited before input completion')
            key, cls = types[topic]
            msg = deserialize_message(data, cls)
            if key in raw_hashes:
                hash_message(raw_hashes[key], key, msg)
            if key == 'imu' and args.algorithm == 'liosam':
                msg = adapter.imu(msg)
            elif key == 'cloud':
                msg = adapter.cloud(msg)
            elif key == 'clock':
                latest_sensor[0] = msg.clock.sec + msg.clock.nanosec * 1e-9
            if key in hashes:
                hashes[key].update(serialize_message(msg))
                hash_message(semantic_hashes[key], key, msg)
            pubs[key].publish(msg)
            counts[key] += 1
            rclpy.spin_once(node, timeout_sec=0.)
            if time.monotonic() - target > .25:
                raise RuntimeError('1x replay deadline exceeded 250 ms')
            if key == 'cloud' and shutil.disk_usage(run).free < 8 * 1024**3:
                raise RuntimeError('disk reserve below 8 GiB')
        publish_wall_s = time.monotonic() - wall_start
        drain = time.monotonic() + 3
        while time.monotonic() < drain:
            rclpy.spin_once(node, timeout_sec=.01)
        if not poses:
            raise RuntimeError('no estimated poses')
        if counts['cloud'] == 0 or counts['imu'] == 0:
            raise RuntimeError('incomplete sensor topic selection')
        if any(observed_counts[k] != counts[k] or observed_hashes[k].hexdigest() != semantic_hashes[k].hexdigest() for k in observed_counts):
            raise RuntimeError('adapted input observer count/hash mismatch')
        lifecycle = 'completed'
    except (Exception, KeyboardInterrupt) as exc:
        failure = repr(exc)
        print(failure, file=sys.stderr, flush=True)
    finally:
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
                lifecycle, failure = 'failed', 'owned child required forced shutdown'
        for f in handles:
            f.close()
        if args.algorithm == 'fastlio':
            # Upstream also writes auxiliary debug files under its compiled-in
            # source root. Preserve each run's copy before a subsequent replay.
            native = BASE / 'sources/fastlio/Log'
            destination = run / 'runtime/fastlio_native_debug'
            destination.mkdir(exist_ok=True)
            for path in native.glob('*.txt'):
                shutil.copy2(path, destination / path.name)
        if node:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        export = run / 'export/odometry'
        export.mkdir(parents=True, exist_ok=True)
        with (export / 'trajectory.csv').open('w') as f:
            writer = csv.writer(f); writer.writerow(['stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']); writer.writerows(poses)
        with (export / 'delivery_latency.csv').open('w') as f:
            writer = csv.writer(f); writer.writerow(['stamp', 'latest_input_stamp_minus_estimate_s']); writer.writerows(latency)
        summary = dict(failure=failure, counts=counts, adapted_sha256={k: h.hexdigest() for k, h in hashes.items()}, poses=len(poses), max_ring_residual_deg=adapter.max_ring_residual_deg if adapter else None, exit_codes=[p.returncode for p in children])
        summary.update(raw_semantic_sha256={k:h.hexdigest() for k,h in raw_hashes.items()}, observed_counts=observed_counts, observed_semantic_sha256={k:h.hexdigest() for k,h in observed_hashes.items()}, publish_wall_s=publish_wall_s)
        summary['adapted_semantic_sha256'] = {k:h.hexdigest() for k,h in semantic_hashes.items()}
        rd.write_subordinate_manifest(run, 'baseline_result', summary)
        if failure:
            rd.write_subordinate_manifest(run, 'input_failure', {'reason': failure})
        rd.finalize_run(run, lifecycle=lifecycle)
        print(json.dumps(summary), flush=True)
    return 0 if lifecycle == 'completed' else 1


if __name__ == '__main__':
    sys.exit(main())
