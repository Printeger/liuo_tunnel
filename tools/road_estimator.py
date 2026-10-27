#!/usr/bin/env python3
"""Owned 1x replay of native road sensor bags; no truth is read by this runner."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import run_directory as rd
from raw_identity import hash_message

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=20.)
    args = parser.parse_args()
    if not 3 <= args.seconds <= 392.6:
        raise ValueError('replay duration must be in [3,392.6] seconds')
    if rd._git_value('status', '--porcelain'):
        raise RuntimeError('commit changes before production replay')
    source = args.source_run.resolve(strict=True)
    run = rd.resolve_run_directory(entrypoint='road_estimator', scenario='road_tunnel_7km')
    print(run, flush=True)
    manifest = run / 'metadata/run_manifest.json'
    meta = json.loads(manifest.read_text())
    meta.update(run_class='validation', retention_class='protected', retained_input_run_ids=[source.name],
                modules=['odometry', 'observer', 'road_sensor_replay'])
    rd._atomic_write_json(manifest, meta)
    children, handles = [], []
    node = None
    lifecycle = 'failed'
    try:
        if shutil.disk_usage(run).free < 8 * 1024**3:
            raise RuntimeError('disk reserve below 8 GiB')
        config_path = run / 'metadata/config/phase1.json'
        shutil.copyfile(args.config, config_path)
        rd.register_config_snapshot(run, config_path)
        config = json.loads(config_path.read_text())
        if config['odometry']['backend'] != 'gpu' or config['uwb']['mode'] != 'record_only':
            raise ValueError('first road smoke requires GPU LIO-only; unknown-A mode is not yet connected here')
        if config['scan_model'] != 'rolling':
            raise ValueError('native road bag must retain rolling scan model')
        bag = source / 'export/capture/raw'
        files = list(bag.glob('*.mcap'))
        if len(files) != 1:
            raise ValueError('expected one frozen road MCAP')
        source_artifacts = json.loads((source / 'metadata/manifests/scene_artifacts.json').read_text())['files']
        sha = digest(files[0])
        if sha != source_artifacts[str(files[0].relative_to(source))]['sha256']:
            raise RuntimeError('frozen road MCAP content mismatch')
        env = os.environ.copy()
        env.update(LIUO_RUN_DIR=str(run), ROS_DOMAIN_ID=str(30 + os.getpid() % 180), ROS_LOG_DIR=str(run / 'runtime/ros'), ROS_LOCALHOST_ONLY='1')
        os.environ.update({k: env[k] for k in ['LIUO_RUN_DIR', 'ROS_DOMAIN_ID', 'ROS_LOG_DIR', 'ROS_LOCALHOST_ONLY']})
        binary = Path(env['LIUO_INSTALL_BASE']) / 'liuo_tunnel/lib/liuo_tunnel/liuo_tunnel_node'
        linked = subprocess.check_output(['ldd', str(binary)], text=True, env=env)
        if '/src/iap/' in linked or '/src/demo/' in linked:
            raise RuntimeError('forbidden runtime dependency')
        libraries = {}
        for line in linked.splitlines():
            parts = line.split('=>')
            if len(parts) == 2:
                path = Path(parts[1].strip().split()[0])
                if path.is_absolute() and path.is_file(): libraries[str(path)] = digest(path)
        rd.write_subordinate_manifest(run, 'input_identity', dict(source_run=str(source), raw_sha256=sha,
            config_sha256=digest(config_path), binary_sha256=digest(binary), shared_libraries=libraries,
            scan_model='rolling', state_reference='frame_start', rate=1., seconds=args.seconds))
        commands = [[str(binary), '--ros-args', '-p', 'config_file:=' + str(config_path), '-p', 'use_sim_time:=true'],
                    [sys.executable, str(ROOT / 'tools/observe.py'), '--ros-args', '-p', 'use_sim_time:=true']]
        for name, command in zip(['odometry', 'observer'], commands):
            handle = (run / 'runtime' / (name + '.log')).open('w')
            handles.append(handle)
            child = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT)
            children.append(child)
            rd._atomic_write_json(run / 'metadata/processes' / (name + '_launch.json'), dict(pid=child.pid, argv=command))
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from nav_msgs.msg import Odometry
        import rosbag2_py
        rclpy.init()
        node = rclpy.create_node('road_sensor_replay')
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        kinds = {'/sensors/lidar': 'cloud', '/sensors/imu': 'imu', '/liuo/uwb/observations': 'uwb', '/clock': 'clock'}
        publishers, types = {}, {}
        for topic in reader.get_all_topics_and_types():
            if topic.name not in kinds:
                raise ValueError('non-sensor input forbidden: ' + topic.name)
            types[topic.name] = get_message(topic.type)
            publishers[topic.name] = node.create_publisher(types[topic.name], topic.name,
                QoSProfile(depth=2048, reliability=ReliabilityPolicy.RELIABLE if topic.name == '/clock' else ReliabilityPolicy.BEST_EFFORT))
        if set(publishers) != set(kinds): raise ValueError('incomplete road sensor topics')
        progress = [time.monotonic(), None]
        def odometry(message):
            progress[:] = [time.monotonic(), message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec]
        subscription = node.create_subscription(Odometry, '/liuo/odom', odometry, 100)
        ready = time.monotonic() + 20
        while any(publishers[t].get_subscription_count() < 2 for t in kinds if t != '/clock'):
            if time.monotonic() > ready or any(c.poll() is not None for c in children): raise RuntimeError('road replay consumers not ready')
            rclpy.spin_once(node, timeout_sec=.01)
        # Warm deserialization before the first timed delivery.
        prefix, prepared = [], set()
        while prepared != set(types):
            if not reader.has_next(): raise RuntimeError('missing source topic')
            item = reader.read_next(); prefix.append(item)
            if item[0] not in prepared:
                deserialize_message(item[1], types[item[0]]); prepared.add(item[0])
            if len(prefix) > 256: raise RuntimeError('startup buffer limit')
        import itertools
        def remaining():
            while reader.has_next(): yield reader.read_next()
        counts = {k: 0 for k in kinds.values()}
        hashes = {k: hashlib.sha256() for k in counts}
        cloud_stamps = []
        first = prefix[0][2]
        started = time.monotonic(); progress[0] = started
        for topic, data, timestamp in itertools.chain(prefix, remaining()):
            elapsed = (timestamp - first) * 1e-9
            if elapsed > args.seconds + 1e-8: break
            deadline = started + elapsed
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(.005, max(0., deadline-time.monotonic())))
            if any(c.poll() is not None for c in children): raise RuntimeError('owned child exited during replay')
            if time.monotonic() - started > args.seconds + 30 or time.monotonic() - progress[0] > 30:
                raise TimeoutError('road replay wall/no-odometry-progress limit')
            if shutil.disk_usage(run).free < 8 * 1024**3: raise RuntimeError('disk reserve')
            message = deserialize_message(data, types[topic])
            publishers[topic].publish(message)
            kind = kinds[topic]; counts[kind] += 1; hash_message(hashes[kind], kind, message)
            if kind == 'cloud': cloud_stamps.append(message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec)
            rclpy.spin_once(node, timeout_sec=0.)
            if time.monotonic() - deadline > .25: raise TimeoutError('1x replay deadline exceeded 250 ms')
        for _ in range(200): rclpy.spin_once(node, timeout_sec=.01)
        for child in children: child.send_signal(signal.SIGINT)
        for child in children:
            if child.wait(timeout=15) not in [0, -signal.SIGINT]: raise RuntimeError('owned child failed at shutdown')
        audit = json.loads((run / 'metadata/manifests/observation_audit.json').read_text())
        expected_hashes = {k: v.hexdigest() for k, v in hashes.items()}
        if audit['counts'] != counts or audit['raw_sha256'] != expected_hashes or audit['violations']:
            raise RuntimeError('raw observer count/hash/field audit mismatch: ' + str(audit['violations']))
        trajectory = list(csv.DictReader((run / 'export/odometry/trajectory.csv').open()))
        if not trajectory or abs(float(trajectory[-1]['stamp']) - cloud_stamps[-1]) > 1e-6:
            raise RuntimeError('estimator did not consume final scan')
        if (run / 'runtime/odometry_failure.json').exists(): raise RuntimeError('estimator failure artifact')
        report = dict(scope='native rolling GPU LIO smoke, not curved self-calibration or localization acceptance',
                      result='PASS', raw_counts=counts, raw_sha256=expected_hashes, output_count=len(trajectory),
                      first_output=float(trajectory[0]['stamp']), last_output=float(trajectory[-1]['stamp']), source_run=str(source))
        rd._atomic_write_json(run / 'export/analysis/rolling_smoke.json', report)
        print(json.dumps(report, indent=2), flush=True)
        lifecycle = 'completed'
    except BaseException as error:
        rd._atomic_write_json(run / 'runtime/owner_failure.json', dict(error=str(error)))
        raise
    finally:
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try: child.wait(timeout=15)
                except subprocess.TimeoutExpired: child.kill(); child.wait(timeout=5)
        for handle in handles: handle.close()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        rd.finalize_run(run, lifecycle=lifecycle)


if __name__ == '__main__':
    main()
