"""Independent bag reader: bytes, timing, physics and source identities. No localization."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from wire import semantic_bytes
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def seconds(stamp):
  return stamp.sec + stamp.nanosec / 1e9 - 1


def read_bag(path):
  reader = rosbag2_py.SequentialReader()
  reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
  types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
  while reader.has_next():
    topic, data, t = reader.read_next()
    yield topic, data, t, deserialize_message(data, types[topic])


def audit(run):
  run = Path(run)
  summary = json.loads((run / 'export/analysis/generation.json').read_text())
  cfg = json.loads((run / 'metadata/config/scene.json').read_text())
  truth = np.load(run / 'export/simulation/trajectory.npz')
  allowed = {'/sensors/lidar', '/sensors/imu', '/liuo/uwb/observations', '/clock'}
  counts = {t: 0 for t in allowed}
  hashes = {t: hashlib.sha256() for t in allowed}
  last = -1
  scan_starts = []
  imu_times = []
  identities = set()
  time_min = float('inf')
  time_max = -1
  max_age = 0.
  native_points = 0
  batch_last = 0
  imu_error = []
  gyro_error = []
  ring_seen = set()
  # Parse raw PointCloud2 directly using field offsets, not generator dtype.
  dtype = np.dtype({'names': ['x', 'y', 'z', 'ring', 'time'], 'formats': ['<f4', '<f4', '<f4', '<u2', '<f4'], 'offsets': [0, 4, 8, 16, 20], 'itemsize': 24})
  physical = {}
  with (run / 'export/simulation/physical_uwb.csv').open() as f:
    for row in csv.DictReader(f):
      if row['valid'] == '1':
        physical[int(row['source_seq'])] = row
  for topic, data, t, msg in read_bag(run / 'export/capture/raw'):
    assert topic in allowed, 'truth or unknown topic in raw bag'
    assert t >= last, 'delivery order'
    last = t
    assert semantic_bytes(deserialize_message(serialize_message(msg), type(msg))) == semantic_bytes(msg), 'serialization semantic roundtrip'
    counts[topic] += 1
    hashes[topic].update(semantic_bytes(msg))
    if topic == '/sensors/lidar':
      ts = seconds(msg.header.stamp)
      scan_starts.append(ts)
      assert msg.point_step == 24 and msg.row_step == 24 * msg.width and msg.height == 1 and not msg.is_bigendian
      assert {f.name: (f.offset, f.datatype) for f in msg.fields} == {'x': (0, 7), 'y': (4, 7), 'z': (8, 7), 'intensity': (12, 7), 'ring': (16, 4), 'time': (20, 7)}
      points = np.frombuffer(msg.data, dtype=dtype)
      assert np.isfinite(np.c_[points['x'], points['y'], points['z'], points['time']]).all()
      rel = points['time'].astype(float)
      assert np.all(np.diff(rel) > 0) and np.all((rel >= 0) & (rel < .1))
      ray_index = np.rint(rel * 204800).astype(int)
      assert np.all(ray_index % 40 == points['ring']), 'native ring lost'
      assert np.all(np.abs(rel - ray_index / 204800) < 1e-8)
      assert t / 1e9 - 1 >= ts + rel.max() - 1e-8
      time_min = min(time_min, ts + rel.min())
      time_max = max(time_max, ts + rel.max())
      native_points += len(points)
      ring_seen.update(map(int, points['ring']))
    elif topic == '/sensors/imu':
      ts = seconds(msg.header.stamp)
      imu_times.append(ts)
      i = len(imu_times) - 1
      assert abs(ts - truth['t'][i]) < 1e-8 and msg.orientation_covariance[0] == -1
      assert msg.orientation.x == msg.orientation.y == msg.orientation.z == msg.orientation.w == 0
      observed = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
      gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
      imu_error.append(observed - truth['force'][i] - truth['acc_bias'][i])
      gyro_error.append(gyro - truth['omega'][i] - truth['gyro_bias'][i])
    elif topic == '/liuo/uwb/observations':
      assert msg.batch_seq > batch_last
      batch_last = msg.batch_seq
      for o in msg.observations:
        key = (msg.driver_session_id, o.reporter_id, o.source_session_id, o.source_frame_seq, o.node_b)
        assert key not in identities, 'repeated source frame was emitted'
        identities.add(key)
        assert o.source_frame_seq_valid and o.source_local_time_valid and o.source_system_time_valid
        assert o.time_basis == (1 if o.kind == 0 else 2)
        assert o.node_a == o.reporter_id and o.node_a != o.node_b and msg.receiver_id == 0
        assert 0 <= o.shard_index < o.shard_count and o.shard_count == 1
        assert not o.rssi_valid and np.isnan(o.fp_rssi) and np.isnan(o.rx_rssi)
        assert o.pair_measurement_freshness_verified
        source = seconds(o.source_frame_stamp)
        received = seconds(o.received_stamp)
        published = seconds(msg.header.stamp)
        assert source <= received < published and abs(published - (t / 1e9 - 1)) < 1e-8
        assert abs(o.source_system_ms / 1000 - (source + 1)) <= .000501
        row = physical[o.source_frame_seq]
        assert abs(float(row['t']) - source) < 1e-8
        assert int(row['a']) == o.node_a and int(row['b']) == o.node_b
        assert abs(float(row['distance']) - o.distance_m) < .0001
        if o.kind:
          assert o.source_hold_age_valid and abs((received - .012 - source) - o.source_hold_age_ms / 1000) < .000501
          max_age = max(max_age, received - source)
        else:
          assert not o.source_hold_age_valid and o.source_hold_age_ms == 65535
  assert counts == summary['counts']
  assert {t: h.hexdigest() for t, h in hashes.items()} == summary['raw_semantic_sha256']
  assert len(scan_starts) == summary['frames'] and np.allclose(np.diff(scan_starts), .1, atol=1e-8)
  assert np.allclose(np.diff(imu_times), .005, atol=1e-8)
  assert imu_times[0] <= time_min and imu_times[-1] >= time_max
  assert ring_seen == set(range(40)) and native_points == summary['points']
  # IMU theory independently from finite differences of saved world velocity/rotation.
  dt = .005
  a_fd = (truth['v'][2:] - truth['v'][:-2]) / (2 * dt)
  f_fd = np.einsum('nji,nj->ni', truth['R'][1:-1], a_fd - [0, 0, -9.80665])
  force_error = float(np.max(np.abs(f_fd - truth['force'][1:-1])))
  assert force_error < .002
  imu_sigma = np.std(imu_error, axis=0)
  gyro_sigma = np.std(gyro_error, axis=0)
  # Statistical checks only on sufficient samples, preregistered 10% tolerance.
  if len(imu_times) >= 2000:
    assert np.all(np.abs(imu_sigma / cfg['imu']['sigma_acc_sample'] - 1) < .1)
    assert np.all(np.abs(gyro_sigma / cfg['imu']['sigma_gyro_sample'] - 1) < .1)
  slots = []
  with (run / 'export/simulation/physical_uwb.csv').open() as f:
    for row in csv.DictReader(f):
      slots.append(float(row['t']))
  assert np.all(np.diff(slots) >= .004 - 1e-8), 'physical exchange budget'
  installed = json.loads((run / 'metadata/config/installation_metadata.json').read_text())
  assert all(set(a) == {'id', 'side', 'lateral_offset_m'} for a in installed['anchors'])
  assert len(installed['anchors']) == 18
  result = dict(pass_=True,
                scope='scene/raw sensor data only; no localization',
                counts=counts,
                unique_uwb=len(identities),
                imu_coverage=[imu_times[0], imu_times[-1]],
                ray_time_range=[time_min, time_max],
                imu_force_fd_error=force_error,
                acc_noise_std=imu_sigma.tolist(),
                gyro_noise_std=gyro_sigma.tolist(),
                maximum_received_source_age_s=max_age,
                point_count=native_points,
                semantic_sha256={
                    t: h.hexdigest() for t, h in hashes.items()
                })
  (run / 'export/analysis/bag_audit.json').write_text(json.dumps(result, indent=2) + '\n')
  return result
