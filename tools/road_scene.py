#!/usr/bin/env python3
"""Independent scene owner. No estimator modules or baseline runner imports."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import sys
import time
import traceback
import signal
import platform
import importlib.metadata

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'packages/liuo_tunnel_sim/road_scene'
sys.path.insert(0, str(MODEL))
import numpy as np
from model import Road, Motion
from geometry import Geometry
from checks import validate
import run_directory as rd


def write_json(path, value):
  path.write_text(json.dumps(value, indent=2) + '\n')


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--check', action='store_true')
  parser.add_argument('--seconds', type=float, help='short capture; omitted means full route')
  parser.add_argument('--start', type=float, default=0, help='short test absolute trajectory time')
  parser.add_argument('--allow-dirty', action='store_true', help='development fixtures only')
  args = parser.parse_args()
  signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt('owned scene timeout/termination')))
  configured_root = Path(os.environ.get('LIUO_RUN_ROOT', ''))
  if not configured_root.is_absolute() or not configured_root.resolve().is_relative_to('/home/dev/ws_iap'):
    raise RuntimeError('invoke through data-run; data-disk LIUO_RUN_ROOT required')
  cfg = json.loads((MODEL / 'scene.json').read_text())
  if not args.allow_dirty and rd._git_value('status', '--porcelain'):
    raise RuntimeError('formal generation requires clean committed source')
  run = rd.resolve_run_directory(entrypoint='road_scene', scenario='road_tunnel_7km')
  os.environ['LIUO_RUN_DIR'] = str(run)
  os.environ['ROS_LOG_DIR'] = str(run / 'runtime/ros')
  print('LIUO_RUN_DIR=' + str(run), flush=True)
  started = time.monotonic()
  manifest = run / 'metadata/run_manifest.json'
  value = json.loads(manifest.read_text())
  value.update(modules=['scene_generator', 'independent_checks'],
               run_class='formal' if not args.allow_dirty else 'development',
               retention_class='protected',
               localization='NOT_RUN')
  rd._atomic_write_json(manifest, value)
  try:
    from configuration import validate_configuration
    validate_configuration(cfg)
    write_json(run / 'metadata/config/scene.json', cfg)
    rd.register_config_snapshot(run, run / 'metadata/config/scene.json')
    write_json(run / 'metadata/processes/owner.json', dict(pid=os.getpid(), argv=sys.argv, adopted_run_dir=str(run), executable=sys.executable, algorithm_processes_started=[]))
    import subprocess
    dependencies = {name: importlib.metadata.version(name) for name in ['numpy', 'scipy', 'trimesh', 'embreex', 'matplotlib']}
    dependencies['python'] = platform.python_version()
    shared = ROOT.parent / 'nlink_parser_uwb/packages/liuo_uwb_msgs/msg'
    dependencies['shared_msg_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in shared.glob('*.msg')}
    dependencies['driver_commit'] = subprocess.check_output(['git', '-C', str(ROOT.parent / 'nlink_parser_uwb'), 'rev-parse', 'HEAD'], text=True).strip()
    dependencies['checker_sha256'] = hashlib.sha256(Path('/home/dev/ws_iap/road_scene_build/road_scene_wire_check').read_bytes()).hexdigest()
    write_json(run / 'metadata/manifests/dependencies.json', dependencies)
    from interface_tests import packet_tests
    write_json(run / 'export/analysis/packet_fixtures.json', packet_tests())
    road = Road()
    motion = Motion(road)
    geometry = Geometry(road, cfg['mesh_step_m'])
    checks = validate(road, motion, geometry, cfg)
    write_json(run / 'export/analysis/geometry_motion_checks.json', checks)
    write_json(run / 'export/simulation/motion_plan.json',
               dict(duration_s=motion.duration, lane_length_m=road.length, brake_station_m=motion.brake_s, segments=motion.segments.tolist()))
    write_json(run / 'export/simulation/facilities.json', geometry.facilities + geometry.continuous_facilities)
    if not args.check:
      generate(run, cfg, road, motion, geometry, args, started)
      from audit import audit
      print(json.dumps(audit(run)), flush=True)
      from preview import plots
      plots(run)
      from road_scene_io import replay
      replay(run / 'export/capture/raw', run / 'export/analysis/wire_check.json', min(2., args.seconds or 2.), Path('/home/dev/ws_iap/road_scene_build/road_scene_wire_check'))
      artifacts = {}
      for path in sorted(run.rglob('*')):
        if not path.is_file() or path == manifest or '/runtime/' in str(path):
          continue
        digest = hashlib.sha256()
        with path.open('rb') as f:
          for chunk in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(chunk)
        artifacts[str(path.relative_to(run))] = dict(bytes=path.stat().st_size, sha256=digest.hexdigest(), retention='keep; scene experiment input/evidence')
      rd.write_subordinate_manifest(run, 'scene_artifacts',
                                    dict(files=artifacts, full_route=args.seconds is None, source_commit=rd._git_value('rev-parse', 'HEAD'), seed=cfg['seed']))
    value = json.loads(manifest.read_text())
    value['acceptance'] = 'pass' if not args.check else 'not_evaluated'
    value['scene_checks'] = 'pass'
    value['wall_seconds'] = time.monotonic() - started
    rd._atomic_write_json(manifest, value)
    rd.finalize_run(run, lifecycle='completed')
  except BaseException as error:
    write_json(run / 'metadata/manifests/input_failure.json', dict(error=str(error), traceback=traceback.format_exc()))
    rd.finalize_run(run, lifecycle='failed')
    raise


def generate(run, cfg, road, motion, geometry, args, started):
  import csv
  import rosbag2_py
  from rclpy.serialization import serialize_message, deserialize_message
  from wire import semantic_bytes
  from rosgraph_msgs.msg import Clock
  from uwb import simulate, anchors, stamp
  from sensors import scan, cloud_message, imu_arrays, imu_message, rolling_fixture
  end = motion.duration if args.seconds is None else args.start + args.seconds
  if args.start < 0 or end > motion.duration + .1 or end <= args.start:
    raise ValueError('invalid capture window')
  frames = int(np.ceil((end - args.start) * 10 - 1e-9))
  end = args.start + frames / 10
  truth_dir = run / 'export/simulation'
  write_json(run / 'export/analysis/rolling_fixture.json', rolling_fixture(road, motion, geometry))
  times = args.start + np.arange(frames * 20 + 1) / 200
  state, force, ba, bg, observed_force, observed_omega = imu_arrays(motion, times, cfg)
  np.savez(truth_dir / 'trajectory.npz', t=times, force=force, acc_bias=ba, gyro_bias=bg, **state)
  ids, stations, sides, heights, positions = anchors(road)
  write_json(truth_dir / 'anchors.json',
             [dict(id=int(i), s=float(s), side=int(side), height=float(h), position=p.tolist()) for i, s, side, h, p in zip(ids, stations, sides, heights, positions)])
  write_json(
      run / 'metadata/config/installation_metadata.json',
      dict(source='exact synthetic relation; not field calibrated; no covariance or algorithm prior',
           definition='horizontal signed normal offset from design axis to antenna phase center, metres; valid ordinary wall band',
           anchors=sorted([dict(id=int(i), side='left' if side == 1 else 'right', lateral_offset_m=float(side * 5.52)) for i, side in zip(ids, sides)], key=lambda x: x['id'])))
  messages, uwb_summary = simulate(road, motion, geometry, cfg, end, truth_dir)
  messages = [(t, m) for t, m in messages if t >= args.start]
  write_json(run / 'export/analysis/uwb_summary.json', uwb_summary)
  geometry.mesh.export(truth_dir / 'scene.ply')
  writer = rosbag2_py.SequentialWriter()
  writer.open(rosbag2_py.StorageOptions(uri=str(run / 'export/capture/raw'), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
  types = {
      '/sensors/lidar': 'sensor_msgs/msg/PointCloud2',
      '/sensors/imu': 'sensor_msgs/msg/Imu',
      '/liuo/uwb/observations': 'liuo_uwb_msgs/msg/UwbObservationBatch',
      '/clock': 'rosgraph_msgs/msg/Clock'
  }
  for topic, kind in types.items():
    writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr'))
  hashes = {topic: hashlib.sha256() for topic in types}
  counts = {topic: 0 for topic in types}

  def emit(topic, msg, t):
    data = serialize_message(msg)
    writer.write(topic, data, int(round((t + 1) * 1e9)))
    hashes[topic].update(semantic_bytes(deserialize_message(data, type(msg))))
    counts[topic] += 1

  rng = np.random.default_rng(np.random.SeedSequence([cfg['seed'], 1]))
  ui = 0
  ii = 0
  total_points = 0
  with (run / 'profiling/resources.csv').open('w') as f:
    w = csv.writer(f)
    w.writerow(['frame', 'simulation_s', 'wall_s', 'peak_rss_kib', 'free_bytes', 'points'])
    for frame in range(frames):
      t = args.start + frame / 10
      delivery = t + .1
      if time.monotonic() - started > cfg['limits']['wall_timeout_s']:
        raise TimeoutError('scene wall deadline')
      free = shutil.disk_usage(run).free
      if free < cfg['limits']['disk_reserve_bytes']:
        raise RuntimeError('disk reserve')
      points, diag = scan(motion, geometry, t, rng, cfg['lidar']['sigma_m'])
      total_points += len(points)
      # Merge independent streams by delivery timestamp. LiDAR arrives only after scan end.
      while (ii < len(times) and times[ii] <= delivery + 1e-9) or (ui < len(messages) and messages[ui][0] <= delivery + 1e-9):
        it = times[ii] if ii < len(times) and times[ii] <= delivery + 1e-9 else float('inf')
        ut = messages[ui][0] if ui < len(messages) and messages[ui][0] <= delivery + 1e-9 else float('inf')
        event = min(it, ut)
        clock = Clock()
        clock.clock = stamp(event)
        emit('/clock', clock, event)
        if it <= ut:
          emit('/sensors/imu', imu_message(it, observed_force[ii], observed_omega[ii], cfg), it)
          ii += 1
        else:
          emit('/liuo/uwb/observations', messages[ui][1], ut)
          ui += 1
      emit('/sensors/lidar', cloud_message(t, points), delivery)
      if frame % 10 == 0 or frame == frames - 1:
        w.writerow([frame, delivery, time.monotonic() - started, resource.getrusage(resource.RUSAGE_SELF).ru_maxrss, free, len(points)])
        f.flush()
      if frame % 100 == 0:
        print(f'frame {frame}/{frames}, sim={delivery:.1f}s, wall={time.monotonic()-started:.1f}s', flush=True)
      if frame in [0, frames // 2, frames - 1]:
        np.savez(truth_dir / f'preview_{frame}.npz', points=points, world=diag['origins'][diag['valid']] + diag['directions'][diag['valid']] * diag['ranges'][diag['valid'], None])
  del writer
  summary = dict(start_s=args.start,
                 end_s=end,
                 planned_duration_s=motion.duration,
                 frames=frames,
                 points=total_points,
                 counts=counts,
                 raw_semantic_sha256={
                     k: v.hexdigest() for k, v in hashes.items()
                 },
                 wall_s=time.monotonic() - started,
                 real_time_factor=(end - args.start) / (time.monotonic() - started),
                 peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                 localization='NOT_RUN',
                 scan_model='rolling_per_ray')
  write_json(run / 'export/analysis/generation.json', summary)
  print(json.dumps(summary), flush=True)


if __name__ == '__main__':
  main()
