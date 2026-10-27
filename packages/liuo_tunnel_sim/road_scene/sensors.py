"""Raw rolling scans, IMU and lossless PointCloud2 field construction."""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, Imu
from std_msgs.msg import Header
from uwb import stamp

POINT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': 24
})


def ray_pattern():
  index = np.arange(512 * 40)
  ring = index % 40
  elevation = np.deg2rad(-25 + 50 * ring / 39)
  azimuth = 2 * np.pi * (index // 40) / 512
  direction = np.c_[np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation)]
  return direction, ring, index / (512 * 40 * 10)


def scan(motion, geometry, t, rng, sigma=.02):
  direction, ring, relative = ray_pattern()
  state = motion.state(t + relative)
  world_direction = np.einsum('nij,nj->ni', state['R'], direction)
  ranges = geometry.hit(state['p'], world_direction)
  valid = np.isfinite(ranges)
  noisy = ranges[valid] + rng.normal(0, sigma, int(valid.sum()))
  points = np.zeros(int(valid.sum()), dtype=POINT_DTYPE)
  xyz = direction[valid] * noisy[:, None]
  for j, key in enumerate(['x', 'y', 'z']):
    points[key] = xyz[:, j]
  points['intensity'] = 1
  points['ring'] = ring[valid]
  points['time'] = relative[valid]
  return points, dict(origins=state['p'], directions=world_direction, ranges=ranges, valid=valid)


def cloud_message(t, points):
  fields = [PointField(name=k, offset=POINT_DTYPE.fields[k][1], datatype=PointField.UINT16 if k == 'ring' else PointField.FLOAT32, count=1) for k in POINT_DTYPE.names]
  return PointCloud2(header=Header(stamp=stamp(t), frame_id='lidar'),
                     height=1,
                     width=len(points),
                     fields=fields,
                     is_bigendian=False,
                     point_step=24,
                     row_step=len(points) * 24,
                     data=points.tobytes(),
                     is_dense=True)


def imu_arrays(motion, times, cfg):
  state = motion.state(times)
  params = cfg['imu']
  streams = [np.random.default_rng(np.random.SeedSequence([cfg['seed'], i])) for i in range(10, 14)]
  n = len(times)
  ba = streams[0].normal(0, params['initial_acc_bias_sigma'], 3) + np.cumsum(streams[0].normal(0, params['sigma_ba_density'] * np.sqrt(.005), (n, 3)), axis=0)
  bg = streams[1].normal(0, params['initial_gyro_bias_sigma'], 3) + np.cumsum(streams[1].normal(0, params['sigma_bg_density'] * np.sqrt(.005), (n, 3)), axis=0)
  force = np.einsum('nji,nj->ni', state['R'], state['a'] - [0, 0, -9.80665])
  observed_force = force + ba + streams[2].normal(0, params['sigma_acc_sample'], (n, 3))
  observed_omega = state['omega'] + bg + streams[3].normal(0, params['sigma_gyro_sample'], (n, 3))
  return state, force, ba, bg, observed_force, observed_omega


def imu_message(t, force, omega, cfg):
  msg = Imu()
  msg.header = Header(stamp=stamp(t), frame_id='imu')
  msg.orientation_covariance[0] = -1.
  msg.orientation.w = 0.
  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, force)
  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, omega)
  for j in range(3):
    msg.linear_acceleration_covariance[j * 3 + j] = cfg['imu']['sigma_acc_sample']**2
    msg.angular_velocity_covariance[j * 3 + j] = cfg['imu']['sigma_gyro_sample']**2
  return msg


def rolling_fixture(road, motion, geometry):
  # Independent straight-section plane equations, tested while travelling 80 km/h.
  from scipy.optimize import brentq
  from model import heading
  t = brentq(lambda t: float(motion.state(t)['s']) - 3400, 0, motion.duration)
  points, diag = scan(motion, geometry, t, np.random.default_rng(0), sigma=0)
  valid = diag['valid']
  origins = diag['origins'][valid]
  rays = diag['directions'][valid]
  xyz = np.c_[points['x'], points['y'], points['z']]
  pose = motion.state(t + points['time'].astype(float))
  restored = pose['p'] + np.einsum('nij,nj->ni', pose['R'], xyz)
  base = motion.state(t)
  snapshot = base['p'] + xyz @ base['R'].T
  a = float(heading(3400))
  N = np.array([-np.sin(a), np.cos(a), 0.])
  F = np.array([np.cos(a), np.sin(a), 0.])
  c = road.world(3400, 0, 0)
  yr = (restored - c) @ N
  zr = (restored - c) @ np.array([0., 0., 1.]) - .005 * ((restored - c) @ F)
  # Sidewall and pavement hits only; exclude drainage/arch/fixtures by height and y.
  wall = (zr > .5) & (zr < 4.9) & (np.abs(yr) > 5.5)
  floor = (np.abs(yr) < 4.3) & (zr < .2)
  error = np.r_[np.abs(np.abs(yr[wall]) - 5.6), np.abs(zr[floor] - .02 * yr[floor])]
  if len(error) < 1000 or error.max() > .005:
    raise AssertionError('independent rolling plane residual')
  displacement = np.linalg.norm(snapshot - restored, axis=1)
  if displacement.max() < 2.:
    raise AssertionError('expected moving-scan distortion missing')
  return dict(time=t,
              plane_points=len(error),
              plane_residual_max_m=float(error.max()),
              snapshot_displacement_max_m=float(displacement.max()),
              scope='truth restoration only inside offline check; raw output unchanged')
