"""Independent analytical/numerical fixtures; thresholds frozen in scene.json."""
import numpy as np
from scipy.integrate import quad
from scipy.spatial.transform import Rotation
from model import heading, curvature


def validate(road, motion, geometry, cfg):
  results = {}

  def check(name, value, limit):
    results[name] = dict(value=float(value), limit=float(limit), pass_=bool(value <= limit))
    if value > limit:
      raise AssertionError(f'{name}: {value} > {limit}')

  tol = cfg['tolerances']
  for s in [1000, 1150, 2350, 2500, 4000, 4150, 5350, 5500, 7000]:
    x = quad(lambda u: np.cos(float(heading(u))), 0, s, points=[p for p in [1000, 1150, 2350, 2500, 4000, 4150, 5350, 5500] if p < s], epsabs=1e-8)[0]
    y = quad(lambda u: np.sin(float(heading(u))), 0, s, points=[p for p in [1000, 1150, 2350, 2500, 4000, 4150, 5350, 5500] if p < s], epsabs=1e-8)[0]
    check(f'axis_quadrature_{s}', np.linalg.norm(road.xy(s) - [x, y]), tol['route_position_m'])
  t = np.arange(.01, motion.duration - .01, .031)
  h = .0005
  state = motion.state(t)
  plus = motion.state(t + h)
  minus = motion.state(t - h)
  check('velocity_fd', np.max(np.abs((plus['p'] - minus['p']) / (2 * h) - state['v'])), .0001)
  check('acceleration_fd', np.max(np.abs((plus['v'] - minus['v']) / (2 * h) - state['a'])), tol['finite_difference_acc_mps2'])
  rot = Rotation.from_matrix(np.swapaxes(minus['R'], -1, -2) @ plus['R']).as_rotvec() / (2 * h)
  check('omega_fd', np.max(np.abs(rot - state['omega'])), tol['finite_difference_omega_radps'])
  check('speed_excess', max(0, state['speed'].max() - 80 / 3.6), tol['speed_mps'])
  check('accel_excess', max(0, state['acc'].max() - 1, -1.5 - state['acc'].min()), tol['acc_mps2'])
  check('jerk_excess', max(0, np.max(np.abs(state['jerk'])) - .5), tol['jerk_mps3'])
  check('reverse', max(0, -state['speed'].min()), 0)
  specific = np.einsum('nji,nj->ni', state['R'], state['a'] - [0, 0, -9.80665])
  check('friction_demand', np.max(np.linalg.norm(specific[:, :2], axis=1) / specific[:, 2]), .6)
  check('endpoint_station', abs(float(motion.state(motion.duration)['s']) - 7000), .0001)
  check('start_hold', np.max(np.linalg.norm(motion.state(np.linspace(0, 5, 20))['v'], axis=1)), 1e-12)
  check('end_hold', np.max(np.linalg.norm(motion.state(np.linspace(motion.duration - 5, motion.duration, 20))['v'], axis=1)), 1e-12)
  # Straight wall / road / crown at grade-adjusted origins: independent distances.
  origins = road.world([500, 500, 500], [0, 0, 0], [2, 2, 2])
  hit = geometry.hit(origins, [[0, 1, 0], [0, 0, -1], [0, 0, 1]])
  check('analytic_wall', abs(hit[0] - 5.6), .005)
  check('analytic_road', abs(hit[1] - 2), .005)
  check('analytic_crown', abs(hit[2] - 5.2), .005)
  portal = geometry.hit(road.world([0, 7000], [0, 0], [2, 2]), [[-1, 0, -.005], [1, 0, .005]])
  check('portal_false_returns', np.isfinite(portal).sum(), 0)
  check('straight_LOS_blocked', int(not geometry.los(road.world(100, 5.52, 2.5), road.world(500, -5.52, 3.5))[0]), 0)
  check('curve_wall_leak', int(geometry.los(road.world(1300, 5.52, 2.5), road.world(2100, 5.52, 2.5))[0]), 0)
  # Complete horizontal driving clearance, including crossfall height, is inside arch.
  y = np.linspace(-5.25, 5.25, 1001)
  ceiling = 5.2 + 2 * np.sqrt(1 - (y / 5.6)**2)
  check('clearance_violation', max(0, np.max(.02 * y + 5 - ceiling)), 0)
  # Envelope corners: conservatively within roadway boundaries, with road support margin.
  stations = np.linspace(0, 7000, 14001)
  p, R = road.pose(stations)
  worst = 0.
  for x in [-2.3, 2.3]:
    for y in [-.9, .9]:
      for z in [-1.5, 0]:
        corners = p + np.einsum('nij,j->ni', R, [x, y, z])
        approximate_s = stations + x
        delta = corners - road.world(approximate_s, road.offset, road.crossfall * road.offset)
        pavement_delta = delta[:, 2] - .02 * (delta[:, 1] * np.cos(heading(approximate_s)) - delta[:, 0] * np.sin(heading(approximate_s)))
        worst = max(worst, float(np.max(-pavement_delta)))
  check('body_road_penetration', worst, 0)
  # Continuity from both sides of every prescribed alignment join.
  from scipy.optimize import brentq
  for station in [1000, 1150, 2350, 2500, 4000, 4150, 5350, 5500]:
    join = brentq(lambda t: float(motion.state(t)['s']) - station, 0, motion.duration)
    lo = motion.state(join - 1e-6)
    hi = motion.state(join + 1e-6)
    check(f'continuous_acc_{station}', np.linalg.norm(hi['a'] - lo['a']), .00001)
    check(f'continuous_omega_{station}', np.linalg.norm(hi['omega'] - lo['omega']), .000001)
  # 80 km/h stopping sight demand: 2.5 s reaction + braking at 3.4 m/s².
  sight_distance = (80 / 3.6) * 2.5 + (80 / 3.6)**2 / (2 * 3.4)
  stations = np.arange(0, 7000 - sight_distance, 5.)
  eyes = road.world(stations, -1.875, .02 * (-1.875) + 1.2)
  targets = road.world(stations + sight_distance, -1.875, .02 * (-1.875) + .2)
  check('stopping_sight_blocked', np.count_nonzero(~geometry.los(eyes, targets)), 0)
  results['sight_model'] = dict(distance_m=sight_distance, reaction_s=2.5, deceleration_mps2=3.4, source='experimental sight envelope; not whole-road certification')
  # Wall panels, recess backs and longitudinal mesh joins have no seams/leaks.
  station = np.array([999.999, 1000, 1000.001, 1150, 2350, 2500, 4000, 5350, 5500])
  origins = road.world(station, 0, 2)
  a = heading(station)
  rays = np.c_[-np.sin(a), np.cos(a), np.zeros_like(a)]
  check('mesh_join_wall', np.max(np.abs(geometry.hit(origins, rays) - 5.6)), .005)
  for f in geometry.facilities:
    if 'recess' not in f['kind']:
      continue
    a = float(heading(f['s']))
    direction = f['side'] * np.array([-np.sin(a), np.cos(a), 0.])
    distance = geometry.hit(road.world(f['s'], 0, 2), direction)[0]
    check(f"recess_back_{f['s']}", abs(distance - 6.), .005)
  from geometry import Geometry
  fine = Geometry(road, .5)
  rng = np.random.default_rng(7)
  stations = rng.uniform(100, 6900, 2000)
  origins = road.pose(stations)[0]
  direction = rng.normal(size=(len(stations), 3))
  direction /= np.linalg.norm(direction, axis=1)[:, None]
  coarse = geometry.hit(origins, direction)
  refined = fine.hit(origins, direction)
  finite = np.isfinite(coarse) & np.isfinite(refined)
  check('mesh_refinement_return_disagreement', np.count_nonzero(np.isfinite(coarse) != np.isfinite(refined)), 0)
  check('mesh_refinement_range', np.max(np.abs(coarse[finite] - refined[finite])), .005)
  return results
