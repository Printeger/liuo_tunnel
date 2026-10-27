"""Independent road geometry and continuous kinematics; simulation truth only."""
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.interpolate import CubicSpline


def curvature(s):
  s = np.asarray(s)
  k = np.zeros_like(s, dtype=float)
  dk = np.zeros_like(k)
  for start, sign in [(1000, 1), (4000, -1)]:
    u = s - start
    k += sign / 3000 * np.clip(np.minimum(u / 150, (1500 - u) / 150), 0, 1)
    dk += sign / 450000 * ((u > 0) & (u < 150)) - sign / 450000 * ((u > 1350) & (u < 1500))
  return k, dk


def heading(s):
  s = np.asarray(s)
  a = np.zeros_like(s, dtype=float)
  for start, sign in [(1000, 1), (4000, -1)]:
    u = np.clip(s - start, 0, 1500)
    a += sign / 3000 * np.where(u < 150, u * u / 300, np.where(u < 1350, u - 75, 1350 - (1500 - u)**2 / 300))
  return a


class Road:
  offset = -1.875
  grade = .005
  crossfall = .02

  def __init__(self):
    grid = np.arange(-100., 7100.25, .25)
    a = heading(grid)
    xy = cumulative_trapezoid(np.c_[np.cos(a), np.sin(a)], grid, axis=0, initial=0)
    xy -= xy[400]
    self.xy = CubicSpline(grid, xy)
    k, _ = curvature(grid)
    q = (grid - self.offset * heading(grid)) * np.sqrt(1 + self.grade**2)
    self.q = CubicSpline(grid, q)
    self.s_of_q = CubicSpline(q, grid)
    self.length = float(self.q(7000))

  def world(self, s, y, z):
    s, y, z = np.broadcast_arrays(s, y, z)
    a = heading(s)
    xy = self.xy(s) + np.stack([-np.sin(a) * y, np.cos(a) * y], axis=-1)
    return np.concatenate([xy, (self.grade * s + z)[..., None]], axis=-1)

  def pose(self, s):
    s = np.asarray(s)
    k, _ = curvature(s)
    a = heading(s)
    b = 1 - self.offset * k
    forward = np.stack([np.cos(a), np.sin(a), np.full_like(a, self.grade)], axis=-1)
    forward /= np.linalg.norm(forward, axis=-1)[..., None]
    left = np.stack([-np.sin(a), np.cos(a), np.full_like(a, self.crossfall)], axis=-1)
    left -= np.sum(left * forward, axis=-1)[..., None] * forward
    left /= np.linalg.norm(left, axis=-1)[..., None]
    R = np.stack([forward, left, np.cross(forward, left)], axis=-1)
    p = self.world(s, self.offset, self.crossfall * self.offset + 1.7 - self.grade * self.offset * a)
    return p, R


class Motion:
  """Quintic velocity ramps, exact polynomial distance; q is actual lane arc length."""

  def __init__(self, road):
    self.road = road
    self.segments = []
    t, q, v = 0., 0., 0.

    def add(T, target):
      nonlocal t, q, v
      self.segments.append((t, T, q, v, target - v))
      q += .5 * (v + target) * T
      t += T
      v = target

    add(5., 0.)
    for end_s, target in [(650, 60 / 3.6), (2500, 70 / 3.6), (4000, 80 / 3.6), (4700, 60 / 3.6), (5500, 70 / 3.6), (None, 80 / 3.6)]:
      dv = abs(target - v)
      T = max(1.875 * dv / (1. if target > v else 1.5), np.sqrt(5.774 * dv / .5)) * 1.02
      add(T, target)
      if end_s is not None:
        add((float(road.q(end_s)) - q) / v, v)
    Tstop = max(1.875 * v / 1.5, np.sqrt(5.774 * v / .5)) * 1.02
    add((road.length - q - .5 * v * Tstop) / v, v)
    self.brake_s = float(road.s_of_q(q))
    add(Tstop, 0.)
    add(5., 0.)
    self.duration = t
    self.segments = np.array(self.segments)
    assert np.all(self.segments[:, 1] > 0)

  def scalar(self, t):
    t = np.asarray(t)
    idx = np.clip(np.searchsorted(self.segments[:, 0], t, side='right') - 1, 0, len(self.segments) - 1)
    start, T, q0, v0, dv = self.segments[idx].T
    u = np.clip((t - start) / T, 0, 1)
    q = q0 + v0 * T * u + dv * T * (2.5 * u**4 - 3 * u**5 + u**6)
    v = v0 + dv * (10 * u**3 - 15 * u**4 + 6 * u**5)
    acc = dv / T * (30 * u**2 - 60 * u**3 + 30 * u**4)
    jerk = dv / T**2 * (60 * u - 180 * u**2 + 120 * u**3)
    return q, v, acc, jerk

  def state(self, t):
    q, speed, acc, jerk = self.scalar(t)
    s = self.road.s_of_q(q)
    p, R = self.road.pose(s)
    k, dk = curvature(s)
    yaw = heading(s)
    b = 1 - self.road.offset * k
    h = np.sqrt(1 + self.road.grade**2)
    sd = speed / (b * h)
    tangent = np.stack([np.cos(yaw), np.sin(yaw), np.full_like(s, self.road.grade)], axis=-1) / h
    normal = np.stack([-np.sin(yaw), np.cos(yaw), np.zeros_like(s)], axis=-1)
    vel = tangent * speed[..., None]
    acceleration = tangent * acc[..., None] + normal * (speed * k * sd / h)[..., None]
    # Constant chassis grade/crossfall, yaw follows the offset-lane tangent.
    # <=4.22 mm passive ride-height variation avoids artificial acceleration /
    # pitch-rate jumps at the prescribed clothoid curvature-slope boundaries.
    omega = R[..., 2, :] * (k * sd)[..., None]
    return dict(s=s, p=p, R=R, v=vel, a=acceleration, omega=omega, speed=speed, acc=acc, jerk=jerk)
