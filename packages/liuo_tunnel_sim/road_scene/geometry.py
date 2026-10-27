"""Watertight cross-section sweep with open portals and real recessed walls."""
import numpy as np
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector


def profile():
  # Horizontal transverse y, vertical z relative to grade axis; road z=.02*y.
  positive = [(0, 0), (4.32, .0864), (4.32, .0064), (4.44, .0088), (4.44, .0888), (4.5, .09), (4.5, .24), (5.3, .256), (5.3, .216), (5.45, .219), (5.45, .259), (5.6, .262)]
  # Negative drainage and sidewalk independently follow the SAME one-sided slope.
  negative = [(-5.6, .038), (-5.45, .041), (-5.45, .001), (-5.3, .004), (-5.3, .044), (-4.5, .06), (-4.5, -.09), (-4.44, -.0888), (-4.44, -.1688), (-4.32, -.1664), (-4.32, -.0864)]
  floor = negative + positive
  rightwall = [(5.6, .8), (5.6, 1.), (5.6, 3.), (5.6, 3.2), (5.6, 5.2)]
  theta = np.linspace(0, np.pi, 65)[1:]
  arch = list(zip(5.6 * np.cos(theta), 5.2 + 2 * np.sin(theta)))
  leftwall = [(-5.6, 3.2), (-5.6, 3.), (-5.6, 1.), (-5.6, .8)]
  return np.array(floor + rightwall + arch + leftwall)


class Geometry:

  def __init__(self, road, step=1.):
    self.road = road
    self.facilities = []
    self.continuous_facilities = [
        dict(kind='inspection_walkways',
             s_interval=[0, 7000],
             sides=['left', 'right'],
             y_band_abs_m=[4.5, 5.6],
             step_m=.15,
             source='experimental implementation of referenced walkway geometry'),
        dict(kind='drainage_channels', s_interval=[0, 7000], sides=['left', 'right'], y_band_abs_m=[4.32, 4.44], depth_m=.08, source='experimental'),
        dict(kind='cable_channels', s_interval=[0, 7000], sides=['left', 'right'], y_band_abs_m=[5.3, 5.45], depth_m=.04, source='experimental')
    ]

    for s in np.arange(250, 7000, 500):
      self.facilities.append(dict(kind='lamp_bracket', s=float(s), side=1, y=3.5, z=5.9, size=[.6, .35, .25], source='experimental'))
    for i, s in enumerate(np.arange(325, 7000, 1000)):
      self.facilities.append(
          dict(kind='equipment_recess' if i % 2 == 0 else 'door_recess', s=float(s), side=1 if i % 2 == 0 else -1, size=[2., .4, 2.], z=1., source='experimental'))
    sections = np.arange(0, 7000 + step / 2, step)
    # Add explicit tight transition sections for recess lips; never leave a wall hole.
    extra = [f['s'] + d for f in self.facilities if 'recess' in f['kind'] for d in [-1.1, -1, 1, 1.1]]
    sections = np.unique(np.r_[sections, extra])
    yz = profile()
    ys = np.broadcast_to(yz[:, 0], (len(sections), len(yz))).copy()
    zs = np.broadcast_to(yz[:, 1], ys.shape)
    for f in self.facilities:
      if 'recess' not in f['kind']:
        continue
      taper = np.clip((1.1 - np.abs(sections - f['s'])) / .1, 0, 1)
      mask = (yz[:, 0] * f['side'] > 5.5) & (yz[:, 1] >= 1) & (yz[:, 1] <= 3)
      ys += f['side'] * .4 * taper[:, None] * mask
    vertices = road.world(sections[:, None], ys, zs).reshape(-1, 3)
    n = len(yz)
    a = (np.arange(len(sections) - 1)[:, None] * n + np.arange(n)).ravel()
    b = (np.arange(len(sections) - 1)[:, None] * n + (np.arange(n) + 1) % n).ravel()
    faces = np.vstack([np.c_[a, b, a + n], np.c_[b, b + n, a + n]])
    meshes = [trimesh.Trimesh(vertices=vertices, faces=faces, process=False)]
    # Portal pavement extension only, no end caps or invented end walls.
    for lo, hi in [(-60, 0), (7000, 7060)]:
      v = road.world(np.array([lo, lo, hi, hi]), np.array([-4.5, 4.5, 4.5, -4.5]), np.array([-.09, .09, .09, -.09]))
      meshes.append(trimesh.Trimesh(vertices=v, faces=[[0, 1, 2], [0, 2, 3]], process=False))
    for f in self.facilities:
      if f['kind'] != 'lamp_bracket':
        continue
      box = trimesh.creation.box(extents=f['size'])
      s, y, z = f['s'], f['y'], f['z']
      # Transform all eight vertices through the road chart, keeping true solid faces.
      v = box.vertices
      box.vertices = road.world(s + v[:, 0], y + v[:, 1], z + v[:, 2])
      meshes.append(box)
    self.mesh = trimesh.util.concatenate(meshes)
    self.rays = RayMeshIntersector(self.mesh, scale_to_box=True)

  def hit(self, origins, directions, max_range=40.):
    origins = np.asarray(origins).reshape(-1, 3)
    directions = np.asarray(directions).reshape(-1, 3)
    distance = np.full(len(origins), np.inf)
    locations, ray, _ = self.rays.intersects_location(origins, directions, multiple_hits=False)
    distance[ray] = np.linalg.norm(locations - origins[ray], axis=1)
    distance[distance > max_range] = np.inf
    return distance

  def los(self, a, b, endpoint_tolerance=.002):
    a, b = np.broadcast_arrays(a, b)
    a, b = a.reshape(-1, 3), b.reshape(-1, 3)
    delta = b - a
    length = np.linalg.norm(delta, axis=1)
    direction = delta / np.maximum(length[:, None], 1e-12)
    distance = self.hit(a + endpoint_tolerance * direction, direction, 800.)
    return (length <= 800) & (length > 2 * endpoint_tolerance) & (distance >= length - 2 * endpoint_tolerance)
