"""Standalone scientific plots from simulation-only artifacts."""
import csv
import json
import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from model import Road
from geometry import profile


def plots(run):
  out = run / 'export/analysis'
  truth = np.load(run / 'export/simulation/trajectory.npz')
  road = Road()
  s = np.linspace(0, 7000, 7001)
  axis = road.world(s, 0, 0)
  anchors = json.loads((run / 'export/simulation/anchors.json').read_text())
  ap = np.array([a['position'] for a in anchors])
  fig, axs = plt.subplots(2, 1, figsize=(12, 7))
  axs[0].plot(axis[:, 0], axis[:, 1], label='design axis')
  axs[0].scatter(ap[:, 0], ap[:, 1], s=15, label='anchors (truth)')
  axs[0].set_aspect('equal')
  axs[0].legend()
  axs[0].set(xlabel='world x (m)', ylabel='world y (m)', title='7 km road plan; geometry only')
  axs[1].plot(s, axis[:, 2])
  axs[1].set(xlabel='design station (m)', ylabel='grade elevation (m)')
  fig.tight_layout()
  fig.savefig(out / 'route_plan.png', dpi=160)
  plt.close(fig)
  fig = plt.figure(figsize=(12, 6))
  ax = fig.add_subplot(projection='3d')
  ax.plot(*axis.T)
  ax.scatter(*ap.T, s=12)
  ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)', title='Full route 3D (vertical scale exaggerated)')
  fig.tight_layout()
  fig.savefig(out / 'route_3d.png', dpi=160)
  plt.close(fig)
  yz = profile()
  fig, ax = plt.subplots(figsize=(10, 6))
  ax.plot(*np.vstack([yz, yz[0]]).T, label='inner surface')
  y = np.array([-4.5, -4.5, 4.5, 4.5])
  z = .02 * y + np.array([0, 5, 5, 0])
  ax.plot(y, z, '--', label='5 m driving clearance')
  ax.axvline(-5.25, color='gray', linestyle=':')
  ax.axvline(5.25, color='gray', linestyle=':', label='10.50 m basic clearance width')
  ax.scatter([-5.52, 5.52], [3.5, 2.5], label='antenna phase centers')
  ax.set(xlabel='horizontal left offset (m)', ylabel='vertical height above grade (m)', title='Experimental symmetric section')
  ax.set_aspect('equal')
  ax.legend()
  fig.tight_layout()
  fig.savefig(out / 'cross_section.png', dpi=160)
  plt.close(fig)
  fig, axs = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
  for ax, key, label in zip(axs, ['s', 'speed', 'acc', 'jerk'], ['design station (m)', 'actual speed (m/s)', 'longitudinal acceleration (m/s²)', 'jerk (m/s³)']):
    ax.plot(truth['t'], truth[key])
    ax.set_ylabel(label)
    ax.grid()
  axs[-1].set_xlabel('scene time (s)')
  fig.tight_layout()
  fig.savefig(out / 'motion.png', dpi=160)
  plt.close(fig)
  with (run / 'export/simulation/coverage.csv').open() as f:
    rows = list(csv.DictReader(f))
  fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
  t = np.array([float(r['t']) for r in rows])
  station = np.array([float(r['s']) for r in rows])
  axs[0].step(station, [int(r['ta_count']) for r in rows], where='post')
  axs[0].set_ylabel('LOS TA count')
  for r in rows[::4]:
    nodes = list(map(int, r['reachable_ids'].split()))
    axs[1].scatter([float(r['s'])] * len(nodes), nodes, s=2, c='tab:blue')
  axs[1].set_ylabel('reachable anchor ID')
  with (run / 'export/simulation/delivery_uwb.csv').open() as f:
    deliveries = [r for r in csv.DictReader(f) if r['action'] == 'delivered']
  edges = sorted({r['reporter'] + '->' + r['peer'] for r in deliveries})
  for row in deliveries:
    edge = row['reporter'] + '->' + row['peer']
    axs[2].scatter(np.interp(float(row['tx']), t, station), edges.index(edge), s=2, c='tab:orange')
  axs[2].set_yticks(range(len(edges)), edges, fontsize=6)
  axs[2].set(xlabel='design station (m)', ylabel='delivered AA edge')
  fig.tight_layout()
  fig.savefig(out / 'uwb_coverage.png', dpi=160)
  plt.close(fig)
  previews = sorted((run / 'export/simulation').glob('preview_*.npz'))
  fig = plt.figure(figsize=(12, 5 * len(previews)))
  for i, path in enumerate(previews, 1):
    pts = np.load(path)['world'][::8]
    ax = fig.add_subplot(len(previews), 1, i, projection='3d')
    ax.scatter(*pts.T, s=.2, c=pts[:, 2], cmap='viridis')
    ax.set_title(path.stem + ' noiseless collision geometry (truth preview only)')
    ax.set(xlabel='x', ylabel='y', zlabel='z')
  fig.tight_layout()
  fig.savefig(out / 'scene_cloud.png', dpi=130)
  plt.close(fig)
