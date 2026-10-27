#!/usr/bin/env python3
"""Additional independent artifact acceptance; reads saved truth only offline.

Uses a direct float64 segment/triangle test, not the generator Embree intersector.
Does not import simulation geometry or motion implementations.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import trimesh
from scipy.integrate import quad


def run_checks(run, report=None):
  out = report or run / 'export/analysis/independent_acceptance.json'
  if out.exists():
    raise FileExistsError(out)
  metadata = json.loads((run / 'metadata/run_manifest.json').read_text())
  assert metadata['lifecycle'] == 'completed' and metadata['acceptance'] == 'pass'
  assert metadata['source']['git_worktree_clean'] and metadata['localization'] == 'NOT_RUN'
  inventory = json.loads((run / 'metadata/manifests/scene_artifacts.json').read_text())
  assert inventory['full_route'] and inventory['source_commit'] == metadata['source']['git_commit']
  for relative, entry in inventory['files'].items():
    p = run / relative
    h = hashlib.sha256()
    with p.open('rb') as f:
      for chunk in iter(lambda: f.read(1024 * 1024), b''):
        h.update(chunk)
    assert p.stat().st_size == entry['bytes'] and h.hexdigest() == entry['sha256'], relative
  truth = np.load(run / 'export/simulation/trajectory.npz')
  t = truth['t']
  assert t[0] == 0 and abs(truth['s'][-1] - 7000) < 1e-4
  assert np.all(truth['speed'][t <= 5] == 0) and np.all(truth['speed'][t >= t[-1] - 5] == 0)
  anchors = json.loads((run / 'export/simulation/anchors.json').read_text())
  position = {a['id']: np.array(a['position']) for a in anchors}
  assert len(anchors) == 18 and np.allclose(sorted(a['s'] for a in anchors), 100 + 400 * np.arange(18))

  # Independent closed-form heading and quadrature (not the simulator table).
  def angle(s):
    result = 0.
    for start, sign in [(1000, 1), (4000, -1)]:
      u = min(max(s - start, 0), 1500)
      integral = u * u / 300 if u <= 150 else (u - 75 if u <= 1350 else 1350 - (1500 - u)**2 / 300)
      result += sign * integral / 3000
    return result

  wall_errors = []
  for a in anchors:
    s = a['s']
    yaw = angle(s)
    breaks = [v for v in [1000, 1150, 2350, 2500, 4000, 4150, 5350, 5500] if v < s]
    center = np.array([quad(lambda u: np.cos(angle(u)), 0, s, points=breaks, epsabs=1e-8)[0], quad(lambda u: np.sin(angle(u)), 0, s, points=breaks, epsabs=1e-8)[0], .005 * s])
    local = np.array([np.cos(yaw), np.sin(yaw), 0.])
    normal = np.array([-np.sin(yaw), np.cos(yaw), 0.])
    delta = position[a['id']] - center
    wall_errors.extend([abs(delta @ normal - a['side'] * 5.52), abs(delta @ local), abs(delta[2] - a['height'])])
  assert max(wall_errors) < .0001
  mesh = trimesh.load_mesh(run / 'export/simulation/scene.ply', process=False)
  tri = np.asarray(mesh.triangles, dtype=np.float64)
  lower = tri.min(axis=1)
  upper = tri.max(axis=1)

  def los(a, b):
    length = np.linalg.norm(b - a)
    if length > 800:
      return False
    lo = np.minimum(a, b) - .002
    hi = np.maximum(a, b) + .002
    candidates = tri[np.all(upper >= lo, axis=1) & np.all(lower <= hi, axis=1)]
    direction = (b - a) / length
    edge1 = candidates[:, 1] - candidates[:, 0]
    edge2 = candidates[:, 2] - candidates[:, 0]
    h = np.cross(direction, edge2)
    det = np.sum(edge1 * h, axis=1)
    valid = np.abs(det) > 1e-10
    inv = np.zeros_like(det)
    inv[valid] = 1 / det[valid]
    offset = a - candidates[:, 0]
    u = np.sum(offset * h, axis=1) * inv
    q = np.cross(offset, edge1)
    v = q @ direction * inv
    d = np.sum(edge2 * q, axis=1) * inv
    hit = valid & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (d > .002) & (d < length - .002)
    return not np.any(hit)

  summary = json.loads((run / 'export/analysis/uwb_summary.json').read_text())
  pairs = {frozenset(p) for p in summary['aa_geometry_pairs']}
  for i, a in enumerate(anchors):
    for b in anchors[i + 1:]:
      assert los(position[a['id']], position[b['id']]) == (frozenset([a['id'], b['id']]) in pairs)

  def car(at):
    return np.array([np.interp(at, t, truth['p'][:, j]) for j in range(3)])

  with (run / 'export/simulation/delivery_uwb.csv').open() as f:
    delivery = list(csv.DictReader(f))
  tested = 0
  for action in ['delivered', 'unreachable']:
    selected = [r for r in delivery if r['action'] == action]
    for idx in np.linspace(0, len(selected) - 1, min(100, len(selected))).astype(int):
      r = selected[idx]
      tx = float(r['tx'])
      a = position[int(r['reporter'])]
      reachable = los(a, car(tx)) and los(a, car(tx + .012))
      assert reachable == (action == 'delivered'), r
      tested += 1
  errors = []
  physical_times = []
  noise_by_kind = {0: [], 1: []}
  with (run / 'export/simulation/physical_uwb.csv').open() as f:
    physical = list(csv.DictReader(f))
  for r in physical:
    at = float(r['t'])
    physical_times.append(at)
    if r['valid'] != '1':
      continue
    a = car(at) if int(r['a']) == 0 else position[int(r['a'])]
    b = position[int(r['b'])]
    noise_by_kind[int(r['kind'])].append(float(r['distance']) - np.linalg.norm(a - b))
  for kind, noise in noise_by_kind.items():
    assert abs(np.std(noise) - .1) < .01 and abs(np.mean(noise)) < .01
  for idx in np.linspace(0, len(physical) - 1, 100).astype(int):
    r = physical[idx]
    at = float(r['t'])
    a = car(at) if int(r['a']) == 0 else position[int(r['a'])]
    assert los(a, position[int(r['b'])]) == (r['valid'] == '1')
  assert np.min(np.diff(physical_times)) >= .004 - 1e-8
  # Raw radial noise from retained preview rays; restoration is an offline test only.
  radial = []
  for file in (run / 'export/simulation').glob('preview_*.npz'):
    frame = int(file.stem.split('_')[1])
    d = np.load(file)
    points = d['points']
    pt = frame * .1 + points['time']
    origin = np.column_stack([np.interp(pt, t, truth['p'][:, j]) for j in range(3)])
    raw = np.c_[points['x'], points['y'], points['z']]
    radial.extend(np.linalg.norm(raw, axis=1) - np.linalg.norm(d['world'] - origin, axis=1))
  assert abs(np.std(radial) - .02) < .002 and abs(np.mean(radial)) < .002
  coverage = list(csv.DictReader((run / 'export/simulation/coverage.csv').open()))
  zero = sum(int(r['ta_count']) == 0 for r in coverage) * .05
  assert abs(zero - summary['zero_coverage_s']) < 1e-8
  data = dict(pass_=True,
              source_commit=metadata['source']['git_commit'],
              scope='S01-S12 scene/data; algorithms NOT_RUN',
              artifact_hashes_verified=len(inventory['files']),
              wall_offset_error_max_m=max(wall_errors),
              aa_pairs_checked=153,
              delivery_segments_checked=tested,
              physical_segments_checked=100,
              uwb_noise={
                  str(k): dict(n=len(v), mean=float(np.mean(v)), std=float(np.std(v))) for k, v in noise_by_kind.items()
              },
              lidar_radial_noise=dict(n=len(radial), mean=float(np.mean(radial)), std=float(np.std(radial))),
              minimum_exchange_spacing_s=float(np.min(np.diff(physical_times))),
              zero_coverage_s=zero)
  out.write_text(json.dumps(data, indent=2) + '\n')
  print(json.dumps(data, indent=2))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('run', type=Path)
  parser.add_argument('--report', type=Path, help='new audit output path; never overwrite evidence')
  args = parser.parse_args()
  run_checks(args.run, args.report)
