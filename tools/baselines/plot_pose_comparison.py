#!/usr/bin/env python3
"""Plot recorded body poses, without whole-trajectory fitting or estimator reruns."""
import hashlib
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from evaluate_ape import ROOT, NAMES, load


def main():
    spec = json.loads((ROOT / 'tools/baselines/runs.json').read_text())
    truth_path = ROOT / spec['source'] / 'export/simulation/truth.csv'
    truth = load(truth_path)
    truth_rot = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))
    keys = ('fastlio', 'self', 'reuse')
    data, metadata = {}, {}
    for key in keys:
        path = ROOT / spec['runs'][key] / 'export/odometry/trajectory.csv'
        values = load(path)
        if np.any(np.diff(values[:, 0]) <= 0):
            raise ValueError('Nonmonotonic timestamps: ' + key)
        first = values[0]
        r = truth_rot(first[0]) * Rotation.from_quat(first[4:8]).inv()
        target = np.array([np.interp(first[0], truth[:, 0], truth[:, i]) for i in (1, 2, 3)])
        t = target - r.apply(first[1:4])
        fixed = values.copy()
        fixed[:, 1:4] = r.apply(values[:, 1:4]) + t
        fixed[:, 4:8] = (r * Rotation.from_quat(values[:, 4:8])).as_quat()
        assert np.allclose(fixed[0, 1:4], target, atol=1e-9)
        assert (truth_rot(first[0]).inv() * Rotation.from_quat(fixed[0, 4:8])).magnitude() < 1e-9
        assert np.allclose(np.linalg.norm(np.diff(values[:, 1:4], axis=0), axis=1),
                           np.linalg.norm(np.diff(fixed[:, 1:4], axis=0), axis=1), atol=1e-9)
        data[key] = {'native': values, 'first_pose': fixed}
        metadata[key] = dict(path=str(path.relative_to(ROOT)), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                             poses=len(values), first_stamp=float(first[0]), last_stamp=float(values[-1, 0]),
                             rotation=r.as_matrix().tolist(), translation=t.tolist())
    out = ROOT / 'docs/figures'
    plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': .25, 'figure.dpi': 160})
    for mode in ('native', 'first_pose'):
        title = ('Native coordinates: different frames; NOT absolute error comparison' if mode == 'native'
                 else 'One first-pose transform per method; NO whole-trajectory fit')
        curves = [('Truth', 'black', truth)] + [(NAMES[k][1], NAMES[k][2], data[k][mode]) for k in keys]
        for quantity in ('xyz', 'rpy'):
            fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
            for label, color, values in curves:
                if quantity == 'xyz':
                    y = values[:, 1:4]
                else:
                    # R = Rz(yaw) Ry(pitch) Rx(roll); unwrap each continuous run.
                    y = np.rad2deg(np.unwrap(Rotation.from_quat(values[:, 4:8]).as_euler('xyz'), axis=0))
                for i, ax in enumerate(axes):
                    # Break any missing-output interval rather than bridging it.
                    cuts = np.r_[0, np.flatnonzero(np.diff(values[:, 0]) > .25) + 1, len(values)]
                    for j, (a, b) in enumerate(zip(cuts[:-1], cuts[1:])):
                        ax.plot(values[a:b, 0], y[a:b, i], color=color, lw=1.2,
                                ls='--' if label == 'Truth' else '-', label=label if j == 0 else None)
                    ax.set_ylabel((('X', 'Y', 'Z')[i] + ' [m]') if quantity == 'xyz'
                                  else (('Roll', 'Pitch', 'Yaw')[i] + ' [deg]'))
            axes[0].legend(ncol=2, fontsize=9)
            axes[-1].set_xlabel('ROS measurement stamp [s]')
            fig.suptitle(title)
            fig.tight_layout()
            fig.savefig(out / f'pose_four_{mode}_{quantity}.png')
            plt.close(fig)
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        for label, color, values in curves:
            cuts = np.r_[0, np.flatnonzero(np.diff(values[:, 0]) > .25) + 1, len(values)]
            for j, (a, b) in enumerate(zip(cuts[:-1], cuts[1:])):
                ax.plot(*values[a:b, 1:4].T, color=color, lw=1.2, ls='--' if label == 'Truth' else '-',
                        label=label if j == 0 else None)
            ax.scatter(*values[0, 1:4], color=color, s=22)
        ax.set(xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]')
        ax.set_box_aspect((2.5, 1, 1))
        ax.view_init(elev=24, azim=-65)
        ax.legend(fontsize=9, loc='upper left')
        fig.suptitle(title + '\nUnequal axis scales for visibility; dots = first available output')
        fig.tight_layout()
        fig.savefig(out / f'pose_four_{mode}_3d.png')
        plt.close(fig)
    (ROOT / 'docs/pose_four_plot_metadata.json').write_text(json.dumps(dict(
        truth_path=str(truth_path.relative_to(ROOT)), truth_sha256=hashlib.sha256(truth_path.read_bytes()).hexdigest(),
        protocol='All recorded samples; native and first-pose only; no full-window fit; no scale; no estimator changes',
        rpy='body-to-reference rotation Rz(yaw) Ry(pitch) Rx(roll), degrees, unwrapped', methods=metadata), indent=2) + '\n')
    print('Wrote six figures and metadata; first-pose and rigid-distance checks passed.')


if __name__ == '__main__':
    main()
