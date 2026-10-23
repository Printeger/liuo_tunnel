#!/usr/bin/env python3
"""Plot recorded poses without any whole-trajectory fit; offline truth only."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate_ape import ROOT, NAMES, load

KEYS = ('fastlio', 'self', 'reuse', 'off')


def broken_line(times, values, angular=False):
    """Do not draw through missing output or Euler wrap discontinuities."""
    values = np.array(values, copy=True)
    cuts = np.diff(times) > .25
    if angular:
        cuts |= np.abs(np.diff(values)) > 180
    values[np.flatnonzero(cuts) + 1] = np.nan
    return values


def main():
    spec = json.loads((ROOT/'tools/baselines/runs.json').read_text())
    truth_path = ROOT/spec['source']/'export/simulation/truth.csv'
    truth = load(truth_path)
    truth_rotation = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))
    out = ROOT/'docs/figures'
    records = {}
    metadata = {'coordinate_policy': 'native and one first-output-pose transform; no whole-window fit, scale fit or subsequent correction',
                'euler_convention': 'extrinsic xyz, R=Rz(yaw)Ry(pitch)Rx(roll), degrees; wrapped [-180,180]; discontinuities broken',
                'truth_sha256': hashlib.sha256(truth_path.read_bytes()).hexdigest(), 'runs': {}}
    for key in KEYS:
        path = ROOT/spec['runs'][key]/'export/odometry/trajectory.csv'
        v = load(path)
        if v[0, 0] < truth[0, 0] or v[-1, 0] > truth[-1, 0]:
            raise ValueError('trajectory outside truth time support: '+key)
        p0 = np.array([np.interp(v[0, 0], truth[:, 0], truth[:, i]) for i in (1, 2, 3)])
        r = truth_rotation(v[0, 0]) * Rotation.from_quat(v[0, 4:8]).inv()
        t = p0-r.apply(v[0, 1:4])
        transformed = v.copy()
        transformed[:, 1:4] = r.apply(v[:, 1:4])+t
        transformed[:, 4:8] = (r*Rotation.from_quat(v[:, 4:8])).as_quat()
        records[key] = {'native': v, 'first_pose': transformed}
        metadata['runs'][key] = {'source': str(path.relative_to(ROOT)), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                'poses': len(v), 'first_stamp': float(v[0, 0]), 'rotation': r.as_matrix().tolist(), 'translation': t.tolist()}
        # The coordinate transform must map the initial pose exactly, without changing relative motion.
        assert np.allclose(transformed[0, 1:4], p0)
        assert (Rotation.from_quat(transformed[0, 4:8]).inv()*truth_rotation(v[0, 0])).magnitude() < 1e-10
        assert np.allclose(np.linalg.norm(np.diff(v[:, 1:4], axis=0), axis=1),
                           np.linalg.norm(np.diff(transformed[:, 1:4], axis=0), axis=1))
    plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': .25, 'figure.dpi': 160})
    for mode in ('native', 'first_pose'):
        title = 'Native frames (NOT a common world frame)' if mode == 'native' else 'One first-pose transform only (NO whole-window fit)'
        series = {'truth': truth, **{k: records[k][mode] for k in KEYS}}
        for quantity in ('xyz', 'rpy'):
            fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
            for key, v in series.items():
                values = v[:, 1:4] if quantity == 'xyz' else Rotation.from_quat(v[:, 4:8]).as_euler('xyz', degrees=True)
                label, color = ('Truth', 'black') if key == 'truth' else (NAMES[key][1], NAMES[key][2])
                for col in range(2):
                    if col == 1 and key == 'off':
                        continue
                    for i in range(3):
                        axes[i, col].plot(v[:, 0], broken_line(v[:, 0], values[:, i], quantity == 'rpy'),
                                          color=color, label=label, lw=1.1, ls='--' if key == 'truth' else '-')
            labels = ('X [m]', 'Y [m]', 'Z [m]') if quantity == 'xyz' else ('Roll [deg]', 'Pitch [deg]', 'Yaw [deg]')
            for i, label in enumerate(labels):
                for col in range(2):
                    axes[i, col].set_ylabel(label)
                    axes[i, col].set_xlim(truth[0, 0], truth[-1, 0])
            axes[0, 0].set_title('All four methods + truth (full range)')
            axes[0, 1].set_title('Detail: FAST-LIO2 + UWB methods + truth; OFF at left')
            for ax in axes[0]:
                ax.legend(fontsize=8)
            for ax in axes[-1]:
                ax.set_xlabel('ROS measurement time [s]')
            fig.suptitle(title+' | '+quantity.upper())
            fig.tight_layout(rect=(0, 0, 1, .96))
            fig.savefig(out/f'pose_four_{mode}_{quantity}.png')
            plt.close(fig)
        fig = plt.figure(figsize=(16, 7))
        for col in range(2):
            ax = fig.add_subplot(1, 2, col+1, projection='3d')
            for key, v in series.items():
                if col == 1 and key == 'off':
                    continue
                p = v[:, 1:4].copy()
                p[np.flatnonzero(np.diff(v[:, 0]) > .25)+1] = np.nan
                label, color = ('Truth', 'black') if key == 'truth' else (NAMES[key][1], NAMES[key][2])
                ax.plot(*p.T, color=color, label=label, lw=1, ls='--' if key == 'truth' else '-')
            ax.set(xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]')
            ax.set_title('All four + truth' if col == 0 else 'Detail: OFF shown at left')
            ax.legend(fontsize=8)
            ax.set_box_aspect((1.6, 1, 1))
        fig.suptitle(title+'\n3D axes use independent scales; see ticks (not equal metric aspect)')
        fig.tight_layout(rect=(0, 0, 1, .91))
        fig.savefig(out/f'pose_four_{mode}_3d.png')
        plt.close(fig)
    (ROOT/'docs/pose_four_plot_provenance.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print('Saved six pose figures; verified first-pose transforms and invariant step lengths.')


if __name__ == '__main__':
    main()
