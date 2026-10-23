#!/usr/bin/env python3
"""Frozen empty-corridor comparison; no whole-window fitted transform."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate_ape import ROOT, NAMES, load, associate, stats
sys.path.insert(0, str(ROOT / 'tools'))
from evaluation_frames import map_evaluation_datum


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=Path, default=ROOT / 'tools/baselines/empty_runs.json')
    parser.add_argument('--prefix', default='empty_tunnel')
    args = parser.parse_args()
    spec_path = args.runs
    NAMES.update({
        'self_ta': ('本方法无图 TA-only', 'Ours: self, TA only', '#D55E00'),
        'reuse_ta': ('本方法有图 TA-only', 'Ours: reuse, TA only', '#9467bd'),
    })
    spec = json.loads(spec_path.read_text())
    if any(k.endswith('_ta') for k in spec['runs']):
        NAMES['self'] = ('本方法无图 TA+AA', 'Ours: self, TA+AA', '#009E73')
        NAMES['reuse'] = ('本方法有图 TA+AA', 'Ours: reuse, TA+AA', '#CC79A7')
    truth = load(ROOT / spec['source'] / 'export/simulation/truth.csv')
    tr = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))
    def truth_xyz(t):
        return np.column_stack([np.interp(t, truth[:, 0], truth[:, i]) for i in (1, 2, 3)])
    grid = np.round(np.arange(61., 360.9001, .1), 4)
    raw, world, matched, masks, results = {}, {}, {}, {}, {}
    for key, rel in spec['runs'].items():
        run = ROOT / rel
        path = run / 'export/odometry/trajectory.csv'
        values = load(path) if path.exists() else np.zeros((0, 8))
        raw[key] = values
        matched[key], masks[key] = associate(values, grid)
        manifest = json.loads((run / 'metadata/run_manifest.json').read_text())
        identity_path = run / 'metadata/manifests' / ('baseline_input.json' if key == 'fastlio' else 'input_identity.json')
        identity = json.loads(identity_path.read_text())
        declared_source = identity['source'] if key == 'fastlio' else identity['frozen_input']['source_run']
        if Path(declared_source).resolve() != (ROOT / spec['source']).resolve():
            raise ValueError('different raw input source: ' + key)
        expected = identity['capture_sha256'] if key == 'fastlio' else identity['frozen_input']['capture_sha256']
        raw_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT / spec['source'] / 'export/capture/raw').glob('*.mcap')}
        if {Path(k).name:v for k,v in expected.items() if k.endswith('.mcap')} != raw_hashes:
            raise ValueError('different raw input hash: ' + key)
        result = dict(run=rel, input_identity_path=str(identity_path.relative_to(ROOT)), raw_input_sha256=raw_hashes,
                      lifecycle=manifest['lifecycle'], poses=len(values),
                      coverage_percent=float(100 * masks[key].mean()),
                      trajectory_sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None)
        if key.endswith('_ta'):
            config = json.loads((run / 'metadata/config/phase1.json').read_text())
            if config['uwb'].get('use_aa', False) is not False:
                raise ValueError('TA-only comparison config enables AA: ' + key)
            range_path = run / 'export/odometry/uwb.csv'
            ranges = list(csv.DictReader(range_path.open())) if range_path.exists() else []
            graph_path = run / 'export/odometry/graph.csv'
            graphs = list(csv.DictReader(graph_path.open())) if graph_path.exists() else []
            aa_fields = ('uwb_aa', 'aa_summary_factors', 'aa_accumulated_measurements')
            aa_max = {field: max([int(row[field]) for row in graphs], default=0) for field in aa_fields}
            accepted_aa = sum(row['reason'] == 'ACCEPT_AA' for row in ranges)
            accepted_ta = sum(row['reason'] == 'ACCEPT_TA' for row in ranges)
            if accepted_aa or any(aa_max.values()):
                raise ValueError('AA information entered disabled run: ' + key)
            inherited = None
            if config['uwb']['mode'] == 'reuse_map':
                inherited = json.loads((run / 'metadata/config/map.json').read_text()).get('uwb_information')
                if not inherited or inherited.get('use_aa') is not False or inherited.get('accepted_aa_measurements') != 0:
                    raise ValueError('TA-only reuse inherited AA or unknown information')
            result['aa_disabled_audit'] = dict(accepted_aa=accepted_aa, accepted_ta=accepted_ta,
                disabled_aa=sum(row['reason'] == 'AA_ABLATION_DISABLED' for row in ranges), graph_max=aa_max,
                inherited_map_information=inherited, graph_rows=len(graphs))
        failure = run / 'metadata/manifests/input_failure.json'
        if failure.exists(): result['failure'] = json.loads(failure.read_text())
        if len(values):
            if key == 'fastlio':
                # Fixed simulation start datum, not estimated first output or trajectory fit.
                scenario = json.loads((ROOT / spec['source'] / 'metadata/config/scenario.json').read_text())
                x, y, z, yaw = scenario['simulation_start_xyz_yaw']
                r = Rotation.from_euler('z', yaw)
                t = np.array([x, y, z])
            else:
                datum = map_evaluation_datum(run)
                r = Rotation.from_quat([datum[k] for k in ('qx','qy','qz','qw')])
                t = np.array([datum[k] for k in ('x','y','z')])
            w = values.copy()
            w[:, 1:4] = r.apply(values[:, 1:4]) + t
            w[:, 4:8] = (r * Rotation.from_quat(values[:, 4:8])).as_quat()
            world[key] = w
            result.update(first_stamp=float(values[0,0]), last_stamp=float(values[-1,0]),
                          fixed_datum_rotation=r.as_matrix().tolist(), fixed_datum_translation=t.tolist())
            state_counts = {}
            for row in csv.DictReader(path.open()):
                state = row.get('state', 'not_published')
                state_counts[state] = state_counts.get(state, 0) + 1
            result['state_counts'] = state_counts
            resource = run / 'export/simulation/runtime.json'
            if resource.exists(): result['simulation_runtime'] = json.loads(resource.read_text())
            resource = run / 'metadata/manifests/baseline_result.json'
            if resource.exists(): result['external_runtime'] = json.loads(resource.read_text())
        results[key] = result
    active = [k for k in raw if len(raw[k]) and raw[k][-1,0] >= 360.]
    common = np.logical_and.reduce([masks[k] for k in active]) if active else np.zeros(len(grid), dtype=bool)
    errors = {}
    for key, w in world.items():
        own = w[matched[key][masks[key]]]
        if len(own): results[key]['own_window_fixed_datum_ape'] = stats(own[:,1:4]-truth_xyz(own[:,0]))
        chosen = w[matched[key][common & masks[key]]]
        if not len(chosen): continue
        error = chosen[:,1:4] - truth_xyz(chosen[:,0])
        errors[key] = (common & masks[key], error)
        result = results[key]
        result['common_fixed_datum_ape'] = stats(error)
        result['evaluated_samples'] = len(chosen)
        result['partial_window_only'] = key not in active
        attitude = (tr(chosen[:,0]).inv()*Rotation.from_quat(chosen[:,4:8])).magnitude()
        result['attitude_rmse_deg'] = float(np.rad2deg(np.sqrt(np.mean(attitude**2))))
        v = raw[key]
        r0 = tr(v[0,0]) * Rotation.from_quat(v[0,4:8]).inv()
        t0 = truth_xyz([v[0,0]])[0] - r0.apply(v[0,1:4])
        c = v[matched[key][common & masks[key]]]
        result['common_first_pose_ape'] = stats(r0.apply(c[:,1:4])+t0-truth_xyz(c[:,0]))
    out = ROOT/'docs'; figs = out/'figures'
    metadata = dict(source=spec['source'], calibration=spec['calibration'], common_samples=int(common.sum()),
                    grid_samples=len(grid), common_methods=active, excluded_incomplete_methods=[k for k in raw if k not in active], policy='Fixed declared original simulation datum, no fitted transform; offline only', results=results)
    (out/f'{args.prefix}_results.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+'\n')
    plt.rcParams.update({'axes.grid':True,'grid.alpha':.25,'figure.dpi':150})
    curves = [('Truth','black',truth)] + [(NAMES[k][1],NAMES[k][2],v) for k,v in world.items()]
    for quantity in ('xyz','rpy'):
        fig, axes = plt.subplots(3,1,figsize=(12,9),sharex=True)
        for label,color,v in curves:
            y = v[:,1:4] if quantity=='xyz' else np.rad2deg(np.unwrap(Rotation.from_quat(v[:,4:8]).as_euler('xyz'),axis=0))
            cut = np.r_[0,np.flatnonzero(np.diff(v[:,0])>.25)+1,len(v)]
            for i,ax in enumerate(axes):
                for j,(a,b) in enumerate(zip(cut[:-1],cut[1:])):
                    ax.plot(v[a:b,0],y[a:b,i],color=color,ls='--' if label=='Truth' else '-',lw=1,label=label if j==0 else None)
                ax.set_ylabel((('X','Y','Z')[i]+' [m]') if quantity=='xyz' else (('Roll','Pitch','Yaw')[i]+' [deg]'))
        axes[0].legend();axes[-1].set_xlabel('ROS measurement stamp [s]')
        fig.suptitle('Empty tunnel | fixed original datum | NO trajectory fitting')
        fig.tight_layout();fig.savefig(figs/f'{args.prefix}_fixed_{quantity}.png');plt.close(fig)
    fig=plt.figure(figsize=(12,8));ax=fig.add_subplot(111,projection='3d')
    for label,color,v in curves:
        p=v[:,1:4].copy();p[np.flatnonzero(np.diff(v[:,0])>.25)+1]=np.nan
        ax.plot(*p.T,color=color,label=label,ls='--' if label=='Truth' else '-',lw=1)
    ax.set(xlabel='X [m]',ylabel='Y [m]',zlabel='Z [m]');ax.set_box_aspect((2.5,1,1));ax.legend()
    fig.suptitle('Empty tunnel | fixed original datum | NO trajectory fitting\nIndependent axis scales; read ticks')
    fig.tight_layout();fig.savefig(figs/f'{args.prefix}_fixed_3d.png');plt.close(fig)
    fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True)
    for key,(mask,e) in errors.items():
        for i,ax in enumerate(axes):
            ax.plot(grid[mask],np.linalg.norm(e,axis=1) if i==0 else e[:,i-1],label=NAMES[key][1],color=NAMES[key][2],lw=1)
    for ax,label in zip(axes,['APE norm [m]','X error [m]','Y error [m]','Z error [m]']): ax.set_ylabel(label)
    axes[0].legend();axes[-1].set_xlabel('ROS measurement stamp [s]')
    fig.suptitle('Empty tunnel | fixed datum errors | no fitted alignment')
    fig.tight_layout();fig.savefig(figs/f'{args.prefix}_fixed_errors.png');plt.close(fig)
    print(json.dumps({k:{'poses':v['poses'],'ape':v.get('common_fixed_datum_ape'),'states':v.get('state_counts')} for k,v in results.items()},ensure_ascii=False,indent=2))

if __name__=='__main__': main()
