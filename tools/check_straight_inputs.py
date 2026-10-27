#!/usr/bin/env python3
"""Offline full ordinary-S coverage and independent noise audit; never estimator input."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def mix(value):
    # Independent Python implementation of the declared SplitMix64 counter.
    with np.errstate(over='ignore'):
        x = np.asarray(value, dtype=np.uint64) + np.uint64(0x9e3779b97f4a7c15)
        x = (x ^ (x >> np.uint64(30))) * np.uint64(0xbf58476d1ce4e5b9)
        x = (x ^ (x >> np.uint64(27))) * np.uint64(0x94d049bb133111eb)
        return x ^ (x >> np.uint64(31))


def noise(seed, stream, samples, component):
    key = mix(seed) ^ mix(stream + 1000) ^ mix(samples + 1000000) ^ mix(component + 1000000000)
    with np.errstate(over='ignore'):
        u = ((mix(key) >> np.uint64(11)).astype(float) + .5) / 2**53
        v = ((mix(key + np.uint64(1)) >> np.uint64(11)).astype(float) + .5) / 2**53
    return np.sqrt(-2*np.log(u))*np.cos(2*np.pi*v)


def inspect(run, seed, start):
    scenario = json.loads((run/'metadata/config/scenario.json').read_text())
    config = json.loads((run/'metadata/config/phase1.json').read_text())
    assert scenario['seed'] == seed and scenario['simulation_start_xyz_yaw'] == start
    assert scenario['duration_s'] == 540 and scenario['motion'] == 'outbound'
    assert scenario['scan_model'] == 'snapshot' and not scenario['return_trip']
    assert config['uwb']['enforce_reporter_delivery'] and config['uwb']['mode'] == 'record_only'
    identity = json.loads((run/'metadata/manifests/input_identity.json').read_text())
    assert identity['capture_only'] and 'odometry' not in identity['binaries']
    capture = json.loads((run/'metadata/manifests/capture_only_result.json').read_text())
    assert capture['raw_input_valid'] and capture['localization'] == 'NOT_RUN'
    audit = json.loads((run/'metadata/manifests/observation_audit.json').read_text())
    assert not audit['violations'] and not audit['duplicate_uwb'] and audit['odometry_count'] == 0
    assert audit['counts']['cloud'] == 5400 and audit['counts']['imu'] == audit['counts']['clock'] == 108000
    files = json.loads((run/'metadata/manifests/captured_inputs.json').read_text())
    raw_hashes = {}
    for relative, expected in files.items():
        with (run/relative).open('rb') as stream:
            actual = hashlib.file_digest(stream, 'sha256').hexdigest()
        assert actual == expected, relative
        if relative.endswith('.mcap'):
            raw_hashes[relative] = actual
    assert raw_hashes
    truth = read_csv(run/'export/simulation/truth.csv')
    t = np.array([float(r['stamp']) for r in truth])
    p = np.array([[float(r[k]) for k in ('x','y','z')] for r in truth])
    v = np.array([[float(r[k]) for k in ('vx','vy','vz')] for r in truth])
    q = np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in truth])
    assert abs(t[0]-1) < 1e-8 and t[-1] >= 540.9-1e-8
    assert np.max(np.abs(p[0]-start[:3])) < 1e-8
    assert abs(p[-1,0]-2350) < 1e-8 and np.linalg.norm(v[-1]) < 1e-8
    assert np.min(np.diff(p[:,0])) >= -1e-8 and np.max(np.linalg.norm(v,axis=1)) <= 5+1e-8
    assert np.max(np.abs(p[:,1:]-np.asarray(start[1:3]))) < 1e-8
    assert np.max(np.abs(q-q[0])) < 1e-8
    assert np.linalg.norm(Rotation.from_quat(q[0]).as_matrix()-Rotation.from_euler('z',start[3]).as_matrix()) < 1e-8
    assert np.max(np.linalg.norm(v[(t<=6)|(t>=531)],axis=1)) < 1e-8
    imu = np.genfromtxt(run/'export/capture/imu.csv',delimiter=',',names=True)
    assert len(imu) == 108000 and np.max(np.abs(imu['stamp']-(1+np.arange(len(imu))*.005))) < 1e-8
    acceleration = CubicHermiteSpline(t,p,v)(imu['stamp'],2)
    force = Rotation.from_quat(q[0]).inv().apply(acceleration + [0,0,9.80665])
    raw = np.column_stack([imu[k] for k in ('ax','ay','az','wx','wy','wz')])
    theory = np.column_stack([force,np.zeros_like(force)])
    residual = raw-theory
    sample = np.arange(len(imu),dtype=np.uint64)
    s = config['sensors']
    for j in range(6):
        stream = 20 if j<3 else 21
        density = s['sigma_ba_density' if j<3 else 'sigma_bg_density']
        residual[:,j] -= np.cumsum(noise(seed,stream,sample,j%3))*density*np.sqrt(.005)
        # An independent raw-field check bounds interpolation error before correlation.
        sigma = s['sigma_acc_sample' if j<3 else 'sigma_gyro_sample']
        discrepancy = residual[:,j]-sigma*noise(seed,10 if j<3 else 11,sample,j%3)
        assert np.max(np.abs(discrepancy)) < (.002 if j<3 else 1e-10)
    physical = read_csv(run/'export/simulation/physical_uwb.csv')
    ranges = {kind: {(r['a'],r['b'],r['sample_index']):float(r['range'])-float(r['true_range'])
                     for r in physical if r['a_type']==kind} for kind in ('0','1')}
    return dict(run=str(run),seed=seed,raw_sha256=raw_hashes,semantic_sha256=audit['raw_sha256'],
                counts=audit['counts'],start=p[0].tolist(),end=p[-1].tolist(),last_truth_stamp=float(t[-1])), residual, ranges


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first',type=Path,required=True)
    parser.add_argument('--second',type=Path)
    args=parser.parse_args()
    first, noise_a, ranges_a=inspect(args.first,0,[0,0,1.2,0])
    result=dict(result='PASS_FULL_S_CAPTURE',scope='ordinary S raw input only; no localization acceptance',first=first)
    destination=args.first
    if args.second:
        second,noise_b,ranges_b=inspect(args.second,1,[0,.2,1.6,np.pi/2])
        assert set(first['raw_sha256'].values()).isdisjoint(second['raw_sha256'].values())
        for kind in ('cloud','imu','uwb'):
            assert first['semantic_sha256'][kind] != second['semantic_sha256'][kind]
        assert first['semantic_sha256']['clock'] == second['semantic_sha256']['clock']
        imu_correlations=[float(np.corrcoef(noise_a[:,j],noise_b[:,j])[0,1]) for j in range(6)]
        assert max(abs(v) for v in imu_correlations) < .05
        uwb_correlations={}
        for kind in ('0','1'):
            common=sorted(ranges_a[kind].keys() & ranges_b[kind].keys())
            assert len(common)>1000
            value=float(np.corrcoef([ranges_a[kind][k] for k in common],[ranges_b[kind][k] for k in common])[0,1])
            assert abs(value)<.05
            uwb_correlations[kind]=dict(samples=len(common),correlation=value)
        result.update(result='PASS_FULL_S_INDEPENDENT_INPUTS',second=second,
                      imu_white_noise_correlations=imu_correlations,uwb_noise_correlations=uwb_correlations)
        destination=args.second
    output=destination/'export/analysis/straight_inputs.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
