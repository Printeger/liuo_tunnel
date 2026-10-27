#!/usr/bin/env python3
"""Offline two-seed input check. Truth is never passed to an estimator."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def read_json(path):
    return json.loads(path.read_text())


def imu_noise(run, truth):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / 'export/capture/raw'), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/sensors/imu']))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, Imu)
        i = len(rows)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - 1
        if abs(t-truth['t'][i]) > 1e-8: raise ValueError('IMU/truth time mismatch')
        acc, gyro = m.linear_acceleration, m.angular_velocity
        rows.append(np.r_[np.array([acc.x,acc.y,acc.z])-truth['force'][i]-truth['acc_bias'][i],
                          np.array([gyro.x,gyro.y,gyro.z])-truth['omega'][i]-truth['gyro_bias'][i]])
    if len(rows) != len(truth['t']): raise ValueError('incomplete IMU')
    return np.array(rows)


def correlations(a, b):
    return [float(np.corrcoef(a[:,i], b[:,i])[0,1]) for i in range(a.shape[1])]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--first', type=Path, required=True)
    p.add_argument('--second', type=Path, required=True)
    args = p.parse_args()
    runs = [args.first.resolve(strict=True), args.second.resolve(strict=True)]
    configs = [read_json(r/'metadata/config/scene.json') for r in runs]
    seeds = [c.pop('seed') for c in configs]
    if seeds != [20261027, 20261028] or configs[0] != configs[1]: raise ValueError('frozen input configuration changed')
    for name in ['export/simulation/anchors.json','export/simulation/facilities.json','export/simulation/motion_plan.json','metadata/config/installation_metadata.json']:
        if read_json(runs[0]/name) != read_json(runs[1]/name): raise ValueError('physical scene changed: '+name)
    # Materialize once: NpzFile indexing rereads the whole stored array.
    truths = []
    for run in runs:
        with np.load(run/'export/simulation/trajectory.npz') as archive:
            truths.append({key: archive[key] for key in archive.files})
    physical_keys = [k for k in truths[0] if k not in ['acc_bias','gyro_bias']]
    for key in physical_keys:
        if not np.array_equal(truths[0][key], truths[1][key]): raise ValueError('trajectory changed: '+key)
    for key in ['acc_bias','gyro_bias']:
        if np.array_equal(truths[0][key], truths[1][key]): raise ValueError('bias stream reused')
    summaries = [read_json(r/'export/analysis/generation.json') for r in runs]
    for key in ['frames','points','counts','end_s','start_s']:
        if summaries[0][key] != summaries[1][key]: raise ValueError('schedule changed: '+key)
    for topic in summaries[0]['raw_semantic_sha256']:
        equal = summaries[0]['raw_semantic_sha256'][topic] == summaries[1]['raw_semantic_sha256'][topic]
        if equal != (topic == '/clock'): raise ValueError('unexpected stream identity: '+topic)
    hashes = []
    for run in runs:
        files = list((run/'export/capture/raw').glob('*.mcap'))
        if len(files) != 1: raise ValueError('expected one raw MCAP')
        with files[0].open('rb') as f: digest = hashlib.file_digest(f,'sha256').hexdigest()
        declared = read_json(run/'metadata/manifests/scene_artifacts.json')['files'][str(files[0].relative_to(run))]['sha256']
        if digest != declared: raise ValueError('MCAP checksum mismatch')
        hashes.append(digest)
    white = [imu_noise(r,t) for r,t in zip(runs,truths)]
    imu_corr = correlations(*white)
    physical = [list(csv.DictReader((r/'export/simulation/physical_uwb.csv').open())) for r in runs]
    if len(physical[0]) != len(physical[1]): raise ValueError('exchange count changed')
    positions = {int(x['id']):np.array(x['position']) for x in read_json(runs[0]/'export/simulation/anchors.json')}
    noise = {0:[[],[]], 1:[[],[]]}
    for a,b in zip(*physical):
        if {k:v for k,v in a.items() if k!='distance'} != {k:v for k,v in b.items() if k!='distance'}:
            raise ValueError('physical exchange schedule/LOS changed')
        if a['valid'] != '1': continue
        kind = int(a['kind']); t = float(a['t'])
        first = positions[int(a['a'])] if kind else np.array([np.interp(t,truths[0]['t'],truths[0]['p'][:,j]) for j in range(3)])
        distance = np.linalg.norm(first-positions[int(a['b'])])
        for i,row in enumerate([a,b]): noise[kind][i].append(float(row['distance'])-distance)
    uwb_corr = {str(kind):float(np.corrcoef(*rows)[0,1]) for kind,rows in noise.items()}
    if not all(np.isfinite(c) and abs(c)<.05 for c in imu_corr+list(uwb_corr.values())):
        raise ValueError('preregistered cross-stream correlation tolerance failed')
    report = dict(result='PASS', scope='offline independent synthetic noise and identical physical scene; no localization acceptance',
                  runs=[str(r) for r in runs], seeds=seeds, raw_sha256=hashes, exact_trajectory_keys=physical_keys,
                  imu_white_cross_correlation=imu_corr, uwb_cross_correlation=uwb_corr,
                  uwb_samples={str(k):len(v[0]) for k,v in noise.items()}, correlation_absolute_limit=.05,
                  source_identity='new physical samples; same deployment; no old sample relabeling')
    (runs[1]/'export/analysis/independent_input.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__ == '__main__':
    main()
