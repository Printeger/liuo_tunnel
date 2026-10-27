#!/usr/bin/env python3
"""Audit D2 graph, decisions and biases without reading vehicle truth."""
import argparse
import collections
import csv
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--observations', type=Path, required=True)
    parser.add_argument('--start', type=float, required=True)
    parser.add_argument('--end', type=float, required=True)
    args = parser.parse_args()
    run = args.run
    manifest = json.loads((run/'metadata/manifests/conditional_diagnostic.json').read_text())
    assert manifest['role'] == 'D2_ORACLE_ANCHORS_NOT_DEPLOYABLE'
    samples = json.loads(args.observations.read_text())
    assert samples['raw_sha256'] == manifest['raw_sha256']
    read = lambda path: list(csv.DictReader((run/path).open()))
    graph = read('export/odometry/graph.csv')
    assert graph and not list((run/'export/map').glob('*.json'))
    for row in graph:
        for name in ('A', 'uwb_aa', 'aa_summary_factors', 'aa_accumulated_measurements',
                     'map_priors', 'joint_map_priors', 'installation_observation_factors'):
            assert int(row[name]) == 0, (name, row[name])
    decisions = read('export/odometry/uwb.csv')
    # Match physical input values exactly, separately from their accept/reject outcome.
    expected = collections.Counter((v['measurement']['acquisition_ns'], v['measurement']['endpoints'][1],
                                    v['measurement']['range_m'], v['measurement']['variance_m2'])
                                   for v in samples['samples']['ta'].values())
    actual = collections.Counter()
    for row in decisions:
        if int(row['a_type']) == 0:
            # The raw message is float32; the decimal CSV prints 16 significant
            # digits of its widened double. Recover the exact wire value rather
            # than treating decimal serialization roundoff as changed data.
            key = (round(float(row['measurement_stamp'])*1e9), int(row['b']), float(np.float32(row['range'])), float(row['variance']))
            actual[key] += 1
    # A decision may be logged once at enqueue and again on later acceptance.
    assert set(actual) == set(expected), 'D2 decision coverage differs from frozen TA input'
    focus = lambda rows, field='stamp': [r for r in rows if args.start <= float(r[field]) <= args.end]
    selected = focus(decisions, 'measurement_stamp')
    accepted = [r for r in selected if r['reason'] == 'ACCEPT_TA']
    residuals = np.array([float(r['residual']) for r in accepted])
    finite = residuals[np.isfinite(residuals)]
    timing = focus(read('profiling/odometry.csv'))
    biases = np.array([[float(r[k]) for k in ('ba_x','ba_y','ba_z','bg_x','bg_y','bg_z')] for r in timing])
    trajectory = focus(read('export/odometry/trajectory.csv'))
    result = dict(role=manifest['role'], scope='D2 factor/input/bias audit; not joint or full-route localization acceptance',
                  raw_sha256=manifest['raw_sha256'], measurement_set_sha256=samples['measurement_set_sha256'],
                  focus=[args.start,args.end], output_samples=len(trajectory),
                  states=dict(collections.Counter(r['state'] for r in trajectory)),
                  ta_physical_input_covered=len(actual), ta_input_decision_entries=sum(actual.values()),
                  focus_decisions=dict(collections.Counter(r['reason'] for r in selected)),
                  focus_ta_accepted=len(accepted), finite_logged_ta_residuals=len(finite),
                  logged_ta_residual_rmse_m=float(np.sqrt(np.mean(finite**2))) if len(finite) else None,
                  bias_columns=['ba_x','ba_y','ba_z','bg_x','bg_y','bg_z'],
                  bias_min=biases.min(axis=0).tolist(), bias_max=biases.max(axis=0).tolist(),
                  exact_anchor_constants=True, aa_vehicle_factors=0, vehicle_truth_input=False,
                  limitations=['Logged TA residuals use their recorded decision state; not a reconstructed common batch residual.',
                               'Input coverage is separate from actual acceptance and fixed-window retention.'])
    path = run/'export/analysis/conditional_vehicle_audit.json'
    path.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
