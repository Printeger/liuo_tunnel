#!/usr/bin/env python3
"""Audit a matched CAL TA+AA/TA-only pair; no claimed statistical causal effect."""
import argparse
from collections import Counter
import copy
import csv
import json
from pathlib import Path


def read(run, name):
    return json.loads((run/name).read_text())


def rows(run, name):
    with (run/name).open() as stream:
        return list(csv.DictReader(stream))


def static_messages(callbacks):
    # CDR alignment padding is not message data and need not be deterministic.
    # Compare every decoded field (including delivery/acquisition time), without
    # dropping identities, headers, payload fields or duplicate multiplicity.
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from liuo_tunnel_msgs.msg import StaticAnchorRecord
    return Counter(json.dumps(message_to_ordereddict(
        deserialize_message(bytes.fromhex(row['cdr_hex']), StaticAnchorRecord)),
        sort_keys=True, allow_nan=False) for row in callbacks)


def compare(full, ta_only):
    runs = [full, ta_only]
    identities = [read(run, 'metadata/manifests/input_identity.json') for run in runs]
    for field in ('raw_sha256', 'binary_sha256', 'shared_libraries', 'scan_model', 'seconds', 'rate', 'end_exclusive'):
        assert identities[0][field] == identities[1][field], field
    configs = [read(run, 'metadata/config/phase1.json') for run in runs]
    assert configs[0]['uwb']['use_aa'] is True and configs[1]['uwb']['use_aa'] is False
    comparable = copy.deepcopy(configs)
    for config in comparable:
        assert config['uwb']['mode'] == 'self_calibrate_free', 'CAL-only comparison; no inherited AA prior'
        config['uwb'].pop('use_aa')
    assert comparable[0] == comparable[1], 'configuration differs beyond use_aa'
    assert read(full, 'metadata/config/local_installation.json') == read(ta_only, 'metadata/config/local_installation.json')
    publications = [read(run, 'metadata/manifests/published_input.json') for run in runs]
    for field in ('counts', 'raw_sha256', 'requested_seconds'):
        assert publications[0][field] == publications[1][field], field
    callbacks = []
    for run in runs:
        callbacks.append([json.loads(line) for line in (run/'export/odometry/static_edges.jsonl').read_text().splitlines()
                          if json.loads(line)['event'] == 'input'])
        assert not (run/'metadata/config/reuse_map.json').exists(), 'inherited map forbidden'
        smoke = read(run, 'export/analysis/'+identities[0]['scan_model']+'_smoke.json')
        assert smoke['result'] == 'PASS' and smoke['raw_counts'] == publications[0]['counts']
        assert smoke['raw_sha256'] == publications[0]['raw_sha256']
    assert callbacks[0] and callbacks[1], 'no real static delivery'
    assert all(row['reason'] == 'QUEUED' for row in callbacks[0])
    assert all(row['reason'] == 'AA_ABLATION_DISABLED' for row in callbacks[1])
    assert static_messages(callbacks[0]) == static_messages(callbacks[1]), 'decoded static inputs differ'
    audit = read(full, 'export/analysis/road_static_audit.json')
    assert audit['result'] == 'PASS' and audit['final_fused_leaves'] > 0, 'comparator has no actual AA fusion'
    models = [read(run, 'export/map/calibrated_map.json') for run in runs]
    assert models[0]['static_edge_ledger']['leaves'] and not models[1]['static_edge_ledger']['leaves']
    assert models[1]['uwb_information'] == dict(accepted_aa_measurements=0, ta_enabled=True, use_aa=False)
    graph = rows(ta_only, 'export/odometry/graph.csv')
    for row in graph:
        assert all(int(row[field]) == 0 for field in ('uwb_aa', 'aa_summary_factors', 'aa_accumulated_measurements', 'map_priors'))
    accepted_ta = [sum(row['reason'] == 'ACCEPT_TA' for row in rows(run, 'export/odometry/uwb.csv')) for run in runs]
    assert min(accepted_ta) > 0, 'no actual TA use'
    diagnostics = [read(run, 'export/analysis/road_fixed_start_diagnostic.json') for run in runs]
    assert all(d['gauge'] == 'first_output_yaw_and_translation_only_no_route_fit' for d in diagnostics)
    result = dict(result='PASS_MATCHED_INPUT_AND_AA_ABLATION',
                  scope='same sensor input and every decoded static field/multiplicity, only AA switch differs, actual graph/ledger use; not statistical causal or localization acceptance',
                  static_comparison='all decoded fields including headers; CDR alignment padding excluded',
                  runs=[str(run) for run in runs], raw_sha256=identities[0]['raw_sha256'], seconds=identities[0]['seconds'],
                  static_records_each=len(callbacks[0]), full_fused_aa=audit['final_fused_leaves'], ta_only_fused_aa=0,
                  accepted_ta=accepted_ta, initialized_anchors=[[a['id'] for a in model['anchors']] for model in models],
                  position_rmse_m=[d['position_rmse_m'] for d in diagnostics],
                  maximum_position_error_m=[d['maximum_position_error_m'] for d in diagnostics],
                  attitude_rmse_deg=[d['attitude_rmse_deg'] for d in diagnostics],
                  velocity_rmse_mps=[d.get('velocity_rmse_mps') for d in diagnostics])
    (ta_only/'export/analysis/ta_ablation_pair.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--full', type=Path, required=True)
    parser.add_argument('--ta-only', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(compare(args.full, args.ta_only), indent=2))
