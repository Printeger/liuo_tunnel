#!/usr/bin/env python3
"""Read-only ledger/graph audit of the retained legacy CAL -> REUSE loop.

This is not the retransmission stress test or a localization acceptance test.
"""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cal', type=Path, required=True)
    parser.add_argument('--reuse', type=Path, required=True)
    args = parser.parse_args()
    source = args.cal / 'export/map/calibrated_map.json'
    loaded = args.reuse / 'metadata/config/map.json'
    assert source.read_bytes() == loaded.read_bytes(), 'loaded map differs from CAL export'
    m = json.loads(source.read_text())
    old = [leaf['physical_id'] for leaf in m['static_edge_ledger']['leaves']]
    assert len(old) == len(set(old)), 'duplicate absorbed leaf'
    delivery = json.loads((args.reuse / 'export/simulation/static_delivery.json').read_text())
    new = set(delivery['unique_physical_ids'])
    assert not set(old) & new, 'independent REUSE input overlaps CAL physical information'
    events = [json.loads(x) for x in (args.reuse / 'export/odometry/static_edges.jsonl').read_text().splitlines()]
    inputs = [x for x in events if x['event'] == 'input']
    queued = {x['record_id'] for x in inputs if x['reason'] == 'QUEUED'}
    # Legacy simulator emits independent one-leaf records (not cumulative blocks).
    assert queued <= new, 'unrecognized queued record identity'
    graphs = list(csv.DictReader((args.reuse / 'export/odometry/graph.csv').open()))
    final = graphs[-1]
    update = [x for x in events if x['event'] == 'graph_update'][-1]
    assert int(final['aa_accumulated_measurements']) == update['fused_leaves'] == len(queued)
    assert int(final['aa_summary_factors']) == update['explicit_factors']
    assert update['pending'] == 0
    assert all(int(x['joint_map_priors']) == 1 for x in graphs)
    assert all(int(x['installation_observation_factors']) == 0 for x in graphs)
    # graph.uwb_aa is the update's AA count, including static summaries;
    # raw acceptance is separately observable in the UWB decision stream.
    raw = list(csv.DictReader((args.reuse / 'export/odometry/uwb.csv').open()))
    assert not any(x['reason'] == 'ACCEPT_AA' for x in raw), 'raw AA duplicated static AA'
    result = dict(
        scope='legacy_engineering_loop_ledger_not_retransmission_or_accuracy_acceptance',
        cal=str(args.cal), reuse=str(args.reuse), map_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        absorbed_prior_leaves=len(old), independent_delivered_leaves=len(new),
        explicit_new_leaves=len(queued), explicit_factors=update['explicit_factors'],
        input_reasons=dict(collections.Counter(x['reason'] for x in inputs)),
        old_new_identity_intersection=0, joint_prior_count=1, installation_factors=0,
        final_stamp=float(final['stamp']),
        limitations=['Does not resend old blocks; production retransmission belongs to A6.',
                     'Does not certify localization accuracy, complete anchor initialization or RF reachability.'])
    output = args.reuse / 'export/analysis/legacy_static_loop_audit.json'
    output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
