#!/usr/bin/env python3
"""Audit recorded candidate TA identities and reconstruct pending AA prefixes."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
from rclpy.serialization import deserialize_message
from liuo_tunnel_msgs.msg import StaticAnchorRecord


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--observations', type=Path, required=True)
    args = parser.parse_args()
    saved = json.loads(args.observations.read_text())
    identity = json.loads((args.run/'metadata/manifests/input_identity.json').read_text())
    assert identity['raw_sha256'] == saved['raw_sha256']
    raw_ta = {}
    for physical, row in saved['samples']['ta'].items():
        for delivery in row['deliveries']:
            raw_ta[(delivery['source_session'],delivery['source_sequence'],row['measurement']['endpoints'][1])] = (physical,row['measurement'])
    aa_by_pair = collections.defaultdict(list)
    seen = set()
    for row in map(json.loads,(args.run/'export/odometry/static_edges.jsonl').read_text().splitlines()):
        if row['event'] != 'input' or row['reason'] != 'QUEUED':
            continue
        msg = deserialize_message(bytes.fromhex(row['cdr_hex']),StaticAnchorRecord)
        for leaf in msg.leaves:
            if leaf.physical_id in seen:
                continue
            seen.add(leaf.physical_id)
            assert leaf.physical_id in saved['samples']['aa']
            aa_by_pair[tuple(sorted((msg.first,msg.second)))].append((leaf.physical_id,leaf.range_m))
    contexts = [json.loads(x) for x in (args.run/'profiling/candidate_inputs.jsonl').read_text().splitlines()]
    patches = {(r['session'],r['pose']):r for r in map(json.loads,(args.run/'profiling/local_geometry.jsonl').read_text().splitlines())}
    results = []
    for row in contexts:
        ta_ids = []
        for sample in row['ta']:
            physical, expected = raw_ta[(sample['session'],sample['sequence'],row['anchor'])]
            assert round(sample['stamp']*1e9) == expected['acquisition_ns']
            assert sample['range'] == expected['range_m']
            assert sample['variance'] == .1*.1
            assert 0 <= sample['beta'] <= 1
            ta_ids.append(physical)
        assert len(ta_ids) == len(set(ta_ids))
        aa_ids = []
        for summary in row['static_aa']:
            # New candidate endpoint is not yet known: incident static leaves
            # are pending and have not been absorbed into a prior/explicit edge.
            prefix = aa_by_pair[tuple(sorted((summary['first'],summary['second'])))][:summary['count']]
            assert len(prefix) == summary['count']
            mean = math.fsum(x[1] for x in prefix)/len(prefix)
            scatter = math.fsum((x[1]-mean)**2/.01 for x in prefix)
            assert abs(summary['mean']-mean) <= 1e-9
            assert abs(summary['scatter']-scatter) <= 1e-7
            assert summary['weight'] == 100.*len(prefix)
            aa_ids += [x[0] for x in prefix]
        if 'local' in row:
            local = row['local']
            assert patches[(row['session'],local['pose'])]['stamp'] == local['stamp']
            assert len(row['seeds_local']) == len(row['seeds'])
        ids = dict(ta=ta_ids,aa=aa_ids)
        results.append(dict(anchor=row['anchor'],stamp=row['stamp'],oldest=row['oldest'],
                            oldest_age=row['stamp']-row['oldest'],factor_count=row['factors'],
                            seed_count=len(row['seeds']),effective_ta=len(ta_ids),effective_aa=len(aa_ids),
                            identities=ids,subset_sha256=hashlib.sha256(json.dumps(ids,sort_keys=True).encode()).hexdigest()))
    output = dict(scope='actual logged candidate TA; AA received-order pending prefix verified against count/mean/weight/scatter',
                  raw_sha256=saved['raw_sha256'],available_measurement_set_sha256=saved['measurement_set_sha256'],
                  covariance_assumption='declared independent synthetic samples; identity alone does not establish independence',
                  limitations=['AA identities reconstructed from ordered received prefixes; not a separate per-factor leaf export.',
                               'Candidate graph also includes existing joint factors and marginal information; these lists are newly introduced range factors.'],
                  batches=results)
    (args.run/'export/analysis/candidate_input_audit.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(dict(batches=len(results),maximum_oldest_age=max(r['oldest_age'] for r in results))))


if __name__ == '__main__':
    main()
