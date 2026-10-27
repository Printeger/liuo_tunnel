#!/usr/bin/env python3
"""Audit predeclared identities against actual production callback and graph logs."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def audit(run):
    plan_path = run / 'metadata/config/uwb_contract_expected.json'
    plan = json.loads(plan_path.read_text())
    config = json.loads((run / 'metadata/config/phase1.json').read_text())
    manifest = json.loads((run / 'metadata/run_manifest.json').read_text())
    failures = []
    def check(ok, why):
        if not ok:
            failures.append(why)
    def rows(path):
        with (run / path).open() as stream:
            return list(csv.DictReader(stream))
    batches = rows('export/odometry/uwb_batches.csv')
    decisions = rows('export/odometry/uwb_input_decisions.csv')
    graph = rows('export/odometry/graph.csv')
    expected = []
    check(manifest['lifecycle'] == 'completed', 'lifecycle incomplete')
    check(manifest['source']['git_worktree_clean'], 'dirty source')
    check(len(batches) == len(plan['cases']), 'batch delivery count mismatch')
    for case, batch in zip(plan['cases'], batches):
        wire = case['message']
        check(int(batch['driver_session']) == wire['driver_session_id'] and int(batch['batch_seq']) == wire['batch_seq'], case['name'] + ': batch identity')
        check(batch['reason'] == case['batch_reason'], case['name'] + ': batch reason')
        check(int(batch['observations']) == len(wire['observations']), case['name'] + ': batch size')
        for index, reason in enumerate(case['reasons']):
            observation = wire['observations'][index]
            expected.append((case['name'], wire, observation, index, reason))
    check(len(decisions) == len(expected), 'item delivery count mismatch')
    for (name, wire, o, index, reason), row in zip(expected, decisions):
        fields = dict(driver_session=wire['driver_session_id'], batch_seq=wire['batch_seq'], item=index,
                      reporter=o['reporter_id'], source_session=o['source_session_id'], source_sequence=o['source_frame_seq'],
                      kind=o['kind'], node_a=o['node_a'], node_b=o['node_b'])
        check(all(int(row[k]) == v for k,v in fields.items()), name + ': item identity')
        if reason == 'ACCEPT':
            reason = 'RECORD_ONLY' if config['uwb']['mode'] == 'record_only' else 'QUEUED'
        check(row['reason'] == reason, name + ': expected ' + reason + ', got ' + row['reason'])
    check(bool(graph), 'no estimator graph output')
    if config['uwb']['mode'] == 'record_only':
        check(all(all(int(r[k]) == 0 for k in ['uwb_ta', 'uwb_aa', 'aa_summary_factors', 'aa_accumulated_measurements']) for r in graph), 'OFF added UWB information')
    else:
        check(False, 'graph fusion fixture assertions not implemented; callback-only result cannot accept fusion')
    observer = json.loads((run / 'metadata/manifests/observation_audit.json').read_text())
    runtime = json.loads((run / 'export/simulation/runtime.json').read_text())
    check(observer['raw_sha256'] == runtime['delivered_sha256'], 'published/observer semantic hashes differ')
    result = dict(result='FAIL' if failures else 'PASS', failures=failures,
                  scope='public production callback contract and OFF graph isolation only',
                  plan_sha256=hashlib.sha256(plan_path.read_bytes()).hexdigest(),
                  source=manifest['source'], batches=len(batches), decisions=len(decisions), graph_updates=len(graph),
                  generic_evaluation='intentionally malformed inputs remain failures in nominal evaluator; no nominal failures suppressed')
    (run / 'export/analysis/uwb_contract_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    result = audit(parser.parse_args().run)
    print(json.dumps(result, indent=2))
    raise SystemExit(result['result'] != 'PASS')
