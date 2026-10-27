#!/usr/bin/env python3
"""Compare new samples to immutable pre-migration generated-distance evidence."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(run, baseline):
    old_path = baseline / 'export/simulation/uwb.csv'
    new_path = run / 'export/simulation/uwb.csv'
    with old_path.open() as f:
        old = list(csv.DictReader(f))
    with new_path.open() as f:
        new = list(csv.DictReader(f))
    failures = []
    maximum = 0.
    error_sum = 0.
    mismatches = []
    if len(old) < len(new):
        failures.append('historical evidence shorter than candidate')
    config = json.loads((run / 'metadata/config/phase1.json').read_text())
    mapping = {(n['type'], n['internal_id']): n['logical_id'] for n in config['uwb']['input']['nodes']}
    # Persist explicit old/new physical identity mapping, not just aggregate hashes.
    with (run / 'export/analysis/physical_identity_mapping.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['old_session','old_sequence','new_driver_session','new_source_sequence','kind','logical_a','logical_b','stamp','old_raw_distance','new_pre_float32','new_wire_distance','float32_error'])
        for i, (a,b) in enumerate(zip(old,new)):
            reasons = []
            for key in ['sequence','stamp','a_type','a','b','true_range','variance']:
                if a[key] != b[key]:
                    reasons.append(key)
            if a['range'] != b['physical_range']:
                reasons.append('physical_distance')
            expected = struct.unpack('<f', struct.pack('<f', float(a['range'])))[0]
            # CSV is printed at 16 significant digits, not a binary float dump.
            if abs(expected - float(b['range'])) > 5e-13:
                reasons.append('wire_float32')
            error = float(b['range']) - float(a['range'])
            maximum = max(maximum, abs(error)); error_sum += error*error
            if reasons:
                mismatches.append(dict(index=i, fields=reasons))
            kind = int(b['a_type'])
            writer.writerow([a['session'],a['sequence'],b['session'],b['sequence'],kind,
                             mapping[(kind,int(b['a']))],mapping[(1,int(b['b']))],b['stamp'],a['range'],b['physical_range'],b['range'],error])
    if mismatches:
        failures.append('sample correspondence mismatch')
    result = dict(result='FAIL' if failures else 'PASS', failures=failures,
                  scope='generated physical input prefix equivalence; graph acceptance comparison separate',
                  baseline=str(baseline), baseline_sha256=digest(old_path), candidate_sha256=digest(new_path),
                  baseline_rows=len(old), compared_rows=len(new), mismatches=mismatches[:20],
                  maximum_float32_error_m=maximum, rms_float32_error_m=(error_sum/max(1,len(new)))**.5,
                  weight_source=dict(legacy_variance_m2=.01, current_model=config['uwb']['input']['model']),
                  limitations='Historical CSV decimal precision (16 significant digits); no claim of CDR byte identity or identical optimizer trajectory')
    (run / 'export/analysis/physical_equivalence.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('run',type=Path); p.add_argument('baseline',type=Path)
    a=p.parse_args(); result=audit(a.run,a.baseline); print(json.dumps(result,indent=2)); raise SystemExit(result['result']!='PASS')
