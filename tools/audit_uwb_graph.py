#!/usr/bin/env python3
"""Audit production consumption identities and AA information lifecycle."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path


def audit(run):
    def rows(name):
        with (run/'export/odometry'/name).open() as f:
            return list(csv.DictReader(f))
    raw=rows('uwb.csv'); graph=rows('graph.csv'); trajectory=rows('trajectory.csv')
    config=json.loads((run/'metadata/config/phase1.json').read_text())
    manifest=json.loads((run/'metadata/run_manifest.json').read_text())
    accepted=[r for r in raw if r['reason'] in ('ACCEPT_TA','ACCEPT_AA')]
    identities=[tuple(r[k] for k in ('network','driver_session','producer','session','sequence','a_type','a','b_type','b')) for r in accepted]
    reasons=Counter(r['reason'] for r in raw); failures=[]
    def check(ok,why):
        if not ok: failures.append(why)
    check(manifest['lifecycle']=='completed','incomplete lifecycle')
    check(manifest['source']['git_worktree_clean'],'dirty source')
    check(len(identities)==len(set(identities)),'duplicate fused report identity')
    check(bool(graph) and bool(trajectory),'no sustained estimator output')
    for r in accepted:
        if r['reason']=='ACCEPT_AA':
            check(r['a_type']=='1' and r['b_type']=='1' and not r['key_left'] and not r['key_right'], 'AA references vehicle pose')
        else:
            check(r['a_type']=='0' and r['b_type']=='1' and r['key_left'].startswith('x'),'TA lacks vehicle association')
    mode=config['uwb']['mode']; aa=config['uwb']['use_aa']
    if mode=='record_only':
        check(not accepted,'OFF fused observations')
    else:
        check(reasons['ACCEPT_TA']>0,'no TA fusion')
    if aa and mode in ('self_calibrate','reuse_map'):
        check(reasons['ACCEPT_AA']>0,'no AA fusion')
        check(int(graph[-1]['aa_accumulated_measurements'])==reasons['ACCEPT_AA'],'AA precision/count differs from unique accepted reports')
    if not aa or mode=='record_only':
        check(reasons['ACCEPT_AA']==0 and all(int(g['aa_summary_factors'])==0 and int(g['aa_accumulated_measurements'])==0 for g in graph),'disabled AA added information')
    if mode=='self_calibrate':
        check(all(int(g['installation_observation_factors'])==1 for g in graph),'installation information repeated/missing')
    if mode=='reuse_map':
        check(all(int(g['joint_map_priors'])==1 and int(g['installation_observation_factors'])==0 for g in graph),'map/installation information lifecycle')
    result=dict(result='FAIL' if failures else 'PASS',failures=failures,scope='production graph identities and information lifecycle',
                source=manifest['source'],mode=mode,use_aa=aa,reasons=dict(reasons),unique_fused_reports=len(set(identities)),
                trajectory_rows=len(trajectory),graph_updates=len(graph),
                first_stamp=float(trajectory[0]['stamp']) if trajectory else None,
                last_stamp=float(trajectory[-1]['stamp']) if trajectory else None,
                sha256={n:hashlib.sha256((run/'export/odometry'/n).read_bytes()).hexdigest() for n in ['uwb.csv','graph.csv','trajectory.csv']})
    (run/'export/analysis/uwb_graph_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);r=audit(p.parse_args().run);print(json.dumps(r,indent=2));raise SystemExit(r['result']!='PASS')
