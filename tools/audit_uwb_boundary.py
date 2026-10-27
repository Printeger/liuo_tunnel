#!/usr/bin/env python3
"""Check expired/future queue evidence and real post-reset graph recovery."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path


def audit(run):
    def rows(path):
        with (run/path).open() as f:return list(csv.DictReader(f))
    expected=rows('export/simulation/boundary_sent.csv')
    decisions=rows('export/odometry/uwb_input_decisions.csv'); consumed=rows('export/odometry/uwb.csv')
    graph=rows('export/odometry/graph.csv'); trajectory=rows('export/odometry/trajectory.csv')
    failures=[]
    def check(ok,why):
        if not ok:failures.append(why)
    index={(r['driver_session'],r['batch_seq'],r['source_session'],r['source_sequence'],r['node_a'],r['node_b']):r for r in decisions}
    for e in expected:
        k=tuple(e[x] for x in ['driver_session','batch_seq','source_session','sequence','node_a','node_b'])
        check(k in index,'missing planned item '+str(k))
        if k in index:check(index[k]['reason']==e['expected_input'],'input reason mismatch '+str(k)+': '+index[k]['reason'])
    future=[e for e in expected if e['scenario']=='future']
    future_ids={(e['driver_session'],e['source_session'],e['sequence']) for e in future}
    drained=[r for r in consumed if (r['driver_session'],r['session'],r['sequence']) in future_ids and r['reason']=='CLOCK_RESET']
    fused_future=[r for r in consumed if (r['driver_session'],r['session'],r['sequence']) in future_ids and r['reason'].startswith('ACCEPT_')]
    check(len(future)==4097 and len(drained)==4096 and not fused_future,'future capacity/reset information invariant')
    expired=next(e for e in expected if e['scenario']=='expired')
    expired_rows=[r for r in consumed if (r['driver_session'],r['session'],r['sequence'])==(expired['driver_session'],expired['source_session'],expired['sequence'])]
    check(any(r['reason']=='WINDOW_EXPIRED' for r in expired_rows),'expired input not rejected at estimator association')
    check(max(int(r['queue_size']) for r in decisions)==4096,'queue peak must hit declared bound exactly')
    post=[r for r in trajectory if int(r['session'])==2]
    check(len(post)>=20 and float(post[-1]['stamp'])-float(post[0]['stamp'])>=2,'no sustained second-session localization')
    new_driver=str(int(expired['driver_session'])+1000000000)
    accepted=[r for r in consumed if r['driver_session']==new_driver and r['reason'] in ['ACCEPT_TA','ACCEPT_AA']]
    check(any(r['reason']=='ACCEPT_TA' for r in accepted),'new dataset did not recover TA fusion')
    check(any(r['reason']=='ACCEPT_AA' for r in accepted),'new dataset did not recover AA fusion')
    check({int(r['session']) for r in graph}=={1,2},'graph did not reset into a second session')
    identities=[tuple(r[k] for k in ['driver_session','producer','session','sequence','a_type','a','b_type','b']) for r in accepted]
    check(len(set(identities))==len(identities),'duplicate post-reset fusion')
    result=dict(result='FAIL' if failures else 'PASS',failures=failures,scope='public expired/future/capacity/ROS reset and production graph recovery',
                planned_items=len(expected),future_drained=len(drained),future_fused=len(fused_future),queue_peak=max(int(r['queue_size']) for r in decisions),
                callback_reasons=dict(Counter(r['reason'] for r in decisions)),consumer_reasons=dict(Counter(r['reason'] for r in consumed)),
                post_reset_states=len(post),post_reset_fused=len(accepted))
    (run/'export/analysis/uwb_boundary_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);r=audit(p.parse_args().run);print(json.dumps(r,indent=2));raise SystemExit(r['result']!='PASS')
