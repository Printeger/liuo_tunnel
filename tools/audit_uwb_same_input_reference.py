#!/usr/bin/env python3
"""Strict physical/report/weight comparison to the retained legacy estimator."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def audit(current,reference):
    def rows(run,path):
        with (run/path).open() as f:return list(csv.DictReader(f))
    def doc(run,path):return json.loads((run/path).read_text())
    failures=[]
    def check(ok,why):
        if not ok:failures.append(why)
    identity=doc(reference,'metadata/manifests/input_identity.json')
    check(identity['legacy_reference'] is not None,'not a declared legacy reference')
    check(identity['frozen_input']['source_run']==str(current),'different frozen source')
    old_audit=doc(reference,'metadata/manifests/observation_audit.json');new_audit=doc(current,'metadata/manifests/observation_audit.json')
    check(old_audit['raw_sha256']==new_audit['raw_sha256'],'frozen IMU/LiDAR/clock/new-message observation differs')
    check((reference/'metadata/config/map.json').read_bytes()==(current/'metadata/config/map.json').read_bytes(),'different joint map prior')
    a_cfg=doc(reference,'metadata/config/phase1.json');b_cfg=doc(current,'metadata/config/phase1.json')
    for key in ['sensors','odometry','uwb','scan_model']:check(a_cfg[key]==b_cfg[key],'different estimator settings '+key)
    old=rows(reference,'export/odometry/uwb.csv');new=rows(current,'export/odometry/uwb.csv')
    delivered=rows(reference,'export/simulation/legacy_reference_delivered.csv')
    legacy_key=lambda r:tuple(r[k] for k in ['session','sequence','a_type','a','b_type','b'])
    sent={tuple(r[k] for k in ['legacy_session','legacy_sequence','a_type','a','b_type','b']):r for r in delivered}
    check(Counter(legacy_key(r) for r in old)==Counter(sent.keys()),'legacy receipt multiplicity differs from sent original samples')
    for r in old:
        s=sent[legacy_key(r)]
        check(abs(float(r['range'])-float(s['physical_distance']))<=1e-12,'legacy distance changed')
        check(abs(float(r['variance'])-float(s['variance']))<=1e-15,'legacy variance changed')
        check(abs(float(r['measurement_stamp'])-float(s['stamp']))<=1e-12,'legacy source time changed')
    physical_key=lambda r:tuple(r[k] for k in ['sequence','a_type','a','b_type','b'])
    accepted=lambda rr:{physical_key(r):r for r in rr if r['reason'] in ['ACCEPT_TA','ACCEPT_AA']}
    a,b=accepted(old),accepted(new);common=set(a)&set(b)
    check(set(a)==set(b),'accepted physical report sets differ')
    association_changes=[k for k in common if any(a[k][f]!=b[k][f] for f in ['reason','key_left','key_right','key_anchor'])]
    check(not association_changes,'physical report graph association differs')
    time_error=max(abs(float(a[k]['measurement_stamp'])-float(b[k]['measurement_stamp'])) for k in common)
    check(time_error<=1e-12,'source time differs beyond decimal float reconstruction')
    precision={}
    for reason in ['ACCEPT_TA','ACCEPT_AA']:
        va=sum(1/float(r['variance']) for r in a.values() if r['reason']==reason)
        vb=sum(1/float(r['variance']) for r in b.values() if r['reason']==reason)
        precision[reason]=dict(legacy=va,unified=vb)
        check(abs(va-vb)<=1e-8,'raw measurement precision differs '+reason)
    old_graph=rows(reference,'export/odometry/graph.csv');new_graph=rows(current,'export/odometry/graph.csv')
    for key in ['aa_summary_factors','aa_accumulated_measurements','joint_map_priors','installation_observation_factors']:
        check(old_graph[-1][key]==new_graph[-1][key],'final information structure differs '+key)
    indexed=lambda rr:{round(float(r['stamp']),6):r for r in rr}
    ta=indexed(rows(reference,'export/odometry/trajectory.csv'));tb=indexed(rows(current,'export/odometry/trajectory.csv'));times=sorted(set(ta)&set(tb))
    delta=np.array([[float(tb[t][k])-float(ta[t][k]) for k in ['x','y','z']] for t in times])
    ca=indexed(rows(reference,'export/odometry/covariance.csv'));cb=indexed(rows(current,'export/odometry/covariance.csv'))
    cov_delta=[]
    for t in times:
        if t not in ca or t not in cb:continue
        x=np.array([float(ca[t][f'xv_{i}_{j}']) for i in range(9) for j in range(9)])
        y=np.array([float(cb[t][f'xv_{i}_{j}']) for i in range(9) for j in range(9)])
        cov_delta.append(float(np.linalg.norm(y-x)/np.linalg.norm(x)))
    result=dict(result='FAIL' if failures else 'PASS',failures=sorted(set(failures)),scope='same frozen sensor input and joint map; retained pre-migration binary versus unified production',
                legacy_binary=identity['legacy_reference'],current_binary=doc(current,'metadata/manifests/input_identity.json')['binaries']['odometry'],
                delivered_samples=len(delivered),legacy_accepted=len(a),unified_accepted=len(b),association_changes=len(association_changes),max_time_representation_difference_s=time_error,
                raw_precision_per_m2=precision,common_trajectory_samples=len(times),trajectory_difference_axis_rmse_m=np.sqrt(np.mean(delta**2,axis=0)).tolist(),trajectory_difference_axis_max_m=np.max(np.abs(delta),axis=0).tolist(),
                covariance_relative_frobenius_p95=float(np.percentile(cov_delta,95)),
                max_float32_distance_error_m=max(abs(float(r['physical_distance'])-float(r['wire_distance'])) for r in delivered),
                source_map_sha256=hashlib.sha256((current/'metadata/config/map.json').read_bytes()).hexdigest(),
                limits='Legacy distances reconstructed from immutable 16-significant-digit pre-float32 CSV; solver timing/nonlinearity may change states. Reference is not unified production acceptance.')
    (reference/'export/analysis/same_input_migration_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return {k:v for k,v in result.items() if k not in ['legacy_binary','current_binary']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('current',type=Path);p.add_argument('reference',type=Path);a=p.parse_args();r=audit(a.current,a.reference);print(json.dumps(r,indent=2));raise SystemExit(r['result']!='PASS')
