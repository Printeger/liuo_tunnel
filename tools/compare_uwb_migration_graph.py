#!/usr/bin/env python3
"""Compare graph decisions only after explicit physical input correspondence."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path


def compare(run,baseline):
    proof=json.loads((run/'export/analysis/physical_equivalence.json').read_text())
    if proof['result']!='PASS':raise ValueError('physical equivalence must pass first')
    def read(root,name):
        with (root/'export/odometry'/name).open() as f:return list(csv.DictReader(f))
    old=read(baseline,'uwb.csv');new=read(run,'uwb.csv')
    key=lambda r:tuple(r[k] for k in ['sequence','a_type','a','b_type','b'])
    select=lambda rows:{key(r):r for r in rows if r['reason'] in ['ACCEPT_TA','ACCEPT_AA']}
    a,b=select(old),select(new); common=set(a)&set(b)
    changes=[]
    for k in sorted(common):
        fields=[f for f in ['reason','measurement_stamp','key_left','key_right','key_anchor','variance'] if a[k][f]!=b[k][f]]
        if fields:changes.append(dict(identity=k,fields=fields,before={f:a[k][f] for f in fields},after={f:b[k][f] for f in fields}))
    with (run/'export/analysis/migration_acceptance_changes.json').open('w') as f:
        json.dump(dict(only_old=sorted(set(a)-set(b)),only_new=sorted(set(b)-set(a)),common_changes=changes),f,indent=2)
    old_graph=read(baseline,'graph.csv');new_graph=read(run,'graph.csv')
    keys=['uwb_ta','uwb_aa','aa_summary_factors','aa_accumulated_measurements','joint_map_priors','installation_observation_factors']
    result=dict(scope='historical physical sample correspondence and production graph comparison; numerical optimization need not be bit-identical',
                old_graph_run=str(baseline),current_graph_run=str(run),physical_proof=proof,
                old_accepted=len(a),new_accepted=len(b),common_accepted=len(common),only_old=len(set(a)-set(b)),only_new=len(set(b)-set(a)),
                common_association_or_weight_changes=len(changes),old_reasons=dict(Counter(r['reason'] for r in old)),new_reasons=dict(Counter(r['reason'] for r in new)),
                old_final_graph={k:old_graph[-1][k] for k in keys},new_final_graph={k:new_graph[-1][k] for k in keys},
                review_required=bool(changes or set(a)!=set(b)),differences='see migration_acceptance_changes.json; no automatic equivalence verdict from same seed')
    (run/'export/analysis/migration_graph_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    return {k:v for k,v in result.items() if k!='physical_proof'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('baseline',type=Path);a=p.parse_args();print(json.dumps(compare(a.run,a.baseline),indent=2))
