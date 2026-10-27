#!/usr/bin/env python3
"""Offline compare logged joint-candidate world and same-vehicle branch distances."""
import argparse
from collections import Counter
import csv
import io
import json
import math
from pathlib import Path
import numpy as np


def analyze(run):
    config=json.loads((run/'metadata/config/phase1.json').read_text())
    iteration_limit=config['uwb'].get('bootstrap_max_iterations',30)
    content=(run/'profiling/anchor_extension.csv').read_text()
    incomplete_tail=not content.endswith('\n')
    if incomplete_tail:
        content=content.rsplit('\n',1)[0]+'\n'
    batches={}; rejected=[]
    for row in csv.DictReader(io.StringIO(content)):
        key=(row['session'],row['anchor']);event=row['event']
        if event=='CANDIDATES_STARTED':
            batches[key]=dict(start=float(row['stamp']),candidates=[],current={})
        batch=batches.get(key)
        if batch is None:
            continue
        current=batch['current']
        if event=='CANDIDATE_ITERATIONS':
            current['iterations']=int(float(row['value_a']))
            current['iteration_limit_reached']=current['iterations']>iteration_limit
        elif event=='CANDIDATE_TERMINATION':
            current['termination_flags']=int(float(row['value_a']))
            change=float(row['value_b'])
            current['last_absolute_error_change']=change if math.isfinite(change) else None
        elif event=='CANDIDATE_COMMON_REFERENCE':
            current['reference']=int(float(row['value_a']))
        elif event=='CANDIDATE_RELATIVE_XY':
            current['relative_xy']=[float(row['value_a']),float(row['value_b'])]
        elif event=='CANDIDATE_RELATIVE_Z':
            current['relative']=current['relative_xy']+[float(row['value_a'])]
        elif event=='CANDIDATE_SOLVED':
            current.update(z=float(row['value_a']),cost=float(row['value_b']))
        elif event=='CANDIDATE_POSITION_XY':
            current['world']=[float(row['value_a']),float(row['value_b']),current['z']]
            if math.isfinite(current['cost']):
                batch['candidates'].append(current)
            batch['current']={}
        elif event=='UNRESOLVED_POSITION_BRANCH':
            candidates=batch['candidates']
            entry=dict(session=int(key[0]),anchor=int(key[1]),start=batch['start'],stamp=float(row['stamp']),finite_candidates=len(candidates))
            if len(candidates)<2 or any('relative' not in c or 'reference' not in c for c in candidates):
                entry['classification']='INSUFFICIENT_COMMON_REFERENCE_LOG'
            elif len({c['reference'] for c in candidates})!=1:
                entry['classification']='REFERENCE_KEYS_DIFFER'
            else:
                best=min(range(len(candidates)),key=lambda i:candidates[i]['cost'])
                world=np.array([c['world'] for c in candidates]);relative=np.array([c['relative'] for c in candidates])
                wd=float(np.linalg.norm(world-world[best],axis=1).max())
                rd=float(np.linalg.norm(relative-relative[best],axis=1).max())
                entry.update(reference=candidates[best]['reference'],maximum_world_separation_m=wd,
                             maximum_relative_separation_m=rd,
                             classification='RELATIVE_BRANCH_REMAINS' if rd>=.5 else 'WORLD_ONLY_SEPARATION',
                             candidates=candidates)
            rejected.append(entry)
            batches.pop(key,None)
    result=dict(scope='logged candidate comparison only; no truth or localization acceptance',run=str(run),gate_m=.5,
                convergence_scope='finite candidates are numerical outputs, not proven stationary solutions; iteration limit may coincide with another stopping criterion',
                configured_iteration_limit=iteration_limit,
                incomplete_tail_ignored=incomplete_tail,counts=dict(Counter(r['classification'] for r in rejected)),rejected_batches=rejected)
    output=run/'export/analysis/candidate_branch_diagnostic.json';output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    result=analyze(parser.parse_args().run)
    print(json.dumps({k:v for k,v in result.items() if k!='rejected_batches'},indent=2))
