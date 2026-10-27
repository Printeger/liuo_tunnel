#!/usr/bin/env python3
"""Run missing/invalid model and invalid registry against the actual node owner."""
import argparse
import csv
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
SOURCE='/home/dev/ws_iap/.runtime/0/liuo/20261027T025815Z_012'


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for name,error in [('missing_model','MODEL_UNAVAILABLE'),('invalid_model','INVALID_UWB_MODEL'),('invalid_registry','INVALID_UWB_REGISTRY')]:
        cfg=json.loads((ROOT/'packages/liuo_tunnel/config/phase2_continuous_walls_off.json').read_text())
        if name=='missing_model':del cfg['uwb']['input']['model']
        elif name=='invalid_model':cfg['uwb']['input']['model']['ta_sigma_m']=0
        else:cfg['uwb']['input']['nodes'][1]['internal_id']=cfg['uwb']['input']['nodes'][2]['internal_id']
        path=a.output/(name+'.json');log=a.output/(name+'.log')
        if log.exists():raise RuntimeError('do not overwrite evidence')
        path.write_text(json.dumps(cfg,indent=2)+'\n')
        command=[str(ROOT/'tools/run.sh'),'--replay',SOURCE,'--config',str(path),'--wall-timeout','60']
        with log.open('w') as f:code=subprocess.run(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=100).returncode
        run=Path(log.read_text().splitlines()[0]);manifest=json.loads((run/'metadata/run_manifest.json').read_text())
        failures=[]
        if name=='missing_model':
            with (run/'export/odometry/uwb_input_decisions.csv').open() as f:rows=list(csv.DictReader(f))
            with (run/'export/odometry/graph.csv').open() as f:graph=list(csv.DictReader(f))
            if len(rows)!=384 or any(r['reason']!=error for r in rows):failures.append('missing model did not reject every input')
            if not graph or any(int(r['uwb_ta']) or int(r['uwb_aa']) for r in graph):failures.append('missing model graph isolation')
            if manifest['lifecycle']!='completed':failures.append('missing model prevented LIO output')
        else:
            text=(run/'runtime/odometry_console.log').read_text()
            if manifest['lifecycle']!='failed' or error not in text:failures.append('wrong startup rejection')
        result=dict(case=name,result='FAIL' if failures else 'PASS',failures=failures,run=str(run),owner_exit=code,command=command)
        results.append(result);print(json.dumps(result),flush=True)
    (a.output/'model_audit.json').write_text(json.dumps(results,indent=2)+'\n')
    return int(any(r['failures'] for r in results))

if __name__=='__main__':raise SystemExit(main())
