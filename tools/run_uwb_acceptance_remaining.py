#!/usr/bin/env python3
"""Bounded sequential public-ROS acceptance cases; retain nominal and case verdicts."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
RUNS=Path('/home/dev/ws_iap/.runtime/0/liuo')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cases',nargs='+',choices=['contract','boundary','load','fault_on','fault_off'],default=['contract','boundary','load','fault_on','fault_off'])
    p.add_argument('--derived-source',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    evidence=[];derived=a.derived_source
    common=['--map',str(RUNS/'20261027T030245Z_221/export/map/calibrated_map.json'),'--wall-timeout','150']
    for name in a.cases:
        if name=='contract':
            arguments=['--replay',str(RUNS/'20261027T025815Z_012'),'--config','packages/liuo_tunnel/config/phase2_continuous_walls_off.json','--uwb-contract-fixture','--uwb-topic','/acceptance/contract','--uwb-qos-depth','37','--wall-timeout','60']
            auditor='audit_uwb_contract.py'
        elif name=='boundary':
            arguments=['--replay',str(RUNS/'20261027T030429Z_985'),'--config','packages/liuo_tunnel/config/phase3_reuse_map.json','--use-aa','--uwb-boundary-fixture','--formal']+common
            auditor='audit_uwb_boundary.py'
        elif name=='load':
            arguments=['--replay',str(RUNS/'20261027T030429Z_985'),'--config','packages/liuo_tunnel/config/phase3_reuse_map.json','--use-aa','--uwb-load-fixture','--capture-derived','--uwb-topic','/acceptance/load','--uwb-qos-depth','37','--formal']+common
            auditor='audit_uwb_load.py'
        else:
            if derived is None:raise ValueError('fault cases require a completed derived load source')
            arguments=['--replay',str(derived),'--config',str(derived/'metadata/config/phase1.json'),'--use-aa' if name=='fault_on' else '--no-use-aa','--delivery-profile','packages/liuo_tunnel/config/delivery/unified_batch_acceptance.json','--uwb-topic','/acceptance/fault','--uwb-qos-depth','37','--visualize','--formal']+common
            auditor='audit_uwb_graph.py'
        log=a.output/(name+'.log')
        if log.exists():raise ValueError('refusing to overwrite previous evidence '+str(log))
        command=[str(ROOT/'tools/run.sh')]+arguments
        print(name,flush=True)
        with log.open('w') as f:
            child=subprocess.Popen(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=child.wait(timeout=210)
            except subprocess.TimeoutExpired:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    child.kill();child.wait(timeout=10)
                raise RuntimeError('owner timed out; retained '+str(log))
        lines=log.read_text().splitlines()
        run=Path(lines[0]);manifest=json.loads((run/'metadata/run_manifest.json').read_text())
        if manifest['lifecycle']!='completed':raise RuntimeError(name+' lifecycle failed: '+str(run))
        with (a.output/(name+'-audit.log')).open('w') as f:
            audit_code=subprocess.run([sys.executable,str(ROOT/'tools'/auditor),str(run)],stdout=f,stderr=subprocess.STDOUT,timeout=60).returncode
        evidence.append(dict(case=name,run=str(run),command=command,owner_exit=code,case_audit_exit=audit_code))
        (a.output/'suite.json').write_text(json.dumps(evidence,indent=2)+'\n')
        print(json.dumps(evidence[-1]),flush=True)
        if audit_code:raise RuntimeError(name+' case audit failed; investigate retained evidence')
        if name=='load':derived=run
        if name.startswith('fault') and code:raise RuntimeError(name+' nominal delivery evaluator failed')
    return 0

if __name__=='__main__':raise SystemExit(main())
