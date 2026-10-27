#!/usr/bin/env python3
"""Verify and bind the declared T01-T14 software acceptance evidence."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
DATA=Path('/home/dev/ws_iap/liuo_observation')
RUNS=Path('/home/dev/ws_iap/.runtime/0/liuo')
ROLES={
 'default_live':'20261027T035815Z_500','default_replay':'20261027T040815Z_067',
 'contract_capture':'20261027T040326Z_895','contract_replay':'20261027T040646Z_535',
 'boundary':'20261027T033259Z_396','load':'20261027T033415Z_020',
 'fault_on':'20261027T034312Z_074','fault_off':'20261027T034428Z_811',
 'calibration':'20261027T031155Z_389','reuse':'20261027T031819Z_304',
 'legacy_reference':'20261027T034937Z_909',
 'missing_model':'20261027T034841Z_670','invalid_model':'20261027T034853Z_096','invalid_registry':'20261027T034855Z_623',
}

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def doc(p):return json.loads(p.read_text())
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    failures=[]
    def check(ok,why):
        if not ok:failures.append(why)
    def pass_audit(role,name):
        return doc(RUNS/ROLES[role]/'export/analysis'/name)['result']=='PASS'
    records={}
    for role,run_id in ROLES.items():
        run=RUNS/run_id;m=doc(run/'metadata/run_manifest.json');i=doc(run/'metadata/manifests/input_identity.json')
        check(m['source']['git_worktree_clean'],role+': dirty source')
        check(digest(run/'metadata/config/phase1.json')==i['config_sha256'],role+': effective configuration hash differs')
        audits={}
        for name in ['report.json','uwb_contract_audit.json','uwb_load_audit.json','uwb_boundary_audit.json','uwb_graph_audit.json','physical_equivalence.json','same_input_migration_audit.json']:
            path=run/'export/analysis'/name
            if path.exists():audits[name]=dict(path=str(path),sha256=digest(path),result=doc(path).get('result'))
        records[role]=dict(run=str(run),source=m['source'],lifecycle=m['lifecycle'],nominal_acceptance=m['acceptance'],configuration_sha256=i['config_sha256'],uwb_interface=i['uwb_interface'],uwb_transport=i['uwb_transport'],binaries=i['binaries'],commands=i['commands'],audits=audits)
    check(all(pass_audit(r,'report.json') for r in ['default_live','default_replay','fault_on','fault_off','calibration','reuse','legacy_reference']),'positive nominal run failed')
    check(all(pass_audit(r,'uwb_contract_audit.json') for r in ['contract_capture','contract_replay']),'contract case audit failed')
    check(pass_audit('boundary','uwb_boundary_audit.json'),'boundary audit failed')
    check(pass_audit('load','uwb_load_audit.json'),'load audit failed')
    check(pass_audit('legacy_reference','same_input_migration_audit.json'),'strict migration reference audit failed')
    for role in ['calibration','reuse','fault_on','fault_off','load']:check(pass_audit(role,'uwb_graph_audit.json'),role+': graph audit failed')
    for role in ['calibration','reuse']:check(pass_audit(role,'physical_equivalence.json'),role+': historical physical sample correspondence failed')
    model=doc(DATA/'evidence/model-cases-1/model_audit.json');check(all(r['result']=='PASS' for r in model),'model/registry production cases failed')
    ca=doc(RUNS/ROLES['contract_capture']/'metadata/manifests/observation_audit.json');cb=doc(RUNS/ROLES['contract_replay']/'metadata/manifests/observation_audit.json')
    check(ca['raw_sha256']==cb['raw_sha256'] and ca['counts']==cb['counts'],'negative capture/replay semantic identity differs')
    calibration=RUNS/ROLES['calibration'];reuse=RUNS/ROLES['reuse']
    exported=calibration/'export/map/calibrated_map.json';loaded=reuse/'metadata/config/map.json'
    check(exported.read_bytes()==loaded.read_bytes(),'map bytes changed on independent reuse')
    check(not (calibration/'metadata/config/map.json').exists(),'first calibration supplied a coordinate map')
    map_doc=doc(exported);cov=np.array(map_doc['joint_covariance'])
    check(np.linalg.eigvalsh(cov).min()>0 and np.linalg.norm(cov[:-2,-2:])>0,'joint correlated map information missing')
    scenarios=[doc(r/'metadata/config/scenario.json') for r in [calibration,reuse]]
    check(scenarios[0]['seed']!=scenarios[1]['seed'] and scenarios[0]['simulation_start_xyz_yaw']!=scenarios[1]['simulation_start_xyz_yaw'],'reuse start/noise not independent')
    compatibility={}
    guard='''    // Pending observations may wait at most one lag on an advancing ROS clock.
    // A mapped timestamp is not permission to retain arbitrarily distant data.
    if (o.stamp > o.received + cfg.at("odometry").at("lag_s").get<double>()) {
      log_range(o, "FUTURE_TIME_LIMIT");
      return "FUTURE_TIME_LIMIT";
    }
'''
    node='packages/liuo_tunnel/apps/liuo_tunnel_node.cpp'
    old=subprocess.check_output(['git','show','d5ce11a:'+node],cwd=ROOT,text=True)
    current=(ROOT/node).read_text()
    check(current.replace(guard,'')==old,'representative evidence compatibility requires additional code review')
    for role in ['calibration','reuse']:
        r=RUNS/ROLES[role];rr=rows(r/'export/odometry/uwb.csv');lag=doc(r/'metadata/config/phase1.json')['odometry']['lag_s']
        ahead=max(float(x['measurement_stamp'])-float(x['receive_stamp']) for x in rr)
        compatibility[role]=dict(maximum_source_ahead_of_callback_s=ahead,current_future_bound_s=lag,would_reject_existing_reports=sum(float(x['measurement_stamp'])>float(x['receive_stamp'])+lag for x in rr))
        check(ahead<=lag,role+': new guard affects reused full-route evidence')
    legacy_log=DATA/'evidence/legacy-bag-rejection.log';legacy_run=Path(legacy_log.read_text().splitlines()[0]);text=(legacy_run/'runtime/simulation_console.log').read_text()
    check('Legacy UwbRange bag: use historical LIUO revision; no implicit conversion' in text,'legacy bag was not explicitly rejected')
    matrix={
      'T01':['default_live','default_replay','fault_on'], 'T02':['contract_capture','contract_replay','load'],
      'T03':['contract_capture','invalid_registry'], 'T04':['contract_capture','contract_replay','boundary'],
      'T05':['contract_capture','contract_replay','boundary'], 'T06':['contract_capture','contract_replay','boundary'],
      'T07':['contract_capture','missing_model','invalid_model','load'], 'T08':['contract_replay','fault_on','fault_off','calibration','reuse'],
      'T09':['default_live','default_replay','contract_capture','contract_replay'], 'T10':['fault_on','fault_off','boundary'],
      'T11':['contract_capture','missing_model','calibration','reuse','load'], 'T12':['load','boundary','contract_capture'],
      'T13':['calibration','reuse','legacy_reference'], 'T14':['calibration','reuse'],
    }
    result=dict(result='FAIL' if failures else 'PASS',failures=failures,scope='declared software integration, synthetic load and representative GPU simulation/map reuse only',
                evidence_script_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                matrix={k:dict(status='FAIL' if failures else 'PASS',evidence=v) for k,v in matrix.items()},runs=records,
                legacy_bag_rejection=dict(run=str(legacy_run),console_sha256=digest(legacy_run/'runtime/simulation_console.log')),
                joint_map=dict(sha256=digest(exported),dimension=len(cov),minimum_eigenvalue=float(np.linalg.eigvalsh(cov).min()),anchor_reference_cross_covariance_norm=float(np.linalg.norm(cov[:-2,-2:])),internal_anchor_ids=[r['id'] for r in map_doc['anchors']],deployment_validated=map_doc['deployment_validated']),
                final_code_compatibility=dict(only_production_node_change_since_representative_binary='one-lag future-time admission guard',node_source_sha256=digest(ROOT/node),unchanged_valid_input_paths=compatibility),
                limitations=['software reporting frequencies are not physical independent TWR rates','60 seconds is not hours-long stability','snapshot only; weak height, local covariance and intermittent LOST/DEGRADED remain','negative cases retain nominal evaluator failures; their explicit case audits define the test verdict','legacy inverse representation is only a separately identified reference, never the unified production input'])
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(result=result['result'],failures=failures,output=str(a.output)),indent=2));return bool(failures)

if __name__=='__main__':raise SystemExit(main())
