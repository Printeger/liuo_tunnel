#!/usr/bin/env python3
"""Single owner: allocate, snapshot, start owned processes, retain failure, finalize."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import run_directory as rd

ROOT = Path(__file__).resolve().parents[1]
def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--duration',type=float,default=45)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--reset-at',type=float,default=-1.0)
    p.add_argument('--config',type=Path,default=ROOT/'packages/liuo_tunnel/config/phase1.json')
    p.add_argument('--diagnostic-capture',action='store_true')
    p.add_argument('--inject-failure',choices=['simulation','odometry'])
    args=p.parse_args()
    run=rd.resolve_run_directory(entrypoint='rail_sim',scenario='rail_tunnel_straight_v1')
    print(run,flush=True)
    children={}; handles=[]; failure=None; lifecycle='failed'
    try:
        if not rd._git_value('rev-parse','HEAD') or rd._git_value('status','--porcelain'):
            raise RuntimeError('LIVE_BLOCKED_BY_DIRTY_WORKTREE: commit project changes before live evidence')
        config=run/'metadata/config/phase1.json';shutil.copyfile(args.config,config);rd.register_config_snapshot(run,config)
        rd._atomic_write_json(run/'metadata/config/scenario.json',{'scene':'rail_tunnel_straight_v1','length_m':2400,'width_m':6,'height_m':5,'gauge_m':1.435,'duration_s':args.duration,'seed':args.seed,'scan_model':'snapshot','reset_at_s':args.reset_at,'clock_start_s':1,'dt_s':.005,'exchange_dt_s':.004,'lidar_samples':[512,40],'lidar_range_m':40,'world_voxel_m':.1})
        rd.register_config_snapshot(run,run/'metadata/config/scenario.json')
        env=os.environ.copy();env.update(LIUO_RUN_DIR=str(run),ROS_LOG_DIR=str(run/'runtime/ros'),ROS_DOMAIN_ID=str(100+os.getpid()%100),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        env['LIUO_DIAGNOSTIC_CAPTURE']='1' if args.diagnostic_capture else '0'
        binaries={name:ROOT/f'install/{pkg}/lib/{pkg}/{exe}' for name,pkg,exe in [('odometry','liuo_tunnel','liuo_tunnel_node'),('simulation','liuo_tunnel_sim','liuo_rail_sim')]}
        identity={'source_iap_commit':'fbccae16802402d7300bb817d2627f86e6a495df','config_sha256':digest(config),'seed':args.seed,'scan_model':'snapshot','binaries':{},'environment':{k:env.get(k) for k in ['AMENT_PREFIX_PATH','LD_LIBRARY_PATH','ROS_DOMAIN_ID','ROS_LOG_DIR']},'commands':{}}
        for name,binary in binaries.items():
            linked=subprocess.check_output(['ldd',str(binary)],text=True)
            if '/iap/' in linked or '/ws_iap/' in linked: raise RuntimeError('forbidden runtime dependency: '+linked)
            identity['binaries'][name]={'path':str(binary),'sha256':digest(binary),'ldd':linked}
        commands={name:[str(binary),'--ros-args','-p','config_file:='+str(config),'-p','use_sim_time:=true'] for name,binary in binaries.items()}
        commands['simulation']+=['-p',f'duration_s:={args.duration}','-p',f'seed:={args.seed}','-p',f'reset_at_s:={args.reset_at}']
        commands['observer']=[sys.executable,str(ROOT/'tools/observe.py')]
        for name in ['observer','odometry','simulation']:
            f=open(run/f'runtime/{name}_console.log','w');handles.append(f)
            children[name]=subprocess.Popen(commands[name],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            identity['commands'][name]=commands[name]
            if name!='simulation':time.sleep(1)
        rd.write_subordinate_manifest(run,'input_identity',identity)
        start=time.monotonic();injected=False
        while True:
            for name,child in children.items():
                code=child.poll()
                if code is not None and (name!='simulation' or code!=0):
                    raise RuntimeError(f'{name} exited unexpectedly: {code}; see runtime/{name}_console.log')
            if children['simulation'].poll()==0:break
            if args.inject_failure and not injected and time.monotonic()-start>5:
                children[args.inject_failure].kill();injected=True
            if time.monotonic()-start>max(120,args.duration*8):raise RuntimeError('wall time budget exceeded; see timing evidence')
            time.sleep(.1)
        time.sleep(2)
        lifecycle='completed'
    except KeyboardInterrupt:
        lifecycle='interrupted';failure='user interrupt'
    except Exception as exc:
        failure=str(exc);print(failure,file=sys.stderr,flush=True)
    finally:
        for name,child in children.items():
            if child.poll() is None:child.send_signal(signal.SIGINT)
        for name,child in children.items():
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.kill();child.wait();failure=f'{name} did not stop after owned SIGINT';lifecycle='failed'
        for f in handles:f.close()
        if failure:rd.write_subordinate_manifest(run,'input_failure',{'reason':failure,'exit_codes':{k:v.returncode for k,v in children.items()}})
        rd.finalize_run(run,lifecycle=lifecycle)
    if lifecycle=='completed':
        result=subprocess.run([sys.executable,str(ROOT/'tools/evaluate.py'),str(run)])
        manifest_path=run/'metadata/run_manifest.json'
        manifest=json.loads(manifest_path.read_text())
        manifest['acceptance']='pass' if result.returncode==0 else 'fail'
        rd._atomic_write_json(manifest_path,manifest)
        return result.returncode
    return 1
if __name__=='__main__':sys.exit(main())
