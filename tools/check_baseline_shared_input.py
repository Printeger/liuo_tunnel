#!/usr/bin/env python3
"""Independently audit FAST-LIO published input against the frozen source and optional LIUO run."""
import argparse
import hashlib
import json
from pathlib import Path
from raw_identity import hash_message


def check_hashes(result, counts, raw_hashes):
    for key in ('imu','cloud'):
        if result['counts'][key]!=counts[key] or result['raw_semantic_sha256'][key]!=raw_hashes[key]:
            raise ValueError('original sensor count/hash mismatch: '+key)
        if result['observed_counts'][key]!=counts[key] or result['observed_semantic_sha256'][key]!=result['adapted_semantic_sha256'][key]:
            raise ValueError('adapted subscriber count/hash mismatch: '+key)
    if result['counts']['clock']!=counts['clock']: raise ValueError('clock count mismatch')


def audit(run, liuo_run=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu,PointCloud2
    identity=json.loads((run/'metadata/manifests/baseline_input.json').read_text())
    result=json.loads((run/'metadata/manifests/baseline_result.json').read_text())
    manifest=json.loads((run/'metadata/run_manifest.json').read_text())
    if not manifest['source']['git_worktree_clean'] or identity['algorithm']!='fastlio':
        raise ValueError('expected committed FAST-LIO input evidence')
    source=Path(identity['source'])
    actual={p.name:hashlib.file_digest(p.open('rb'),'sha256').hexdigest() for p in (source/'export/capture/raw').glob('*.mcap')}
    if not actual or actual!=identity['capture_sha256']: raise ValueError('source capture changed')
    if identity['scan_model'] not in ('snapshot','rolling') or identity['end_exclusive']!=(identity['scan_model']=='snapshot'):
        raise ValueError('unexpected scan model or interval boundary')
    if identity['rate']!=1. or result['publish_wall_s'] is None or abs(result['publish_wall_s']-identity['duration_s'])>.25:
        raise ValueError('incomplete 1x publication interval')
    lidar='/sensors/lidar' if identity['scan_model']=='rolling' else '/sensors/lidar/points'
    selected=['/sensors/imu',lidar,'/clock']; boundary=identity['sensor_access_boundary']
    if set(boundary['selected_topics'])!=set(selected) or not boundary['storage_filter_before_first_read']:
        raise ValueError('baseline sensor access boundary not established')
    if boundary['startup_topics']!=['/sensors/imu'] or identity['uwb_published'] or identity['truth_published']:
        raise ValueError('forbidden baseline input')
    stream=rosbag2_py.SequentialReader()
    stream.open(rosbag2_py.StorageOptions(uri=str(source/'export/capture/raw'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    stream.set_filter(rosbag2_py.StorageFilter(topics=selected))
    types={'/sensors/imu':('imu',Imu),lidar:('cloud',PointCloud2)}
    counts=dict(imu=0,cloud=0,clock=0)
    hashes={k:hashlib.sha256() for k in ('imu','cloud')}
    first=None; last_published=None; duration=round(identity['duration_s']*1e9)
    while stream.has_next():
        topic,data,stamp=stream.read_next()
        first=stamp if first is None else first
        elapsed=stamp-first
        if elapsed>duration or (identity['end_exclusive'] and elapsed==duration): break
        last_published=elapsed
        if topic=='/clock': counts['clock']+=1; continue
        key,cls=types[topic]; counts[key]+=1; hash_message(hashes[key],key,deserialize_message(data,cls))
    raw={k:v.hexdigest() for k,v in hashes.items()}
    if last_published is None or duration-last_published>20_000_000:
        raise ValueError('source does not cover requested publication interval')
    check_hashes(result,counts,raw)
    if liuo_run:
        name='rolling_smoke.json' if identity['scan_model']=='rolling' else 'snapshot_smoke.json'
        liuo=json.loads((liuo_run/'export/analysis'/name).read_text())
        if Path(liuo['source_run']).resolve()!=source.resolve(): raise ValueError('LIUO used a different source')
        for key in ('imu','cloud'):
            if liuo['raw_counts'][key]!=counts[key] or liuo['raw_sha256'][key]!=raw[key]:
                raise ValueError('LIUO shared input mismatch: '+key)
        if liuo['raw_counts']['clock']!=counts['clock']: raise ValueError('LIUO clock count mismatch')
    report=dict(scope='original published sensors and independent adapted subscriber; not algorithm-internal consumption or localization acceptance',
                result='PASS',baseline_run=str(run),source_run=str(source),liuo_run=str(liuo_run) if liuo_run else None,
                paired_liuo_verified=liuo_run is not None,counts=counts,raw_sha256=raw,
                source_capture_sha256=actual,baseline_failure=result['failure'],estimated_poses=result['poses'])
    output=run/'export/analysis';output.mkdir(exist_ok=True,parents=True)
    (output/'shared_input_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--liuo-run',type=Path)
    a=p.parse_args();print(json.dumps(audit(a.run,a.liuo_run),indent=2))
