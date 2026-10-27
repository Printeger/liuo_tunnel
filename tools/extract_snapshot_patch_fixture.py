#!/usr/bin/env python3
"""Extract an offline S wall-observation fixture from raw points and estimated attitude."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2


def extract(source, run, stamp):
    config = json.loads((run/'metadata/config/phase1.json').read_text())
    identity = json.loads((run/'metadata/manifests/input_identity.json').read_text())
    if config['scan_model'] != 'snapshot' or Path(identity['source_run']).resolve() != source.resolve():
        raise ValueError('requires matching native snapshot input; no rolling approximation')
    paths = list((source/'export/capture/raw').glob('*.mcap'))
    if len(paths) != 1:
        raise ValueError('one frozen capture required')
    with paths[0].open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    if digest != identity['raw_sha256']:
        raise ValueError('frozen input changed')
    with (run/'export/odometry/trajectory.csv').open() as stream:
        poses = [r for r in csv.DictReader(stream) if abs(float(r['stamp'])-stamp) < 1e-8]
    if len(poses) != 1:
        raise ValueError('exact estimated scan pose required')
    pose = poses[0]
    attitude = Rotation.from_quat([float(pose[k]) for k in ('qx','qy','qz','qw')])
    velocity = attitude.inv().apply([float(pose[k]) for k in ('vx','vy','vz')])
    topic = config['sensors'].get('lidar_topic','/sensors/lidar/points')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source/'export/capture/raw'),storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr','cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    selected = None
    while reader.has_next():
        _, payload, _ = reader.read_next()
        message = deserialize_message(payload, PointCloud2)
        when = message.header.stamp.sec + 1e-9*message.header.stamp.nanosec
        if abs(when-stamp) < 1e-8:
            selected = message
            break
        if when > stamp:
            break
    if selected is None:
        raise ValueError('exact input scan missing')
    fields = {f.name:f for f in selected.fields}
    for name in ('x','y','z'):
        if fields[name].datatype != 7 or fields[name].count != 1:
            raise ValueError('float32 XYZ required')
    endian = '>' if selected.is_bigendian else '<'
    dtype = np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,
                         offsets=[fields[k].offset for k in ('x','y','z')],itemsize=selected.point_step))
    cloud = np.ndarray((selected.height,selected.width),dtype=dtype,buffer=selected.data,
                       strides=(selected.row_step,selected.point_step)).reshape(-1)
    # The production split is made before voxelization, by original raw index.
    reserved = np.arange(len(cloud)) % 4 == 0
    points = np.column_stack([cloud[k][reserved] for k in ('x','y','z')]).astype(float)
    if not np.isfinite(points).all():
        raise ValueError('fixture contains nonfinite reserved points')
    extrinsic = Rotation.from_euler('z',config['sensors']['T_BL_yaw'])
    points = extrinsic.apply(points) + config['sensors']['T_BL_translation']
    output = run/'export/analysis'/('snapshot_patch_'+format(stamp,'.6f').replace('.','p'))
    output.parent.mkdir(parents=True,exist_ok=True)
    np.savetxt(output.with_suffix('.csv'),points,delimiter=',',header='x,y,z',comments='',fmt='%.17g')
    report = dict(scope='offline geometry fixture; estimated attitude/velocity, no truth or estimator feedback',
                  source_run=str(source),estimator_run=str(run),stamp=stamp,raw_sha256=digest,
                  tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  source_topic=topic,raw_points=len(cloud),reserved_points=len(points),
                  points_within_25m=int(np.sum(np.linalg.norm(points,axis=1)<=25)),
                  range_sigma=config['sensors']['sigma_lidar_range'],
                  attitude_scope='optimized scan output; not an exact log of the pre-update predicted attitude used online',
                  rotation_body_to_estimated_world=attitude.as_matrix().tolist(),velocity_body=velocity.tolist(),
                  point_file=str(output.with_suffix('.csv')))
    output.with_suffix('.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--stamp',type=float,required=True)
    args = parser.parse_args()
    print(json.dumps(extract(args.source,args.run,args.stamp),indent=2))
