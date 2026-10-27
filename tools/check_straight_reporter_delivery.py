#!/usr/bin/env python3
"""Offline independent delivery audit for the ordinary straight S route."""
import argparse
import collections
import csv
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
import rosbag2_py
from rclpy.serialization import deserialize_message
from liuo_uwb_msgs.msg import UwbObservationBatch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--reference', type=Path, help='same-seed ungated run, for unchanged physical acquisition check')
    args = parser.parse_args(); run = args.run
    config = json.loads((run / 'metadata/config/phase1.json').read_text())
    assert config['uwb']['enforce_reporter_delivery'] is True
    physical = list(csv.DictReader((run / 'export/simulation/physical_uwb.csv').open()))
    truth = list(csv.DictReader((run / 'export/simulation/truth.csv').open()))
    t = np.array([float(r['stamp']) for r in truth])
    p = np.array([[float(r[k]) for k in ['x','y','z']] for r in truth])
    v = np.array([[float(r[k]) for k in ['vx','vy','vz']] for r in truth])
    q = np.array([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in truth])
    assert np.max(np.abs(q-q[0])) < 1e-12, 'ordinary fixed-heading S required'
    interpolation = CubicHermiteSpline(t, p, v)
    lever = Rotation.from_quat(q[0]).apply(config['sensors']['T_BU_translation'])
    fixture = json.loads((Path(__file__).resolve().parents[1] / 'packages/liuo_tunnel/config/maps/oracle_map.json').read_text())
    anchors = {a['id']:np.array(a['mean_m']) for a in fixture['anchors']}
    # Independent convex-box segment test. This intentionally does not import
    # the simulator implementation or trust the recorded delivered flag.
    def inside(a):
        return abs(a[1]) < 3 and 0 < a[2] < 5
    expected, blocked = set(), set()
    per_second = collections.Counter(); max_distance_error = 0
    for row in physical:
        stamp = float(row['stamp']); kind = int(row['a_type'])
        assert t[0] <= stamp <= t[-1]+.1+1e-8
        tag = interpolation(stamp) + lever
        reporter = anchors[int(row['a'])] if kind else tag
        endpoint = anchors[int(row['b'])]
        assert np.linalg.norm(endpoint-reporter) <= 800+0.002
        distance = np.linalg.norm(reporter-tag) if kind else 0.
        max_distance_error = max(max_distance_error, abs(distance-float(row['reporter_to_tag_m'])))
        # 2 mm predeclared interpolation/error allowance; reject an ambiguous
        # threshold crossing rather than accepting a self-reported gate.
        assert abs(distance-800) > .002
        delivered = distance <= 800 and inside(reporter) and inside(tag)
        identity = (int(row['session']),int(row['physical_sequence']))
        (expected if delivered else blocked).add(identity)
        assert delivered == bool(int(row['delivered']))
        per_second[int(np.floor(stamp-1+1e-8))] += 1
    assert len(expected) + len(blocked) == len(physical)
    assert max_distance_error <= .002
    assert max(per_second.values()) <= 250
    assert blocked and expected, 'must exercise delivery and blocking'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run/'export/capture/raw'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    actual = set()
    while reader.has_next():
        topic,data,_ = reader.read_next()
        if topic != '/liuo/uwb/observations': continue
        batch = deserialize_message(data,UwbObservationBatch)
        identity = (int(batch.driver_session_id),int(batch.batch_seq))
        assert identity not in actual
        actual.add(identity)
        assert len(batch.observations)==1
        assert batch.observations[0].source_frame_seq == batch.batch_seq
    assert actual == expected and actual.isdisjoint(blocked)
    aa = [r for r in physical if r['a_type']=='1']
    result = dict(scope='S ordinary reporter delivery and physical budget only',result='PASS',
        physical_samples=len(physical),physical_aa=len(aa),aa_delivered=sum(int(r['delivered']) for r in aa),
        aa_blocked=len(blocked),max_physical_exchanges_per_second=max(per_second.values()),
        maximum_independent_reporter_distance_error_m=max_distance_error,bag_delivered_samples=len(actual))
    if args.reference:
        reference = list(csv.DictReader((args.reference/'export/simulation/physical_uwb.csv').open()))
        fields = ['physical_sequence','stamp','a_type','a','b','true_range','range','sample_index']
        assert [[r[k] for k in fields] for r in reference] == [[r[k] for k in fields] for r in physical]
        result['unchanged_physical_acquisition_reference'] = str(args.reference)
    out = run/'export/analysis/straight_reporter_delivery.json';out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
