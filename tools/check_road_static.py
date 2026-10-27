#!/usr/bin/env python3
"""Independently compare delivered road AA, received records and absorbed leaves."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from liuo_uwb_msgs.msg import UwbObservationBatch
from liuo_tunnel_msgs.msg import StaticAnchorRecord


def check(run):
    identity = json.loads((run / 'metadata/manifests/input_identity.json').read_text())
    config = json.loads((run / 'metadata/config/phase1.json').read_text())
    source = Path(identity['source_run'])
    files = list((source / 'export/capture/raw').glob('*.mcap'))
    assert len(files) == 1
    with files[0].open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    assert digest == identity['raw_sha256']
    namespace = 'sha256:' + digest
    assert config['uwb']['static_edges']['source_identity'] == namespace
    registry = {int(n['logical_id']): (int(n['type']), int(n['internal_id'])) for n in config['uwb']['input']['nodes']}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source / 'export/capture/raw'), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    first = reader.read_next()
    start = first[2]
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/liuo/uwb/observations']))
    def remaining():
        while reader.has_next():
            yield reader.read_next()
    expected, raw_records = {}, 0
    for topic, data, timestamp in itertools.chain([first], remaining()):
        if timestamp - start > round(identity['seconds'] * 1e9):
            break
        if topic != '/liuo/uwb/observations':
            continue
        batch = deserialize_message(data, UwbObservationBatch)
        for observation in batch.observations:
            if observation.kind != 1:
                continue
            assert observation.source_frame_seq_valid and observation.pair_measurement_freshness_verified
            assert registry[observation.node_a][0] == registry[observation.node_b][0] == 1
            pair = sorted([registry[observation.node_a][1], registry[observation.node_b][1]])
            key = f'{namespace}:road-exchange:{observation.source_frame_seq}:{pair[0]}:{pair[1]}'
            value = dict(pair=pair, range=observation.distance_m,
                         acquired=observation.source_frame_stamp.sec * 10**9 + observation.source_frame_stamp.nanosec)
            if key in expected:
                assert expected[key] == value
            expected[key] = value
            raw_records += 1
    audit = [json.loads(line) for line in (run / 'export/odometry/static_edges.jsonl').read_text().splitlines()]
    received, received_count = {}, 0
    for row in audit:
        if row['event'] != 'input':
            continue
        assert row['reason'] == 'QUEUED', row['reason']
        message = deserialize_message(bytes.fromhex(row['cdr_hex']), StaticAnchorRecord)
        assert len(message.leaves) == 1 and message.source_identity == namespace
        leaf = message.leaves[0]
        assert message.covered_blocks == [leaf.physical_id]
        expected_leaf = expected[leaf.physical_id]
        assert [message.first, message.second] == expected_leaf['pair']
        assert leaf.physical_source == message.first and leaf.range_m == expected_leaf['range']
        assert leaf.acquisition_stamp_valid and leaf.time_basis == 'shared_simulation_clock'
        assert leaf.acquisition_stamp.sec * 10**9 + leaf.acquisition_stamp.nanosec == expected_leaf['acquired']
        assert leaf.variance_m2 == .01
        received[leaf.physical_id] = expected_leaf
        received_count += 1
    assert received == expected and received_count == raw_records
    published = json.loads((run / 'export/simulation/static_delivery.json').read_text())
    assert published['records'] == raw_records and set(published['unique_physical_ids']) == set(expected)
    model = json.loads((run / 'export/map/calibrated_map.json').read_text())
    assert model['static_information_policy'] == 'EXACT_LEDGER'
    assert model['static_edge_ledger']['network'] == config['uwb']['static_edges']['network']
    assert model['static_edge_ledger']['deployment_epoch'] == config['uwb']['static_edges']['deployment_epoch']
    leaves = model['static_edge_ledger']['leaves']
    assert len({leaf['physical_id'] for leaf in leaves}) == len(leaves)
    for leaf in leaves:
        source_leaf = expected[leaf['physical_id']]
        assert [leaf['first'], leaf['second']] == source_leaf['pair']
        assert leaf['range_m'] == source_leaf['range'] and leaf['variance_m2'] == .01
    updates = [row for row in audit if row['event'] == 'graph_update']
    checkpoint = [row for row in updates if row['stamp'] <= model['measurement_stamp']][-1]
    assert checkpoint['fused_leaves'] == len(leaves)
    assert updates[-1]['fused_leaves'] + updates[-1]['pending'] == len(expected)
    covariance = np.asarray(model['joint_covariance'])
    assert covariance.shape == (3*len(model['anchors']),)*2
    assert np.allclose(covariance,covariance.T,rtol=0,atol=1e-10)
    assert np.linalg.eigvalsh(covariance).min() > 0
    first_fused = next((row['stamp'] for row in updates if row['fused_leaves']), None)
    oldest = min((expected[leaf['physical_id']]['acquired']*1e-9 for leaf in leaves), default=None)
    report = dict(scope='delivered raw AA to callback and map ledger; not localization/reuse acceptance', result='PASS',
                  raw_records=raw_records, unique_physical_leaves=len(expected), received_records=received_count,
                  map_absorbed_leaves=len(leaves), final_fused_leaves=updates[-1]['fused_leaves'],
                  final_pending_leaves=updates[-1]['pending'], explicit_edges=updates[-1]['explicit_factors'],
                  first_fused_stamp=first_fused, oldest_absorbed_acquisition=oldest,
                  map_stamp=model['measurement_stamp'], anchor_ids=[a['id'] for a in model['anchors']],
                  minimum_map_eigenvalue=float(np.linalg.eigvalsh(covariance).min()), raw_sha256=digest)
    (run / 'export/analysis/road_static_audit.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    print(json.dumps(check(parser.parse_args().run), indent=2))
