#!/usr/bin/env python3
"""Freeze delivered noisy TA/AA for conditional diagnostics; never read truth."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from liuo_uwb_msgs.msg import UwbObservationBatch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='actual joint comparator with frozen input identity')
    parser.add_argument('--seconds', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert args.output.resolve().is_relative_to('/home/dev/ws_iap')
    assert not args.output.exists(), 'diagnostic input is immutable; choose a new path explicitly'
    identity = json.loads((args.run/'metadata/manifests/input_identity.json').read_text())
    config = json.loads((args.run/'metadata/config/phase1.json').read_text())
    assert args.seconds <= identity['seconds']
    source = Path(identity['source_run'])
    files = list((source/'export/capture/raw').glob('*.mcap'))
    assert len(files) == 1
    with files[0].open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    assert digest == identity['raw_sha256']
    namespace = 'sha256:' + digest
    source_model = config['uwb']['static_edges']['source_model']
    assert source_model in ('road_sim_global_exchange_v1', 'rail_sim_independent_exchange_v1')
    registry = {n['logical_id']: (n['type'], n['internal_id']) for n in config['uwb']['input']['nodes']}
    assert config['uwb']['input']['model']['ta_sigma_m'] == .1
    assert config['uwb']['input']['model']['aa_sigma_m'] == .1
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source/'export/capture/raw'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    first = reader.read_next()
    origin = first[2]
    topic = '/liuo/uwb/observations'
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    def records():
        yield first
        while reader.has_next():
            yield reader.read_next()
    samples = {'ta': {}, 'aa': {}}
    multiplicity = collections.Counter()
    ns = lambda stamp: stamp.sec*10**9 + stamp.nanosec
    for name, data, timestamp in records():
        elapsed = timestamp-origin
        if elapsed > round(args.seconds*1e9) or (identity.get('end_exclusive', False) and elapsed >= round(args.seconds*1e9)):
            break
        if name != topic:
            continue
        batch = deserialize_message(data, UwbObservationBatch)
        for o in batch.observations:
            assert o.source_frame_seq_valid and o.pair_measurement_freshness_verified
            assert o.time_basis in (1, 2) and o.time_uncertainty_s == 0
            a, b = registry[o.node_a], registry[o.node_b]
            if o.kind == 0:
                assert a[0] == 0 and b[0] == 1
                kind = 'ta'
                key = f'{namespace}:ta:{o.reporter_id}:{o.source_session_id}:{o.source_frame_seq}:{a[1]}:{b[1]}'
                endpoints = [a[1], b[1]]
            else:
                assert o.kind == 1 and a[0] == b[0] == 1
                kind = 'aa'
                endpoints = sorted([a[1], b[1]])
                if source_model == 'road_sim_global_exchange_v1':
                    key = f'{namespace}:road-exchange:{o.source_frame_seq}:{endpoints[0]}:{endpoints[1]}'
                else:
                    key = f'{namespace}:{o.source_session_id}:{o.source_frame_seq}:{endpoints[0]}:{endpoints[1]}'
            value = dict(endpoints=endpoints, acquisition_ns=ns(o.source_frame_stamp),
                         range_m=float(o.distance_m), variance_m2=.01)
            previous = samples[kind].get(key)
            if previous:
                assert previous['measurement'] == value, 'same physical identity has conflicting content'
            else:
                samples[kind][key] = dict(measurement=value, deliveries=[])
            samples[kind][key]['deliveries'].append(dict(bag_ns=timestamp, receive_ns=ns(o.received_stamp),
                driver_session=batch.driver_session_id, batch_sequence=batch.batch_seq, reporter=o.reporter_id,
                source_session=o.source_session_id, source_sequence=o.source_frame_seq,
                node_a=o.node_a, node_b=o.node_b))
            multiplicity[kind] += 1
    canonical = {kind: {key: row['measurement'] for key, row in sorted(rows.items())} for kind, rows in samples.items()}
    encoded = json.dumps(canonical, sort_keys=True, separators=(',', ':')).encode()
    output = dict(role='CONDITIONAL_DIAGNOSTIC_INPUT_NOT_DEPLOYABLE', source_run=str(source),
                  comparator_run=str(args.run), raw_sha256=digest, seconds=args.seconds,
                  bag_origin_ns=origin, end_exclusive=identity.get('end_exclusive', False),
                  measurement_set_sha256=hashlib.sha256(encoded).hexdigest(),
                  multiplicity=dict(multiplicity), counts={k: len(v) for k, v in samples.items()},
                  samples=samples, sensors=config['sensors'],
                  scope='Only actually delivered raw noisy observations; no physical-global log or truth. D2 AA has zero vehicle information.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps({k: output[k] for k in ('raw_sha256','measurement_set_sha256','counts','multiplicity')}, indent=2))


if __name__ == '__main__':
    main()
