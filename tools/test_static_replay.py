import hashlib
import tempfile
import unittest
from pathlib import Path

from liuo_uwb_msgs.msg import UwbObservationBatch, UwbObservation
from static_replay import StaticReplayPublisher


class Publisher:
    def __init__(self):
        self.records = []

    def publish(self, record):
        self.records.append(record)


class Node:
    def __init__(self):
        self.publisher = Publisher()

    def create_publisher(self, *args):
        return self.publisher


class StaticReplayTests(unittest.TestCase):
    def test_delivery_restart_preserves_physical_identity_and_acquisition_time(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / 'export/capture/raw'
            raw.mkdir(parents=True)
            (root / 'metadata/manifests').mkdir(parents=True)
            (root / 'export/simulation').mkdir(parents=True)
            payload = b'unit fixture for source content binding, not a parsed bag'
            (raw / 'raw.mcap').write_bytes(payload)
            config = dict(source_model='rail_sim_independent_exchange_v1',
                          source_identity='sha256:' + hashlib.sha256(payload).hexdigest(),
                          network='network', deployment_epoch='deployment', variance_m2=.01)
            node = Node()
            source = StaticReplayPublisher(node, root, root, {5: (1, 4), 8: (1, 9)}, config)
            batch = UwbObservationBatch()
            batch.header.stamp.sec = 90
            observation = UwbObservation()
            observation.kind = 1
            observation.node_a, observation.node_b = 5, 8
            observation.reporter_id = 5
            observation.source_session_id = 1
            observation.source_frame_seq = 7
            observation.source_frame_seq_valid = True
            observation.pair_measurement_freshness_verified = True
            observation.source_frame_stamp.sec = 2
            observation.distance_m = 10.
            batch.observations = [observation]
            source.publish(batch)
            batch.driver_session_id = 999
            batch.header.stamp.sec = 100
            source.publish(batch)
            first, repeated = node.publisher.records
            self.assertEqual(first.covered_blocks, repeated.covered_blocks)
            self.assertEqual(repeated.leaves[0].acquisition_stamp.sec, 2)
            self.assertEqual(repeated.header.stamp.sec, 100)
            self.assertEqual(repeated.leaves[0].physical_source, 4)
            self.assertEqual(len(source.identities), 1)
            config['source_identity'] = 'sha256:wrong'
            with self.assertRaises(ValueError):
                StaticReplayPublisher(node, root, root, {5: (1, 4), 8: (1, 9)}, config)


if __name__ == '__main__':
    unittest.main()
