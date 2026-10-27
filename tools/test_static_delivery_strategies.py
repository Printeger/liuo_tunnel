#!/usr/bin/env python3
"""Real message/CDR tests for delivery identity, reporter isolation and cache bounds."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from rclpy.serialization import serialize_message, deserialize_message
from liuo_uwb_msgs.msg import UwbObservation, UwbObservationBatch
from liuo_tunnel_msgs.msg import StaticAnchorRecord
from static_replay import StaticReplayPublisher


class Node:
    def __init__(self):
        self.records=[]

    def create_publisher(self, *args):
        return self

    def publish(self, record):
        self.records.append(serialize_message(record))


def batch(reporter, other, sequence, delivery_stamp):
    result=UwbObservationBatch()
    result.header.stamp.sec=delivery_stamp
    item=UwbObservation()
    item.kind=1; item.reporter_id=reporter; item.node_a=reporter; item.node_b=other
    item.source_session_id=27; item.source_frame_seq=sequence; item.source_frame_seq_valid=True
    item.pair_measurement_freshness_verified=True
    item.source_frame_stamp.sec=sequence
    item.distance_m=400.
    result.observations=[item]
    return result


class StaticStrategies(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        (self.root/'export/capture/raw').mkdir(parents=True)
        (self.root/'metadata/manifests').mkdir(parents=True)
        (self.root/'export/capture/raw/raw_0.mcap').write_bytes(b'constructor identity fixture; not a bag parser test')
        self.sha=hashlib.sha256((self.root/'export/capture/raw/raw_0.mcap').read_bytes()).hexdigest()

    def tearDown(self):
        self.temp.cleanup()

    def publisher(self, strategy):
        node=Node()
        publisher=StaticReplayPublisher(node,self.root,self.root,{1:(1,10),2:(1,20),3:(1,30)},dict(
            source_model='rail_sim_independent_exchange_v1',source_identity='sha256:'+self.sha,
            network='test',deployment_epoch='fixture',variance_m2=.01,delivery_strategy=strategy))
        return node,publisher

    def test_same_physical_set_and_reporter_isolation(self):
        identities=[]
        for strategy,count in [('cache_sync',3),('continuous_report',4)]:
            node,publisher=self.publisher(strategy)
            for item in [batch(1,2,1,11),batch(2,3,2,12),batch(1,2,3,13)]:
                publisher.publish(item)
            records=[deserialize_message(data,StaticAnchorRecord) for data in node.records]
            self.assertEqual(len(records),count)
            identities.append({r.leaves[0].physical_id for r in records})
            if strategy=='continuous_report':
                repeated=records[-1]
                self.assertEqual(repeated.first,10)
                self.assertEqual(repeated.header.stamp.sec,13)
                self.assertEqual(repeated.leaves[0].acquisition_stamp.sec,1)
                self.assertEqual(repeated.record_id,records[0].record_id)
                self.assertEqual(serialize_message(repeated.leaves[0]),serialize_message(records[0].leaves[0]))
            publisher.close()
            report=json.loads((self.root/'export/simulation/static_delivery.json').read_text())
            self.assertEqual(report['records'],count)
            self.assertEqual(report['serialized_payload_bytes'],sum(map(len,node.records)))
        self.assertEqual(*identities)

    def test_thousand_duplicate_deliveries_do_not_create_new_cache_samples(self):
        node,publisher=self.publisher('cache_sync')
        for stamp in range(1000):
            publisher.publish(batch(1,2,1,stamp))
        self.assertEqual(len(node.records),1)
        self.assertEqual(len(publisher.identities),1)
        changed=batch(1,2,1,1001);changed.observations[0].distance_m=401.
        with self.assertRaisesRegex(ValueError,'changed payload'):
            publisher.publish(changed)

    def test_repeated_report_cache_is_bounded(self):
        node,publisher=self.publisher('continuous_report')
        for sequence in range(1,81):
            before=len(node.records)
            publisher.publish(batch(1,2,sequence,sequence+100))
            self.assertLessEqual(len(node.records)-before,33)
            self.assertLessEqual(len(publisher.reporter_cache[1]),32)
        self.assertEqual(len(publisher.identities),80)


if __name__=='__main__':
    unittest.main()
