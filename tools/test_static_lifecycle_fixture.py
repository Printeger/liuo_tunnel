#!/usr/bin/env python3
import json
import copy
from pathlib import Path
import tempfile
import unittest
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from liuo_tunnel_msgs.msg import StaticAnchorRecord,StaticRangeLeaf
from static_lifecycle_fixture import StaticLifecycleFixture


class Publisher:
    def __init__(self):
        self.messages = []
    def get_subscription_count(self):
        return 1
    def publish(self,message):
        self.messages.append(message)


class Node:
    def __init__(self):
        self.publishers = []
    def create_publisher(self,*args):
        result = Publisher()
        self.publishers.append(result)
        return result
    def destroy_publisher(self,publisher):
        pass


class StaticLifecycleFixtureTest(unittest.TestCase):
    def test_retransmission_preserves_physical_content_across_publisher_recreation(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            for path in ['metadata/config','metadata/manifests','export/odometry','export/simulation']:
                (run/path).mkdir(parents=True,exist_ok=True)
            record = StaticAnchorRecord()
            record.header.stamp.sec=1
            record.source_identity='original-source'
            record.record_id='record'
            record.first=4;record.second=9;record.revision=1
            leaf=StaticRangeLeaf();leaf.physical_id='physical';leaf.physical_source=4
            leaf.range_m=10.;leaf.variance_m2=.01;leaf.acquisition_stamp.sec=1
            record.leaves=[leaf];record.covered_blocks=['physical']
            second=copy.deepcopy(record)
            second.leaves[0].physical_id='physical2'
            second.leaves[0].range_m=10.1
            second.covered_blocks=['physical2']
            (run/'export/odometry/static_edges.jsonl').write_text(''.join(json.dumps(dict(event='input',reason='QUEUED',cdr_hex=serialize_message(r).hex()))+'\n' for r in (record,second)))
            config=dict(source_identity='original-source',max_leaves=2)
            (run/'metadata/config/map.json').write_text(json.dumps(dict(source_run=str(run),static_edge_source=config,
                static_edge_ledger=dict(leaves=[dict(physical_id='physical'),dict(physical_id='physical2')]))))
            node=Node();fixture=StaticLifecycleFixture(node,run,config,negative_probes=True)
            for tick in range(17000):
                ns=10**9+tick*5000000
                fixture.tick(Time(sec=ns//10**9,nanosec=ns%10**9))
            fixture.close()
            messages=[m for p in node.publishers for m in p.messages]
            self.assertEqual(len(messages),8103)
            self.assertEqual(len(node.publishers),3)
            self.assertEqual({m.revision for m in messages},{1,2})
            self.assertEqual({(m.first,m.second) for m in messages},{(4,9),(9,4)})
            self.assertEqual(len({(m.header.stamp.sec,m.header.stamp.nanosec) for m in messages}),8103)
            valid = [m for m in messages if m.header.stamp.nanosec%5000000 == 0]
            negative = [m for m in messages if m.header.stamp.nanosec%5000000 == 1]
            self.assertEqual(len(valid),8100)
            self.assertTrue(all(serialize_message(m.leaves[0])==serialize_message(leaf) for m in valid))
            self.assertTrue(all(len(m.leaves)==m.revision for m in valid))
            self.assertTrue(all(serialize_message(m.leaves[1])==serialize_message(second.leaves[0]) for m in valid if m.revision==2))
            self.assertEqual(len(negative),3)
            self.assertTrue(negative[0].deployment_epoch.endswith('-moved'))
            self.assertEqual(negative[1].leaves[0].range_m,10.25)
            self.assertTrue(negative[2].leaves[0].physical_id.endswith(':negative_capacity_probe'))


if __name__=='__main__':
    unittest.main()
