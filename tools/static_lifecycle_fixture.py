"""A6 software delivery load of already-absorbed, unmodified physical AA.

No vehicle truth, new physical measurement, RF model, or deployable output.
"""
import copy
import csv
import hashlib
import json
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from liuo_tunnel_msgs.msg import StaticAnchorRecord


class StaticLifecycleFixture:
    def __init__(self,node,run,config,negative_probes=False):
        self.node,self.run,self.config = node,run,config
        document = json.loads((run/'metadata/config/map.json').read_text())
        assert document['static_edge_source']['source_identity'] == config['source_identity']
        absorbed = {leaf['physical_id'] for leaf in document['static_edge_ledger']['leaves']}
        assert config['max_leaves'] == len(absorbed), 'exercise a full inherited ledger'
        from pathlib import Path
        source = Path(document['source_run'])/'export/odometry/static_edges.jsonl'
        self.prototype = None
        self.second_leaf = None
        with source.open() as stream:
            for line in stream:
                row = json.loads(line)
                if row['event'] != 'input' or row['reason'] != 'QUEUED':
                    continue
                msg = deserialize_message(bytes.fromhex(row['cdr_hex']),StaticAnchorRecord)
                if len(msg.leaves)==1 and msg.leaves[0].physical_id in absorbed and msg.header.stamp.sec <= 2:
                    if self.prototype is None:
                        self.prototype = msg
                    elif (msg.leaves[0].physical_id != self.prototype.leaves[0].physical_id and
                          sorted((msg.first,msg.second)) == sorted((self.prototype.first,self.prototype.second))):
                        self.second_leaf = msg.leaves[0]
                        break
        assert self.prototype is not None and self.second_leaf is not None
        assert self.prototype.source_identity == config['source_identity']
        self.publisher = self.make_publisher()
        self.next_tick = 200
        self.generation = 0
        self.count = 0
        self.ids = {self.prototype.leaves[0].physical_id,self.second_leaf.physical_id}
        self.stream = (run/'export/simulation/static_load_sent.csv').open('w')
        self.log = csv.writer(self.stream)
        self.log.writerow(['tick','stamp','generation','event','sha256','publish_wall_ns'])
        self.bytes = 0
        self.events = set()
        self.negative_probes = negative_probes
        (run/'metadata/manifests/static_load_plan.json').write_text(json.dumps(dict(
            scope='software delivery only; original absorbed CAL physical leaf, independent REUSE TA/IMU/LiDAR',
            source_audit=str(source), physical_id=self.prototype.leaves[0].physical_id,
            physical_ids=sorted(self.ids),cumulative_revision='revision1 first leaf; revision2 same first plus second already-absorbed leaf',
            inherited_leaves=len(absorbed),configured_capacity=config['max_leaves'],
            rate_hz=100,active_elapsed_s=[2,83],publisher_recreation_elapsed_s=[25,45],
            slow_observer=dict(depth=37,sleep_s=.1,recover_ros_stamp=51),
            physical_deployment='unchanged; alternate direction/revision never changes physical leaf',
            restarts='DDS publisher recreation only; independent estimator restart is covered by A7; not a physical deployment reset',
            new_physical_samples=0,
            negative_packets=({6000:'wrong_epoch',6500:'conflicting_physical_content',7000:'capacity_new_identity'} if negative_probes else {})),indent=2)+'\n')

    def make_publisher(self):
        return self.node.create_publisher(StaticAnchorRecord,self.config.get('topic','/liuo/uwb/static_edges'),
            QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE))

    def publish(self,batch):
        # Raw independent-input AA is still published/audited, but is not
        # converted to the static channel in this explicitly declared load case.
        pass

    def tick(self,clock):
        stamp = clock.sec+clock.nanosec*1e-9
        tick = int(round((stamp-1)*100))
        if tick < self.next_tick or tick >= 8300:
            return
        if tick-self.next_tick > 1:
            raise RuntimeError('static load tick gap')
        self.next_tick = tick+1
        for boundary in (2500,4500):
            if tick >= boundary and boundary not in self.events:
                self.node.destroy_publisher(self.publisher)
                self.publisher = self.make_publisher()
                self.generation += 1
                self.events.add(boundary)
                self.log.writerow([tick,stamp,self.generation,'publisher_recreated','',time.monotonic_ns()])
        if self.publisher.get_subscription_count() < 1:
            self.log.writerow([tick,stamp,self.generation,'not_matched','',time.monotonic_ns()])
            return
        message = copy.deepcopy(self.prototype)
        message.header.stamp = clock
        message.revision = 1+tick%2
        if tick%2:
            message.first,message.second = message.second,message.first
            message.leaves.append(copy.deepcopy(self.second_leaf))
            message.covered_blocks.append(self.second_leaf.physical_id)
        payload = serialize_message(message)
        self.publisher.publish(message)
        self.log.writerow([tick,stamp,self.generation,'published',hashlib.sha256(payload).hexdigest(),time.monotonic_ns()])
        self.count += 1
        self.bytes += len(payload)
        cases = {6000:'wrong_epoch',6500:'conflicting_physical_content',7000:'capacity_new_identity'}
        if self.negative_probes and tick in cases:
            invalid = copy.deepcopy(message)
            invalid.header.stamp.nanosec += 1  # unique delivery identity, never a measurement timestamp
            event = cases[tick]
            if event == 'wrong_epoch':
                invalid.deployment_epoch += '-moved'
            elif event == 'conflicting_physical_content':
                invalid.leaves[0].range_m += .25
            else:
                invalid.record_id = 'A6_NEGATIVE_CAPACITY_NOT_A_MEASUREMENT'
                invalid.leaves[0].physical_id += ':negative_capacity_probe'
                invalid.covered_blocks = [invalid.leaves[0].physical_id]
            payload = serialize_message(invalid)
            self.publisher.publish(invalid)
            invalid_stamp = invalid.header.stamp.sec+invalid.header.stamp.nanosec*1e-9
            self.log.writerow([tick,invalid_stamp,self.generation,'negative_'+event,hashlib.sha256(payload).hexdigest(),time.monotonic_ns()])
            self.count += 1
            self.bytes += len(payload)

    def close(self):
        self.stream.close()
        (self.run/'export/simulation/static_delivery.json').write_text(json.dumps(dict(
            records=self.count,unique_physical_ids=sorted(self.ids),strategy='absorbed_physical_leaf_load',
            serialized_payload_bytes=self.bytes,new_physical_samples=0,
            physical_sampling='unchanged original absorbed physical sample; 100 Hz is software delivery, not measurement frequency'),indent=2)+'\n')
