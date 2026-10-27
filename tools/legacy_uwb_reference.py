"""Reference-only inverse representation for a retained pre-migration binary.

Never used by the unified production node. Only one-edge, independently generated
simulation batches with retained pre-float32 samples are eligible. No unknown
quality or timing field is upgraded. Historical CSV precision is disclosed.
"""
import csv
import math
from copy import deepcopy
from liuo_tunnel_msgs.msg import UwbRange
from rclpy.qos import QoSProfile, ReliabilityPolicy


class LegacyReference:
    def __init__(self,node,source,run,registry):
        self.registry=registry;self.samples={};self.seen=set()
        with (source/'export/simulation/uwb.csv').open() as f:
            for r in csv.DictReader(f):
                if 'physical_range' not in r:raise ValueError('reference requires retained pre-float32 samples')
                self.samples[(int(r['session']),int(r['sequence']))]=r
                if len(self.samples)>200000:raise ValueError('reference sample capacity exceeded')
        self.publisher=node.create_publisher(UwbRange,'/sensors/uwb/range',QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.log=(run/'export/simulation/legacy_reference_delivered.csv').open('w')
        self.csv=csv.writer(self.log)
        self.csv.writerow(['driver_session','reporter','source_session','source_sequence','legacy_producer','legacy_session','legacy_sequence','a_type','a','b_type','b','stamp','physical_distance','wire_distance','variance'])

    def publish(self,b):
        if len(b.observations)!=1:raise ValueError('reference requires original single-event physical batches')
        o=b.observations[0];key=(b.driver_session_id,o.source_frame_seq)
        if key in self.seen:raise ValueError('reference duplicate physical sample')
        if not o.source_frame_seq_valid or o.time_basis not in (1,2) or not math.isfinite(o.time_uncertainty_s) or o.time_uncertainty_s!=0 or not o.pair_measurement_freshness_verified:
            raise ValueError('reference refuses unknown timing/independence')
        r=self.samples[key];a=self.registry[o.node_a];c=self.registry[o.node_b]
        if a!=(int(r['a_type']),int(r['a'])) or c!=(1,int(r['b'])):raise ValueError('reference endpoint mismatch')
        stamp=o.source_frame_stamp.sec+o.source_frame_stamp.nanosec*1e-9
        if abs(stamp-float(r['stamp']))>1e-9:raise ValueError('reference measurement time mismatch')
        m=UwbRange();m.header.stamp=deepcopy(o.source_frame_stamp);m.producer_id=1;m.session_id=b.driver_session_id;m.sequence=o.source_frame_seq
        m.node_a_type,m.node_a_id=a;m.node_b_type,m.node_b_id=c;m.range_m=float(r['physical_range']);m.variance_m2=float(r['variance'])
        self.publisher.publish(m);self.seen.add(key)
        self.csv.writerow([b.driver_session_id,o.reporter_id,o.source_session_id,o.source_frame_seq,1,m.session_id,m.sequence,*a,*c,stamp,m.range_m,o.distance_m,m.variance_m2])

    def close(self):
        self.log.close()
        if len(self.seen)!=len(self.samples):raise ValueError('reference did not deliver every physical sample')
