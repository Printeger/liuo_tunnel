"""60 s software load: independent synthetic noise, not physical TWR scheduling."""
import bisect
import csv
from pathlib import Path
import random
from liuo_uwb_msgs.msg import UwbObservation, UwbObservationBatch
from rclpy.serialization import serialize_message


class LoadFixture:
    def __init__(self, source, run, topic):
        self.topic = topic
        self.series = {}
        with (source/'export/simulation/uwb.csv').open() as f:
            for r in csv.DictReader(f):
                key=(int(r['a_type']),int(r['a']),int(r['b']))
                self.series.setdefault(key,[]).append((float(r['stamp']),float(r['true_range'])))
        self.times={k:[x[0] for x in v] for k,v in self.series.items()}
        self.rng=random.Random(79031)
        self.batch_seq=0
        self.log=(run/'export/simulation/load_sent.csv').open('w')
        self.csv=csv.writer(self.log)
        self.csv.writerow(['elapsed_s','batch_seq','driver_session','reporter','source_session','source_sequence','node_a','node_b','shard_index','shard_count','sent'])

    def distance(self, kind,a,b,t):
        key=(kind,a,b) if not kind or a<b else (kind,b,a)
        points=self.series[key]; times=self.times[key]
        i=bisect.bisect_left(times,t)
        if i==0: value=points[0][1]
        elif i==len(points): value=points[-1][1]
        else:
            left,right=points[i-1],points[i]
            value=left[1]+(right[1]-left[1])*(t-left[0])/(right[0]-left[0])
        return value+self.rng.gauss(0,.1)

    def batch(self,t,sequence,reporter,peers,shard=0,shards=1,sent=True):
        b=UwbObservationBatch();b.driver_session_id=70001;b.batch_seq=self.batch_seq;self.batch_seq+=1
        b.trigger_type=int(reporter!=0);b.header.stamp.sec=int(t);b.header.stamp.nanosec=round((t-int(t))*1e9)
        for peer in peers:
            o=UwbObservation();o.kind=b.trigger_type;o.reporter_id=o.node_a=reporter;o.node_b=peer
            o.source_session_id=100+reporter;o.source_frame_seq=sequence;o.source_frame_seq_valid=True
            o.tx_cycle_seq=sequence if reporter else 0;o.shard_index=shard;o.shard_count=shards
            o.source_frame_stamp=b.header.stamp;o.received_stamp=b.header.stamp;o.time_basis=1;o.time_uncertainty_s=0.
            o.source_hold_age_ms=65535;o.fp_rssi=o.rx_rssi=float('nan');o.pair_measurement_freshness_verified=True
            o.distance_m=self.distance(o.kind,reporter-1 if reporter else 0,peer-1,t)
            b.observations.append(o)
            self.csv.writerow([t-1,b.batch_seq,b.driver_session_id,reporter,o.source_session_id,sequence,reporter,peer,shard,shards,int(sent)])
        return b

    def records(self,records):
        origin=None; tick=0
        for topic,data,stamp in records:
            if origin is None:origin=stamp
            if topic==self.topic:continue
            yield topic,data,stamp
            if topic!='/clock':continue
            elapsed=tick*.005
            if tick<12000 and tick%4==0:
                yield self.topic,serialize_message(self.batch(1+elapsed,tick//4,0,[1,2,3])),stamp
            if tick<12000 and tick%40==0:
                cycle=tick//40
                yield self.topic,serialize_message(self.batch(1+elapsed,cycle,2,[1],0,2)),stamp
                sent=cycle%10!=9
                tail=self.batch(1+elapsed,cycle,2,[3],1,2,sent)
                if sent:yield self.topic,serialize_message(tail),stamp
            tick+=1
        self.log.close()
