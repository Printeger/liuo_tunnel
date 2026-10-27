"""Bounded queue/clock-reset experiment on frozen public ROS sensor streams.

At 25 s send an expired TA, at 30 s send 4097 future reports, then rewind
all sensor stamps by 35 s and create a new dataset at 35 s. Physical payloads
are retained. The replay's monotonic delivery schedule is unchanged.
"""
from copy import deepcopy
import csv
from rclpy.serialization import deserialize_message, serialize_message


class BoundaryFixture:
    def __init__(self,run,topic,types):
        self.run=run;self.topic=topic;self.types=types
        self.expired=False;self.far_future=False;self.future=0;self.late=False;self.old=None
        self.log=(run/'export/simulation/boundary_sent.csv').open('w')
        self.csv=csv.writer(self.log)
        self.csv.writerow(['scenario','driver_session','batch_seq','source_session','sequence','node_a','node_b','expected_input'])

    def record(self,label,b,expected):
        for o in b.observations:
            self.csv.writerow([label,b.driver_session_id,b.batch_seq,o.source_session_id,o.source_frame_seq,o.node_a,o.node_b,expected])

    def records(self,records):
        origin=None
        for topic,data,ns in records:
            if origin is None:origin=ns
            elapsed=(ns-origin)*1e-9
            if elapsed<25:
                yield topic,data,ns;continue
            m=deserialize_message(data,self.types[topic])
            if elapsed>=35:
                if topic=='/clock':m.clock.sec-=35
                else:m.header.stamp.sec-=35
                if topic==self.topic:
                    m.driver_session_id+=1000000000
                    for o in m.observations:
                        o.source_frame_stamp.sec-=35;o.received_stamp.sec-=35
                data=serialize_message(m)
            if topic==self.topic and m.trigger_type==0:
                if not self.expired:
                    self.expired=True;self.old=deepcopy(m)
                    m.batch_seq=900000;m.observations=m.observations[:1]
                    o=m.observations[0];o.source_frame_seq=100000;o.source_frame_stamp.sec=1;o.source_frame_stamp.nanosec=0
                    self.record('expired',m,'QUEUED');data=serialize_message(m)
                elif elapsed>=26 and not self.far_future:
                    self.far_future=True;m=deepcopy(self.old);m.batch_seq=900011;m.observations=m.observations[:1]
                    o=m.observations[0];o.source_frame_seq=150000;o.source_frame_stamp.sec=1000;o.source_frame_stamp.nanosec=0
                    self.record('far_future',m,'FUTURE_TIME_LIMIT');data=serialize_message(m)
                elif 30<=elapsed<35 and self.future<4097:
                    template=deepcopy(self.old.observations[0]);m=deepcopy(self.old);m.observations=[]
                    m.batch_seq=900001+self.future//512
                    for _ in range(min(512,4097-self.future)):
                        o=deepcopy(template);o.source_frame_seq=200000+self.future;o.source_frame_stamp.sec=40;o.source_frame_stamp.nanosec=0
                        m.observations.append(o);self.future+=1
                    # Exactly the last entry exceeds the 4096 pending limit.
                    self.record('future',m,'QUEUE_LIMIT' if self.future==4097 else 'QUEUED');data=serialize_message(m)
                elif elapsed>=36 and not self.late:
                    self.late=True;m=deepcopy(self.old);m.batch_seq=900010
                    m.observations=m.observations[:1];m.observations[0].source_frame_seq=300000
                    self.record('retired_driver_after_clock_reset',m,'RETIRED_DRIVER_SESSION');data=serialize_message(m)
            yield topic,data,ns
        self.log.close()
        if not(self.expired and self.future==4097 and self.late):raise RuntimeError('boundary scenario did not complete')
