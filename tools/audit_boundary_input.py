"""Independent original-bag versus received 35+35s boundary transformation.

Does not import the replay or fixture implementation. Checks full received UWB
fields, including declared negative packets, and original repeated sensor hashes.
"""
import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path


def audit(run):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from liuo_uwb_msgs.msg import UwbObservationBatch
    from raw_identity import hash_message,semantic_fields
    frozen=json.loads((run/'metadata/manifests/frozen_input.json').read_text())
    source=Path(frozen['source_run'])
    kinds={'/clock':'clock','/sensors/imu':'imu','/sensors/lidar/points':'cloud'}
    hashes={k:hashlib.sha256() for k in kinds.values()}
    counts={k:0 for k in hashes}
    originals=[]
    for repeat in range(2):
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(source/'export/capture/raw'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
        types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types()}
        origin=None
        while reader.has_next():
            topic,data,stamp=reader.read_next()
            if origin is None: origin=stamp
            elapsed=(stamp-origin)/1e9
            if elapsed>=35: break
            if topic in kinds:
                kind=kinds[topic]; counts[kind]+=1
                hash_message(hashes[kind],kind,deserialize_message(data,types[topic]))
            elif types[topic] is UwbObservationBatch and repeat==0:
                originals.append((elapsed,deserialize_message(data,UwbObservationBatch)))
    observed=json.loads((run/'metadata/manifests/observation_audit.json').read_text())
    expected_hashes={k:h.hexdigest() for k,h in hashes.items()}
    assert all(observed['counts'][k]==counts[k] and observed['raw_sha256'][k]==expected_hashes[k] for k in hashes)
    with (run/'export/odometry/uwb_batches.csv').open() as stream:
        batches=list(csv.DictReader(stream))
    assert len(batches)==2*len(originals)==observed['counts']['uwb']
    exceptional={}
    expired_base=None
    received_hash=hashlib.sha256()
    for index,row in enumerate(batches):
        half,offset=divmod(index,len(originals))
        elapsed,base=originals[offset]
        delivery_elapsed=elapsed+half*35
        expected=copy.deepcopy(base)
        if half: expected.driver_session_id+=1000000000
        actual=deserialize_message(bytes.fromhex(row['cdr_hex']),UwbObservationBatch)
        seq=actual.batch_seq
        if 900000<=seq<=900011:
            assert base.trigger_type==0 and seq not in exceptional
            exceptional[seq]=delivery_elapsed
            if seq==900000:
                assert 25<=delivery_elapsed<25.1
                expired_base=copy.deepcopy(base)
                expected.observations=expected.observations[:1]
                expected.observations[0].source_frame_seq=100000
                expected.observations[0].source_frame_stamp.sec=1
                expected.observations[0].source_frame_stamp.nanosec=0
            elif seq in (900010,900011):
                assert expired_base is not None
                expected=copy.deepcopy(expired_base)
                expected.observations=expected.observations[:1]
                o=expected.observations[0]
                o.source_frame_seq=300000 if seq==900010 else 150000
                if seq==900011:
                    assert 26<=delivery_elapsed<26.1
                    o.source_frame_stamp.sec=1000; o.source_frame_stamp.nanosec=0
                else: assert 36<=delivery_elapsed<36.1
            else:
                block=seq-900001
                assert 30+block*.5<=delivery_elapsed<30+block*.5+.1
                expected=copy.deepcopy(originals[0][1])
                expected.observations=[]
                for number in range(block*512,min((block+1)*512,4097)):
                    o=copy.deepcopy(originals[0][1].observations[0])
                    o.source_frame_seq=200000+number
                    o.source_frame_stamp.sec=40; o.source_frame_stamp.nanosec=0
                    expected.observations.append(o)
            expected.batch_seq=seq
        assert semantic_fields(actual)==semantic_fields(expected),f'unplanned UWB change at delivery {index}'
        hash_message(received_hash,'uwb',actual)
    assert set(exceptional)==set(range(900000,900012))
    assert received_hash.hexdigest()==observed['raw_sha256']['uwb']
    expected_hashes['uwb']=received_hash.hexdigest()
    runtime=json.loads((run/'export/simulation/runtime.json').read_text())
    assert runtime['delivered_sha256']==expected_hashes
    result=dict(result='PASS_DECLARED_BOUNDARY_TRANSFORM',source=str(source),
        original_prefix_s=35,replays=2,negative_replacements=len(exceptional),
        received_uwb_batches=len(batches),counts=observed['counts'],hashes=expected_hashes,
        scope='exact original sensors repeated and independently reconstructed declared UWB faults; not identity with untransformed 70s source')
    (run/'export/analysis/boundary_input_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path)
    print(json.dumps(audit(parser.parse_args().run),indent=2))
