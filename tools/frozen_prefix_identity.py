"""Read-only semantic identity of an exact, end-exclusive frozen bag prefix."""
import hashlib
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from raw_identity import hash_message


def prefix_identity(source, seconds, uwb_topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source/'export/capture/raw'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr','cdr'))
    kinds = {'/clock':'clock','/sensors/imu':'imu','/sensors/lidar/points':'cloud',uwb_topic:'uwb'}
    types = {row.name:get_message(row.type) for row in reader.get_all_topics_and_types() if row.name in kinds}
    assert set(types) == set(kinds)
    counts = {kind:0 for kind in kinds.values()}
    hashes = {kind:hashlib.sha256() for kind in counts}
    origin = None
    last_cloud = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if origin is None:
            origin = stamp
        if stamp-origin >= round(seconds*1e9):
            break
        if topic in kinds:
            kind = kinds[topic]
            counts[kind] += 1
            message = deserialize_message(data,types[topic])
            hash_message(hashes[kind],kind,message)
            if kind == 'cloud':
                last_cloud = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
    if not all(counts.values()):
        raise ValueError('prefix lacks required sensors')
    return dict(counts=counts,raw_sha256={k:h.hexdigest() for k,h in hashes.items()},
                prefix_seconds=seconds,prefix_end_exclusive=True,prefix_origin_bag_ns=origin,prefix_last_cloud_stamp=last_cloud)


def write_truth_prefix(source,destination,last_cloud_stamp):
    """Keep original truth rows only through the actually included last scan."""
    with source.open() as input_stream, destination.open('w') as output_stream:
        reader = csv.DictReader(input_stream)
        writer = csv.DictWriter(output_stream,fieldnames=reader.fieldnames)
        writer.writeheader()
        for row in reader:
            if round(float(row['stamp'])*1e9) <= round(last_cloud_stamp*1e9):
                writer.writerow(row)


def write_boundary_truth(source,destination,duration):
    """Evaluation-only truth for BoundaryFixture's original-prefix seek.

    Both streams replay the exact original trajectory, never shift positions
    or sensor time. Only the second delivery session label changes.
    """
    if not 60 <= duration <= 70:
        raise ValueError('boundary fixture supplies at most 35+35 seconds')
    with source.open() as stream:
        reader=csv.DictReader(stream)
        fields=reader.fieldnames
        rows=list(reader)
    if not rows or len({r['session'] for r in rows})!=1:
        raise ValueError('boundary fixture requires one original session')
    start=round(float(rows[0]['stamp'])*1e9)
    with destination.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader()
        for index,seconds in enumerate((35,duration-35)):
            for row in rows:
                if round(float(row['stamp'])*1e9)-start >= round(seconds*1e9):
                    break
                transformed=dict(row)
                transformed['session']=str(int(row['session'])+index*1000000000)
                writer.writerow(transformed)
