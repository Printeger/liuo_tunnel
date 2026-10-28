"""Read-only semantic identity of an exact, end-exclusive frozen bag prefix."""
import hashlib
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
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if origin is None:
            origin = stamp
        if stamp-origin >= round(seconds*1e9):
            break
        if topic in kinds:
            kind = kinds[topic]
            counts[kind] += 1
            hash_message(hashes[kind],kind,deserialize_message(data,types[topic]))
    if not all(counts.values()):
        raise ValueError('prefix lacks required sensors')
    return dict(counts=counts,raw_sha256={k:h.hexdigest() for k,h in hashes.items()},
                prefix_seconds=seconds,prefix_end_exclusive=True,prefix_origin_bag_ns=origin)
