"""Canonical semantic stream identity, independent of CDR padding."""

import json
import struct
import math


def semantic_fields(value):
    """All declared ROS fields, including invalid optional values; canonical NaN."""
    if hasattr(value, "get_fields_and_field_types"):
        return {name: semantic_fields(getattr(value, name)) for name in value.get_fields_and_field_types()}
    if isinstance(value, (list, tuple)):
        return [semantic_fields(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else ("+Infinity" if value > 0 else "-Infinity")
    return value


def observation_identity(batch, observation, network):
    return (network, batch.driver_session_id, observation.reporter_id, observation.source_session_id,
            observation.source_frame_seq_valid, observation.source_frame_seq, observation.kind,
            observation.node_a, observation.node_b)



def hash_message(hasher, kind, m):
    # Canonical semantic bytes avoid CDR alignment padding differences.
    if kind == "clock":
        data = [m.clock.sec, m.clock.nanosec]
    else:
        data = [m.header.stamp.sec, m.header.stamp.nanosec, m.header.frame_id]
        if kind == "imu":
            data += [
                m.orientation.x,
                m.orientation.y,
                m.orientation.z,
                m.orientation.w,
                m.angular_velocity.x,
                m.angular_velocity.y,
                m.angular_velocity.z,
                m.linear_acceleration.x,
                m.linear_acceleration.y,
                m.linear_acceleration.z,
                list(m.orientation_covariance),
                list(m.angular_velocity_covariance),
                list(m.linear_acceleration_covariance),
            ]
        elif kind == "uwb":
            data = ["liuo_uwb_observation_batch_v1", semantic_fields(m)]
        else:
            data += [
                m.height,
                m.width,
                m.is_bigendian,
                m.point_step,
                m.row_step,
                m.is_dense,
                [(f.name, f.offset, f.datatype, f.count) for f in m.fields],
            ]
    encoded = json.dumps(data, separators=(",", ":"), allow_nan=True).encode()
    hasher.update(struct.pack("<Q", len(encoded)))
    hasher.update(encoded)
    if kind == "cloud":
        payload = bytes(m.data)
        hasher.update(struct.pack("<Q", len(payload)))
        hasher.update(payload)
