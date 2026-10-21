"""Canonical semantic stream identity, independent of CDR padding."""

import json
import struct


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
            data += [
                m.producer_id,
                m.session_id,
                m.sequence,
                m.node_a_type,
                m.node_a_id,
                m.node_b_type,
                m.node_b_id,
                m.status,
                m.range_m,
                m.variance_m2,
                m.propagation_status,
            ]
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
