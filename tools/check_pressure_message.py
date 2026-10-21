#!/usr/bin/env python3
"""Verify installed ROS message round-trip for explicit measurement stress."""
import copy
import json
from pathlib import Path
from rclpy.serialization import serialize_message, deserialize_message
from liuo_tunnel_msgs.msg import UwbRange
from delivery_schedule import DeliverySchedule

root = Path(__file__).resolve().parents[1]
profile = json.loads(
    (root / "packages/liuo_tunnel/config/delivery/correlated_nlos.json").read_text()
)
model = DeliverySchedule(profile)
base = UwbRange()
base.header.stamp.sec = 41
base.header.stamp.nanosec = 300000000
base.producer_id = 1
base.session_id = 123456789
base.sequence = 900
base.node_a_type = base.NODE_TAG
base.node_a_id = 0
base.node_b_type = base.NODE_ANCHOR
base.node_b_id = 4
base.status = base.STATUS_OK
base.range_m = 799.9
base.variance_m2 = 0.01
base.propagation_status = base.LOS_UNKNOWN
payload = serialize_message(base)
for elapsed in [39.9, 40.3, 60.0]:
    message = deserialize_message(payload, UwbRange)
    original_range, original_status, original_variance, bias = model.apply_range_stress(
        message, elapsed
    )
    assert (original_range, original_status, original_variance) == (799.9, 0, 0.01)
    expected = copy.deepcopy(base)
    if 40 <= elapsed < 60:
        assert bias > 0
        expected.range_m += bias
        expected.propagation_status = expected.NLOS_REPORTED
        assert message.range_m > 800
    else:
        assert bias == 0
    assert deserialize_message(serialize_message(message), UwbRange) == expected
    duplicate = deserialize_message(payload, UwbRange)
    assert model.apply_range_stress(duplicate, elapsed)[3] == bias
    assert duplicate == message
nominal = deserialize_message(payload, UwbRange)
assert DeliverySchedule().apply_range_stress(nominal, 40.3)[3] == 0
assert nominal == base
for name in ["single_anchor_loss.json", "correlated_nlos.json", "long_transport.json"]:
    DeliverySchedule(json.loads((root / "packages/liuo_tunnel/config/delivery" / name).read_text()))
print(
    "PASS: installed ROS UwbRange CDR round-trip, full-field preservation, no 800m clipping, duplicate identity, nominal no-op, all profile preflights"
)
