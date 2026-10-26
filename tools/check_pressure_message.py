#!/usr/bin/env python3
"""Actual generated interface serialization, all-field hash, and distance-only stress."""
import copy
import hashlib
import json
from pathlib import Path
from rclpy.serialization import serialize_message, deserialize_message
from liuo_uwb_msgs.msg import UwbObservation, UwbObservationBatch
from delivery_schedule import DeliverySchedule
from raw_identity import hash_message, semantic_fields
root = Path(__file__).resolve().parents[1]
profile = json.loads((root / "packages/liuo_tunnel/config/delivery/correlated_nlos.json").read_text())
model = DeliverySchedule(profile)
base = UwbObservationBatch(driver_session_id=2**50, batch_seq=900)
o = UwbObservation(node_b=5, source_session_id=2**40, source_frame_seq=8, source_frame_seq_valid=True,
                   shard_count=1, distance_m=799.9, fp_rssi=float('nan'), rx_rssi=float('nan'), time_uncertainty_s=float('nan'))
o.source_frame_stamp.sec = 41
base.observations = [o]
payload = serialize_message(base)
base = deserialize_message(payload, UwbObservationBatch)
for elapsed in [39.9, 40.3, 60.0]:
    message = deserialize_message(payload, UwbObservationBatch)
    original, bias = model.apply_range_stress(message.observations[0], elapsed, [4])
    assert original == base.observations[0].distance_m
    before, after = semantic_fields(base), semantic_fields(message)
    after['observations'][0]['distance_m'] = before['observations'][0]['distance_m']
    assert before == after
    if 40 <= elapsed < 60: assert bias > 0 and message.observations[0].distance_m > 800
    else: assert bias == 0
    h1, h2 = hashlib.sha256(), hashlib.sha256()
    hash_message(h1, 'uwb', message)
    hash_message(h2, 'uwb', deserialize_message(serialize_message(message), UwbObservationBatch))
    assert h1.digest() == h2.digest()
print('PASS: batch CDR/hash round-trip, NaN preservation, distance-only stress, no clipping')
