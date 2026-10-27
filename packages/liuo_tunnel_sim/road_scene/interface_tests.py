"""Driver-shaped packet fixtures, independent of road physical samples."""
import copy
from liuo_uwb_msgs.msg import UwbObservationBatch, UwbObservation
from rclpy.serialization import serialize_message, deserialize_message
from wire import semantic_bytes


def packet_tests():
  accepted = set()

  def consume(batch):
    fresh = []
    for o in batch.observations:
      if not o.source_frame_seq_valid or not 0 <= o.shard_index < o.shard_count:
        continue
      key = (batch.driver_session_id, o.reporter_id, o.source_session_id, o.source_frame_seq, o.node_b)
      if key not in accepted:
        accepted.add(key)
        fresh.append(o)
    return fresh

  def packet(shard, cycle=1, frame=9):
    b = UwbObservationBatch()
    b.driver_session_id = 1
    b.trigger_type = 1
    for peer in ([2, 3, 4, 5, 6] if shard == 0 else [7, 8]):
      o = UwbObservation()
      o.kind = 1
      o.reporter_id = 1
      o.node_a = 1
      o.node_b = peer
      o.source_session_id = 2
      o.source_frame_seq = frame
      o.source_frame_seq_valid = True
      o.shard_index = shard
      o.shard_count = 2
      o.tx_cycle_seq = cycle
      o.distance_m = 10.
      o.fp_rssi = float('nan')
      o.rx_rssi = float('nan')
      o.rssi_valid = False
      b.observations.append(o)
    return b

  first = packet(0)
  tail = packet(1)
  assert len(consume(first)) == 5  # Missing tail never blocks first shard.
  assert len(consume(packet(0, cycle=2))) == 0  # New report cycle is not fresh ranging.
  assert len(consume(tail)) == 2
  assert len(consume(packet(0, cycle=3, frame=10))) == 5  # Same range, independent source frame.
  for b in [first, tail]:
    assert semantic_bytes(b) == semantic_bytes(deserialize_message(serialize_message(b), UwbObservationBatch))
  return dict(pass_=True,
              first_shard_immediate=5,
              late_tail=2,
              repeat_cycle_fresh=0,
              same_value_new_frame=5,
              scope='synthetic interface fixture, not road ranging or driver hardware certification')
