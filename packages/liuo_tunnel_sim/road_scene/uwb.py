"""Physical pair sampling and independently gated software delivery; no estimator imports."""
import csv
import numpy as np
from liuo_uwb_msgs.msg import UwbObservation, UwbObservationBatch
from builtin_interfaces.msg import Time


def stamp(t):
  ns = int(round((1 + float(t)) * 1e9))
  return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def anchors(road):
  ids = np.array([11, 3, 16, 7, 1, 14, 9, 5, 18, 2, 13, 8, 4, 17, 6, 12, 10, 15])
  s = 100 + 400 * np.arange(18)
  sides = np.where(np.arange(18) % 2 == 0, 1, -1)
  z = np.where(sides == 1, 2.5, 3.5)
  p = road.world(s, sides * 5.52, z)
  return ids, s, sides, z, p


def simulate(road, motion, geometry, cfg, duration, output):
  ids, s, sides, z, p = anchors(road)
  rng = np.random.default_rng(np.random.SeedSequence([cfg['seed'], 30]))
  params = cfg['uwb']
  times = np.arange(0, duration - .014, .05)
  car = motion.state(times)['p']
  visible = geometry.los(np.broadcast_to(car[:, None, :], (len(times), 18, 3)), np.broadcast_to(p, (len(times), 18, 3))).reshape(-1, 18)
  aa = [(i, j) for i in range(18) for j in range(i + 1, 18) if geometry.los(p[i], p[j])[0]]
  requests = [(float(t), 0, i, -1) for row, t in enumerate(times) for i in np.flatnonzero(visible[row])]
  requests += [(float(t), 1, i, j) for t in np.arange(0, duration - .014, 1.) for i, j in aa]
  requests.sort()
  physical = []
  slot = 0
  for due, kind, i, j in requests:
    slot = max(slot, int(np.ceil(due * 250 - 1e-9)))
    t = slot / 250
    if t - due > (.05 if kind == 0 else 1.) or t + .014 > duration:
      continue
    physical.append([t, kind, i, j, due])
    slot += 1
  at = np.array([x[0] for x in physical])
  poses = motion.state(at)['p']
  a = np.array([poses[k] if x[1] == 0 else p[x[2]] for k, x in enumerate(physical)])
  b = np.array([p[x[2]] if x[1] == 0 else p[x[3]] for x in physical])
  valid = geometry.los(a, b)
  distances = np.linalg.norm(a - b, axis=1) + rng.normal(0, params['sigma_m'], len(a))
  events = []
  local_seq = 0
  # Each independent physical frame has explicit unique source identity.
  snapshots = {i: [] for i in range(18)}
  with (output / 'physical_uwb.csv').open('w') as f:
    w = csv.writer(f)
    w.writerow(['t', 'kind', 'a', 'b', 'due', 'valid', 'distance', 'source_seq'])
    for k, (t, kind, i, j, due) in enumerate(physical):
      source_seq = k + 1
      w.writerow([t, kind, 0 if kind == 0 else ids[i], ids[i] if kind == 0 else ids[j], due, int(valid[k]), distances[k] if valid[k] else '', source_seq])
      if not valid[k]:
        continue
      if kind == 0:
        local_seq += 1
        events.append((t + .013, 0, [(t, 0, int(ids[i]), source_seq, float(distances[k]), 0, 0, 1, 0.)]))
      else:
        snapshots[i].append((t, int(ids[i]), int(ids[j]), source_seq, float(distances[k])))
  emitted = set()
  delivery_rows = []
  report_times = np.arange(0, duration - .014, .2)
  txcar = motion.state(report_times)['p']
  rxcar = motion.state(report_times + .012)['p']
  txvis = geometry.los(np.broadcast_to(txcar[:, None, :], (len(report_times), 18, 3)), np.broadcast_to(p, (len(report_times), 18, 3))).reshape(-1, 18)
  rxvis = geometry.los(np.broadcast_to(rxcar[:, None, :], (len(report_times), 18, 3)), np.broadcast_to(p, (len(report_times), 18, 3))).reshape(-1, 18)
  cursors = {i: 0 for i in range(18)}
  latest = {i: {} for i in range(18)}
  for cycle, t in enumerate(report_times, 1):
    for i in range(18):
      source = snapshots[i]
      while cursors[i] < len(source) and source[cursors[i]][0] <= t:
        x = source[cursors[i]]
        latest[i][x[2]] = x
        cursors[i] += 1
      selected = [x for x in latest[i].values() if t - x[0] <= params['max_age_s']]
      # One physical pair sample per source frame. Each envelope has one shard;
      # do not combine different source frames into an impossible driver packet.
      for x in selected:
        identity = (x[1], x[2], x[3])
        action = 'unreachable'
        if txvis[cycle - 1, i] and rxvis[cycle - 1, i]:
          action = 'duplicate' if identity in emitted else 'delivered'
          if action == 'delivered':
            emitted.add(identity)
            events.append((t + .013, 1, [(*x, cycle, 0, 1, t - x[0])]))
        delivery_rows.append([t, int(ids[i]), x[2], x[3], t - x[0], 0, 1, action])
  events.sort(key=lambda x: x[0])
  # Finite-delay queue occupancy, including concurrent reports. Reject overflow.
  queue_events = []
  for published, kind, _ in events:
    queue_events.extend([(published - .013, 1), (published, -1)])
  occupancy = 0
  peak_queue = 0
  for _, delta in sorted(queue_events):
    occupancy += delta
    peak_queue = max(peak_queue, occupancy)
  if peak_queue > params['queue_limit']:
    raise RuntimeError('UWB delivery queue overflow')
  messages = []
  for seq, (published, kind, rows) in enumerate(events, 1):
    batch = UwbObservationBatch()
    batch.header.stamp = stamp(published)
    batch.driver_session_id = cfg['seed']
    batch.batch_seq = seq
    batch.receiver_id = 0
    batch.trigger_type = kind
    for source, a, b, source_seq, distance, cycle, shard, shards, age in rows:
      o = UwbObservation()
      o.kind = kind
      o.reporter_id = a
      o.node_a = a
      o.node_b = b
      o.source_session_id = a + 1
      o.source_frame_seq = source_seq
      o.source_frame_seq_valid = True
      o.tx_cycle_seq = cycle
      o.shard_index = shard
      o.shard_count = shards
      o.source_local_ms = int(round((source + 1) * 1000))
      o.source_system_ms = o.source_local_ms
      o.source_local_time_valid = True
      o.source_system_time_valid = True
      o.time_basis = 1 if kind == 0 else 2
      o.source_frame_stamp = stamp(source)
      o.received_stamp = stamp(published - .001)
      o.source_hold_age_ms = int(round(age * 1000)) if kind else 65535
      o.source_hold_age_valid = bool(kind)
      o.time_uncertainty_s = 0.
      o.distance_m = distance
      o.fp_rssi = float('nan')
      o.rx_rssi = float('nan')
      o.rssi_valid = False
      o.pair_measurement_freshness_verified = True
      batch.observations.append(o)
    messages.append((published, batch))
  with (output / 'delivery_uwb.csv').open('w') as f:
    w = csv.writer(f)
    w.writerow(['tx', 'reporter', 'peer', 'source_seq', 'hold_s', 'shard', 'shard_count', 'action'])
    w.writerows(delivery_rows)
  with (output / 'coverage.csv').open('w') as f:
    w = csv.writer(f)
    w.writerow(['t', 's', 'ta_count', 'reachable_ids'])
    for t, station, row in zip(times, motion.state(times)['s'], visible):
      w.writerow([t, station, int(row.sum()), ' '.join(map(str, ids[row]))])
  physical_counts = {}
  for k, (_, kind, i, j, _) in enumerate(physical):
    if not valid[k]:
      continue
    key = f"{0 if kind==0 else ids[i]}->{ids[i] if kind==0 else ids[j]}"
    physical_counts[key] = physical_counts.get(key, 0) + 1
  counts = {}
  for _, batch in messages:
    for o in batch.observations:
      key = f'{o.node_a}->{o.node_b}'
      counts[key] = counts.get(key, 0) + 1
  return messages, dict(physical_requests=len(requests),
                        physical_slots=len(physical),
                        valid_physical=int(valid.sum()),
                        budget_dropped=len(requests) - len(physical),
                        delivered_per_edge=counts,
                        zero_coverage_s=float(np.sum(visible.sum(axis=1) == 0) * .05),
                        coverage_switches=int(np.sum(np.any(visible[1:] != visible[:-1], axis=1))),
                        aa_geometry_pairs=[(int(ids[i]), int(ids[j])) for i, j in aa],
                        queue_model='offline scheduled fixed-delay events; maximum occupancy audited',
                        queue_limit=params['queue_limit'],
                        queue_peak=peak_queue,
                        physical_per_edge=physical_counts,
                        physical_rate_hz={
                            k: v / duration for k, v in physical_counts.items()
                        },
                        delivered_rate_hz={
                            k: v / duration for k, v in counts.items()
                        },
                        rate_denominator_s=duration)
