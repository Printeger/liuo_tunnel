"""Predeclared software messages for the public production ROS callback.

This fixture supplies no physical localization evidence. Valid distances are
synthetic constants; timing is an exact simulated clock, sigma=.10m is the
explicit synthetic model. Graph/representative evidence is evaluated separately.
"""
from copy import deepcopy
import json
from raw_identity import semantic_fields
from liuo_uwb_msgs.msg import UwbObservation, UwbObservationBatch


def cases():
    result = []
    seq = 0

    def observation(**changes):
        nonlocal seq
        seq += 1
        o = UwbObservation()
        o.reporter_id = o.node_a = 0
        o.node_b = 1
        o.source_session_id = 100
        o.source_frame_seq = seq
        o.source_frame_seq_valid = True
        o.shard_count = 1
        o.time_basis = 1
        o.source_frame_stamp.sec = 1
        o.received_stamp.sec = 1
        o.source_hold_age_ms = 65535
        o.distance_m = 10.
        o.time_uncertainty_s = 0.
        o.fp_rssi = o.rx_rssi = float('nan')
        o.pair_measurement_freshness_verified = True
        for key, value in changes.items():
            setattr(o, key, value)
        return o

    def add(name, observations, reasons, batch_reason='', **changes):
        b = UwbObservationBatch()
        b.driver_session_id = 9001
        b.batch_seq = len(result) + 1
        b.header.stamp.sec = 1
        b.observations = observations
        for key, value in changes.items():
            setattr(b, key, value)
        result.append((name, b, reasons, batch_reason))
        return b

    o = observation()
    peer = deepcopy(o); peer.node_b = 2
    add('same_frame_multiple_peers', [o, peer], ['ACCEPT', 'ACCEPT'])
    add('cross_cycle_retransmit', [deepcopy(o)], ['DUPLICATE'])
    result[-1][1].observations[0].tx_cycle_seq = 99
    add('same_value_new_frame', [observation()], ['ACCEPT'])
    aa = observation(kind=1, reporter_id=3, node_a=3, node_b=2, shard_count=3)
    aa_peer = deepcopy(aa); aa_peer.node_b = 1; aa_peer.shard_index = 1
    add('larger_id_reporter_first_shard', [aa], ['ACCEPT'], trigger_type=1)
    add('interleaved_local', [observation()], ['ACCEPT'])
    add('second_shard_missing_tail', [aa_peer], ['ACCEPT'], trigger_type=1)
    add('reverse_aa_not_independent', [observation(kind=1, reporter_id=2, node_a=2, node_b=3)], ['AA_OTHER_SOURCE'], trigger_type=1)
    add('empty', [], [])
    add('invalid_batch_receiver', [observation()], [], 'INVALID_BATCH', receiver_id=7)
    add('oversized_batch', [observation()] * 1025, [], 'BATCH_LIMIT')
    mutations = [
        ('unknown_endpoint', dict(node_b=255), 'INVALID_ENDPOINTS'),
        ('self_loop', dict(node_b=0), 'INVALID_ENDPOINTS'),
        ('reporter_mismatch', dict(reporter_id=1), 'INVALID_ENDPOINTS'),
        ('invalid_kind', dict(kind=2), 'INVALID_ENUM_OR_SHARD'),
        ('invalid_shard', dict(shard_count=0), 'INVALID_ENUM_OR_SHARD'),
        ('nan_range', dict(distance_m=float('nan')), 'INVALID_DISTANCE'),
        ('infinite_range', dict(distance_m=float('inf')), 'INVALID_DISTANCE'),
        ('zero_range', dict(distance_m=0.), 'INVALID_DISTANCE'),
        ('negative_range', dict(distance_m=-1.), 'INVALID_DISTANCE'),
        ('invalid_rssi', dict(rssi_valid=True), 'INVALID_RSSI'),
        ('invalid_hold_age', dict(source_hold_age_valid=True), 'INVALID_HOLD_AGE'),
        ('unknown_sequence', dict(source_frame_seq_valid=False), 'SOURCE_SEQUENCE_UNKNOWN'),
        ('zero_session', dict(source_session_id=0), 'INVALID_SOURCE_SESSION'),
        ('driver_unknown_time', dict(time_basis=0, time_uncertainty_s=float('nan'), pair_measurement_freshness_verified=False), 'MEASUREMENT_TIME_UNKNOWN'),
        ('receive_only', dict(time_basis=3), 'MEASUREMENT_TIME_UNKNOWN'),
        ('nan_uncertainty', dict(time_uncertainty_s=float('nan')), 'TIME_UNCERTAINTY_UNSUPPORTED'),
        ('excess_uncertainty', dict(time_uncertainty_s=10.), 'TIME_UNCERTAINTY_UNSUPPORTED'),
        ('unverified_freshness', dict(pair_measurement_freshness_verified=False), 'INDEPENDENCE_UNVERIFIED'),
        ('system_mapped', dict(time_basis=2), 'ACCEPT'),
    ]
    for name, changes, reason in mutations:
        add(name + '_mixed_with_legal_peer', [observation(**changes), observation(node_b=2)], [reason, 'ACCEPT'])
    for name, sec, ns in [('negative_time', -1, 0), ('zero_time', 0, 0), ('illegal_nanosecond', 1, 1000000000)]:
        bad = observation(); bad.source_frame_stamp.sec = sec; bad.source_frame_stamp.nanosec = ns
        add(name, [bad, observation(node_b=2)], ['INVALID_SOURCE_TIME', 'ACCEPT'])
    add('forward', [observation(source_frame_seq=100)], ['ACCEPT'])
    add('reordered', [observation(source_frame_seq=99)], ['ACCEPT'])
    add('retire_window', [observation(source_frame_seq=300)], ['ACCEPT'])
    add('retired_sequence_late', [observation(source_frame_seq=99)], ['IDENTITY_TOO_OLD'])
    add('source_restart_near_wrap', [observation(source_session_id=101, source_frame_seq=4294967295)], ['ACCEPT'])
    add('sequence_wrap', [observation(source_session_id=101, source_frame_seq=0)], ['ACCEPT'])
    add('old_source_late', [observation(source_session_id=100, source_frame_seq=301)], ['RETIRED_SOURCE_SESSION'])
    add('driver_restart', [observation(source_session_id=1, source_frame_seq=1)], ['ACCEPT'], driver_session_id=9002)
    add('old_driver_late', [observation(source_session_id=101, source_frame_seq=1)], ['RETIRED_DRIVER_SESSION'])
    for session in range(2, 66):
        add('source_retirement_capacity_' + str(session), [observation(source_session_id=session, source_frame_seq=1)], ['ACCEPT'], driver_session_id=9002)
    add('source_retirement_exhausted', [observation(source_session_id=66, source_frame_seq=1)], ['SOURCE_SESSION_CAPACITY'], driver_session_id=9002)
    add('active_source_after_exhaustion', [observation(source_session_id=65, source_frame_seq=2)], ['ACCEPT'], driver_session_id=9002)
    add('retired_source_stays_retired', [observation(source_session_id=1, source_frame_seq=2)], ['RETIRED_SOURCE_SESSION'], driver_session_id=9002)
    for driver in range(9003, 9066):
        add('driver_retirement_capacity_' + str(driver), [observation(source_session_id=1, source_frame_seq=1)],
            ['ACCEPT' if driver < 9065 else 'IDENTITY_CAPACITY_EXHAUSTED'], driver_session_id=driver)
    add('capacity_cannot_resurrect_old_driver', [observation(source_session_id=1, source_frame_seq=2)], ['IDENTITY_CAPACITY_EXHAUSTED'], driver_session_id=9002)
    return result


def prepare(run):
    plan = cases()
    document = dict(schema='liuo_public_callback_fixture_v1', source='software_synthetic',
                    scope='callback_contract; no localization accuracy claim',
                    expected_information='record_only: zero TA and AA factors; fused cases require separate graph audit',
                    cases=[dict(name=n, message=semantic_fields(b), reasons=r, batch_reason=br) for n,b,r,br in plan])
    (run / 'metadata/config/uwb_contract_expected.json').write_text(json.dumps(document, indent=2) + '\n')
    return plan
