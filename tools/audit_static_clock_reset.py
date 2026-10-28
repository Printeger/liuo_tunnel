"""Independent static identity/factor recovery checks for the fixed 35+35s seek.

Invoked by audit_uwb_boundary --static-channel; its original TA fault assertions
remain required. No vehicle truth or accuracy criterion is used here.
"""
import collections
import hashlib
import json


def check_static_recovery(run, graph, raw):
    from rclpy.serialization import deserialize_message
    from liuo_tunnel_msgs.msg import StaticAnchorRecord
    document=json.loads((run/'metadata/config/map.json').read_text())
    old={x['physical_id'] for x in document['static_edge_ledger']['leaves']}
    known={int(x['id']) for x in document['anchors']}
    events=[json.loads(x) for x in (run/'export/odometry/static_edges.jsonl').read_text().splitlines()]
    physical={}
    counts=collections.Counter()
    eligible=set()
    pairs=set()
    for event in events:
        if event['event']!='input':
            continue
        message=deserialize_message(bytes.fromhex(event['cdr_hex']),StaticAnchorRecord)
        assert len(message.leaves)==1 and message.covered_blocks==[message.leaves[0].physical_id]
        leaf=message.leaves[0]
        pair=tuple(sorted((message.first,message.second)))
        value=(pair,leaf.physical_source,leaf.range_m,leaf.variance_m2,
               leaf.acquisition_stamp.sec,leaf.acquisition_stamp.nanosec,
               leaf.acquisition_stamp_valid,leaf.time_basis,message.network,
               message.deployment_epoch,message.source_identity)
        key=leaf.physical_id
        assert key not in old, 'independent REUSE fixture overlaps CAL identities'
        assert key not in physical or physical[key]==value, 'physical content changed across seek'
        physical[key]=value
        counts[key]+=1
        ready=set(pair)<=known
        assert event['reason']==('QUEUED' if ready else 'UNKNOWN_ANCHOR')
        if ready:
            eligible.add(key)
            pairs.add(pair)
    assert physical and set(counts.values())=={2}, 'each original AA must appear in both deliveries'
    delivery=json.loads((run/'export/simulation/static_delivery.json').read_text())
    assert set(delivery['unique_physical_ids'])==set(physical)
    updates=[x for x in events if x['event']=='graph_update']
    breaks=[i for i in range(1,len(updates)) if updates[i]['stamp']<updates[i-1]['stamp']]
    assert len(breaks)==1, 'expected one actual graph clock rollback'
    sessions=[updates[:breaks[0]],updates[breaks[0]:]]
    for rows in sessions:
        assert rows[-1]['fused_leaves']==len(eligible)>0
        assert rows[-1]['explicit_factors']==len(pairs)>0
        assert rows[-1]['pending']==0
        assert all(x['fused_leaves']<=len(eligible) and x['explicit_factors']<=len(pairs) for x in rows)
    # Reset discards the graph and indices, retains the physical history, then
    # stages its eligible leaves once; replaying them must not replace again.
    assert sessions[1][0]['fused_leaves']==len(eligible)
    assert sessions[1][0]['added_factors']==len(pairs)
    assert sessions[1][0]['removed_factors']==0
    assert all(x['added_factors']==0 and x['removed_factors']==0 for x in sessions[1][1:])
    assert {int(x['session']) for x in graph}=={1,2}
    for row in graph:
        assert int(row['joint_map_priors'])==int(row['map_priors'])==1
        assert int(row['installation_observation_factors'])==0
        assert int(row['aa_accumulated_measurements'])<=len(eligible)
    assert not any(x['reason']=='ACCEPT_AA' for x in raw)
    for session in (1,2):
        last=[x for x in graph if int(x['session'])==session][-1]
        assert int(last['aa_accumulated_measurements'])==len(eligible)
        assert int(last['aa_summary_factors'])==len(pairs)
    return dict(result='PASS_STATIC_GRAPH_CLOCK_RECOVERY',
        inherited_leaves=len(old),unique_delivered_leaves=len(physical),deliveries_per_leaf=2,
        eligible_leaves_each_graph=len(eligible),explicit_edges_each_graph=len(pairs),
        graph_sessions=2,prior_per_graph=1,second_graph_duplicate_factor_updates=0,
        physical_content_sha256=hashlib.sha256(json.dumps(physical,sort_keys=True).encode()).hexdigest(),
        limitation='Fixed synthetic-source clock seek and driver-session change; not hardware restart or localization certification')
