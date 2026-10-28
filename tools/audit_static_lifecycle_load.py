#!/usr/bin/env python3
"""Independent A6 delivery/factor audit; separate producer, consumer, observer."""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
from rclpy.serialization import deserialize_message
from liuo_tunnel_msgs.msg import StaticAnchorRecord


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    run=parser.parse_args().run
    read=lambda path:list(csv.DictReader((run/path).open()))
    plan=json.loads((run/'metadata/manifests/static_load_plan.json').read_text())
    document=json.loads((run/'metadata/config/map.json').read_text())
    leaves={row['physical_id']:row for row in document['static_edge_ledger']['leaves']}
    assert len(leaves)==plan['configured_capacity']==plan['inherited_leaves']
    old=leaves[plan['physical_id']]
    sent_rows=read('export/simulation/static_load_sent.csv')
    sent={round(float(row['stamp'])*1e9):row for row in sent_rows if row['event']=='published' or row['event'].startswith('negative_')}
    assert len(sent)==sum(row['event']=='published' or row['event'].startswith('negative_') for row in sent_rows)
    events=[json.loads(line) for line in (run/'export/odometry/static_edges.jsonl').read_text().splitlines()]
    inputs=[row for row in events if row['event']=='input']
    received={}
    for row in inputs:
        message=deserialize_message(bytes.fromhex(row['cdr_hex']),StaticAnchorRecord)
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        assert stamp in sent and stamp not in received
        received[stamp]=row
        expected=sent[stamp]['event']
        assert len(message.leaves) in (1,2)
        leaf=message.leaves[0]
        if expected=='published':
            assert row['reason']=='QUEUED'
            allowed=set(plan.get('physical_ids',[plan['physical_id']]))
            assert len(message.leaves)==(2 if len(allowed)==2 and message.revision==2 else 1)
            assert set(message.covered_blocks)=={item.physical_id for item in message.leaves}
            for item in message.leaves:
                assert item.physical_id in allowed
                original=leaves[item.physical_id]
                assert item.range_m==original['range_m'] and item.variance_m2==original['variance_m2']
            assert sorted((message.first,message.second))==[old['first'],old['second']]
            assert message.deployment_epoch==document['static_edge_ledger']['deployment_epoch']
        elif expected=='negative_wrong_epoch':
            assert row['reason']=='STATIC_RECORD_SOURCE_MISMATCH'
        else:
            assert row['reason']=='QUEUED'  # rejected transactionally by estimator owner, below
    assert set(received)==set(sent),'production consumer lost static deliveries'
    rejection=collections.Counter(row['reason'] for row in events if row['event']=='rejected_before_graph')
    negative_packets=plan.get('negative_packets',{})
    assert {row['event'] for row in sent.values() if row['event'].startswith('negative_')} == {'negative_'+event for event in negative_packets.values()}
    expected_rejection=collections.Counter({'STATIC_EDGE_IDENTITY_CONFLICT':1,'STATIC_EDGE_BLOCK_CAPACITY':1}) if negative_packets else collections.Counter()
    assert rejection==expected_rejection
    updates=[row for row in events if row['event']=='graph_update']
    assert updates
    assert all(all(row[key]==0 for key in ('fused_leaves','pending','explicit_factors','added_factors','removed_factors')) for row in updates)
    graph=read('export/odometry/graph.csv')
    assert graph and all(int(row['joint_map_priors'])==1 and int(row['map_priors'])==1 for row in graph)
    assert all(int(row['installation_observation_factors'])==0 and int(row['aa_accumulated_measurements'])==0 and int(row['aa_summary_factors'])==0 for row in graph)
    trajectory=read('export/odometry/trajectory.csv')
    valid_stamps=[stamp/1e9 for stamp,row in sent.items() if row['event']=='published']
    overlap=max(valid_stamps)-max(min(valid_stamps),float(trajectory[0]['stamp']))
    assert overlap>=60,'less than 60 seconds of active estimation under retransmission'
    observer=read('profiling/slow_static_subscriber.csv')
    observed={round(float(row['stamp'])*1e9):row for row in observer}
    assert len(observed)==len(observer) and set(observed)<=set(sent)
    late={stamp for stamp in sent if stamp>=55*10**9}
    assert late<=set(observed),'slow observer did not recover by declared late window'
    resources=read('profiling/owner_resources.csv')
    resource_summary={}
    for process in sorted({row['process'] for row in resources}):
        values=[float(row['rss_kib'])/1024 for row in resources if row['process']==process]
        resource_summary[process]=dict(max_rss_mib=max(values),first_rss_mib=values[0],last_rss_mib=values[-1])
    report=json.loads((run/'export/analysis/report.json').read_text())
    result=dict(result='PASS_STATIC_LIFECYCLE_LOAD' if report['result']=='PASS' else 'INVARIANTS_OBSERVED_BUT_RUN_REPORT_FAILED',
                original_run_report=report['result'],original_failures=report['failures'],
                scope='declared absorbed-AA production delivery/ledger/resource workload only; no localization or RF acceptance',
                sent=len(sent),production_received=len(received),production_lost=0,
                observer_received=len(observed),observer_lost=len(sent)-len(observed),observer_late_lost=len(late-set(observed)),
                active_estimation_load_seconds=overlap,pressure_stamp_range=[min(valid_stamps),max(valid_stamps)],
                publisher_recreations=sum(r['event']=='publisher_recreated' for r in sent_rows),
                not_matched_opportunities=sum(r['event']=='not_matched' for r in sent_rows),
                callback_queue_leaves_max=max(row['queue_leaves'] for row in inputs),callback_queue_limit=2048,
                dds_depths=dict(production=100,observer=37),
                observer_max_callback_delay_s=max((int(row['receive_wall_ns'])-int(sent[stamp]['publish_wall_ns']))/1e9 for stamp,row in observed.items()),
                inherited_physical_leaves=len(leaves),new_fused_leaves=0,new_static_factors=0,joint_map_priors=1,
                controlled_rejections=dict(rejection),resource_by_process=resource_summary,
                map_sha256=hashlib.sha256((run/'metadata/config/map.json').read_bytes()).hexdigest(),
                source_prefix_verified=report['frozen_input_verified'],
                limitations=['DDS occupancy is not directly exposed; configured depths, callback queue and observed delivery delay/loss are separate.',
                             'Finite workload and bounded data structures do not certify arbitrary-duration leak freedom.',
                             'Publisher recreation is not a physical deployment change; changed epoch must be explicitly rejected.'])
    assert result['source_prefix_verified']
    (run/'export/analysis/static_lifecycle_load_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
