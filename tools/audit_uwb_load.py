#!/usr/bin/env python3
"""Per-edge 60s/10s delivery, production decisions, resource and slow-probe audit."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
from audit_uwb_graph import audit as graph_audit


def audit(run):
    def rows(path):
        with (run/path).open() as f:return list(csv.DictReader(f))
    sent=rows('export/simulation/load_sent.csv'); decisions=rows('export/odometry/uwb_input_decisions.csv')
    slow=rows('profiling/slow_subscriber.csv'); resources=rows('profiling/resources.csv')
    consumed=rows('export/odometry/uwb.csv'); failures=[]
    def check(ok,why):
        if not ok:failures.append(why)
    published=[r for r in sent if r['sent']=='1']
    expected=Counter((r['batch_seq'],r['node_a'],r['node_b']) for r in published)
    received=Counter((r['batch_seq'],r['node_a'],r['node_b']) for r in decisions)
    check(expected==received,'production item delivery differs from declared publication')
    check(all(r['reason']=='QUEUED' for r in decisions),'unexpected input rejection in modeled load')
    by_batch={r['batch_seq']:float(r['elapsed_s']) for r in sent}
    per_edge={}
    for edge,total in [('0->1',3000),('0->2',3000),('0->3',3000),('2->1',300),('2->3',270)]:
        a,b=edge.split('->'); s=[r for r in published if r['node_a']==a and r['node_b']==b]
        d=[r for r in decisions if r['node_a']==a and r['node_b']==b]
        check(len(s)==total and len(d)==total,'per-edge total mismatch '+edge)
        windows=[]
        for i in range(6):
            inside=lambda t:i*10<=round(t,6)<(i+1)*10
            ns=sum(inside(float(r['elapsed_s'])) for r in s);nd=sum(inside(by_batch[r['batch_seq']]) for r in d)
            check(ns==total//6 and nd==total//6,'10s edge window mismatch '+edge+' '+str(i))
            windows.append(dict(begin_s=i*10,sent=ns,received=nd,report_hz=nd/10))
        per_edge[edge]=dict(sent=len(s),received=len(d),report_hz=len(d)/60,windows=windows)
    sent_batches=set(r['batch_seq'] for r in published); slow_batches=set(r['batch_seq'] for r in slow)
    check(slow_batches<=sent_batches,'slow probe saw foreign batch')
    check(len(slow_batches)<len(sent_batches),'slow probe did not exercise delivery loss')
    recovery_sent={b for b in sent_batches if 51<=by_batch[b]<60}
    recovery_received=slow_batches & recovery_sent
    check(recovery_sent==recovery_received,'slow probe did not recover in last nine seconds')
    check(max(int(r['identity_size']) for r in decisions)<=640,'identity state exceeds 128-frame per-source bound')
    check(max(int(r['retired_size']) for r in decisions)==0,'unexpected session retirement')
    graph=graph_audit(run);check(graph['result']=='PASS','graph identity/information audit failed')
    def stats(values):
        values=sorted(values)
        return dict(min=values[0],p50=values[len(values)//2],p95=values[int((len(values)-1)*.95)],max=values[-1]) if values else None
    owner = rows("profiling/owner_resources.csv")
    production_memory = [r for r in owner if r["process"] == "odometry"]
    publishing = {r["batch_seq"]: int(r["publish_wall_ns"]) for r in rows("export/simulation/load_delivery.csv")}
    check(len(production_memory) >= 60, "incomplete owner memory sampling")
    wall_latency_ms = [(int(r["callback_wall_ns"]) - publishing[r["batch_seq"]]) / 1e6 for r in decisions]
    result=dict(result='FAIL' if failures else 'PASS',failures=failures,scope='60 seconds software synthetic reporting load, not hardware TWR rate or hours-long stability',
                batches_sent=len(sent_batches),items_sent=len(published),unique_reports_sent=len(expected),
                batches_received=len(set(r['batch_seq'] for r in decisions)),items_received=len(decisions),
                input_reasons=dict(Counter(r['reason'] for r in decisions)),consumer_reasons=dict(Counter(r['reason'] for r in consumed)),
                fused=graph['unique_fused_reports'],per_edge=per_edge,missing_shard_items=sum(r['sent']=='0' for r in sent),
                publish_to_callback_wall_ms=stats(wall_latency_ms),
                owner_rss_kib={name:stats([int(r["rss_kib"]) for r in owner if r["process"]==name]) for name in sorted({r["process"] for r in owner})},
                owner_production_rss_change_kib=int(production_memory[-1]["rss_kib"])-int(production_memory[0]["rss_kib"]),
                owner_sample_count=len(production_memory),
                callback_processing_us=stats([float(r['processing_us']) for r in decisions]),
                received_to_callback_sim_s=stats([float(r['callback_stamp'])-float(r['received_stamp']) for r in decisions]),
                queue_peak=max(int(r['queue_size']) for r in decisions),identity_peak=max(int(r['identity_size']) for r in decisions),retired_peak=max(int(r['retired_size']) for r in decisions),
                rss_kib=stats([int(r['rss_kib']) for r in resources]),rss_change_kib=int(resources[-1]['rss_kib'])-int(resources[0]['rss_kib']),
                resource_sample_count=len(resources),resource_first_stamp=float(resources[0]['stamp']),resource_last_stamp=float(resources[-1]['stamp']),
                slow_probe=dict(received=len(slow_batches),missing=len(sent_batches-slow_batches),recovery_sent=len(recovery_sent),recovery_received=len(recovery_received),independent_process=True,production_delivery_loss=sum((expected-received).values())))
    (run/'export/analysis/uwb_load_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);r=audit(p.parse_args().run);print(json.dumps(r,indent=2));raise SystemExit(r['result']!='PASS')
