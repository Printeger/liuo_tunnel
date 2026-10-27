#!/usr/bin/env python3
"""Compare production AA identity/weight and software traffic on one frozen input."""
import argparse
from collections import defaultdict
import copy
import json
from pathlib import Path


def read(run, relative):
    return json.loads((run/relative).read_text())


def information(model):
    leaves=model['static_edge_ledger']['leaves']
    records={r['physical_id']:r for r in leaves}
    assert len(records)==len(leaves), 'duplicate physical posterior leaf'
    grouped=defaultdict(lambda: [0,0.,0.,0.])
    for key in sorted(records):
        r=records[key]; weight=1/r['variance_m2']; value=r['range_m']
        row=grouped[f"{r['first']}-{r['second']}"]
        row[0]+=1;row[1]+=weight;row[2]+=weight*value;row[3]+=weight*value*value
    return records,dict(grouped)


def compare(first, second):
    runs=[first,second]
    identities=[read(r,'metadata/manifests/input_identity.json') for r in runs]
    for field in ('raw_sha256','binary_sha256','shared_libraries','scan_model','seconds'):
        assert identities[0][field]==identities[1][field], field
    configs=[read(r,'metadata/config/phase1.json') for r in runs]
    strategies=[c['uwb']['static_edges'].get('delivery_strategy','same_time') for c in configs]
    assert set(strategies)=={'continuous_report','cache_sync'}
    comparable=copy.deepcopy(configs)
    for c in comparable:
        c['uwb']['static_edges'].pop('delivery_strategy',None)
    assert comparable[0]==comparable[1], 'non-delivery configuration changed'
    if configs[0]['uwb']['mode']=='reuse_anchor_map':
        assert read(first,'metadata/manifests/reuse_map_input.json')['sha256']==read(second,'metadata/manifests/reuse_map_input.json')['sha256']
    sent=[read(r,'metadata/manifests/published_input.json') for r in runs]
    assert sent[0]['counts']==sent[1]['counts'] and sent[0]['raw_sha256']==sent[1]['raw_sha256']
    deliveries=[read(r,'export/simulation/static_delivery.json') for r in runs]
    assert set(deliveries[0]['unique_physical_ids'])==set(deliveries[1]['unique_physical_ids'])
    assert deliveries[0]['delivery_opportunities']==deliveries[1]['delivery_opportunities']
    models=[read(r,'export/map/calibrated_map.json') for r in runs]
    assert models[0]['measurement_stamp']==models[1]['measurement_stamp'], 'posterior checkpoints differ'
    records0,summary0=information(models[0]);records1,summary1=information(models[1])
    assert records0==records1 and summary0==summary1, 'actual absorbed AA set/weight differs'
    assert records0, 'no actual AA posterior information'
    for run in runs:
        name='road_reuse_seam_audit.json' if configs[0]['uwb']['mode']=='reuse_anchor_map' else 'road_static_audit.json'
        audit=read(run,'export/analysis/'+name)
        assert audit['result'] in ('PASS','PASS_INPUT_GRAPH_AND_EXACT_POSTERIOR_LEDGER'), 'production callback/graph audit required'
    continuous=deliveries[strategies.index('continuous_report')]
    cached=deliveries[strategies.index('cache_sync')]
    assert continuous['repeated_records']>0 and cached['repeated_records']==0
    assert continuous['records']>cached['records'] and continuous['serialized_payload_bytes']>cached['serialized_payload_bytes']
    report=dict(result='PASS_SAME_ABSORBED_AA_INFORMATION_DIFFERENT_TRAFFIC',
        scope='actual posterior AA physical leaves and Gaussian scalar range sufficient statistics; not whole nonlinear trajectory equality or RF/MAC performance',
        runs=[str(r) for r in runs],strategies=strategies,raw_sha256=identities[0]['raw_sha256'],
        posterior_stamp=models[0]['measurement_stamp'],absorbed_leaves=len(records0),
        edge_statistic_fields=['count','weight','weighted_range','weighted_squared_range'],edge_statistics=summary0,
        continuous_records=continuous['records'],cache_records=cached['records'],
        continuous_payload_bytes=continuous['serialized_payload_bytes'],cache_payload_bytes=cached['serialized_payload_bytes'],
        payload_reduction_fraction=1-cached['serialized_payload_bytes']/continuous['serialized_payload_bytes'])
    (second/'export/analysis/static_pair.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    for ax,values,label in zip(axes,
                              ([continuous['records'],cached['records']],
                               [continuous['serialized_payload_bytes']/2**20,cached['serialized_payload_bytes']/2**20]),
                              ('Published static records','Serialized payload (MiB)')):
        bars=ax.bar(['Continuous report','Cache sync'],values,color=['tab:orange','tab:blue'])
        ax.bar_label(bars,labels=[f'{v:,.3f}' if label.endswith('(MiB)') else str(v) for v in values],padding=3)
        ax.set_ylabel(label);ax.set_ylim(0,max(values)*1.18);ax.grid(axis='y',alpha=.25)
    fig.suptitle(f"Same {len(records0)} absorbed physical leaves at {models[0]['measurement_stamp']:g} s\nSoftware traffic only; no RF/MAC or trajectory-equivalence claim")
    fig.tight_layout();fig.savefig(second/'export/analysis/static_pair.png',dpi=150);plt.close(fig)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first',type=Path,required=True);parser.add_argument('--second',type=Path,required=True)
    args=parser.parse_args(); print(json.dumps(compare(args.first,args.second),indent=2))
