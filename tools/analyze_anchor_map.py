#!/usr/bin/env python3
"""Offline A-only map diagnostics in the already declared CAL gauge; no fitting."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def distance_variance(covariance, means, i, j):
    direction = means[i] - means[j]
    distance = np.linalg.norm(direction)
    if distance <= 1e-9:
        raise ValueError('coincident map anchors')
    direction /= distance
    H = np.zeros(len(means)*3)
    H[3*i:3*i+3], H[3*j:3*j+3] = direction, -direction
    variance = float(H @ covariance @ H)
    if variance < -1e-8:
        raise ValueError('negative distance variance')
    return max(variance, 0.)


def topology(positions, counts):
    ids = sorted(positions); index = {value:i for i,value in enumerate(ids)}
    adjacency = {i:set() for i in ids}; matrix = np.zeros((len(counts), 3*len(ids)))
    for row, (a, b) in enumerate(sorted(counts)):
        if a == b or a not in index or b not in index:
            raise ValueError('invalid graph endpoint')
        adjacency[a].add(b); adjacency[b].add(a)
        delta = np.asarray(positions[a])-positions[b]
        if np.linalg.norm(delta) <= 1e-9:
            raise ValueError('coincident truth anchors')
        u = delta/np.linalg.norm(delta)
        matrix[row,3*index[a]:3*index[a]+3] = u
        matrix[row,3*index[b]:3*index[b]+3] = -u
    unseen = set(ids); components = []
    while unseen:
        pending = [min(unseen)]; component = set()
        while pending:
            node = pending.pop()
            if node in component: continue
            component.add(node); pending.extend(adjacency[node]-component)
        unseen -= component; components.append(sorted(component))
    # Geometric rank only, with all coordinates supplied by OFFLINE truth.
    # Neither these directions nor a physical edge absent from delivery enter LIUO.
    singular = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.sum(singular > 1e-9))
    centered=np.array([positions[i] for i in ids]); centered-=centered.mean(axis=0)
    rigid=np.zeros((3*len(ids),6))
    for i,(x,y,z) in enumerate(centered):
        rigid[3*i:3*i+3,:3]=np.eye(3)
        rigid[3*i:3*i+3,3:]=np.array([[0,z,-y],[-z,0,x],[y,-x,0]])
    gauge_rank=int(np.sum(np.linalg.svd(rigid,compute_uv=False)>1e-9))
    if np.linalg.norm(matrix@rigid)>1e-8: raise ValueError('distance Jacobian violates common rigid-motion nullspace')
    return dict(scope='truth-conditioned unit distance Jacobian; not estimated information',
                registered_nodes=ids, components=components, edges=[dict(first=a,second=b,samples=int(counts[a,b])) for a,b in sorted(counts)],
                rank_absolute_tolerance=1e-9, rigidity_rank=rank,
                jacobian_nullity=3*len(ids)-rank, common_rigid_gauge_rank=gauge_rank,
                nullity_after_common_rigid_gauge=3*len(ids)-rank-gauge_rank,
                singular_values=singular.tolist())


def truth_inputs(truth_run, truth_format):
    physical, delivered = Counter(), Counter()
    if truth_format == 'straight_csv':
        scenario_path = truth_run/'metadata/config/scenario.json'
        scenario = json.loads(scenario_path.read_text())
        truth_path = Path(__file__).resolve().parents[1]/'packages/liuo_tunnel/config/maps/oracle_map.json'
        fixture = json.loads(truth_path.read_text())
        if (scenario['scene'] != 'empty_tunnel_straight_v2' or
                scenario['anchor_layout'] != fixture['simulation_anchor_layout']):
            raise ValueError('straight truth fixture does not match frozen scenario')
        truth = [dict(id=a['id'], position=a['mean_m'], s=a['mean_m'][0]) for a in fixture['anchors']]
        positions = {a['id']: np.array(a['position']) for a in truth}
        with (truth_run/'export/simulation/physical_uwb.csv').open() as stream:
            for row in csv.DictReader(stream):
                if row['a_type'] != '1': continue
                pair = tuple(sorted((int(row['a']), int(row['b']))))
                expected = np.linalg.norm(positions[pair[0]]-positions[pair[1]])
                if abs(expected-float(row['true_range'])) > 1e-8:
                    raise ValueError('straight truth fixture disagrees with physical AA range')
                physical[pair] += 1
                if row['delivered'] == '1': delivered[pair] += 1
        provenance = dict(format=truth_format, scenario_sha256=hashlib.sha256(scenario_path.read_bytes()).hexdigest(),
                          scope='repository oracle fixture for offline analysis only; checked against frozen layout and physical AA truth')
    else:
        truth_path = truth_run/'export/simulation/anchors.json'
        truth = json.loads(truth_path.read_text())
        with (truth_run/'export/simulation/physical_uwb.csv').open() as stream:
            for row in csv.DictReader(stream):
                if row['kind']=='1' and row['valid']=='1': physical[tuple(sorted((int(row['a']),int(row['b']))))]+=1
        with (truth_run/'export/simulation/delivery_uwb.csv').open() as stream:
            for row in csv.DictReader(stream):
                if row['action']=='delivered' and int(row['reporter'])!=0:
                    delivered[tuple(sorted((int(row['reporter']),int(row['peer']))))]+=1
        provenance = dict(format=truth_format)
    provenance.update(path=str(truth_path), sha256=hashlib.sha256(truth_path.read_bytes()).hexdigest())
    return truth, physical, delivered, provenance


def analyze(run, truth_run, gauge_run, truth_format='road_npz'):
    map_path=run/'export/map/calibrated_map.json'
    model=json.loads(map_path.read_text())
    if model['schema_version']!='liuo_anchor_map_v3' or model['state_layout']!='A_only':
        raise ValueError('expected A-only v3 map')
    owner=Path(model.get('coordinate_gauge_source_run',model['source_run'])).resolve()
    if owner!=gauge_run.resolve(): raise ValueError('CAL gauge owner mismatch')
    gauge=json.loads((gauge_run/'export/analysis/road_fixed_start_diagnostic.json').read_text())
    if gauge['gauge']!='first_output_yaw_and_translation_only_no_route_fit':
        raise ValueError('unsupported gauge')
    truth, physical, delivered, truth_provenance = truth_inputs(truth_run, truth_format)
    true_positions={r['id']:np.array(r['position']) for r in truth}
    ids=[r['id'] for r in model['anchors']]; means=np.array([r['mean_m'] for r in model['anchors']])
    if not ids or len(ids)!=len(set(ids)) or not set(ids)<=set(true_positions): raise ValueError('invalid map IDs')
    C=np.array(model['joint_covariance']); dimension=3*len(ids)
    if C.shape!=(dimension,dimension) or not np.isfinite(C).all() or np.max(np.abs(C-C.T))>1e-8:
        raise ValueError('invalid map covariance')
    eigen,vectors=np.linalg.eigh(C)
    if eigen[0]<-1e-8: raise ValueError('indefinite map covariance')
    R=np.array(gauge['yaw_rotation_matrix']); shift=np.array(gauge['translation'])
    aligned=means@R.T+shift; Q=np.kron(np.eye(len(ids)),R); world_C=Q@C@Q.T
    errors=aligned-np.array([true_positions[i] for i in ids])
    anchors=[]
    for i,anchor in enumerate(ids):
        block=world_C[3*i:3*i+3,3*i:3*i+3]
        anchors.append(dict(id=anchor,world_mean_m=aligned[i].tolist(),error_xyz_m=errors[i].tolist(),
                            error_norm_m=float(np.linalg.norm(errors[i])),world_std_xyz_m=np.sqrt(np.maximum(0,np.diag(block))).tolist()))
    ordered=sorted(truth,key=lambda r:r['s']); adjacent={tuple(sorted((a['id'],b['id']))) for a,b in zip(ordered,ordered[1:])}
    pairs=[]
    for i,a in enumerate(ids):
        for j in range(i+1,len(ids)):
            b=ids[j]; distance=float(np.linalg.norm(means[i]-means[j])); actual=float(np.linalg.norm(true_positions[a]-true_positions[b]))
            pairs.append(dict(first=a,second=b,adjacent_in_truth=tuple(sorted((a,b))) in adjacent,
                              distance_m=distance,truth_distance_m=actual,error_m=distance-actual,
                              sigma_m=float(np.sqrt(distance_variance(C,means,i,j)))))
    fused=Counter(tuple(sorted((r['first'],r['second']))) for r in model['static_edge_ledger']['leaves'])
    result=dict(scope='offline fixed-CAL-gauge map diagnosis; not calibration consistency or localization acceptance',
                run=str(run),truth_run=str(truth_run),gauge_run=str(gauge_run),
                map_sha256=hashlib.sha256(map_path.read_bytes()).hexdigest(),truth_anchor_sha256=truth_provenance['sha256'],truth_provenance=truth_provenance,
                map_stamp=model['measurement_stamp'],anchor_rmse_m=float(np.sqrt(np.mean(np.sum(errors**2,axis=1)))),
                anchors=anchors,uninitialized_ids=sorted(set(true_positions)-set(ids)),pairs=pairs,
                joint_covariance_eigenvalues_m2=eigen.tolist(),
                largest_covariance_mode=dict(eigenvalue_m2=float(eigen[-1]),anchor_order=ids,
                                             world_direction=(Q@vectors[:,-1]).reshape(-1,3).tolist()),
                physical_full_input_graph=topology(true_positions,physical),
                delivered_full_input_graph=topology(true_positions,delivered),
                checkpoint_fused_graph=topology(true_positions,fused))
    (run/'export/analysis/anchor_map_diagnostic.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(11,8))
    all_truth=np.array([r['position'] for r in ordered])
    axes[0].plot(all_truth[:,0],all_truth[:,1],'.--',color='gray',label='offline truth anchors')
    axes[0].scatter(aligned[:,0],aligned[:,1],marker='x',label='estimated map in fixed CAL gauge')
    for i,anchor in enumerate(ids):
        target=true_positions[anchor]
        axes[0].plot([target[0],aligned[i,0]],[target[1],aligned[i,1]],color='tab:red',alpha=.5)
        axes[0].annotate(str(anchor),aligned[i,:2])
    axes[0].set(xlabel='world x (m)',ylabel='world y (m)',title='No map alignment fitted; truth used only offline')
    axes[0].legend(); axes[0].grid(True)
    if pairs:
        x=np.arange(len(pairs)); sigmas=np.array([p['sigma_m'] for p in pairs])
        if len(pairs) == 1:
            axes[1].errorbar(x, np.zeros(len(pairs)), yerr=3*sigmas, fmt='none', capsize=8,
                            label='conditional ±3σ with cross covariance')
        else:
            axes[1].fill_between(x,-3*sigmas,3*sigmas,alpha=.2,label='conditional ±3σ with cross covariance')
        axes[1].plot(x,[p['error_m'] for p in pairs],'o',label='distance error')
        axes[1].set_xticks(x,[f"{p['first']}–{p['second']}" for p in pairs],rotation=90)
        axes[1].legend()
    axes[1].set(xlabel='initialized anchor pair',ylabel='distance error (m)',title='Conditional covariance is not a consistency certification')
    axes[1].grid(True);fig.tight_layout()
    fig.savefig(run/'export/analysis/anchor_map_diagnostic.png',dpi=150);plt.close(fig)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--run',type=Path,required=True)
    p.add_argument('--truth-run',type=Path,required=True); p.add_argument('--gauge-run',type=Path)
    p.add_argument('--truth-format', choices=['road_npz', 'straight_csv'], default='road_npz')
    a=p.parse_args(); r=analyze(a.run,a.truth_run,a.gauge_run or a.run,a.truth_format)
    print(json.dumps(dict(anchor_rmse_m=r['anchor_rmse_m'],anchors=len(r['anchors']),
                         pairs=len(r['pairs']),physical_edges=len(r['physical_full_input_graph']['edges']),
                         delivered_edges=len(r['delivered_full_input_graph']['edges']),
                         fused_edges=len(r['checkpoint_fused_graph']['edges'])),indent=2))
