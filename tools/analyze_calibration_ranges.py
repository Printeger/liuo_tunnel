#!/usr/bin/env python3
"""Offline optimistic diagnostic only. Truth never enters an estimator/map export."""
import argparse
import csv
import itertools
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--until', type=float, default=61)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    truth = list(csv.DictReader(open(args.source / 'export/simulation/truth.csv')))
    ranges = [r for r in csv.DictReader(open(args.source / 'export/simulation/uwb.csv')) if float(r['stamp']) <= args.until]
    t = np.array([float(r['stamp']) for r in truth])
    positions = np.array([[float(r[k]) for k in ('x','y','z')] for r in truth])
    rotations = Rotation.from_quat([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in truth])
    config = json.loads((args.source / 'metadata/config/phase1.json').read_text())
    tag = positions + rotations.apply(np.array(config['sensors']['T_BU_translation']))
    ids = sorted({int(r['b']) for r in ranges if r['a_type'] == '0'})
    ta = [r for r in ranges if r['a_type'] == '0']
    times = np.array([float(r['stamp']) for r in ta])
    observed_tag = np.array([np.interp(times,t,tag[:,j]) for j in range(3)]).T
    targets = np.array([ids.index(int(r['b'])) for r in ta])
    measured = np.array([float(r['range']) for r in ta])
    sigmas = np.sqrt([float(r['variance']) for r in ta])
    aa = [r for r in ranges if r['a_type'] == '1' and int(r['a']) in ids and int(r['b']) in ids]
    ia = np.array([ids.index(int(r['a'])) for r in aa],dtype=int)
    ib = np.array([ids.index(int(r['b'])) for r in aa],dtype=int)
    ar = np.array([float(r['range']) for r in aa])
    av = np.sqrt([float(r['variance']) for r in aa])
    root = Path(__file__).resolve().parents[1]
    installation = json.loads((root/'packages/liuo_tunnel/config/installation/straight_simulated.json').read_text())
    order = [next(i for i,r in enumerate(installation['records']) if r['anchor_id']==id) for id in ids]
    offsets = np.array([installation['records'][i]['signed_offset_m'] for i in order])
    C = np.array(installation['joint_covariance_m2'])[np.ix_(order,order)]
    whiten = np.linalg.inv(np.linalg.cholesky(C))
    axial = []
    for j in range(len(ids)):
        samples = observed_tag[targets==j]
        b = measured[targets==j]**2 - np.sum(samples**2,axis=1)
        axial.append(np.linalg.lstsq(np.column_stack([-2*samples[:,0],np.ones(len(samples))]),b,rcond=None)[0][0])
    results=[]
    for use_aa in (False,True):
        for side in (-1,1):
            def residual(v):
                anchors=v.reshape(-1,3)
                components=[(np.linalg.norm(anchors[targets]-observed_tag,axis=1)-measured)/sigmas,whiten@(side*anchors[:,1]-offsets)]
                if use_aa: components.append((np.linalg.norm(anchors[ia]-anchors[ib],axis=1)-ar)/av)
                return np.concatenate(components)
            for heights in itertools.product((-3.,6.),repeat=len(ids)):
                initial=np.column_stack([axial,side*offsets,heights]).ravel()
                fit=least_squares(residual,initial,max_nfev=300,ftol=1e-10,xtol=1e-10,gtol=1e-10)
                eigen=np.linalg.eigvalsh(fit.jac.T@fit.jac)
                covariance=np.linalg.inv(fit.jac.T@fit.jac) if eigen[0]>1e-12 else None
                results.append(dict(aa=use_aa,side=side,initial_heights=heights,cost=float(fit.cost),success=bool(fit.success),anchors=fit.x.reshape(-1,3).tolist(),information_eigenvalues=eigen.tolist(),conditional_std_xyz=None if covariance is None else np.sqrt(np.diag(covariance)).reshape(-1,3).tolist()))
    output=dict(scope='OFFLINE_ONLY_PERFECT_TRAJECTORY_AND_HORIZONTAL_REFERENCE_NOT_ESTIMATOR',source=str(args.source),until=args.until,ids=ids,ta_count=len(ta),aa_count=len(aa),results=results)
    args.output.write_text(json.dumps(output,indent=2)+'\n')
    for use_aa in (False,True):
        for side in (-1,1):
            best=min((r for r in results if r['aa']==use_aa and r['side']==side),key=lambda r:r['cost'])
            print('AA',use_aa,'side',side,'cost',best['cost'],'anchors',np.round(best['anchors'],3).tolist(),'std',np.round(best['conditional_std_xyz'],3).tolist())


if __name__=='__main__':
    main()
