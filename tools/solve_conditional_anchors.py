#!/usr/bin/env python3
"""D1 optimistic fixed-vehicle-trajectory batch; never loads truth Anchors."""
import argparse
import hashlib
import json
import csv
import time
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation, Slerp
from conditional_anchor_model import AnchorProblem


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observations', type=Path, required=True)
    parser.add_argument('--joint-run', type=Path, required=True)
    parser.add_argument('--truth-format', choices=['road_npz', 'straight_csv'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert args.output.resolve().is_relative_to('/home/dev/ws_iap') and not args.output.exists()
    saved = json.loads(args.observations.read_text())
    source = Path(saved['source_run'])
    identity = json.loads((args.joint_run/'metadata/manifests/input_identity.json').read_text())
    assert identity['raw_sha256'] == saved['raw_sha256'] and identity['seconds'] == saved['seconds']
    if args.truth_format == 'road_npz':
        path = source/'export/simulation/trajectory.npz'
        data = np.load(path)
        t, p, rotation = data['t']+1, data['p'], data['R']
    else:
        path = source/'export/simulation/truth.csv'
        rows = list(csv.DictReader(path.open()))
        t = np.array([float(r['stamp']) for r in rows])
        p = np.array([[float(r[k]) for k in ('x','y','z')] for r in rows])
        rotation = Rotation.from_quat([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows]).as_matrix()
    assert np.all(np.diff(t) > 0)
    slerp = Slerp(t, Rotation.from_matrix(rotation))
    def poses(stamps):
        stamps = np.atleast_1d(stamps)
        assert min(stamps) >= t[0] and max(stamps) <= t[-1]
        return np.column_stack([np.interp(stamps, t, p[:, i]) for i in range(3)]), slerp(stamps).as_matrix()
    ta_rows = list(saved['samples']['ta'].values())
    stamps = np.array([r['measurement']['acquisition_ns']/1e9 for r in ta_rows])
    points, rotations = poses(stamps)
    tags = points+np.einsum('nij,j->ni', rotations, saved['sensors']['T_BU_translation'])
    ta = [dict(anchor=r['measurement']['endpoints'][1], tag=tag,
               range=r['measurement']['range_m'], variance=r['measurement']['variance_m2']) for r, tag in zip(ta_rows, tags)]
    aa = [dict(first=r['measurement']['endpoints'][0], second=r['measurement']['endpoints'][1],
               range=r['measurement']['range_m'], variance=r['measurement']['variance_m2']) for r in saved['samples']['aa'].values()]
    ids = sorted({r['anchor'] for r in ta} | {r[k] for r in aa for k in ('first','second')})
    geometry_path = args.joint_run/'profiling/local_geometry.jsonl'
    candidate_path = args.joint_run/'profiling/candidate_inputs.jsonl'
    geometry = {(r['session'],r['pose']):r for r in map(json.loads, geometry_path.read_text().splitlines())}
    contexts = [json.loads(x) for x in candidate_path.read_text().splitlines()]
    patches, seed_sets, selected = [], {}, {}
    # Exactly one earliest actual candidate association per Anchor. Repeated
    # overlapping patches are not treated as independent installation evidence.
    for row in contexts:
        anchor = row['anchor']
        if anchor in selected or anchor not in ids or 'local' not in row:
            continue
        local = row['local']
        observed = geometry[(row['session'],local['pose'])]
        assert observed['stamp'] == local['stamp']
        pp, rr = poses([local['stamp']])
        patches.append(dict(anchor=anchor, p=pp[0], R=rr[0], offset=local['offset'], geometry=observed))
        seed_sets[anchor] = np.array([pp[0]+rr[0]@np.asarray(seed) for seed in row['seeds_local']])
        selected[anchor] = dict(candidate_stamp=row['stamp'], patch_stamp=local['stamp'], pose=local['pose'])
    problem = AnchorProblem(ids, ta, aa, patches)
    for anchor in ids:
        if anchor not in seed_sets:
            seeds = problem.range_seeds(anchor)
            if seeds is not None:
                seed_sets[anchor] = seeds
    # AA-only endpoints have no surveyed ordering/height. Use finite generic
    # sphere proposals around an already seeded neighbor; expose weak modes.
    for _ in ids:
        for row in aa:
            for a, b in ((row['first'],row['second']), (row['second'],row['first'])):
                if a not in seed_sets and b in seed_sets:
                    direction = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]], dtype=float)
                    seed_sets[a] = seed_sets[b][0]+row['range']*direction
    if set(seed_sets) != set(ids):
        raise ValueError('AA component without any TA support: report disconnected information before solving')
    rng = np.random.default_rng(20261027)
    starts = [np.concatenate([seed_sets[a][k % len(seed_sets[a])] for a in ids]) for k in range(8)]
    starts += [np.concatenate([seed_sets[a][rng.integers(len(seed_sets[a]))] for a in ids]) for _ in range(24)]
    args.output.mkdir(parents=True)
    plan = dict(role='D1_FIXED_TRUE_TRAJECTORY_CONDITIONAL_NOT_JOINT_POSTERIOR',
                anchor_truth_access='NONE; evaluation must be separate', raw_sha256=saved['raw_sha256'],
                measurement_set_sha256=saved['measurement_set_sha256'],
                vehicle_truth_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                geometry_sha256=hashlib.sha256(geometry_path.read_bytes()).hexdigest(),
                candidate_context_sha256=hashlib.sha256(candidate_path.read_bytes()).hexdigest(),
                joint_run=str(args.joint_run), ids=ids, ta=len(ta), aa=len(aa), selected_patches=selected,
                rng_seed=20261027, starts=[x.reshape(-1,3).tolist() for x in starts],
                solver='SLSQP analytic range/plane cost gradient; explicit finite support constraints',
                maxiter=300, ftol=1e-9, final_domain_tolerance_m=1e-7,
                comparable_cost_delta=2., significant_anchor_separation_m=.5,
                local_information='Gauss-Newton J.T J; full spectrum, not global uniqueness proof')
    (args.output/'plan_and_seeds.json').write_text(json.dumps(plan, indent=2)+'\n')
    results = []
    constraints = [dict(type='ineq', fun=problem.domain)] if patches else []
    for number, start in enumerate(starts):
        started = time.monotonic()
        fit = minimize(problem.objective, start, jac=True, method='SLSQP', constraints=constraints,
                       options=dict(maxiter=300, ftol=1e-9))
        residual, jacobian = problem.residual_jacobian(fit.x)
        information = jacobian.T@jacobian
        eigenvalues, eigenvectors = np.linalg.eigh(information)
        margins = problem.domain(fit.x)
        feasible = np.all(np.isfinite(fit.x)) and (not len(margins) or min(margins) >= -1e-7)
        result = dict(start=number, success=bool(fit.success), status=int(fit.status), message=str(fit.message),
                      iterations=int(fit.nit), wall_seconds=time.monotonic()-started, feasible=bool(feasible), cost=float(residual@residual/2),
                      gradient_norm=float(np.linalg.norm(jacobian.T@residual)),
                      minimum_domain_margin=float(min(margins)) if len(margins) else None,
                      anchors=fit.x.reshape(-1,3).tolist(), information_eigenvalues=eigenvalues.tolist(),
                      weakest_direction=eigenvectors[:,0].reshape(-1,3).tolist(),
                      ta_cost=float(residual[:len(ta)]@residual[:len(ta)]/2),
                      aa_cost=float(residual[len(ta):len(ta)+len(aa)]@residual[len(ta):len(ta)+len(aa)]/2),
                      installation_cost=float(residual[len(ta)+len(aa):]@residual[len(ta)+len(aa):]/2))
        result['ta_by_anchor'] = {}
        for index, anchor in enumerate(ids):
            values = residual[:len(ta)][problem.ta_index == index]
            result['ta_by_anchor'][anchor] = dict(count=len(values), whitened_cost=float(values@values/2),
                whitened_rmse=float(np.sqrt(np.mean(values**2))) if len(values) else None,
                information_eigenvalues_other_anchors_fixed=np.linalg.eigvalsh(information[3*index:3*index+3,3*index:3*index+3]).tolist())
        result['aa_by_edge'] = {}
        for first, second in sorted({(r['first'],r['second']) for r in aa}):
            mask = (problem.aa_first == ids.index(first)) & (problem.aa_second == ids.index(second))
            values = residual[len(ta):len(ta)+len(aa)][mask]
            result['aa_by_edge'][f'{first}-{second}'] = dict(count=len(values), whitened_cost=float(values@values/2),
                whitened_rmse=float(np.sqrt(np.mean(values**2))))
        results.append(result)
        (args.output/f'candidate_{number:02d}.json').write_text(json.dumps(result, indent=2)+'\n')
    valid = sorted([r for r in results if r['feasible'] and r['success']], key=lambda r:r['cost'])
    separated = []
    if valid:
        best = valid[0]
        for other in valid[1:]:
            separation = np.linalg.norm(np.asarray(other['anchors'])-best['anchors'], axis=1)
            if other['cost']-best['cost'] <= 2. and max(separation) >= .5:
                separated.append(dict(start=other['start'], delta_cost=other['cost']-best['cost'],
                                      per_anchor_separation_m=separation.tolist()))
    report = dict(plan=plan, candidates=results, converged_feasible_count=len(valid),
                  best_start=valid[0]['start'] if valid else None, comparable_separated_solutions=separated,
                  limitations=['Finite candidates cannot exclude other solutions.',
                               'Fixed true vehicle trajectory is optimistic, not a joint posterior.',
                               'Observed local geometry retains estimated IMU/motion error; no truth deskew.',
                               'SLSQP may evaluate a smooth plane extension while testing constraints; only feasible final candidates are physical.',
                               'Conditional full-prefix information differs from the actual D3 sliding-window information.'])
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(converged_feasible=len(valid), comparable_separated=len(separated))))


if __name__ == '__main__':
    main()
