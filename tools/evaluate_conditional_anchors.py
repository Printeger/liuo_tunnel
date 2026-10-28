#!/usr/bin/env python3
"""Post-solve D1 evaluation. Truth Anchors never select or alter a solution."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--anchor-reference', type=Path, required=True)
    parser.add_argument('--probes', type=Path, required=True)
    parser.add_argument('--output-label', default='truth_evaluation')
    args = parser.parse_args()
    assert args.output_label.replace('_','').isalnum()
    output_path = args.result/(args.output_label+'.json')
    assert not output_path.exists(), 'preserve previous diagnostic evaluations'
    original = (args.result/'report.json').read_bytes()
    report = json.loads(original)
    reference = json.loads(args.anchor_reference.read_text())
    assert reference['input_raw_sha256'] == report['plan']['raw_sha256']
    truth_map = {r['id']:r['mean_m'] for r in reference['anchors']}
    ids = report['plan']['ids']
    truth = np.asarray([truth_map[key] for key in ids])
    probes = {r['start']:r for r in json.loads((args.probes/'feasible_probes.json').read_text())}
    candidates = report['candidates']
    best_index = report['best_start']  # selected previously by physical cost, not by truth
    assert best_index is not None
    best = candidates[best_index]
    rows = []
    for row in candidates:
        errors = np.asarray(row['anchors'])-truth
        probe = probes[row['start']]
        rows.append(dict(start=row['start'], original_cost=row['cost'], cost_delta=row['cost']-best['cost'],
                         solver_success=row['success'], feasible=row['feasible'],
                         demonstrated_feasible_descent=any(p['minimum_cost_change'] < -1e-6 for p in probe['probes']),
                         minimum_probe_cost_change=min(p['minimum_cost_change'] for p in probe['probes']),
                         errors_world_m=errors.tolist(), per_anchor_error_m=np.linalg.norm(errors,axis=1).tolist()))
    output = dict(role='D1_POST_SOLVE_TRUTH_EVALUATION_ONLY_NOT_DEPLOYABLE',
                  report_sha256=hashlib.sha256(original).hexdigest(), anchor_reference_sha256=hashlib.sha256(args.anchor_reference.read_bytes()).hexdigest(),
                  probe_sha256=hashlib.sha256((args.probes/'feasible_probes.json').read_bytes()).hexdigest(),
                  ids=ids, cost_selected_best_start=best_index, evaluations=rows,
                  comparable_separated_solutions=report['comparable_separated_solutions'],
                  limitation='No truth-based branch selection, alignment, initial seed, solver feedback or deployable map.')
    output_path.write_text(json.dumps(output,indent=2)+'\n')
    near = [r for r in candidates if r['feasible'] and r['success'] and r['cost'] <= best['cost']+2.]
    fig, axes = plt.subplots(2,4,figsize=(15,7),squeeze=False)
    for i, anchor in enumerate(ids):
        ax = axes.flat[i]
        points = np.asarray([r['anchors'][i] for r in near])
        ax.scatter(points[:,1],points[:,2],c=[r['cost']-best['cost'] for r in near],vmin=0,vmax=2,cmap='viridis',label='cost delta <= 2')
        ax.scatter(truth[i,1],truth[i,2],marker='x',color='red',label='truth: evaluation only')
        ax.set(title=f'A{anchor} world y-z',xlabel='y [m]',ylabel='z [m]')
        ax.grid(alpha=.2)
    for ax in list(axes.flat)[len(ids):]:
        ax.set_visible(False)
    axes.flat[0].legend(fontsize=7)
    fig.suptitle('D1 fixed true vehicle trajectory: finite near-cost candidates (not a uniqueness proof)')
    fig.tight_layout()
    fig.savefig(args.result/(args.output_label+'_candidates.png'),dpi=160)
    plt.close(fig)
    assert (args.result/'report.json').read_bytes() == original
    print(json.dumps(dict(best_start=best_index,best_errors=rows[best_index]['per_anchor_error_m'],
                          near_cost_candidates=len(near),near_with_proved_descent=sum(rows[r['start']]['demonstrated_feasible_descent'] for r in near))))


if __name__ == '__main__':
    main()
