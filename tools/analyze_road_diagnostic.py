#!/usr/bin/env python3
"""Offline-only fixed-start-gauge diagnostics; never used by an estimator."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def route_error_components(t, error, truth_t, truth_v):
    """Offline horizontal travel tangent/left normal/world vertical, no fit."""
    velocity = np.column_stack([np.interp(t, truth_t, truth_v[:, j]) for j in range(2)])
    moving = np.linalg.norm(truth_v[:, :2], axis=1) > 1e-6
    if not np.any(moving):
        raise ValueError('no moving truth sample defines a route direction')
    times, directions = truth_t[moving], truth_v[moving, :2]
    stopped = np.linalg.norm(velocity, axis=1) <= 1e-6
    for index in np.flatnonzero(stopped):
        right = min(int(np.searchsorted(times, t[index])), len(times)-1)
        left = max(0, right-1)
        nearest = left if abs(times[left]-t[index]) <= abs(times[right]-t[index]) else right
        velocity[index] = directions[nearest]
    tangent = velocity / np.linalg.norm(velocity, axis=1)[:, None]
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    return np.column_stack([np.sum(error[:, :2]*tangent, axis=1),
                            np.sum(error[:, :2]*normal, axis=1), error[:, 2]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--truth-run', type=Path, required=True)
    parser.add_argument('--truth-format', choices=['road_npz', 'straight_csv'], default='road_npz')
    parser.add_argument('--map-reference-run', type=Path, help='CAL run supplying fixed offline gauge; never align REUSE to its own first truth pose')
    parser.add_argument('--oracle-world', action='store_true', help='D2 only: retain the fixed Anchor world frame without truth alignment')
    parser.add_argument('--focus-start', type=float, help='inclusive ROS stamp of predeclared diagnostic interval')
    parser.add_argument('--focus-end', type=float, help='inclusive ROS stamp of predeclared diagnostic interval')
    args = parser.parse_args()
    if (args.focus_start is None) != (args.focus_end is None) or (args.focus_start is not None and args.focus_start >= args.focus_end):
        raise ValueError('both increasing focus boundaries are required')
    if args.oracle_world:
        role = json.loads((args.run/'metadata/manifests/conditional_diagnostic.json').read_text())
        if args.map_reference_run or role['role'] != 'D2_ORACLE_ANCHORS_NOT_DEPLOYABLE':
            raise ValueError('oracle world gauge requires the independent D2 run')
    rows = list(csv.DictReader((args.run / 'export/odometry/trajectory.csv').open()))
    if not rows:
        raise ValueError('no trajectory')
    all_count = len(rows)
    gauge_reference = None
    if args.map_reference_run:
        prior = json.loads((args.run / 'metadata/config/reuse_map.json').read_text())
        if Path(prior.get('coordinate_gauge_source_run', prior['source_run'])).resolve() != args.map_reference_run.resolve():
            raise ValueError('reference CAL run does not own loaded map')
        rows = [r for r in rows if r['map_id'] == prior['map_id']]
        if not rows: raise ValueError('map frame was never attached; no fixed-map output to evaluate')
        gauge_reference = json.loads((args.map_reference_run / 'export/analysis/road_fixed_start_diagnostic.json').read_text())
        if gauge_reference['gauge'] != 'first_output_yaw_and_translation_only_no_route_fit':
            raise ValueError('unsupported CAL gauge')
    t = np.array([float(r['stamp']) - 1 for r in rows])
    if np.any(np.diff(t) <= 0):
        raise ValueError('diagnostic requires one increasing-time session')
    if args.truth_format == 'road_npz':
        truth = np.load(args.truth_run / 'export/simulation/trajectory.npz')
    else:
        with (args.truth_run / 'export/simulation/truth.csv').open() as stream:
            saved_truth = list(csv.DictReader(stream))
        truth = dict(t=np.array([float(r['stamp'])-1 for r in saved_truth]),
                     p=np.array([[float(r[k]) for k in ['x','y','z']] for r in saved_truth]),
                     v=np.array([[float(r[k]) for k in ['vx','vy','vz']] for r in saved_truth]),
                     R=Rotation.from_quat([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in saved_truth]).as_matrix())
    if np.any(np.diff(truth['t']) <= 0):
        raise ValueError('truth requires one increasing-time session')
    if t[0] < truth['t'][0] or t[-1] > truth['t'][-1]:
        raise ValueError('trajectory outside truth time coverage')
    positions = np.array([[float(r[k]) for k in ['x', 'y', 'z']] for r in rows])
    rotations = Rotation.from_quat([[float(r[k]) for k in ['qx', 'qy', 'qz', 'qw']] for r in rows])
    target_positions = np.column_stack([np.interp(t, truth['t'], truth['p'][:, j]) for j in range(3)])
    target_rotations = Slerp(truth['t'], Rotation.from_matrix(truth['R']))(t)
    # Declare only yaw and translation gauge from the FIRST output. Do not fit
    # the route or remove roll/pitch error; no later truth influences transform.
    yaw = lambda r: np.arctan2(r.as_matrix()[1, 0], r.as_matrix()[0, 0])
    if args.oracle_world:
        transform = Rotation.identity()
        translation = np.zeros(3)
    elif gauge_reference:
        transform = Rotation.from_matrix(gauge_reference['yaw_rotation_matrix'])
        translation = np.array(gauge_reference['translation'])
    else:
        transform = Rotation.from_euler('z', yaw(target_rotations[0]) - yaw(rotations[0]))
        translation = target_positions[0] - transform.apply(positions[0])
    aligned_positions = transform.apply(positions) + translation
    aligned_rotations = transform * rotations
    error = aligned_positions - target_positions
    route_error = route_error_components(t, error, truth['t'], truth['v'])
    attitude = (target_rotations.inv() * aligned_rotations).magnitude()
    result = dict(scope='offline diagnostic only, not stage acceptance', truth_run=str(args.truth_run), truth_format=args.truth_format,
        run=str(args.run), gauge='oracle_world_no_truth_alignment' if args.oracle_world else ('inherited_CAL_gauge_no_REUSE_alignment' if gauge_reference else 'first_output_yaw_and_translation_only_no_route_fit'),
        reference_run=str(args.map_reference_run) if gauge_reference else str(args.run),
        total_output_samples=all_count, pre_map_output_samples=all_count-len(rows),
        reference_stamp=gauge_reference['reference_stamp'] if gauge_reference else float(t[0]+1), yaw_rotation_matrix=transform.as_matrix().tolist(), translation=translation.tolist(),
        samples=len(t), first_stamp=float(t[0]+1), last_stamp=float(t[-1]+1),
        covered_seconds=float(t[-1]-t[0]), position_rmse_m=float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
        xyz_rmse_m=np.sqrt(np.mean(error**2, axis=0)).tolist(), maximum_position_error_m=float(np.max(np.linalg.norm(error, axis=1))),
        attitude_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(attitude**2)))), rpe={})
    result['route_frame'] = dict(
        definition='horizontal truth travel tangent, left horizontal normal, world vertical; no alignment or estimator input',
        stationary_direction='nearest moving truth sample in time; ties choose earlier; speed threshold 1e-6 m/s',
        along_cross_height_rmse_m=np.sqrt(np.mean(route_error**2, axis=0)).tolist())
    # Relative pose increments at exact interpolated 1 and 10 second intervals.
    for interval in [1., 10.]:
        mask = t + interval <= t[-1]
        if not np.any(mask):
            continue
        begin, end = t[mask], t[mask] + interval
        end_est = np.column_stack([np.interp(end, t, aligned_positions[:, j]) for j in range(3)])
        end_true = np.column_stack([np.interp(end, truth['t'], truth['p'][:, j]) for j in range(3)])
        rot_est = Slerp(t, aligned_rotations)(end)
        rot_true = Slerp(truth['t'], Rotation.from_matrix(truth['R']))(end)
        delta_est = aligned_rotations[mask].inv().apply(end_est - aligned_positions[mask])
        delta_true = target_rotations[mask].inv().apply(end_true - target_positions[mask])
        delta_R_est = aligned_rotations[mask].inv() * rot_est
        delta_R_true = target_rotations[mask].inv() * rot_true
        result['rpe'][str(interval)] = dict(samples=len(begin),
            translation_rmse_m=float(np.sqrt(np.mean(np.sum((delta_est-delta_true)**2, axis=1)))),
            rotation_rmse_deg=float(np.rad2deg(np.sqrt(np.mean((delta_R_true.inv()*delta_R_est).magnitude()**2)))))
    if 'vx' in rows[0]:
        velocities = transform.apply([[float(r[k]) for k in ['vx','vy','vz']] for r in rows])
        target_v = np.column_stack([np.interp(t, truth['t'], truth['v'][:,j]) for j in range(3)])
        result['velocity_rmse_mps'] = float(np.sqrt(np.mean(np.sum((velocities-target_v)**2, axis=1))))
    if args.focus_start is not None:
        focus = (t+1 >= args.focus_start-1e-8) & (t+1 <= args.focus_end+1e-8)
        if not np.any(focus): raise ValueError('no output in fixed diagnostic focus interval')
        result['focus'] = dict(requested_ros_interval=[args.focus_start, args.focus_end],
            boundary='inclusive; preceding output fixes the same gauge, no focus realignment',
            samples=int(focus.sum()), first_stamp=float(t[focus][0]+1), last_stamp=float(t[focus][-1]+1),
            position_rmse_m=float(np.sqrt(np.mean(np.sum(error[focus]**2, axis=1)))),
            xyz_rmse_m=np.sqrt(np.mean(error[focus]**2, axis=0)).tolist(),
            along_cross_height_rmse_m=np.sqrt(np.mean(route_error[focus]**2, axis=0)).tolist(),
            maximum_position_error_m=float(np.max(np.linalg.norm(error[focus], axis=1))),
            attitude_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(attitude[focus]**2)))))
        if 'vx' in rows[0]:
            result['focus']['velocity_rmse_mps'] = float(np.sqrt(np.mean(np.sum((velocities[focus]-target_v[focus])**2, axis=1))))
    output = args.run / 'export/analysis'
    output.mkdir(exist_ok=True, parents=True)
    prefix = 'conditional_oracle_vehicle' if args.oracle_world else ('road_fixed_map_diagnostic' if gauge_reference else 'road_fixed_start_diagnostic')
    if args.focus_start is not None: prefix += f'_focus_{args.focus_start:g}_{args.focus_end:g}'
    (output / (prefix+'.json')).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plot_selection = focus if args.focus_start is not None else np.ones(len(t), dtype=bool)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, label in enumerate(['world x error (m)', 'world y error (m)', 'world z error (m)']):
        axes[i].plot(t[plot_selection]+1, error[plot_selection,i]); axes[i].set_ylabel(label); axes[i].grid(True)
    axes[-1].set_xlabel('ROS stamp (s)')
    fig.suptitle('ORACLE Anchor diagnostic — world frame, no truth alignment' if args.oracle_world else ('Inherited CAL gauge — no REUSE alignment' if gauge_reference else 'Offline fixed-start yaw/translation diagnostic — no route fitting'))
    fig.tight_layout(); fig.savefig(output / (prefix+'.png'), dpi=150); plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, label in enumerate(['horizontal along-route error (m)', 'left cross-route error (m)', 'world height error (m)']):
        axes[i].plot(t[plot_selection]+1, route_error[plot_selection, i]); axes[i].set_ylabel(label); axes[i].grid(True)
    axes[-1].set_xlabel('ROS stamp (s)')
    fig.suptitle('Same fixed gauge; truth route frame used only for offline error components')
    fig.tight_layout(); fig.savefig(output / (prefix+'_route.png'), dpi=150); plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
