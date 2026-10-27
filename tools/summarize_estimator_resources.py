#!/usr/bin/env python3
"""Summarize recorded resource/output evidence without turning extrema into acceptance."""
import argparse
from collections import Counter
import csv
import json
import math
import statistics
from pathlib import Path


def read(path):
    with path.open() as stream:
        rows=list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'empty evidence: {path}')
    return rows


def temporal_summary(rows, key, scale=1.):
    """Describe each session separately; trends are observations, not leak tests."""
    sessions = {}
    for row in rows:
        t, value = float(row['stamp']), float(row[key]) * scale
        if not math.isfinite(t) or not math.isfinite(value):
            raise ValueError(f'nonfinite temporal resource: {key}')
        sessions.setdefault(row.get('session', '1'), []).append((t, value))
    result = {}
    for session, samples in sessions.items():
        if any(b[0] < a[0] for a, b in zip(samples, samples[1:])):
            raise ValueError(f'nonmonotonic resource stamps in session {session}')
        begin, end = samples[0][0], samples[-1][0]
        mean_t = statistics.mean(t - begin for t, _ in samples)
        mean_v = statistics.mean(v for _, v in samples)
        denominator = sum((t - begin - mean_t)**2 for t, _ in samples)
        slope = (sum((t - begin - mean_t) * (v - mean_v) for t, v in samples) / denominator
                 if denominator else None)
        windows = []
        for i in range(4):
            lo, hi = begin + (end-begin)*i/4, begin + (end-begin)*(i+1)/4
            selected = [v for t, v in samples if lo <= t and (t < hi or (i == 3 and t <= hi))]
            windows.append(dict(begin_stamp=lo, end_stamp=hi, samples=len(selected),
                                median=statistics.median(selected) if selected else None,
                                maximum=max(selected) if selected else None))
        result[session] = dict(samples=len(samples), first_stamp=begin, last_stamp=end,
                               first=samples[0][1], last=samples[-1][1],
                               change=samples[-1][1]-samples[0][1],
                               least_squares_change_per_second=slope, time_quartiles=windows)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args=parser.parse_args(); run=args.run
    graph=read(run/'export/odometry/graph.csv')
    trajectory=read(run/'export/odometry/trajectory.csv')
    resources=read(run/'profiling/resources.csv')
    profile=read(run/'profiling/odometry.csv')
    peak=lambda rows,key:max(float(r[key]) for r in rows)
    states=Counter(r['state'] for r in trajectory)
    result=dict(scope='recorded extrema, temporal trends and output coverage only; no localization, leak or load-test acceptance',
        states_samples=dict(states),output_samples=len(trajectory),
        first_output_stamp=float(trajectory[0]['stamp']),last_output_stamp=float(trajectory[-1]['stamp']),
        max_retained_vehicle_states=peak(graph,'X'),max_factors=peak(graph,'total_factors'),
        max_rss_mib=peak(resources,'rss_kib')/1024,max_device_mib=peak(resources,'device_used_bytes')/1024**2,
        device_memory_scope='entire CUDA device, not exclusive process allocation',
        max_imu_queue=peak(resources,'imu_queue'),max_cloud_queue=peak(resources,'cloud_queue'),
        max_uwb_queue=peak(resources,'uwb_queue'),maximum_optimization_ms=peak(profile,'optimize_ms'),
        max_retained_cloud_frames=peak(resources,'retained_cloud_frames'),max_keyframes=peak(resources,'keyframes'))
    result['temporal_trends'] = {
        'rss_mib': temporal_summary(resources, 'rss_kib', 1/1024),
        'device_mib': temporal_summary(resources, 'device_used_bytes', 1/1024**2),
        'vehicle_states': temporal_summary(graph, 'X'),
        'total_factors': temporal_summary(graph, 'total_factors'),
        'anchors': temporal_summary(graph, 'A'),
        'optimization_ms': temporal_summary(profile, 'optimize_ms'),
    }
    (run/'export/analysis/full_route_resources.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
