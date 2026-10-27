#!/usr/bin/env python3
"""Summarize recorded resource/output evidence without turning extrema into acceptance."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path


def read(path):
    with path.open() as stream:
        rows=list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'empty evidence: {path}')
    return rows


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
    result=dict(scope='recorded extrema and output coverage only; no localization or load-test acceptance',
        states_samples=dict(states),output_samples=len(trajectory),
        first_output_stamp=float(trajectory[0]['stamp']),last_output_stamp=float(trajectory[-1]['stamp']),
        max_retained_vehicle_states=peak(graph,'X'),max_factors=peak(graph,'total_factors'),
        max_rss_mib=peak(resources,'rss_kib')/1024,max_device_mib=peak(resources,'device_used_bytes')/1024**2,
        device_memory_scope='entire CUDA device, not exclusive process allocation',
        max_imu_queue=peak(resources,'imu_queue'),max_cloud_queue=peak(resources,'cloud_queue'),
        max_uwb_queue=peak(resources,'uwb_queue'),maximum_optimization_ms=peak(profile,'optimize_ms'),
        max_retained_cloud_frames=peak(resources,'retained_cloud_frames'),max_keyframes=peak(resources,'keyframes'))
    (run/'export/analysis/full_route_resources.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
