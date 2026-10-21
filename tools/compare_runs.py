#!/usr/bin/env python3
"""Audit identical raw inputs and compare only common ONLINE publication stamps."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def load(path):
    return json.loads(path.read_text())


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def keyed(records):
    sessions = list(dict.fromkeys(r["session"] for r in records))
    return {(sessions.index(r["session"]), round(float(r["stamp"]), 8)): r for r in records}


def compare(runs):
    audits = [load(r / "metadata/manifests/observation_audit.json") for r in runs]
    expected = audits[0]
    for run, audit in zip(runs, audits):
        if load(run / "metadata/run_manifest.json")["lifecycle"] != "completed":
            raise ValueError(f"incomplete run: {run}")
        if audit["violations"] or audit["counts"] != expected["counts"]:
            raise ValueError(f"input count/contract mismatch: {run}")
        if not audit.get("raw_sha256") or audit["raw_sha256"] != expected["raw_sha256"]:
            raise ValueError(f"input content mismatch: {run}")
    trajectories = [keyed(rows(r / "export/odometry/trajectory.csv")) for r in runs]
    common = set.intersection(*(set(t) for t in trajectories))
    if not common:
        raise ValueError("no common published measurement stamps")
    results = []
    for run, trajectory in zip(runs, trajectories):
        truth = keyed(rows(run / "export/simulation/truth.csv"))
        config = load(run / "metadata/config/phase1.json")
        fused = config["uwb"]["mode"] == "fuse"
        error, axial, attitude = [], [], []
        for key in sorted(common):
            p, q = trajectory[key], truth[key]
            d = [float(p[k]) - float(q[k]) for k in ["x", "y", "z"]]
            if not fused:
                d[2] += 1.2  # Declared phase-one odom datum; no fitted transform.
            error.append(sum(v * v for v in d))
            axial.append(abs(d[0]))
            attitude.append(math.degrees(2 * math.acos(min(1, abs(float(p["qw"]))))))
        quantile = lambda values: sorted(values)[int((len(values) - 1) * 0.95)]
        report = load(run / "export/analysis/report.json")
        results.append(
            {
                "run_id": run.name,
                "mode": config["uwb"]["mode"],
                "map_id": load(run / "metadata/config/map.json")["map_id"] if fused else None,
                "common_position_rmse_m": math.sqrt(statistics.mean(error)),
                "common_axial_p95_m": quantile(axial),
                "common_axial_max_m": max(axial),
                "common_attitude_p95_deg": quantile(attitude),
                "processing_p95_ms": report["processing_p95_ms"],
                "wall_end_to_end_p95_s": report["wall_end_to_end_p95_s"],
                "run_result": report["result"],
                "failures": report["failures"],
            }
        )
    return {
        "scope": "matched_online_stamps_snapshot_only",
        "input_content_verified": True,
        "raw_sha256": expected["raw_sha256"],
        "counts": expected["counts"],
        "common_samples": len(common),
        "common_first": min(common),
        "common_last": max(common),
        "alignment": "none; OFF uses declared z=-1.2 m odom datum, not known-map accuracy",
        "results": results,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", type=Path, nargs="+")
    args = parser.parse_args()
    if len(args.runs) < 2:
        parser.error("at least two runs required")
    runs = [p.resolve() for p in args.runs]
    report = compare(runs)
    output = runs[0] / "export/analysis/matched_comparison.json"
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(output)
    print(json.dumps(report, indent=2))
