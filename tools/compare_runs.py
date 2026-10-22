#!/usr/bin/env python3
"""Audit identical raw inputs and compare only common ONLINE publication stamps."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
from evaluation_frames import world_estimates, run_world_estimates


def load(path):
    return json.loads(path.read_text())


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def keyed(records):
    sessions = list(dict.fromkeys(r["session"] for r in records))
    return {(sessions.index(r["session"]), round(float(r["stamp"]), 8)): r for r in records}


def estimator_settings(config):
    odometry = dict(config["odometry"])
    for key, default in [
        ("max_keyframes", 15),
        ("cloud_tangent_variance", 1.0),
        ("normalize_lidar_by_point_count", False),
        ("project_weak_lidar_translation", False),
        ("relinearize_threshold", 0.05),
        ("use_dogleg", False),
        ("factorization", "CHOLESKY"),
    ]:
        odometry.setdefault(key, default)
    return {"scan_model": config["scan_model"], "sensors": config["sensors"], "odometry": odometry}


def compare(runs, strict_estimator_settings=False):
    audits = [load(r / "metadata/manifests/observation_audit.json") for r in runs]
    expected = audits[0]
    for run, audit in zip(runs, audits):
        if load(run / "metadata/run_manifest.json")["lifecycle"] != "completed":
            raise ValueError(f"incomplete run: {run}")
        if audit["violations"] or audit["counts"] != expected["counts"]:
            raise ValueError(f"input count/contract mismatch: {run}")
        if not audit.get("raw_sha256") or audit["raw_sha256"] != expected["raw_sha256"]:
            raise ValueError(f"input content mismatch: {run}")
    configs = [load(r / "metadata/config/phase1.json") for r in runs]
    settings = [estimator_settings(c) for c in configs]
    binaries = [
        load(r / "metadata/manifests/input_identity.json")["binaries"]["odometry"]["sha256"]
        for r in runs
    ]
    settings_equal = all(s == settings[0] for s in settings)
    binaries_equal = all(b == binaries[0] for b in binaries)
    fused_settings = [dict(c["uwb"]) for c in configs if c["uwb"]["mode"] in ("fuse", "reuse_map", "self_calibrate")]
    fused_equal = not fused_settings or all(c == fused_settings[0] for c in fused_settings)
    if strict_estimator_settings and not (settings_equal and binaries_equal and fused_equal):
        raise ValueError(
            "strict paired comparison requires identical LIO settings, binary and fused UWB settings"
        )
    trajectories = [keyed(rows(r / "export/odometry/trajectory.csv")) for r in runs]
    common = set.intersection(*(set(t) for t in trajectories))
    if not common:
        raise ValueError("no common published measurement stamps")
    results = []
    for run, trajectory in zip(runs, trajectories):
        truth_rows = rows(run / "export/simulation/truth.csv")
        truth = keyed(truth_rows)
        config = load(run / "metadata/config/phase1.json")
        fused = config["uwb"]["mode"] in ("fuse", "reuse_map", "self_calibrate")
        trajectory = keyed(run_world_estimates(run, list(trajectory.values()), truth_rows, config["uwb"]["mode"]))
        error, axial, attitude, component_errors, velocity_errors = [], [], [], [], []
        for key in sorted(common):
            p, q = trajectory[key], truth[key]
            d = [float(p[k]) - float(q[k]) for k in ["x", "y", "z"]]
            error.append(sum(v * v for v in d))
            component_errors.append(d)
            velocity_errors.append([float(p[k])-float(q[k]) for k in ["vx", "vy", "vz"]])
            axial.append(abs(d[0]))
            dot = sum(
                float(p[k]) * float(q.get(k, 1 if k == "qw" else 0))
                for k in ["qx", "qy", "qz", "qw"]
            )
            attitude.append(math.degrees(2 * math.acos(min(1, abs(dot)))))
        quantile = lambda values: sorted(values)[int((len(values) - 1) * 0.95)]
        report = load(run / "export/analysis/report.json")
        results.append(
            {
                "run_id": run.name,
                "mode": config["uwb"]["mode"],
                "map_id": load(run / ("export/map/calibrated_map.json" if config["uwb"]["mode"] == "self_calibrate" else "metadata/config/map.json"))["map_id"] if fused else None,
                "common_position_rmse_m": math.sqrt(statistics.mean(error)),
                "common_axis_rmse_m": [math.sqrt(statistics.mean(e[i]**2 for e in component_errors)) for i in range(3)],
                "common_velocity_axis_rmse_mps": [math.sqrt(statistics.mean(e[i]**2 for e in velocity_errors)) for i in range(3)],
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
        "auditor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "input_content_verified": True,
        "lio_settings_matched": settings_equal,
        "odometry_binary_matched": binaries_equal,
        "fused_uwb_settings_matched": fused_equal,
        "controlled_estimator_comparison": settings_equal and binaries_equal and fused_equal,
        "settings_scope": "Raw input equality alone does not establish a mode-only comparison; estimator settings and binary are audited separately.",
        "raw_sha256": expected["raw_sha256"],
        "counts": expected["counts"],
        "common_samples": len(common),
        "common_first": min(common),
        "common_last": max(common),
        "alignment": "Known-map ON in world; calibrated modes use fixed first-calibration datum; OFF uses its initial datum; no fitted transforms",
        "results": results,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--strict-estimator-settings", action="store_true")
    args = parser.parse_args()
    if len(args.runs) < 2:
        parser.error("at least two runs required")
    runs = [p.resolve() for p in args.runs]
    report = compare(runs, args.strict_estimator_settings)
    output = runs[0] / "export/analysis/matched_comparison.json"
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(output)
    print(json.dumps(report, indent=2))
