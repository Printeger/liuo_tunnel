#!/usr/bin/env python3
"""Phase-one online output assessment. Never optimize or align trajectories."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def quantile(values, p):
    if not values:
        return None
    return sorted(values)[min(len(values) - 1, int((len(values) - 1) * p))]


def evaluate(run):
    manifest = json.loads((run / "metadata/run_manifest.json").read_text())
    failures = []

    def check(ok, reason):
        if not ok:
            failures.append(reason)

    check(manifest["lifecycle"] == "completed", "run lifecycle not completed")
    check(manifest["source"]["git_worktree_clean"], "source not clean")
    audit = json.loads((run / "metadata/manifests/observation_audit.json").read_text())
    check(not audit["violations"], "raw message violations: " + str(audit["violations"]))
    check(all(v > 0 for v in audit["counts"].values()), "raw topics missing")
    check(bool(audit["estimator_subscriptions"]), "estimator graph not observed")
    check(
        audit["imu_static_acc_z_mean"] is not None
        and abs(audit["imu_static_acc_z_mean"] - 9.80665) < 0.05,
        "static specific force",
    )
    trajectory = rows(run / "export/odometry/trajectory.csv")
    truth = rows(run / "export/simulation/truth.csv")
    graph = rows(run / "export/odometry/graph.csv")
    icp = rows(run / "export/odometry/icp.csv")
    timing = rows(run / "profiling/odometry.csv")
    uwb = rows(run / "export/odometry/uwb.csv")
    check(len(trajectory) > 0, "no optimized trajectory")
    if trajectory and truth:
        check(
            abs(float(trajectory[-1]["stamp"]) - float(truth[-1]["stamp"])) <= 0.11,
            "online output did not drain to final sensor frame",
        )
    runtime = json.loads((run / "export/simulation/runtime.json").read_text())
    if "published_counts" in runtime:
        check(
            audit["counts"] == runtime["published_counts"],
            "raw observer lost published input: counts differ",
        )
    for name in ["odometry", "simulation"]:
        process = json.loads((run / f"metadata/processes/{name}.json").read_text())
        check(process["run_directory"] == str(run), "child allocated different run")
    identity = json.loads((run / "metadata/manifests/input_identity.json").read_text())
    check(bool(identity["shared_libraries"]), "shared library hashes missing")
    check(any(int(r["lidar_factors"]) >= 3 for r in graph), "multi-frame raw lidar graph absent")
    check(
        all(int(r["A"]) == 0 and int(r["fallback"]) == 0 for r in graph),
        "forbidden states/fallback",
    )
    check(all(int(r["X"]) <= 102 for r in graph), "fixed lag vehicle bound exceeded")
    check(all(r["reason"] == "RECORD_ONLY" for r in uwb), "raw UWB rejected")
    truth_sessions = list(dict.fromkeys(r["session"] for r in truth))
    est_sessions = list(dict.fromkeys(r["session"] for r in trajectory))
    check(len(truth_sessions) == len(est_sessions), "session count mismatch")
    truth_map = {
        (truth_sessions.index(r["session"]), round(float(r["stamp"]), 5)): r for r in truth
    }
    sections = {name: [] for name in ["entrance", "transition", "degenerate"]}
    attitude = []
    errors = []
    for r in trajectory:
        key = (est_sessions.index(r["session"]), round(float(r["stamp"]), 5))
        q = truth_map.get(key)
        if q is None:
            failures.append("unmatched online state time")
            continue
        # Fixed documented simulator/initial gauge offset only, never best-fit alignment.
        dx = float(r["x"]) - float(q["x"])
        dy = float(r["y"]) - float(q["y"])
        dz = float(r["z"]) - (float(q["z"]) - 1.2)
        e = math.sqrt(dx * dx + dy * dy + dz * dz)
        check(math.isfinite(e), "nonfinite trajectory")
        name = (
            "entrance"
            if float(q["x"]) <= 50
            else ("transition" if float(q["x"]) < 90 else "degenerate")
        )
        sections[name].append((e, abs(dx)))
        errors.append(e)
        attitude.append(math.degrees(2 * math.acos(min(1, abs(float(r["qw"]))))))
    metrics = {}
    for name, data in sections.items():
        metrics[name] = {
            "samples": len(data),
            "position_rmse_m": math.sqrt(statistics.mean(e * e for e, _ in data)) if data else None,
            "axial_p95_m": quantile([x for _, x in data], 0.95),
            "axial_max_m": max([x for _, x in data], default=None),
        }
    # Basic entrance tracking sanity is additional to process/data contracts.
    check(
        metrics["entrance"]["position_rmse_m"] is not None
        and metrics["entrance"]["position_rmse_m"] <= 1.0,
        "entrance tracking sanity RMSE > 1 m",
    )
    scenario = json.loads((run / "metadata/config/scenario.json").read_text())
    if scenario["duration_s"] >= 35 and scenario["reset_at_s"] < 0:
        check(bool(sections["degenerate"]), "no observation beyond entrance LiDAR visibility")
    if scenario["reset_at_s"] > 0:
        check(
            audit["clock_resets"] == 1 and len(audit["sessions"]) == 2 and len(est_sessions) == 2,
            "clock reset/session behavior",
        )
    elapsed = [float(r["total_ms"]) for r in timing]
    latency = [float(r["latency_s"]) for r in timing]
    backend = json.loads((run / "metadata/config/phase1.json").read_text())["odometry"].get(
        "backend", "cpu"
    )
    if backend == "gpu":
        process = json.loads((run / "metadata/processes/odometry.json").read_text())
        check(
            process.get("backend") == "gpu_RAW_MULTIFRAME_VGICP"
            and bool(process.get("gpu_device")),
            "GPU execution identity missing",
        )
    result = {
        "scope": "phase1_snapshot_" + backend,
        "result": "FAIL" if failures else "PASS",
        "failures": failures,
        "metrics": metrics,
        "attitude_p95_deg": quantile(attitude, 0.95),
        "processing_p95_ms": quantile(elapsed, 0.95),
        "sim_time_latency_p95_s": quantile(latency, 0.95),
        "max_vehicle_states": max((int(r["X"]) for r in graph), default=0),
        "simulation_runtime": runtime,
        "raw_counts": audit["counts"],
        "sessions": len(est_sessions),
        "gpu": ("FAIL" if failures else "PASS") if backend == "gpu" else "NOT_VALIDATED",
        "rolling_deskew": "NOT_VALIDATED",
        "accuracy_thresholds": "Phase-two/three fused-navigation thresholds not applied to phase-one LIO observation baseline.",
    }
    out = run / "export/analysis"
    out.mkdir(exist_ok=True)
    (out / "report.json").write_text(json.dumps(result, indent=2) + "\n")
    with (out / "metrics.csv").open("w") as f:
        writer = csv.DictWriter(
            f, fieldnames=["section", "samples", "position_rmse_m", "axial_p95_m", "axial_max_m"]
        )
        writer.writeheader()
        for section, m in metrics.items():
            writer.writerow({"section": section, **m})
    (out / "report.md").write_text(
        "# Phase-one runtime report\n\n"
        + json.dumps(result, indent=2)
        + "\n\nOnline published trajectory, fixed initial gauge translation z=-1.2 m. No trajectory fitting. LiDAR information is a mixed-unit pose-block diagnostic, not a certified uncertainty bound. Snapshot only.\n"
    )
    # Offline evaluator does not rewrite owner manifest.
    print(json.dumps(result, indent=2))
    return not failures


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("run", type=Path)
    args = p.parse_args()
    try:
        sys.exit(0 if evaluate(args.run.resolve()) else 1)
    except Exception as e:
        print(f"evaluation failed: {e}", file=sys.stderr)
        sys.exit(1)
