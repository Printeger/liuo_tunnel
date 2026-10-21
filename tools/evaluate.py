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


def raw_lidar_graph_present(graph, config):
    # Phase one retains its original >=3 newly added edges check. Phase two
    # connects the current cloud to recent states plus retained keyframes.
    # Two distinct targets already connect three poses with raw factors; unlike
    # a registered-pose prior, the historical pose keys remain in those factors.
    phase2 = config.get("schema_version") == "liuo_phase2_v1"
    required_edges = 2 if phase2 else 3
    multiple = any(
        int(r["X"]) >= required_edges + 1 and int(r["lidar_factors"]) >= required_edges
        for r in graph
    )
    continuous = all(
        int(r["X"]) <= 1 or (int(r["lidar_factors"]) >= 1 and int(r["imu_factors"]) >= 1)
        for r in graph
    )
    return multiple and continuous


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
    fault_profile = (run / "metadata/config/delivery.json").exists()
    runtime = json.loads((run / "export/simulation/runtime.json").read_text())
    check(
        audit.get("duplicate_uwb", []) == runtime.get("duplicate_uwb", []),
        "unexpected duplicate UWB identities",
    )
    if "delivered_sha256" in runtime:
        check(
            audit.get("raw_sha256") == runtime["delivered_sha256"],
            "received input differs from published replay stream",
        )
    frozen_path = run / "metadata/manifests/frozen_input.json"
    if frozen_path.exists():
        frozen = json.loads(frozen_path.read_text())
        for kind in ["imu", "cloud", "clock", "uwb"]:
            if kind == "uwb" and fault_profile:
                check("delivered_sha256" in runtime, "missing fault delivery hash audit")
                continue
            check(
                audit["counts"][kind] == frozen["counts"][kind],
                "frozen input counts differ: " + kind,
            )
            check(
                audit["raw_sha256"][kind] == frozen["raw_sha256"][kind],
                "frozen input content differs: " + kind,
            )
    config = json.loads((run / "metadata/config/phase1.json").read_text())
    fused = config["uwb"]["mode"] == "fuse"
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
    check(raw_lidar_graph_present(graph, config), "multi-frame raw lidar graph absent")
    check(
        all(int(r["fallback"]) == 0 and (fused or int(r["A"]) == 0) for r in graph),
        "forbidden states/fallback",
    )
    check(all(int(r["X"]) <= 102 for r in graph), "fixed lag vehicle bound exceeded")
    if fused:
        check(any(r["reason"] == "ACCEPT_TA" for r in uwb), "no raw TA factors")
        map_config = json.loads((run / "metadata/config/map.json").read_text())
        expected_anchors = len(map_config["anchors"]) if map_config["mode"] == "surveyed_map" else 0
        check(all(int(r["A"]) == expected_anchors for r in graph), "static anchors not retained")
        check(
            all(int(r["map_priors"]) == int(expected_anchors > 0) for r in graph),
            "map prior count changed",
        )
        if graph and "aa_summary_factors" in graph[0]:
            pair_bound = expected_anchors * (expected_anchors - 1) // 2
            check(
                all(int(r["aa_summary_factors"]) <= pair_bound for r in graph),
                "AA summary pair bound exceeded",
            )
            if audit["clock_resets"] == 0:
                check(
                    int(graph[-1]["aa_accumulated_measurements"])
                    == sum(r["reason"] == "ACCEPT_AA" for r in uwb),
                    "AA accumulated/raw identity count mismatch",
                )
            cfg = json.loads((run / "metadata/config/phase1.json").read_text())
            odometry = cfg["odometry"]
            factor_bound = (
                math.ceil(250 * (odometry["lag_s"] + 0.2))
                + 102 * (odometry["connection_window"] + odometry.get("max_keyframes", 15) + 3)
                + pair_bound
                + 16
            )
            check(
                all(int(r["total_factors"]) <= factor_bound for r in graph),
                "factor history exceeds configured fixed-window/static-pair bound",
            )
    else:
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
        dz = float(r["z"]) - (float(q["z"]) - (0 if fused else 1.2))
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
    wall_latency = []
    wall_clock_path = run / "export/simulation/wall_clock.csv"
    if wall_clock_path.exists():
        deadlines = {
            (int(r["session"]), round(float(r["stamp"]), 9)): int(r["scheduled_wall_ns"])
            for r in rows(wall_clock_path)
        }
        for r in trajectory:
            key = (int(r["session"]), round(float(r["stamp"]), 9))
            if key not in deadlines or "publish_wall_ns" not in r:
                check(False, "missing wall publication/deadline association")
                continue
            value = (int(r["publish_wall_ns"]) - deadlines[key]) * 1e-9
            check(value >= 0, "wall publication before measurement deadline")
            wall_latency.append(value)
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
    if fused:
        check(
            math.sqrt(statistics.mean(e * e for e in errors)) <= 0.30,
            "known-map position RMSE >0.30m",
        )
        axial = [x for data in sections.values() for _, x in data]
        check(quantile(axial, 0.95) <= 0.50, "known-map axial P95 >0.50m")
        check(max(axial) <= 1.0, "known-map axial max >1.0m")
        check(quantile(attitude, 0.95) <= 2.0, "known-map attitude P95 >2deg")
        check(quantile(elapsed, 0.95) < 100, "processing P95 >=100ms")
        check(quantile(latency, 0.95) < 0.25, "simulation-time latency P95 >=250ms")
        check(runtime["real_time_factor"] >= 0.99, "actual input rate below 1x tolerance")
        check(len(wall_latency) == len(trajectory), "wall latency evidence incomplete")
        check(
            bool(wall_latency) and quantile(wall_latency, 0.95) < 0.25,
            "wall end-to-end P95 >=250ms or unavailable",
        )
        # Campaign acceptance also requires long-run paired trials, outages and all seeds.
        check(False, "phase2 long-run/paired-campaign acceptance not yet implemented")
    delivery_summary = None
    if fault_profile and fused:
        from delivery_analysis import analyze

        delivery_summary = analyze(
            json.loads((run / "metadata/config/delivery.json").read_text()),
            rows(run / "export/simulation/delivery.csv"),
            uwb,
            trajectory,
            graph,
            rows(run / "export/simulation/wall_clock.csv"),
            runtime,
        )
        for reason in delivery_summary["failures"]:
            check(False, "delivery audit: " + reason)
    covariance_summary = None
    if trajectory and "wx" in trajectory[0]:
        wire_path = run / "export/capture/odometry.csv"
        check(wire_path.exists(), "missing independent odometry wire audit")
        if wire_path.exists():
            wire = rows(wire_path)
            covariance_rows = rows(run / "export/odometry/covariance.csv")
            check(
                len(wire) == len(trajectory) == len(covariance_rows),
                "published/observed odometry counts differ",
            )
            for published, observed, covariance in zip(trajectory, wire, covariance_rows):
                check(
                    abs(float(published["stamp"]) - float(observed["stamp"])) < 1e-8,
                    "wire odometry stamp mismatch",
                )
                check(
                    0 < float(published["twist_interval_s"]) <= 0.15,
                    "invalid angular velocity interval",
                )
                for field in ["wx", "wy", "wz"]:
                    check(
                        abs(float(published[field]) - float(observed[field])) < 1e-10,
                        "wire angular velocity mismatch",
                    )
                for i in range(6):
                    for j in range(6):
                        field = f"twist_{i}_{j}"
                        check(
                            abs(float(covariance[field]) - float(observed[field])) < 1e-10,
                            "wire twist covariance mismatch",
                        )
    covariance_path = run / "export/odometry/covariance.csv"
    if covariance_path.exists():
        from covariance_analysis import summarize

        covariance_summary = summarize(run, trajectory, truth, rows(covariance_path), fused)
    result = {
        "scope": (
            "phase2_iteration_snapshot_"
            if config.get("schema_version") == "liuo_phase2_v1"
            else "phase1_snapshot_"
        )
        + backend,
        "estimator_mode": config["uwb"]["mode"],
        "accuracy_acceptance_scope": (
            "known_map_nominal_gates"
            if fused
            else "entrance_sanity_only; record_only_ablation_is_not_known_map_accuracy_acceptance"
        ),
        "result": "FAIL" if failures else "PASS",
        "failures": failures,
        "metrics": metrics,
        "position_rmse_m": math.sqrt(statistics.mean(e * e for e in errors)),
        "attitude_p95_deg": quantile(attitude, 0.95),
        "processing_p95_ms": quantile(elapsed, 0.95),
        "sim_time_latency_p95_s": quantile(latency, 0.95),
        "wall_end_to_end_p95_s": quantile(wall_latency, 0.95),
        "actual_input_hz": {k: v / runtime["wall_s"] for k, v in audit["counts"].items()},
        "delivery_profile": fault_profile,
        "delivered_input_verified": audit.get("raw_sha256") == runtime.get("delivered_sha256")
        and audit["counts"] == runtime.get("published_counts"),
        "frozen_input_verified": frozen_path.exists()
        and audit.get("raw_sha256") == frozen["raw_sha256"]
        and audit["counts"] == frozen["counts"],
        "max_vehicle_states": max((int(r["X"]) for r in graph), default=0),
        "simulation_runtime": {
            k: v for k, v in runtime.items() if k not in ["duplicate_uwb", "delivered_sha256"]
        },
        "delivery_audit": delivery_summary,
        "raw_counts": audit["counts"],
        "covariance_diagnostics": covariance_summary,
        "sessions": len(est_sessions),
        "gpu": ("FAIL" if failures else "PASS") if backend == "gpu" else "NOT_VALIDATED",
        "rolling_deskew": "NOT_VALIDATED",
        "accuracy_thresholds": (
            "Phase-two known-map thresholds applied without trajectory alignment."
            if fused
            else "Fused-navigation thresholds not applied to phase-one LIO observation baseline."
        ),
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
        "# Online runtime report\n\n"
        + json.dumps(result, indent=2)
        + (
            "\n\nOnline trajectory in external map frame; no alignment. Snapshot only.\n"
            if fused
            else "\n\nOnline trajectory with fixed initial gauge z=-1.2 m; no fitting. Snapshot only.\n"
        )
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
