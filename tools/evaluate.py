#!/usr/bin/env python3
"""Online run checks and reference metrics, not phase-two effectiveness acceptance."""
import argparse
import hashlib
import csv
import json
import math
from pathlib import Path
import statistics
import sys
from evaluation_frames import world_estimates, run_world_estimates, map_local_truth


ACCURACY_REFERENCE_FAILURES = {
    "known-map position RMSE >0.30m",
    "known-map axial P95 >0.50m",
    "known-map axial max >1.0m",
    "known-map attitude P95 >2deg",
    "entrance tracking sanity RMSE > 1 m",
}


def acceptance_policy(failures, phase2):
    """Keep numerical exceedances visible without certifying effectiveness."""
    references = [f for f in failures if phase2 and f in ACCURACY_REFERENCE_FAILURES]
    blocking = [
        f for f in failures if not phase2 or f not in ACCURACY_REFERENCE_FAILURES
    ]
    return {
        "result": "FAIL" if blocking else "PASS",
        "failures": blocking,
        "accuracy_reference_exceedances": references,
        "result_scope": (
            "single_run_contracts_not_system_effectiveness"
            if phase2
            else "phase1_checks"
        ),
        "stage_acceptance": "NOT_ASSESSED" if phase2 else "SEE_PHASE1_ACCEPTANCE",
    }


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
    phase2 = config.get("schema_version") in ("liuo_phase2_v1", "liuo_phase3_v1")
    required_edges = 2 if phase2 else 3
    multiple = any(
        int(r["X"]) >= required_edges + 1 and int(r["lidar_factors"]) >= required_edges
        for r in graph
    )
    continuous = all(
        int(r["X"]) <= 1
        or (int(r["lidar_factors"]) >= 1 and int(r["imu_factors"]) >= 1)
        for r in graph
    )
    return multiple and continuous


def calibrated_map_metrics(run, document):
    """Simulation evaluation only, never an estimator input or fitted alignment."""
    import numpy as np
    from evaluation_frames import map_evaluation_datum, _transform

    fixture = json.loads((Path(__file__).resolve().parent.parent / "packages/liuo_tunnel/config/maps/oracle_map.json").read_text())
    scenario = json.loads((run / "metadata/config/scenario.json").read_text())
    if fixture["simulation_anchor_layout"] != scenario["anchor_layout"]:
        raise ValueError("map evaluation oracle fixture does not match simulation")
    reference = {a["id"]: np.array(a["mean_m"]) for a in fixture["anchors"]}
    datum = map_evaluation_datum(run)
    records = [dict(session="map", x=a["mean_m"][0], y=a["mean_m"][1], z=a["mean_m"][2],
                    vx=0, vy=0, vz=0, qx=0, qy=0, qz=0, qw=1) for a in document["anchors"]]
    world = _transform(records, [datum], False)
    estimates = {a["id"]: np.array([r[k] for k in ("x", "y", "z")]) for a, r in zip(document["anchors"], world)}
    errors = np.array([estimates[a["id"]] - reference[a["id"]] for a in document["anchors"]])
    order = sorted(estimates, key=lambda key: reference[key][0])
    spacing = []
    for a, b in zip(order, order[1:]):
        spacing.append({"ids": [a, b], "distance_error_m": float(np.linalg.norm(estimates[b]-estimates[a]) - np.linalg.norm(reference[b]-reference[a]))})
    covariance = np.array(document["joint_covariance"])
    return {"scope": "observed initialized anchors only; fixed first-run gauge; no alignment",
            "anchor_errors_world_m": {str(a["id"]): e.tolist() for a, e in zip(document["anchors"], errors)},
            "axis_rmse_m": np.sqrt(np.mean(errors**2, axis=0)).tolist(),
            "adjacent_initialized_spacing_errors": spacing,
            "estimated_longitudinal_order": sorted(estimates, key=lambda key: estimates[key][0]),
            "joint_local_std": np.sqrt(np.diag(covariance)).tolist(),
            "joint_order": ["a"+str(a["id"])+".xyz_m" for a in document["anchors"]]+["l0.angle_mrad", "l0.offset_m"],
            "anchor_reference_cross_covariance_norm": float(np.linalg.norm(covariance[:-2,-2:])),
            "not_initialized": document["not_initialized"], "statistical_consistency": "NOT_CERTIFIED"}


def evaluate(run, output_dir=None):
    manifest = json.loads((run / "metadata/run_manifest.json").read_text())
    failures = []

    def check(ok, reason):
        if not ok:
            failures.append(reason)

    check(manifest["lifecycle"] == "completed", "run lifecycle not completed")
    check(manifest["source"]["git_worktree_clean"], "source not clean")
    audit = json.loads((run / "metadata/manifests/observation_audit.json").read_text())
    check(
        not audit["violations"], "raw message violations: " + str(audit["violations"])
    )
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
                check(
                    "delivered_sha256" in runtime, "missing fault delivery hash audit"
                )
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
    phase3 = config.get("schema_version") == "liuo_phase3_v1"
    phase2 = phase3 or config.get("schema_version") == "liuo_phase2_v1"
    mode = config["uwb"]["mode"]
    fused = mode in ("fuse", "self_calibrate", "reuse_map")
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
        map_path = run / ("export/map/calibrated_map.json" if mode == "self_calibrate" else "metadata/config/map.json")
        map_config = json.loads(map_path.read_text())
        expected_anchors = len(map_config["anchors"]) if phase3 or map_config["mode"] == "surveyed_map" else 0
        if phase3:
            import numpy as np
            covariance = np.array(map_config["joint_covariance"], dtype=float)
            check(covariance.shape == (3 * expected_anchors + 2,) * 2, "joint A/L map dimension")
            check(np.isfinite(covariance).all() and np.allclose(covariance, covariance.T), "invalid joint map covariance")
            np.linalg.cholesky(covariance)
            check(map_config["unresolved_branches"] == [], "map contains unresolved branches")
            check(map_config["sensor_identity"] == config["sensors"], "map sensor identity mismatch")
            check(map_config["source_identity"].get("schema_version") == "liuo_map_source_identity_v1", "unsafe legacy map provenance")
            check(bool(graph) and "joint_map_priors" in graph[0], "actual map lifecycle evidence missing")
            check(all(int(r.get("joint_map_priors", -1)) == int(mode == "reuse_map") for r in graph), "joint map prior count")
            if mode == "reuse_map":
                check(all(int(r.get("installation_observation_factors", -1)) == 0 for r in graph), "old installation observations repeated")
            check(all(r["state"] != "TRACKING" for r in trajectory), "provisional map claimed healthy tracking")
        if mode == "self_calibrate":
            counts = [int(r["A"]) for r in graph]
            check(bool(counts) and counts[-1] == expected_anchors and all(a <= b for a, b in zip(counts, counts[1:])),
                  "initialized anchor set shrank or export missed extension")
            check(all(int(r.get("installation_observation_factors", -1)) == 1 for r in graph), "installation subset not exactly once")
        else:
            check(all(int(r["A"]) == expected_anchors for r in graph), "static anchors not retained")
        check(
            all(int(r["map_priors"]) == int(expected_anchors > 0 and mode != "self_calibrate") for r in graph),
            "map prior count changed",
        )
        if graph and "aa_summary_factors" in graph[0]:
            pair_bound = expected_anchors * (expected_anchors - 1) // 2
            check(
                all(int(r["aa_summary_factors"]) <= pair_bound for r in graph),
                "AA summary pair bound exceeded",
            )
            if audit["clock_resets"] == 0:
                if "static_edges" in config["uwb"] and config["uwb"]["use_aa"]:
                    events = [json.loads(line) for line in (run / "export/odometry/static_edges.jsonl").read_text().splitlines()]
                    updates = [event for event in events if event["event"] == "graph_update"]
                    check(bool(updates), "static graph updates missing")
                    check(not any(r["reason"] == "ACCEPT_AA" for r in uwb), "raw/static AA counted simultaneously")
                    if updates:
                        check(int(graph[-1]["aa_accumulated_measurements"]) == updates[-1]["fused_leaves"], "static graph counter mismatch")
                    delivered = json.loads((run / "export/simulation/static_delivery.json").read_text())
                    check(int(graph[-1]["aa_accumulated_measurements"]) <= len(delivered["unique_physical_ids"]), "static count exceeds physical deliveries")
                    if mode == "self_calibrate":
                        leaves = map_config["static_edge_ledger"]["leaves"]
                        check(len({leaf["physical_id"] for leaf in leaves}) == len(leaves), "duplicate map physical leaf")
                        check({leaf["physical_id"] for leaf in leaves} <= set(delivered["unique_physical_ids"]), "map absorbed undelivered static input")
                else:
                    check(
                        int(graph[-1]["aa_accumulated_measurements"])
                        == sum(r["reason"] == "ACCEPT_AA" for r in uwb),
                        "AA accumulated/raw identity count mismatch",
                    )
            cfg = json.loads((run / "metadata/config/phase1.json").read_text())
            odometry = cfg["odometry"]
            factor_bound = (
                math.ceil(250 * (odometry["lag_s"] + 0.2))
                + 102
                * (
                    odometry["connection_window"]
                    + odometry.get("max_keyframes", 15)
                    + 3
                )
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
        (truth_sessions.index(r["session"]), round(float(r["stamp"]), 5)): r
        for r in truth
    }
    sections = {name: [] for name in ["entrance", "transition", "degenerate"]}
    attitude = []
    errors = []
    axis_errors, velocity_errors = [], []
    for r in run_world_estimates(run, trajectory, truth, mode):
        key = (est_sessions.index(r["session"]), round(float(r["stamp"]), 5))
        q = truth_map.get(key)
        if q is None:
            failures.append("unmatched online state time")
            continue
        # World-frame metrics; OFF uses the fixed initial evaluation datum, no fitting.
        dx = float(r["x"]) - float(q["x"])
        dy = float(r["y"]) - float(q["y"])
        dz = float(r["z"]) - float(q["z"])
        e = math.sqrt(dx * dx + dy * dy + dz * dz)
        check(math.isfinite(e), "nonfinite trajectory")
        name = (
            "entrance"
            if float(q["x"]) <= 50
            else ("transition" if float(q["x"]) < 90 else "degenerate")
        )
        sections[name].append((e, abs(dx)))
        errors.append(e)
        axis_errors.append([dx, dy, dz])
        velocity_errors.append([float(r[k]) - float(q[k]) for k in ["vx", "vy", "vz"]])
        # Old capture schema has identity truth; new starts record full orientation.
        dot = sum(
            float(r[k]) * float(q.get(k, 1 if k == "qw" else 0))
            for k in ["qx", "qy", "qz", "qw"]
        )
        attitude.append(math.degrees(2 * math.acos(min(1, abs(dot)))))
    metrics = {}
    for name, data in sections.items():
        metrics[name] = {
            "samples": len(data),
            "position_rmse_m": (
                math.sqrt(statistics.mean(e * e for e, _ in data)) if data else None
            ),
            "axial_p95_m": quantile([x for _, x in data], 0.95),
            "axial_max_m": max([x for _, x in data], default=None),
        }
    # Arbitrary interior starts need not observe the entrance. Phase-one's
    # entrance gate is unchanged; phase-two numerical exceedances are references.
    if not phase2 or metrics["entrance"]["position_rmse_m"] is not None:
        check(
            metrics["entrance"]["position_rmse_m"] is not None
            and metrics["entrance"]["position_rmse_m"] <= 1.0,
            "entrance tracking sanity RMSE > 1 m",
        )
    scenario = json.loads((run / "metadata/config/scenario.json").read_text())
    if scenario["duration_s"] >= 35 and scenario["reset_at_s"] < 0:
        check(
            bool(sections["degenerate"]),
            "no observation beyond entrance LiDAR visibility",
        )
    if scenario["reset_at_s"] > 0:
        check(
            audit["clock_resets"] == 1
            and len(audit["sessions"]) == 2
            and len(est_sessions) == 2,
            "clock reset/session behavior",
        )
    elapsed = [float(r["total_ms"]) for r in timing]
    latency = [float(r["latency_s"]) for r in timing]
    wall_latency = []
    wall_clock_path = run / "export/simulation/wall_clock.csv"
    if wall_clock_path.exists():
        deadlines = {
            (int(r["session"]), round(float(r["stamp"]), 9)): int(
                r["scheduled_wall_ns"]
            )
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
    backend = json.loads((run / "metadata/config/phase1.json").read_text())[
        "odometry"
    ].get("backend", "cpu")
    if backend == "gpu":
        process = json.loads((run / "metadata/processes/odometry.json").read_text())
        check(
            process.get("backend") == "gpu_RAW_MULTIFRAME_VGICP"
            and bool(process.get("gpu_device")),
            "GPU execution identity missing",
        )
    if fused:
        if errors:
            check(
                math.sqrt(statistics.mean(e * e for e in errors)) <= 0.30,
                "known-map position RMSE >0.30m",
            )
            axial = [x for data in sections.values() for _, x in data]
            check(quantile(axial, 0.95) <= 0.50, "known-map axial P95 >0.50m")
            check(max(axial) <= 1.0, "known-map axial max >1.0m")
            check(quantile(attitude, 0.95) <= 2.0, "known-map attitude P95 >2deg")
        check(bool(elapsed) and quantile(elapsed, 0.95) < 100, "processing P95 >=100ms")
        check(
            bool(latency) and quantile(latency, 0.95) < 0.25,
            "simulation-time latency P95 >=250ms",
        )
        check(
            runtime["real_time_factor"] >= 0.99, "actual input rate below 1x tolerance"
        )
        check(len(wall_latency) == len(trajectory), "wall latency evidence incomplete")
        check(
            bool(wall_latency) and quantile(wall_latency, 0.95) < 0.25,
            "wall end-to-end P95 >=250ms or unavailable",
        )
        # Stage effectiveness requires representative paired evidence and honest
        # initialization/status. A single-run evaluator never certifies it;
        # the deferred ten-seed campaign is not an unconditional failure gate.
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
            for published, observed, covariance in zip(
                trajectory, wire, covariance_rows
            ):
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
                            abs(float(covariance[field]) - float(observed[field]))
                            < 1e-10,
                            "wire twist covariance mismatch",
                        )
    covariance_path = run / "export/odometry/covariance.csv"
    if covariance_path.exists() and trajectory:
        from covariance_analysis import summarize

        covariance_summary = summarize(
            run, trajectory, map_local_truth(run, truth) if phase3 else truth, rows(covariance_path), fused
        )
    policy = acceptance_policy(failures, phase2)
    failures = policy["failures"]
    if phase3:
        policy["result_scope"] = "phase3_single_run_contracts_not_system_effectiveness"
    result = {
        "evaluator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": (
            "phase3_iteration_snapshot_" if phase3 else "phase2_iteration_snapshot_"
            if config.get("schema_version") == "liuo_phase2_v1"
            else "phase1_snapshot_"
        )
        + backend,
        "estimator_mode": config["uwb"]["mode"],
        "accuracy_acceptance_scope": (
            "phase2_accuracy_reference_only"
            if phase2
            else "entrance_sanity_only; record_only_ablation_is_not_known_map_accuracy_acceptance"
        ),
        **policy,
        "evaluation_policy": "phase3_fixed_first_run_gauge" if phase3 else ("representative_phase2_2026_10_22" if phase2 else "phase1"),
        "map_evaluation_frame": "fixed original calibration run datum; no reuse-start alignment" if phase3 else None,
        "calibrated_map_metrics": calibrated_map_metrics(run, map_config) if phase3 else None,
        "metrics": metrics,
        "position_rmse_m": (
            math.sqrt(statistics.mean(e * e for e in errors)) if errors else None
        ),
        "axis_rmse_m": [math.sqrt(statistics.mean(v[i] ** 2 for v in axis_errors)) for i in range(3)] if axis_errors else None,
        "velocity_axis_rmse_mps": [math.sqrt(statistics.mean(v[i] ** 2 for v in velocity_errors)) for i in range(3)] if velocity_errors else None,
        "first_output_stamp": float(trajectory[0]["stamp"]) if trajectory else None,
        "initialization_wait_s": float(trajectory[0]["stamp"]) - float(truth[0]["stamp"]) if trajectory and truth else None,
        "states": {state: sum(r["state"] == state for r in trajectory) for state in sorted(set(r["state"] for r in trajectory))},
        "optimized_output_observed": bool(trajectory),
        "attitude_p95_deg": quantile(attitude, 0.95),
        "processing_p95_ms": quantile(elapsed, 0.95),
        "sim_time_latency_p95_s": quantile(latency, 0.95),
        "wall_end_to_end_p95_s": quantile(wall_latency, 0.95),
        "actual_input_hz": {
            k: v / runtime["wall_s"] for k, v in audit["counts"].items()
        },
        "delivery_profile": fault_profile,
        "delivered_input_verified": audit.get("raw_sha256")
        == runtime.get("delivered_sha256")
        and audit["counts"] == runtime.get("published_counts"),
        "frozen_input_verified": frozen_path.exists()
        and audit.get("raw_sha256") == frozen["raw_sha256"]
        and audit["counts"] == frozen["counts"],
        "max_vehicle_states": max((int(r["X"]) for r in graph), default=0),
        "simulation_runtime": {
            k: v
            for k, v in runtime.items()
            if k not in ["duplicate_uwb", "delivered_sha256"]
        },
        "delivery_audit": delivery_summary,
        "raw_counts": audit["counts"],
        "covariance_diagnostics": covariance_summary,
        "sessions": len(est_sessions),
        "gpu": (
            ("FAIL" if failures else "PASS") if backend == "gpu" else "NOT_VALIDATED"
        ),
        "rolling_deskew": "NOT_VALIDATED",
        "accuracy_thresholds": (
            "Reference targets only; exceedances do not block the representative stage. PASS is not effectiveness or stage acceptance."
            if phase2
            else "Fused-navigation thresholds not applied to phase-one LIO observation baseline."
        ),
    }
    out = Path(output_dir) if output_dir is not None else run / "export/analysis"
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.json").write_text(json.dumps(result, indent=2) + "\n")
    with (out / "metrics.csv").open("w") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "section",
                "samples",
                "position_rmse_m",
                "axial_p95_m",
                "axial_max_m",
            ],
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
            else "\n\nOFF uses evaluation-only fixed initial translation/yaw; no trajectory fitting. Snapshot only.\n"
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
