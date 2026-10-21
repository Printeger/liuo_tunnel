#!/usr/bin/env python3
"""Independent full-route continuity/resource audit; never stage acceptance alone."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def sample_key(row):
    return (row["session"], round(float(row["stamp"]), 8))


def normalized(records):
    sessions = list(dict.fromkeys(r["session"] for r in records))
    return [dict(r, session=str(sessions.index(r["session"]))) for r in records]


def continuity(trajectory, truth, config):
    """Denominator is every truth/cloud epoch after declared initialization.

    DEGRADED finite outputs remain available but are reported separately. Accuracy,
    latency and state credibility are separate gates; availability is not accuracy.
    """
    if not truth:
        raise ValueError("no truth/cloud epochs")
    truth = normalized(truth)
    trajectory = normalized(trajectory)
    fused = config["uwb"]["mode"] == "fuse"
    delay = config["uwb"]["bootstrap_s"] if fused else config["odometry"]["initialization_s"] + 0.1
    starts = {}
    for r in truth:
        starts.setdefault(r["session"], float(r["stamp"]) + delay)
    expected = {sample_key(r) for r in truth if float(r["stamp"]) >= starts[r["session"]] - 1e-7}
    observed = {}
    duplicates = 0
    for r in trajectory:
        key = sample_key(r)
        duplicates += key in observed
        observed[key] = r
    states = {
        name: 0 for name in ["TRACKING", "DEGRADED", "LOST", "INITIALIZING", "MISSING", "INVALID"]
    }
    valid = 0
    finite_fields = ["x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz", "wx", "wy", "wz"]
    for key in expected:
        r = observed.get(key)
        if r is None:
            states["MISSING"] += 1
            continue
        state = r.get("state", "INVALID")
        finite = all(math.isfinite(float(r.get(f, "nan"))) for f in finite_fields)
        qnorm = sum(float(r.get(k, "nan")) ** 2 for k in ["qx", "qy", "qz", "qw"])
        usable = finite and abs(qnorm - 1) < 1e-6 and r.get("cov_valid") == "1"
        if not usable or state not in states:
            state = "INVALID"
        states[state] += 1
        valid += state in ["TRACKING", "DEGRADED"]
    return {
        "expected_epochs": len(expected),
        "valid_epochs": valid,
        "availability": valid / len(expected) if expected else 0,
        "states": states,
        "duplicate_epochs": duplicates,
        "unexpected_epochs": len(set(observed) - expected),
        "initialization_sensor_s": delay,
        "definition": "finite unit-quaternion/cov_valid TRACKING or DEGRADED output at each 10 Hz epoch; DEGRADED is not an accuracy claim",
    }


def coverage(truth, return_trip):
    ordered = sorted(truth, key=lambda r: float(r["stamp"]))
    stationary = [r for r in ordered if abs(float(r["vx"])) < 1e-5]
    stops = [600, 1200, 1800, 2350]
    visited = {str(x): any(abs(float(r["x"]) - x) < 0.01 for r in stationary) for x in stops}
    return {
        "required_outbound_stops": visited,
        "maximum_x_m": max(float(r["x"]) for r in ordered),
        "final_x_m": float(ordered[-1]["x"]),
        "reverse_samples": sum(float(r["vx"]) < -0.1 for r in ordered),
        "outbound_complete": all(visited.values()),
        "return_complete": return_trip
        and all(visited.values())
        and abs(float(ordered[-1]["x"])) < 0.01
        and any(float(r["vx"]) < -0.1 for r in ordered),
    }


def quantile(values, p):
    return sorted(values)[int((len(values) - 1) * p)]


def attitude_distributions(trajectory):
    """Signed ZYX Euler errors for the straight-world identity truth attitude."""
    values = {name: [] for name in ["roll", "pitch", "yaw"]}
    for row in trajectory:
        x, y, z, w = [float(row[key]) for key in ["qx", "qy", "qz", "qw"]]
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or abs(norm - 1) > 1e-6:
            raise ValueError("invalid quaternion in attitude distribution")
        x, y, z, w = [q / norm for q in [x, y, z, w]]
        angles = [
            math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1, min(1, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
        ]
        for name, angle in zip(values, angles):
            values[name].append(math.degrees(angle))

    def quantile(data, fraction):
        ordered = sorted(data)
        point = (len(ordered) - 1) * fraction
        low, high = math.floor(point), math.ceil(point)
        return ordered[low] + (ordered[high] - ordered[low]) * (point - low)

    if not trajectory:
        raise ValueError("empty attitude evidence")
    return {
        "convention": "signed ZYX Euler angles in degrees; straight-world identity truth only; no alignment; descriptive distributions, not replacement for geodesic attitude gate",
        "samples": len(trajectory),
        "axes": {
            name: {
                "mean_deg": statistics.mean(data),
                "rmse_deg": math.sqrt(statistics.mean(x * x for x in data)),
                "signed_quantiles_deg": {
                    str(q): quantile(data, q) for q in [0, 0.05, 0.25, 0.5, 0.75, 0.95, 1]
                },
                "absolute_p95_deg": quantile([abs(x) for x in data], 0.95),
            }
            for name, data in values.items()
        },
    }


def audit(run, require_return=False):
    failures = []

    def check(condition, reason):
        if not condition:
            failures.append(reason)

    required = [
        "metadata/run_manifest.json",
        "metadata/config/phase1.json",
        "metadata/config/scenario.json",
        "metadata/manifests/observation_audit.json",
        "export/simulation/runtime.json",
        "export/simulation/truth.csv",
        "export/odometry/trajectory.csv",
        "export/odometry/graph.csv",
        "profiling/resources.csv",
        "export/analysis/report.json",
    ]
    missing = [name for name in required if not (run / name).is_file()]
    if missing:
        return {
            "result": "FAIL",
            "run_id": run.name,
            "failures": ["missing: " + name for name in missing],
        }
    load = lambda name: json.loads((run / name).read_text())
    manifest = load("metadata/run_manifest.json")
    config = load("metadata/config/phase1.json")
    scenario = load("metadata/config/scenario.json")
    observation = load("metadata/manifests/observation_audit.json")
    runtime = load("export/simulation/runtime.json")
    report = load("export/analysis/report.json")
    check(manifest["lifecycle"] == "completed", "run not completed")
    check(manifest["source"]["git_worktree_clean"], "dirty source")
    check(config["odometry"]["backend"] == "gpu", "not GPU")
    check(not observation["violations"], "wire contract violations")
    check(observation["counts"] == runtime["published_counts"], "raw counts mismatch")
    check(
        observation["clock_resets"] == 0, "nominal long-run audit excludes deliberate clock resets"
    )
    check(runtime["real_time_factor"] >= 0.99, "actual input slower than 1x tolerance")
    pending = "phase2 long-run/paired-campaign acceptance not yet implemented"
    other_failures = [x for x in report["failures"] if x != pending]
    check(not other_failures, "per-run failures: " + repr(other_failures))
    # A known pending campaign marker is reported separately, never interpreted
    # as stage acceptance. All physical/accuracy failures above remain failures.
    truth = read_rows(run / "export/simulation/truth.csv")
    trajectory = read_rows(run / "export/odometry/trajectory.csv")
    graph = read_rows(run / "export/odometry/graph.csv")
    resources = read_rows(run / "profiling/resources.csv")
    check(bool(truth and trajectory and graph and resources), "empty required evidence")
    if not (truth and trajectory and graph and resources):
        return {"result": "FAIL", "run_id": run.name, "failures": failures}
    check(len(truth) == runtime["published_counts"]["cloud"], "truth/cloud epoch counts differ")
    check(
        all(
            abs(float(b["stamp"]) - float(a["stamp"]) - 0.1) < 1e-6
            for a, b in zip(truth, truth[1:])
        ),
        "truth/cloud epoch grid not 10 Hz",
    )
    continuity_result = continuity(trajectory, truth, config)
    check(continuity_result["availability"] >= 0.99, "availability below 99%")
    check(continuity_result["duplicate_epochs"] == 0, "duplicate output epochs")
    check(continuity_result["unexpected_epochs"] == 0, "unexpected output epochs")
    trip = scenario.get("return_trip", False)
    route = coverage(truth, trip)
    check(route["outbound_complete"], "full outbound stops not covered")
    check(
        runtime["simulation_s"] >= (1070 if trip or require_return else 540),
        "duration shorter than full-route evidence",
    )
    if trip or require_return:
        check(route["return_complete"], "backward return not completed")
    max_frames = math.ceil(config["odometry"]["lag_s"] * 10) + 2
    check(
        all(
            int(r["X"]) <= max_frames and int(r["reset"]) == 0 and int(r["fallback"]) == 0
            for r in graph
        ),
        "window/reset/fallback invariant",
    )
    check(
        all(
            int(r["retained_cloud_frames"]) <= max_frames
            and int(r["keyframes"]) <= config["odometry"].get("max_keyframes", 15)
            and int(r["imu_queue"]) <= config["odometry"]["imu_queue_limit"]
            and int(r["cloud_queue"]) <= config["odometry"]["cloud_queue_limit"]
            and int(r["uwb_queue"]) <= 4096
            for r in resources
        ),
        "resource structural bound exceeded",
    )
    check(
        float(resources[-1]["stamp"]) >= float(truth[-1]["stamp"]) - 1.1,
        "resource sampling ended early",
    )
    check(
        float(resources[0]["stamp"]) <= float(trajectory[0]["stamp"]) + 0.11,
        "resource sampling started late",
    )
    check(
        all(
            abs(float(b["stamp"]) - float(a["stamp"]) - 1) < 1e-6
            for a, b in zip(resources, resources[1:])
        ),
        "resource samples are not continuous at 1 Hz",
    )
    rss = [int(r["rss_kib"]) for r in resources]
    device = [int(r["device_used_bytes"]) for r in resources]
    check(min(rss) > 0 and min(device) > 0, "invalid RSS/device telemetry")
    tail = resources[len(resources) // 2 :]
    ts = [float(r["stamp"]) for r in tail]
    ys = [int(r["rss_kib"]) for r in tail]
    mean_t, mean_y = statistics.mean(ts), statistics.mean(ys)
    denom = sum((t - mean_t) ** 2 for t in ts)
    slope = sum((t - mean_t) * (y - mean_y) for t, y in zip(ts, ys)) / denom if denom else None
    return {
        "scope": "nominal_full_route_continuity_resources_not_stage_acceptance",
        "run_id": run.name,
        "result": "FAIL" if failures else "PASS",
        "failures": failures,
        "campaign_still_pending": pending in report["failures"],
        "continuity": continuity_result,
        "route": route,
        "attitude_distributions": attitude_distributions(trajectory),
        "resources": {
            "samples": len(resources),
            "rss_range_kib": [min(rss), max(rss)],
            "device_wide_used_bytes_range": [min(device), max(device)],
            "tail_rss_slope_kib_per_s": slope,
            "interpretation": "Structural limits are checked; RSS trend is observed evidence, not a proof of allocator bounds or other-process memory.",
        },
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--require-return", action="store_true")
    args = parser.parse_args()
    try:
        result = audit(args.run.resolve(), args.require_return)
    except (OSError, ValueError, KeyError, TypeError, ZeroDivisionError) as error:
        result = {
            "result": "FAIL",
            "run_id": args.run.name,
            "failures": [f"invalid evidence: {type(error).__name__}: {error}"],
        }
    result["auditor_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    output = args.run / "export/analysis/long_run_audit.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    raise SystemExit(result["result"] != "PASS")
