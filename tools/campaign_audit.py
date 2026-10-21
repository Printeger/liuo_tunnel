#!/usr/bin/env python3
"""Audit explicit nominal seeds 0–9 triplets; not lifecycle/fault or stage acceptance."""
import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path

from compare_runs import compare, estimator_settings
from long_run_audit import audit

MODES = ("record_only", "oracle_map", "surveyed_map")


def coverage_failures(entries):
    failures = []
    for seed in range(10):
        for mode in MODES:
            count = sum(e["seed"] == seed and e["mode"] == mode for e in entries)
            if count != 1:
                failures.append(f"seed {seed} {mode}: expected one run, found {count}")
    for entry in entries:
        if entry["seed"] not in range(10) or entry["mode"] not in MODES:
            failures.append(f"out-of-campaign entry: {entry}")
    return failures


def independent_run_summary(entries):
    """Descriptive equal-run statistics, never a replacement for per-run gates."""
    fields = [
        ("position_rmse_m",),
        ("attitude_p95_deg",),
        ("processing_p95_ms",),
        ("wall_end_to_end_p95_s",),
        ("covariance_diagnostics", "position_nees_mean_3d"),
        ("covariance_diagnostics", "joint_nees_mean_9d"),
        ("covariance_diagnostics", "position_95pct_ellipsoid_coverage"),
    ]
    result = {}
    for mode in MODES:
        group = [e for e in entries if e["mode"] == mode]
        metrics = {}
        for keys in fields:
            observations, unavailable = [], []
            for entry in group:
                value = entry.get("metrics", {})
                for key in keys:
                    value = value.get(key) if isinstance(value, dict) else None
                if not isinstance(value, (float, int)) or not math.isfinite(value):
                    unavailable.append(entry["run_id"])
                else:
                    observations.append(
                        {"run_id": entry["run_id"], "seed": entry["seed"], "value": value}
                    )
            values = [o["value"] for o in observations]
            metrics[".".join(keys)] = {
                "observations": observations,
                "unavailable_runs": unavailable,
                "n_runs": len(values),
                "mean": statistics.mean(values) if values else None,
                "min": min(values) if values else None,
                "max": max(values) if values else None,
                "sample_sd": statistics.stdev(values) if len(values) > 1 else None,
            }
        result[mode] = {"seeds": [e["seed"] for e in group], "metrics": metrics}
    return {
        "scope": "descriptive_equal_weight_per_run_not_pooled_frames_or_acceptance",
        "interpretation": "Each run contributes once. Within-run temporal correlation is not treated as independent samples. OFF is an ablation. Missing seeds and every per-run failure remain campaign failures; no mean replaces a gate.",
        "modes": result,
    }


def campaign(runs):
    failures, entries, groups = [], [], {}
    reference = None
    fused_reference = None
    for run in runs:
        try:
            load = lambda name: json.loads((run / name).read_text())
            config = load("metadata/config/phase1.json")
            scenario = load("metadata/config/scenario.json")
            mode = config["uwb"]["mode"]
            if mode == "fuse":
                if fused_reference is None:
                    fused_reference = config["uwb"]
                elif config["uwb"] != fused_reference:
                    failures.append(f"{run.name}: cross-seed fused UWB settings differ")
                mode = load("metadata/config/map.json")["mode"]
            entry = {"run_id": run.name, "seed": scenario["seed"], "mode": mode}
            entries.append(entry)
            groups.setdefault(entry["seed"], []).append(run)
            identity = load("metadata/manifests/input_identity.json")
            settings = (estimator_settings(config), identity["binaries"]["odometry"]["sha256"])
            if reference is None:
                reference = settings
            elif settings != reference:
                failures.append(f"{run.name}: cross-seed estimator settings/binary differ")
            result = audit(run)
            entry["long_run"] = result
            entry["metrics"] = load("export/analysis/report.json")
            metrics = entry["metrics"]
            # OFF is an ablation, not a known-map accuracy candidate, but still
            # must process the unchanged inputs within the same timing budget.
            if not metrics["processing_p95_ms"] < 100:
                failures.append(f"{run.name}: processing P95 >=100ms")
            if not 0 <= metrics["wall_end_to_end_p95_s"] < 0.25:
                failures.append(f"{run.name}: wall end-to-end P95 outside [0,250ms)")
            failures.extend(f"{run.name}: {reason}" for reason in result["failures"])
        except (OSError, ValueError, KeyError, TypeError, ZeroDivisionError) as error:
            failures.append(f"{run.name}: invalid evidence: {error}")
    failures.extend(coverage_failures(entries))
    comparisons = {}
    for seed, group in groups.items():
        if len(group) != 3:
            continue
        try:
            comparisons[seed] = compare(group, strict_estimator_settings=True)
        except (OSError, ValueError, KeyError, TypeError, ZeroDivisionError) as error:
            failures.append(f"seed {seed}: paired audit: {error}")
    return {
        "scope": "nominal_seeds_0_9_snapshot_campaign_not_stage_acceptance",
        "result": "FAIL" if failures else "PASS",
        "failures": failures,
        "runs": entries,
        "paired_comparisons": comparisons,
        "independent_run_summary": independent_run_summary(entries),
        "remaining_scope": "Fault delivery, lifecycle, return coverage, state semantics and covariance interpretation require separate evidence.",
        "selection": "Explicit list only; no best-run selection or averaging away failures. Preserve earlier attempts in acceptance history.",
        "auditor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = campaign([r.resolve() for r in args.runs])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"result": result["result"], "failures": result["failures"]}, indent=2))
    raise SystemExit(result["result"] != "PASS")
