#!/usr/bin/env python3
"""Handbook 8.1 pressure behavior audit; preserve all nominal accuracy exceedances."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from collections import Counter
from long_run_audit import continuity

ACCURACY_FAILURES = {
    "known-map position RMSE >0.30m",
    "known-map axial P95 >0.50m",
    "known-map axial max >1.0m",
    "known-map attitude P95 >2deg",
}
PENDING = "phase2 long-run/paired-campaign acceptance not yet implemented"


def accuracy_comparison(report):
    # Support immutable legacy reports and the revised reference-only schema.
    exceedances = sorted(
        {f for f in report["failures"] if f in ACCURACY_FAILURES}
        | set(report.get("accuracy_reference_exceedances", []))
    )
    return {
        "result": "FAIL" if exceedances else "PASS",
        "scope": "reference_targets_only_not_phase2_acceptance",
        "exceedances": exceedances,
    }


def interval(records, begin, end):
    selected = [r for r in records if begin <= r["stamp"] < end]
    if not selected:
        return {"samples": 0}
    return {
        "samples": len(selected),
        "position_rmse_m": math.sqrt(sum(r["error_m"] ** 2 for r in selected) / len(selected)),
        "position_max_m": max(r["error_m"] for r in selected),
        "axial_max_m": max(r["axial_m"] for r in selected),
        "states": dict(Counter(r["state"] for r in selected)),
    }


def sustained_return(records, end):
    """Descriptive per-frame 0.30m band, NOT the nominal RMSE acceptance test."""
    after = [r for r in records if r["stamp"] >= end]
    for i in range(max(0, len(after) - 10)):
        window = after[i : i + 11]
        if abs(window[-1]["stamp"] - window[0]["stamp"] - 1) < 1e-6 and all(
            r["error_m"] <= 0.30 for r in window
        ):
            return window[0]["stamp"] - end
    return None


def recovery_bins(records, decisions, begin, end):
    """Compare online diagnostics with evaluation error; never feed truth back."""
    bins = {}
    for record in records:
        if begin <= record["stamp"] < end:
            index = int(record["stamp"] - begin)
            bins.setdefault(index, {"trajectory": [], "innovations": []})["trajectory"].append(
                record
            )
    for decision in decisions:
        stamp = float(decision["measurement_stamp"])
        if begin <= stamp < end and decision["reason"] == "ACCEPT_TA":
            if decision["residual_class"] != "PRE_OPT_INNOVATION":
                raise ValueError("unexpected accepted TA residual class")
            value = float(decision["whitened"])
            if not math.isfinite(value):
                raise ValueError("nonfinite accepted TA innovation")
            index = int(stamp - begin)
            bins.setdefault(index, {"trajectory": [], "innovations": []})["innovations"].append(
                value
            )
    result = []
    for index, data in sorted(bins.items()):
        values = data["innovations"]
        result.append(
            {
                "seconds_after_restore": index,
                "navigation": interval(data["trajectory"], begin + index, begin + index + 1),
                "accepted_ta_count": len(values),
                "measurement_sigma_normalized_innovation_rms": (
                    math.sqrt(sum(value * value for value in values) / len(values))
                    if values
                    else None
                ),
            }
        )
    return result


def output_coverage_failures(summary):
    """Pressure may honestly be LOST; missing/nonfinite output is separate."""
    failures = []
    if (
        not summary["expected_epochs"]
        or summary["states"]["MISSING"] > 0.01 * summary["expected_epochs"]
    ):
        failures.append("pressure odometry publication coverage below 99%")
    if summary["states"]["INVALID"] or summary["duplicate_epochs"] or summary["unexpected_epochs"]:
        failures.append("invalid, duplicate or unexpected pressure output epochs")
    return failures


def recovery_window_check(transitions, trajectory, end, lag):
    # The first renewed-input transition carries the deadline even if a stronger
    # covariance or residual diagnostic masks UWB_RECOVERING_WINDOW.
    candidates = [
        r
        for r in transitions
        if end <= float(r["stamp"]) <= end + 0.5 and float(r["uwb_recovery_until"]) > end
    ]
    if not candidates:
        return None, ["missing recovery deadline transition"]
    entry = candidates[0]
    begin, deadline = float(entry["stamp"]), float(entry["uwb_recovery_until"])
    failures = []
    if deadline < begin + lag - 1e-6:
        failures.append("recovery shorter than lag")
    if any(r["state"] == "TRACKING" for r in trajectory if begin <= r["stamp"] < deadline - 1e-6):
        failures.append("premature TRACKING")
    return deadline, failures


def audit(run):
    load = lambda name: json.loads((run / name).read_text())

    def rows(name):
        with (run / name).open() as stream:
            return list(csv.DictReader(stream))

    report = load("export/analysis/report.json")
    config = load("metadata/config/phase1.json")
    profile = load("metadata/config/delivery.json")
    failures = [f for f in report["failures"] if f not in ACCURACY_FAILURES and f != PENDING]
    if config["uwb"]["mode"] != "fuse" or report["sessions"] != 1:
        failures.append("pressure audit requires fused single-session input")
    if load("metadata/run_manifest.json")["lifecycle"] != "completed":
        failures.append("run not completed")
    if not report["delivery_audit"] or report["delivery_audit"]["result"] != "PASS":
        failures.append("transport/recovery audit did not pass")
    if not report["delivered_input_verified"]:
        failures.append("delivered bytes not verified")
    trajectory = rows("export/odometry/trajectory.csv")
    truth_rows = rows("export/simulation/truth.csv")
    truth = {round(float(r["stamp"]), 7): r for r in truth_rows}
    coverage = continuity(trajectory, truth_rows, config)
    failures.extend(output_coverage_failures(coverage))
    transitions = rows("export/odometry/status.csv")
    decisions = rows("export/odometry/uwb.csv")
    start = float(rows("export/simulation/wall_clock.csv")[0]["stamp"])
    annotated = []
    for r in trajectory:
        stamp = float(r["stamp"])
        t = truth[round(stamp, 7)]
        delta = [float(r[k]) - float(t[k]) for k in ["x", "y", "z"]]
        error = math.sqrt(sum(x * x for x in delta))
        if not math.isfinite(error):
            failures.append("nonfinite published error")
        annotated.append(
            {
                "stamp": stamp,
                "error_m": error,
                "axial_m": abs(delta[0]),
                "state": r["state"],
                "reason": r["state_reason"],
            }
        )
    outages = []
    declared_outages = profile.get("outages", [])
    if not declared_outages and not profile.get("anchor_outages") and not profile.get("nlos_bias"):
        failures.append("no declared outage pressure interval")
    for i, (lo, hi) in enumerate(declared_outages):
        begin, end = start + lo, start + hi
        next_begin = (
            start + declared_outages[i + 1][0]
            if i + 1 < len(declared_outages)
            else annotated[-1]["stamp"] + 0.1
        )
        interior = [r for r in annotated if begin + 0.5 <= r["stamp"] < end]
        if not interior or any(r["state"] not in ["DEGRADED", "LOST"] for r in interior):
            failures.append(f"outage {lo}-{hi}: not reported degraded/lost")
        deadline, recovery_failures = recovery_window_check(
            transitions, annotated, end, config["odometry"]["lag_s"]
        )
        failures.extend(f"outage {lo}-{hi}: {failure}" for failure in recovery_failures)
        tracking = [
            r for r in annotated if end <= r["stamp"] < next_begin and r["state"] == "TRACKING"
        ]
        outages.append(
            {
                "elapsed_interval_s": [lo, hi],
                "recovery_one_second_bins": recovery_bins(
                    annotated, decisions, end, min(end + 30, next_begin)
                ),
                "outage": interval(annotated, begin, end),
                "after_restore_until_next_outage": interval(annotated, end, next_begin),
                "recovery_deadline_stamp": deadline,
                "first_tracking_after_restore_s": tracking[0]["stamp"] - end if tracking else None,
                "position_error_at_first_tracking_m": tracking[0]["error_m"] if tracking else None,
                "sustained_1s_position_0_30m_band_return_s": sustained_return(
                    [r for r in annotated if r["stamp"] < next_begin], end
                ),
            }
        )
    return {
        "scope": "handbook_8_1_pressure_behavior_not_nominal_accuracy_or_stage_acceptance",
        "contract_result": "FAIL" if failures else "PASS",
        "contract_failures": failures,
        "original_report_result": report["result"],
        "original_report_failures": report["failures"],
        "nominal_accuracy_comparison": accuracy_comparison(report),
        "outages": outages,
        "output_coverage": coverage,
        "nlos_navigation": (
            {
                "during_bias": interval(
                    annotated,
                    start + profile["nlos_bias"]["begin_s"],
                    start + profile["nlos_bias"]["end_s"],
                ),
                "first_10s_after_bias": interval(
                    annotated,
                    start + profile["nlos_bias"]["end_s"],
                    start + profile["nlos_bias"]["end_s"] + 10,
                ),
            }
            if profile.get("nlos_bias")
            else None
        ),
        "anchor_outage_navigation": [
            {
                "anchor_id": fault["anchor_id"],
                "elapsed_interval_s": [fault["begin_s"], fault["end_s"]],
                "during_outage": interval(
                    annotated, start + fault["begin_s"], start + fault["end_s"]
                ),
                "first_10s_after_restore": interval(
                    annotated, start + fault["end_s"], start + fault["end_s"] + 10
                ),
            }
            for fault in profile.get("anchor_outages", [])
        ],
        "interpretation": "Pressure errors and failures remain visible. One-second per-frame error band is descriptive, not a replacement for RMSE or a new acceptance gate. No trajectory alignment. TRACKING is operational, not a calibrated error guarantee. Innovation RMS uses measurement sigma only, not predictive covariance or a chi-square consistency test; small residuals do not prove accurate navigation.",
        "auditor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    args = p.parse_args()
    try:
        result = audit(args.run)
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        result = {
            "contract_result": "FAIL",
            "contract_failures": [f"invalid pressure evidence: {error}"],
        }
    (args.run / "export/analysis/pressure_audit.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    print(json.dumps(result, indent=2))
    raise SystemExit(result["contract_result"] != "PASS")
