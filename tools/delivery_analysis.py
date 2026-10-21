"""Audit declared transport faults separately from navigation accuracy."""

from collections import Counter
import math
from delivery_schedule import DeliverySchedule


def analyze(profile, delivery, decisions, trajectory, graph, clock, runtime):
    failures = []
    identity = lambda r: (int(r["producer"]), int(r["session"]), int(r["sequence"]))
    accepted = [r for r in decisions if r["reason"] in ["ACCEPT_TA", "ACCEPT_AA"]]
    accepted_ids = [identity(r) for r in accepted]
    if len(accepted_ids) != len(set(accepted_ids)):
        failures.append("physical measurement entered graph twice")
    expected_duplicates = Counter(identity(r) for r in delivery if r["action"] == "duplicate")
    actual_duplicates = Counter(identity(r) for r in decisions if r["reason"] == "DUPLICATE")
    if expected_duplicates != actual_duplicates:
        failures.append("duplicate rejection identities/counts differ")
    expired = {identity(r) for r in delivery if r["action"] == "expired_delay"}
    expired_rejected = {identity(r) for r in decisions if r["reason"] == "WINDOW_EXPIRED"}
    if not expired <= expired_rejected:
        failures.append("delayed expired measurements were not explicitly rejected")
    dropped = {
        identity(r) for r in delivery if r["action"] in ["outage_drop", "anchor_outage_drop"]
    }
    if dropped & set(accepted_ids):
        failures.append("intentionally withheld measurement entered graph")
    if any(int(r["reset"]) or int(r["fallback"]) for r in graph):
        failures.append("reset/fallback during fault trial")
    if any(int(r["session"]) != 1 for r in clock):
        raise ValueError("combined clock reset and transport fault audit not implemented")
    start_wall = int(clock[0]["scheduled_wall_ns"])
    start_stamp = float(clock[0]["stamp"])
    original = {
        identity(r): r
        for r in delivery
        if r["action"] not in ["duplicate", "outage_drop", "anchor_outage_drop"]
    }
    rejection_reasons = {
        r["reason"]
        for r in decisions
        if r["reason"]
        not in ["ACCEPT_TA", "ACCEPT_AA", "ORACLE_AA_CHECK", "CLOCK_RESET", "SHUTDOWN_PENDING"]
    }

    anchor_outages = []
    anchor_rows = []
    if profile.get("anchor_outages"):
        for row in delivery:
            if row["action"] == "duplicate":
                continue
            elapsed = float(row["source_elapsed_s"])
            endpoints = {int(row[k]) for k in ["a", "b"] if int(row[k + "_type"]) == 1}
            expected_drop = any(
                f["begin_s"] <= elapsed < f["end_s"] and f["anchor_id"] in endpoints
                for f in profile["anchor_outages"]
            )
            full_outage = any(a <= elapsed < b for a, b in profile.get("outages", []))
            expected_action = (
                "outage_drop" if full_outage else "anchor_outage_drop" if expected_drop else None
            )
            if expected_action and row["action"] != expected_action:
                failures.append("anchor outage missed selected physical measurement")
            if not expected_action and row["action"] in ["outage_drop", "anchor_outage_drop"]:
                failures.append("anchor outage incorrectly dropped unrelated link")
            anchor_rows.append((elapsed, endpoints))
    for fault in profile.get("anchor_outages", []):
        selected = [
            endpoints
            for elapsed, endpoints in anchor_rows
            if fault["begin_s"] <= elapsed < fault["end_s"]
        ]
        output_frames = sum(
            fault["begin_s"] <= float(row["stamp"]) - start_stamp < fault["end_s"]
            for row in trajectory
        )
        expected_frames = round((fault["end_s"] - fault["begin_s"]) * 10)
        if output_frames < expected_frames * 0.99:
            failures.append("LIO publication gap during single-anchor outage")
        affected = sum(fault["anchor_id"] in endpoints for endpoints in selected)
        if not affected:
            failures.append("anchor outage had no affected observations")
        recovery = []
        continued = 0
        for decision in accepted:
            if decision["reason"] != "ACCEPT_TA" or identity(decision) not in original:
                continue
            delivered = original[identity(decision)]
            endpoints = {int(delivered[k]) for k in ["a", "b"] if int(delivered[k + "_type"]) == 1}
            elapsed = float(delivered["source_elapsed_s"])
            if fault["begin_s"] <= elapsed < fault["end_s"] and fault["anchor_id"] not in endpoints:
                continued += 1
            if elapsed >= fault["end_s"] and fault["anchor_id"] in endpoints:
                recovery.append(
                    (int(decision["decision_wall_ns"]) - start_wall) * 1e-9 - fault["end_s"]
                )
        if not continued:
            failures.append("no other-anchor TA accepted during single-anchor outage")
        if not recovery:
            failures.append("failed anchor TA never restored after single-anchor outage")
        anchor_outages.append(
            {
                "anchor_id": fault["anchor_id"],
                "begin_s": fault["begin_s"],
                "end_s": fault["end_s"],
                "affected_observations": affected,
                "online_frames_during_outage": output_frames,
                "other_link_observations": len(selected) - affected,
                "other_anchor_ta_accepted_during_outage": continued,
                "failed_anchor_first_ta_recovery_wall_s": min(recovery) if recovery else None,
            }
        )

    def reports_no_accepted_uwb(row):
        reason = row.get("state_reason", "")
        prefix = "UWB_REJECTED_LIO_CONTINUES:"
        return reason == "UWB_ABSENT_LIO_CONTINUES" or (
            reason.startswith(prefix) and reason[len(prefix) :] in rejection_reasons
        )

    outages = []
    for begin, end in profile.get("outages", []):
        interior = [r for r in trajectory if begin + 0.5 <= float(r["stamp"]) - start_stamp < end]
        expected_frames = max(0, round((end - begin - 0.5) * 10))
        if len(interior) < expected_frames * 0.99:
            failures.append(f"LIO publication gap during outage {begin}-{end}")
        if not interior or any(not reports_no_accepted_uwb(r) for r in interior):
            failures.append(f"absence reason not reported during outage {begin}-{end}")
        recovery = [
            r
            for r in accepted
            if r["reason"] == "ACCEPT_TA"
            and identity(r) in original
            and float(original[identity(r)]["source_elapsed_s"]) >= end
        ]
        latency = min(
            ((int(r["decision_wall_ns"]) - start_wall) * 1e-9 - end for r in recovery), default=None
        )
        if latency is None or latency > 0.5:
            failures.append(f"fresh TA not restored within 0.5s after outage {begin}-{end}")
        outages.append(
            {
                "begin_s": begin,
                "end_s": end,
                "interior_online_frames": len(interior),
                "fresh_ta_recovery_wall_s": latency,
            }
        )
    nlos_summary = None
    if profile.get("nlos_bias"):
        model = DeliverySchedule(profile)
        biases = []
        expected_ranges = {}
        for row in delivery:
            endpoints = [int(row[k]) for k in ["a", "b"] if int(row[k + "_type"]) == 1]
            bias = model.range_bias(float(row["source_elapsed_s"]), endpoints)
            expected = float(row["base_range_m"]) + bias
            values = [expected, float(row["injected_bias_m"]), float(row["range_after_stress_m"])]
            if (
                not all(math.isfinite(v) for v in values)
                or abs(values[1] - bias) > 1e-9
                or abs(values[2] - expected) > 1e-9
            ):
                failures.append("NLOS measurement transform differs from declared model")
            expected_status = 2 if bias else int(row["base_propagation_status"])
            if int(row["propagation_status"]) != expected_status:
                failures.append("NLOS propagation flag mismatch")
            before_variance, after_variance = float(row["base_variance_m2"]), float(
                row["variance_after_stress_m2"]
            )
            if not math.isfinite(after_variance) or after_variance != before_variance:
                failures.append("NLOS stress changed baseline variance")
            if row["action"] not in ["duplicate", "outage_drop", "anchor_outage_drop"]:
                expected_ranges[identity(row)] = expected
                if bias:
                    biases.append(bias)
        for decision in decisions:
            if identity(decision) in expected_ranges and (
                not math.isfinite(float(decision["range"]))
                or abs(float(decision["range"]) - expected_ranges[identity(decision)]) > 1e-9
            ):
                failures.append("estimator NLOS measurement differs from delivered value")
        if not biases:
            failures.append("no NLOS-stressed measurements delivered")
        nlos_summary = {
            "model": "stationary_lognormal_AR1_zero_order_hold_per_link",
            "stressed_physical_measurements": len(biases),
            "bias_range_m": [min(biases), max(biases)] if biases else None,
            "bias_values_in_memory": model.bias_values,
            "interpretation": "Explicit additive pressure bias; nominal Gaussian measurement variance unchanged. Not an empirically calibrated NLOS model.",
        }
    return {
        "result": "FAIL" if failures else "PASS",
        "failures": failures,
        "declared_actions": dict(Counter(r["action"] for r in delivery)),
        "duplicate_rejections": sum(actual_duplicates.values()),
        "expired_delayed_rejections": len(expired & expired_rejected),
        "delivery_queue_peak": runtime["delivery_queue_peak"],
        "outages": outages,
        "anchor_outages": anchor_outages,
        "nlos_bias": nlos_summary,
        "scope": "transport/unique ingestion/LIO continuity only; navigation accuracy and covariance evaluated separately",
    }
