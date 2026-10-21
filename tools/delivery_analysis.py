"""Audit declared transport faults separately from navigation accuracy."""

from collections import Counter


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
    dropped = {identity(r) for r in delivery if r["action"] == "outage_drop"}
    if dropped & set(accepted_ids):
        failures.append("intentionally withheld measurement entered graph")
    if any(int(r["reset"]) or int(r["fallback"]) for r in graph):
        failures.append("reset/fallback during fault trial")
    if any(int(r["session"]) != 1 for r in clock):
        raise ValueError("combined clock reset and transport fault audit not implemented")
    start_wall = int(clock[0]["scheduled_wall_ns"])
    start_stamp = float(clock[0]["stamp"])
    original = {identity(r): r for r in delivery if r["action"] not in ["duplicate", "outage_drop"]}
    rejection_reasons = {
        r["reason"]
        for r in decisions
        if r["reason"]
        not in ["ACCEPT_TA", "ACCEPT_AA", "ORACLE_AA_CHECK", "CLOCK_RESET", "SHUTDOWN_PENDING"]
    }

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
    return {
        "result": "FAIL" if failures else "PASS",
        "failures": failures,
        "declared_actions": dict(Counter(r["action"] for r in delivery)),
        "duplicate_rejections": sum(actual_duplicates.values()),
        "expired_delayed_rejections": len(expired & expired_rejected),
        "delivery_queue_peak": runtime["delivery_queue_peak"],
        "outages": outages,
        "scope": "transport/unique ingestion/LIO continuity only; navigation accuracy and covariance evaluated separately",
    }
