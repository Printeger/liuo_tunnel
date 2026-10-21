#!/usr/bin/env python3
"""Sequential GPU nominal campaign. Stop and retain evidence on any physical failure."""
import argparse
import json
import hashlib
import shutil
from collections import Counter
from pathlib import Path
import subprocess
import signal
from datetime import datetime, timezone

from campaign_audit import campaign
from compare_runs import compare, estimator_settings
from generate_known_maps import generate
from long_run_audit import audit

ROOT = Path(__file__).resolve().parents[1]


DISK_RESERVE_BYTES = 2 * 1024**3


def wait_owned_run(owner):
    """Ask only this Popen owner to finalize if the evidence reserve is exhausted."""
    while True:
        try:
            return owner.wait(timeout=5), None
        except subprocess.TimeoutExpired:
            free = shutil.disk_usage(ROOT).free
            if free < DISK_RESERVE_BYTES:
                owner.send_signal(signal.SIGINT)
                return owner.wait(timeout=60), f"DISK_RESERVE_BELOW_2_GIB: {free} bytes available"


def validate_resume_groups(entries, seeds):
    groups = {}
    for entry in entries:
        if entry["seed"] not in seeds:
            raise ValueError("completed run seed is outside requested campaign")
        groups.setdefault(entry["seed"], []).append(entry)
    expected = Counter(["record_only", "oracle_map", "surveyed_map"])
    for seed, group in groups.items():
        if Counter(e["mode"] for e in group) != expected:
            raise ValueError(f"seed {seed}: resume requires exactly one complete three-mode group")
    return groups


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", nargs="+", type=int, default=list(range(10)))
    parser.add_argument(
        "--completed-runs",
        nargs="+",
        type=Path,
        default=[],
        help="explicit completed triplets; re-audit before reuse, never choose best attempts",
    )
    args = parser.parse_args()
    if len(set(args.seeds)) != len(args.seeds) or any(s not in range(10) for s in args.seeds):
        parser.error("provide unique seeds in 0–9")
    directory = (
        ROOT / "log" / ("campaign_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
    )
    directory.mkdir()
    config = ROOT / "packages/liuo_tunnel/config/phase2.json"
    off = json.loads(config.read_text())
    off["uwb"]["mode"] = "record_only"
    off_path = directory / "matched_off.json"
    off_path.write_text(json.dumps(off, indent=2) + "\n")
    record = {
        "scope": "nominal_full_outbound_snapshot",
        "seeds": args.seeds,
        "duration_s": 540,
        "runs": [],
        "status": "active",
        "failure": None,
    }

    def save():
        temporary = directory / "campaign_progress.json.tmp"
        temporary.write_text(json.dumps(record, indent=2) + "\n")
        temporary.replace(directory / "campaign_progress.json")

    def execute(seed, mode, options):
        log = directory / f"seed_{seed}_{mode}.txt"
        command = [str(ROOT / "tools/run.sh"), "--formal", *options]
        entry = {"seed": seed, "mode": mode, "command": command, "console": str(log)}
        record["runs"].append(entry)
        save()
        # Estimated write allowance, backed by a live reserve monitor. This is
        # not a guarantee about external writers or future compression ratios.
        allowance = int((1.5 if "--capture" in options else 0.25) * 1024**3)
        free = shutil.disk_usage(ROOT).free
        entry["disk_preflight"] = {
            "free_bytes": free,
            "write_allowance_bytes": allowance,
            "reserve_bytes": DISK_RESERVE_BYTES,
        }
        save()
        if free < DISK_RESERVE_BYTES + allowance:
            raise RuntimeError(f"INSUFFICIENT_EVIDENCE_SPACE: {free} bytes available")
        with log.open("w") as stream:
            owner = subprocess.Popen(
                command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True
            )
            entry["owner_pid"] = owner.pid
            save()
            try:
                returncode, stop_reason = wait_owned_run(owner)
            except KeyboardInterrupt:
                # The run owner stops only its own children and finalizes them.
                # Do not let subprocess.run kill it before that cleanup.
                owner.send_signal(signal.SIGINT)
                owner.wait(timeout=60)
                raise
        entry["exit_code"] = returncode
        entry["stop_reason"] = stop_reason
        # The run owner prints its allocated directory first, including failures.
        run = Path(log.read_text().splitlines()[0])
        if not run.is_relative_to(ROOT / "log"):
            raise RuntimeError(f"unexpected run directory in {log}")
        entry["run"] = str(run)
        if stop_reason:
            cause = {
                "reason": stop_reason,
                "campaign_directory": str(directory),
                "owner_pid": owner.pid,
                "observed_exit_code": returncode,
                "origin": "campaign disk reserve monitor",
            }
            stop_path = run / "metadata/manifests/campaign_stop.json"
            stop_path.write_text(json.dumps(cause, indent=2) + "\n")
            save()
            raise RuntimeError(stop_reason)
        evidence = audit(run)
        entry["audit"] = evidence
        save()
        print(
            json.dumps({"seed": seed, "mode": mode, "run": str(run), "audit": evidence["result"]}),
            flush=True,
        )
        if evidence["result"] != "PASS":
            raise RuntimeError(f'{run.name}: {evidence["failures"]}')
        # Fused per-run reports retain their explicit pending-stage failure.
        # Its presence is disclosed by audit(), never stage completion here.
        if returncode not in (0, 1):
            raise RuntimeError(f"{run.name}: unexpected owner exit {returncode}")
        return run

    print(directory, flush=True)
    save()
    try:
        base = json.loads(config.read_text())
        with (ROOT / "install/liuo_tunnel/lib/liuo_tunnel/liuo_tunnel_node").open("rb") as stream:
            binary_sha = hashlib.file_digest(stream, "sha256").hexdigest()
        for supplied in args.completed_runs:
            run = supplied.resolve(strict=True)
            load = lambda name: json.loads((run / name).read_text())
            cfg = load("metadata/config/phase1.json")
            seed = load("metadata/config/scenario.json")["seed"]
            mode = cfg["uwb"]["mode"]
            if mode == "fuse":
                known_map = load("metadata/config/map.json")
                mode = known_map["mode"]
                if cfg["uwb"] != base["uwb"]:
                    raise ValueError("completed run fused UWB settings differ")
                if mode == "surveyed_map" and known_map.get("survey_seed") != 67129 + 1000 * seed:
                    raise ValueError("completed run survey realization differs from campaign")
            if estimator_settings(cfg) != estimator_settings(base):
                raise ValueError("completed run LIO settings differ")
            if (
                load("metadata/manifests/input_identity.json")["binaries"]["odometry"]["sha256"]
                != binary_sha
            ):
                raise ValueError("completed run estimator binary differs")
            evidence = audit(run)
            record["runs"].append(
                {
                    "seed": seed,
                    "mode": mode,
                    "run": str(run),
                    "reused_explicitly": True,
                    "audit": evidence,
                }
            )
            save()
            if evidence["result"] != "PASS":
                raise ValueError(f"completed run audit failed: {run.name}: {evidence['failures']}")
        groups = validate_resume_groups(record["runs"], args.seeds)
        for seed, group in groups.items():
            paired = compare([Path(e["run"]) for e in group], strict_estimator_settings=True)
            (directory / f"seed_{seed}_paired.json").write_text(json.dumps(paired, indent=2) + "\n")
        for seed in args.seeds:
            if seed in groups:
                continue
            maps = directory / f"maps_seed_{seed}"
            generate(maps, survey_seed=67129 + 1000 * seed)
            source = execute(
                seed,
                "surveyed_map",
                [
                    "--duration",
                    "540",
                    "--seed",
                    str(seed),
                    "--capture",
                    "--config",
                    str(config),
                    "--map",
                    str(maps / "surveyed_map.json"),
                ],
            )
            oracle = execute(
                seed,
                "oracle_map",
                [
                    "--replay",
                    str(source),
                    "--config",
                    str(config),
                    "--map",
                    str(maps / "oracle_map.json"),
                ],
            )
            baseline = execute(
                seed, "record_only", ["--replay", str(source), "--config", str(off_path)]
            )
            paired = compare([source, oracle, baseline], strict_estimator_settings=True)
            (directory / f"seed_{seed}_paired.json").write_text(json.dumps(paired, indent=2) + "\n")
        report = campaign([Path(r["run"]) for r in record["runs"]])
        (directory / "nominal_campaign_audit.json").write_text(json.dumps(report, indent=2) + "\n")
        record["status"] = (
            "completed_nominal_campaign" if report["result"] == "PASS" else "incomplete_campaign"
        )
        save()
        return int(report["result"] != "PASS")
    except (Exception, KeyboardInterrupt) as error:
        record["status"] = "failed_or_interrupted"
        record["failure"] = f"{type(error).__name__}: {error}"
        save()
        raise


if __name__ == "__main__":
    raise SystemExit(main())
