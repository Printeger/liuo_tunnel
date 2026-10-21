#!/usr/bin/env python3
"""Sequential GPU nominal campaign. Stop and retain evidence on any physical failure."""
import argparse
import json
from pathlib import Path
import subprocess
import signal
from datetime import datetime, timezone

from campaign_audit import campaign
from compare_runs import compare
from generate_known_maps import generate
from long_run_audit import audit

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", nargs="+", type=int, default=list(range(10)))
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
        (directory / "campaign_progress.json").write_text(json.dumps(record, indent=2) + "\n")

    def execute(seed, mode, options):
        log = directory / f"seed_{seed}_{mode}.txt"
        command = [str(ROOT / "tools/run.sh"), "--formal", *options]
        entry = {"seed": seed, "mode": mode, "command": command, "console": str(log)}
        record["runs"].append(entry)
        save()
        with log.open("w") as stream:
            owner = subprocess.Popen(
                command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True
            )
            entry["owner_pid"] = owner.pid
            save()
            try:
                returncode = owner.wait()
            except KeyboardInterrupt:
                # The run owner stops only its own children and finalizes them.
                # Do not let subprocess.run kill it before that cleanup.
                owner.send_signal(signal.SIGINT)
                owner.wait(timeout=60)
                raise
        entry["exit_code"] = returncode
        # The run owner prints its allocated directory first, including failures.
        run = Path(log.read_text().splitlines()[0])
        if not run.is_relative_to(ROOT / "log"):
            raise RuntimeError(f"unexpected run directory in {log}")
        entry["run"] = str(run)
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
        for seed in args.seeds:
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
