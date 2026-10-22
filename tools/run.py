#!/usr/bin/env python3
"""Single owner: allocate, snapshot, start owned processes, retain failure, finalize."""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import run_directory as rd
from generate_known_maps import LAYOUT, validate_deployment

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def watchdog_failure(elapsed, wall_budget, idle_seconds, idle_budget):
    if elapsed > wall_budget:
        return "WALL_TIMEOUT: total run wall budget exceeded"
    for module, idle in idle_seconds.items():
        if idle > idle_budget:
            return f"NO_PROGRESS_TIMEOUT: {module} output unchanged for {idle:.1f}s"
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--delivery-profile",
        type=Path,
        help="explicit frozen-replay UWB transport fault profile",
    )
    p.add_argument("--duration", type=float, default=45)
    p.add_argument(
        "--sim-start",
        type=float,
        nargs=4,
        metavar=("X", "Y", "Z", "YAW"),
        help="simulation-only initial position [m] and body yaw [rad]; never estimator input",
    )
    p.add_argument(
        "--wall-timeout",
        type=float,
        help="owned runtime wall seconds; default max(120, duration/rate*1.5+60)",
    )
    p.add_argument(
        "--no-progress-timeout",
        type=float,
        default=30,
        help="maximum wall seconds without simulation or odometry output growth",
    )
    p.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="explicit simulation/wall reference rate; never automatic fallback",
    )
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--reset-at", type=float, default=-1.0)
    p.add_argument(
        "--return-trip",
        action="store_true",
        help="explicit straight-rail outbound and backward return; use duration >=1070s",
    )
    p.add_argument("--config", type=Path, default=ROOT / "packages/liuo_tunnel/config/phase1.json")
    p.add_argument("--local-calibration", action="store_true", help="simulation-only closed 3D calibration maneuver")
    p.add_argument("--installation", type=Path, help="coordinate-free first-run installation measurements")
    p.add_argument("--map", type=Path, help="external oracle_map or surveyed_map JSON")
    p.add_argument(
        "--replay",
        type=Path,
        help="completed source run with verified original MCAP capture",
    )
    p.add_argument(
        "--debug-throw",
        action="store_true",
        help="diagnostic gdb backtrace at first estimator exception",
    )
    p.add_argument("--formal", action="store_true")
    p.add_argument("--diagnostic-capture", action="store_true")
    p.add_argument(
        "--capture",
        action="store_true",
        help="freeze all original sensor messages once in MCAP",
    )
    p.add_argument("--inject-failure", choices=["simulation", "odometry"])
    args = p.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        p.error("duration must be finite and positive")
    if not math.isfinite(args.no_progress_timeout) or args.no_progress_timeout <= 0:
        p.error("no-progress timeout must be finite and positive")
    if args.wall_timeout is not None and (
        not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0
    ):
        p.error("wall timeout must be finite and positive")
    if args.delivery_profile and not args.replay:
        p.error("delivery profile requires --replay")
    if args.replay and (args.capture or args.rate != 1.0):
        p.error("frozen replay requires 1x and cannot recapture")
    if args.local_calibration and (args.replay or args.return_trip):
        p.error("--local-calibration is a live-only alternative to return trip")
    if args.local_calibration:
        start = args.sim_start or [0, 0, 1.2, 0]
        if abs(start[1]) + .6 >= 2 or start[2] + 1.2 >= 4.5:
            p.error("local maneuver exceeds declared simulation safety envelope")
    if args.sim_start is not None:
        if args.replay or args.return_trip or not all(math.isfinite(v) for v in args.sim_start):
            p.error("--sim-start requires finite live outbound simulation parameters")
        if not (
            0 <= args.sim_start[0] <= 2300
            and abs(args.sim_start[1]) < 2
            and 0.3 < args.sim_start[2] < 4.5
        ):
            p.error("simulation starts require interior positions")
    if args.return_trip and args.replay:
        p.error("--return-trip changes live simulation only; replay preserves its source motion")
    if args.debug_throw and args.formal:
        p.error("debugger runs are diagnostic, not formal timing evidence")
    if not 0 < args.rate <= 1:
        p.error("--rate must be in (0,1]")
    run = rd.resolve_run_directory(entrypoint="rail_sim", scenario="rail_tunnel_straight_v1")
    if args.formal:
        path = run / "metadata/run_manifest.json"
        manifest = json.loads(path.read_text())
        manifest.update(run_class="validation", retention_class="protected")
        rd._atomic_write_json(path, manifest)
    print(run, flush=True)
    children = {}
    handles = []
    failure = None
    lifecycle = "failed"
    try:
        if not rd._git_value("rev-parse", "HEAD") or rd._git_value("status", "--porcelain"):
            raise RuntimeError(
                "LIVE_BLOCKED_BY_DIRTY_WORKTREE: commit project changes before live evidence"
            )
        config = run / "metadata/config/phase1.json"
        shutil.copyfile(args.config, config)
        effective_config = json.loads(config.read_text())
        effective_config["logging"]["raw_bag"] = args.capture
        config.write_text(json.dumps(effective_config, indent=2) + "\n")
        rd.register_config_snapshot(run, config)
        frozen = None
        if args.replay:
            source = args.replay.resolve(strict=True)
            source_manifest = json.loads((source / "metadata/run_manifest.json").read_text())
            source_audit = json.loads(
                (source / "metadata/manifests/observation_audit.json").read_text()
            )
            source_runtime = json.loads((source / "export/simulation/runtime.json").read_text())
            if (
                source_manifest["lifecycle"] != "completed"
                or source_audit["violations"]
                or source_audit.get("duplicate_uwb")
            ):
                raise RuntimeError("FROZEN_SOURCE_INVALID: incomplete lifecycle or raw violations")
            if source_audit["counts"] != source_runtime["published_counts"]:
                raise RuntimeError("FROZEN_SOURCE_INVALID: source lost raw input")
            hashes = json.loads((source / "metadata/manifests/captured_inputs.json").read_text())
            raw_hashes = {k: v for k, v in hashes.items() if k.startswith("export/capture/raw/")}
            if not any(k.endswith(".mcap") for k in raw_hashes):
                raise RuntimeError("FROZEN_SOURCE_INVALID: missing MCAP")
            for name, expected in raw_hashes.items():
                path = (source / name).resolve(strict=True)
                if not path.is_relative_to(source) or digest(path) != expected:
                    raise RuntimeError("FROZEN_SOURCE_INVALID: capture checksum/path " + name)
            source_config = json.loads((source / "metadata/config/phase1.json").read_text())
            for key in ["sensors", "scan_model"]:
                if effective_config[key] != source_config[key]:
                    raise RuntimeError("FROZEN_SENSOR_CONTRACT_MISMATCH: " + key)
            source_scenario = json.loads((source / "metadata/config/scenario.json").read_text())
            if args.formal and source_scenario.get("anchor_layout") != LAYOUT:
                raise RuntimeError(
                    "DEPLOYMENT_LAYOUT_INVALID: dense historical inputs are diagnostic only"
                )
            args.duration = source_runtime["simulation_s"]
            args.seed = source_scenario["seed"]
            frozen = {
                "source_run": str(source),
                "source_run_id": source.name,
                "capture_sha256": raw_hashes,
                "raw_sha256": source_audit["raw_sha256"],
                "counts": source_audit["counts"],
                "source_scenario": source_scenario,
            }
            rd.write_subordinate_manifest(run, "frozen_input", frozen)
            manifest_path = run / "metadata/run_manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest.update(retention_class="protected", retained_input_run_ids=[source.name])
            rd._atomic_write_json(manifest_path, manifest)
            shutil.copyfile(
                source / "export/simulation/truth.csv",
                run / "export/simulation/truth.csv",
            )
        selected_backend = effective_config["odometry"]["backend"]
        map_path = None
        installation_path = None
        if effective_config["uwb"]["mode"] == "self_calibrate":
            if args.map is not None or args.installation is None:
                raise RuntimeError("self_calibrate requires --installation and forbids --map")
            installation_path = run / "metadata/config/installation.json"
            shutil.copyfile(args.installation, installation_path)
            rd.register_config_snapshot(run, installation_path)
        if effective_config["uwb"]["mode"] == "fuse":
            if args.map is None:
                raise RuntimeError("MAP_MISSING: fused mode requires --map")
            map_path = run / "metadata/config/map.json"
            shutil.copyfile(args.map, map_path)
            rd.register_config_snapshot(run, map_path)
            map_data = json.loads(map_path.read_text())
            validate_deployment(map_data["anchors"])
            declared_layout = map_data.get("simulation_anchor_layout")
            physical_layout = source_scenario["anchor_layout"] if args.replay else LAYOUT
            if declared_layout is not None and declared_layout != physical_layout:
                raise RuntimeError(
                    "MAP_SCENE_LAYOUT_MISMATCH: simulation fixture belongs to another layout"
                )
        if args.formal and (selected_backend != "gpu" or args.rate != 1.0):
            raise RuntimeError("formal validation requires GPU backend and requested rate 1.0")
        if selected_backend == "gpu":
            preflight = subprocess.run(["nvidia-smi", "-q"], capture_output=True, text=True)
            (run / "metadata/gpu_preflight.txt").write_text(preflight.stdout + preflight.stderr)
            driver = ctypes.CDLL("libcuda.so.1")
            init_status = driver.cuInit(0)
            count = ctypes.c_int()
            count_status = driver.cuDeviceGetCount(ctypes.byref(count))
            rd._atomic_write_json(
                run / "metadata/gpu_driver.json",
                {
                    "cuInit": init_status,
                    "cuDeviceGetCount": count_status,
                    "device_count": count.value,
                },
            )
            if preflight.returncode or init_status or count_status or count.value < 1:
                raise RuntimeError("GPU_NOT_READY: driver/device preflight failed; no CPU fallback")
        rd._atomic_write_json(
            run / "metadata/config/scenario.json",
            {
                "scene": "rail_tunnel_straight_v1",
                "length_m": 2400,
                "width_m": 6,
                "height_m": 5,
                "gauge_m": 1.435,
                "anchor_layout": LAYOUT,
                "duration_s": args.duration,
                "requested_rate": args.rate,
                "seed": args.seed,
                "scan_model": "snapshot",
                "reset_at_s": args.reset_at,
                "return_trip": args.return_trip,
                "motion": ("closed_local_3d_30s" if args.local_calibration else ("outbound_and_reverse_return" if args.return_trip else "outbound")),
                "simulation_start_xyz_yaw": args.sim_start or [0, 0, 1.2, 0],
                "clock_start_s": 1,
                "dt_s": 0.005,
                "exchange_dt_s": 0.004,
                "lidar_samples": [512, 40],
                "lidar_range_m": 40,
                "world_voxel_m": 0.1,
            },
        )
        if frozen is not None:
            rd._atomic_write_json(run / "metadata/config/scenario.json", frozen["source_scenario"])
        rd.register_config_snapshot(run, run / "metadata/config/scenario.json")
        env = os.environ.copy()
        env.update(
            LIUO_RUN_DIR=str(run),
            ROS_LOG_DIR=str(run / "runtime/ros"),
            ROS_DOMAIN_ID=str(100 + os.getpid() % 100),
            ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
        )
        env["LIUO_DIAGNOSTIC_CAPTURE"] = "1" if args.diagnostic_capture else "0"
        binaries = {
            name: ROOT / f"install/{pkg}/lib/{pkg}/{exe}"
            for name, pkg, exe in [
                ("odometry", "liuo_tunnel", "liuo_tunnel_node"),
                ("simulation", "liuo_tunnel_sim", "liuo_rail_sim"),
            ]
        }
        identity = {
            "source_iap_commit": "fbccae16802402d7300bb817d2627f86e6a495df",
            "config_sha256": digest(config),
            "seed": args.seed,
            "scan_model": "snapshot",
            "binaries": {},
            "environment": {
                k: env.get(k)
                for k in [
                    "AMENT_PREFIX_PATH",
                    "LD_LIBRARY_PATH",
                    "ROS_DOMAIN_ID",
                    "ROS_LOG_DIR",
                ]
            },
            "commands": {},
        }
        for name, binary in binaries.items():
            linked = subprocess.check_output(["ldd", str(binary)], text=True)
            if "/iap/" in linked or "/ws_iap/" in linked:
                raise RuntimeError("forbidden runtime dependency: " + linked)
            identity["binaries"][name] = {
                "path": str(binary),
                "sha256": digest(binary),
                "ldd": linked,
            }
            dependencies = identity.setdefault("shared_libraries", {})
            for line in linked.splitlines():
                parts = line.split("=>")
                if len(parts) == 2:
                    library = Path(parts[1].strip().split()[0])
                    if (
                        library.is_absolute()
                        and library.is_file()
                        and str(library) not in dependencies
                    ):
                        dependencies[str(library)] = {
                            "sha256": digest(library),
                            "resolved": str(library.resolve()),
                        }
        caches = {}
        for package in ["liuo_tunnel", "liuo_tunnel_sim"]:
            cache = ROOT / "build" / package / "CMakeCache.txt"
            caches[package] = {
                "sha256": digest(cache),
                "selected_options": [
                    l
                    for l in cache.read_text().splitlines()
                    if l.startswith(
                        (
                            "CMAKE_BUILD_TYPE:",
                            "CMAKE_CXX_COMPILER:",
                            "GTSAM_DIR:",
                            "gtsam_points_DIR:",
                            "CUDAToolkit_",
                        )
                    )
                ],
            }
        identity["build_caches"] = caches
        commands = {
            name: [
                str(binary),
                "--ros-args",
                "-p",
                "config_file:=" + str(config),
                "-p",
                "use_sim_time:=true",
            ]
            for name, binary in binaries.items()
        }
        if installation_path is not None:
            commands["odometry"] += ["-p", "installation_file:=" + str(installation_path)]
            identity["installation_sha256"] = digest(installation_path)
        if map_path is not None:
            commands["odometry"] += ["-p", "map_file:=" + str(map_path)]
            identity["map_sha256"] = digest(map_path)
        commands["simulation"] += [
            "-p",
            f"duration_s:={args.duration}",
            "-p",
            f"seed:={args.seed}",
            "-p",
            f"reset_at_s:={args.reset_at}",
            "-p",
            f"return_trip:={str(args.return_trip).lower()}",
            "-p",
            f"local_calibration:={str(args.local_calibration).lower()}",
            "-p",
            f"rate:={args.rate}",
            "-p",
            f"capture:={str(args.capture).lower()}",
        ]
        if args.sim_start is not None:
            for key, value in zip(["start_x", "start_y", "start_z", "start_yaw"], args.sim_start):
                commands["simulation"] += ["-p", f"{key}:={value}"]
        if frozen is not None:
            commands["simulation"] = [
                sys.executable,
                str(ROOT / "tools/replay.py"),
                "--source-run",
                frozen["source_run"],
            ]
            identity["frozen_input"] = frozen
        if args.delivery_profile:
            from delivery_schedule import DeliverySchedule

            profile = json.loads(args.delivery_profile.read_text())
            DeliverySchedule(profile)
            profile_path = run / "metadata/config/delivery.json"
            profile_path.write_text(json.dumps(profile, indent=2) + "\n")
            rd.register_config_snapshot(run, profile_path)
            commands["simulation"] += ["--delivery-profile", str(profile_path)]
            identity["delivery_profile_sha256"] = digest(profile_path)
        if args.debug_throw:
            commands["odometry"] = [
                "gdb",
                "--batch",
                "-x",
                str(ROOT / "tools/solver_exception.gdb"),
                "--args",
            ] + commands["odometry"]
        commands["observer"] = [sys.executable, str(ROOT / "tools/observe.py")]
        for name in ["observer", "odometry", "simulation"]:
            f = open(run / f"runtime/{name}_console.log", "w")
            handles.append(f)
            children[name] = subprocess.Popen(
                commands[name],
                env=env,
                stdout=f,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            identity["commands"][name] = commands[name]
            if name != "simulation":
                time.sleep(1)
        rd.write_subordinate_manifest(run, "input_identity", identity)
        start = time.monotonic()
        wall_budget = args.wall_timeout or max(120, args.duration / args.rate * 1.5 + 60)
        progress = {
            "simulation": [run / "export/simulation/wall_clock.csv", -1, start],
            "odometry": [run / "profiling/odometry.csv", -1, start],
        }
        rd.write_subordinate_manifest(
            run,
            "watchdog",
            {
                "wall_timeout_s": wall_budget,
                "no_progress_timeout_s": args.no_progress_timeout,
                "progress_files": {k: str(v[0].relative_to(run)) for k, v in progress.items()},
            },
        )
        injected = False
        runtime_saved = False
        while True:
            for name, child in children.items():
                code = child.poll()
                if code is not None and (name != "simulation" or code != 0):
                    raise RuntimeError(
                        f"{name} exited unexpectedly: {code}; see runtime/{name}_console.log"
                    )
            if children["simulation"].poll() == 0:
                break
            if not runtime_saved and time.monotonic() - start > 3:
                runtime = {}
                for name, child in children.items():
                    maps = Path(f"/proc/{child.pid}/maps").read_text()
                    if "/iap/" in maps or "/ws_iap/" in maps:
                        raise RuntimeError("forbidden loaded runtime library")
                    runtime[name] = {
                        "pid": child.pid,
                        "run_dir": str(run),
                        "maps": maps,
                    }
                rd.write_subordinate_manifest(run, "runtime_identity", runtime)
                runtime_saved = True
            if args.inject_failure and not injected and time.monotonic() - start > 5:
                children[args.inject_failure].kill()
                injected = True
            now = time.monotonic()
            for state in progress.values():
                size = state[0].stat().st_size if state[0].exists() else 0
                if size != state[1]:
                    state[1], state[2] = size, now
            reason = watchdog_failure(
                now - start,
                wall_budget,
                {k: now - v[2] for k, v in progress.items()},
                args.no_progress_timeout,
            )
            if reason:
                raise RuntimeError(reason)
            time.sleep(0.1)
        time.sleep(2)
        for name in ["odometry", "observer"]:
            if children[name].poll() is not None:
                raise RuntimeError(
                    f"{name} exited during sensor drain: {children[name].returncode}"
                )
        lifecycle = "completed"
    except KeyboardInterrupt:
        lifecycle = "interrupted"
        failure = "user interrupt"
    except Exception as exc:
        failure = str(exc)
        print(failure, file=sys.stderr, flush=True)
    finally:
        for name, child in children.items():
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        for name, child in children.items():
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.kill()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass  # Record failure below; never wait indefinitely on an owned child.
                failure = (
                    failure + "; " if failure else ""
                ) + f"{name} did not stop after owned SIGINT"
                lifecycle = "failed"
        for f in handles:
            f.close()
        for name, child in children.items():
            if lifecycle == "completed" and child.returncode != 0:
                failure = f"{name} final exit code {child.returncode}"
                lifecycle = "failed"
        refs = [run / "metadata/manifests/observation_audit.json"]
        for ref in refs:
            if ref.exists():
                rd.register_subordinate_manifest(run, ref)
        captures = {
            str(f.relative_to(run)): digest(f)
            for f in (run / "export/capture").rglob("*")
            if f.is_file()
        }
        rd.write_subordinate_manifest(run, "captured_inputs", captures)
        if failure:
            rd.write_subordinate_manifest(
                run,
                "input_failure",
                {
                    "reason": failure,
                    "exit_codes": {k: v.returncode for k, v in children.items()},
                },
            )
        rd.finalize_run(run, lifecycle=lifecycle)
    if lifecycle == "completed":
        try:
            result = subprocess.run(
                [sys.executable, str(ROOT / "tools/evaluate.py"), str(run)], timeout=120
            )
        except subprocess.TimeoutExpired:
            rd.write_subordinate_manifest(
                run,
                "evaluation_failure",
                {"reason": "EVALUATION_WALL_TIMEOUT", "timeout_s": 120},
            )
            result = subprocess.CompletedProcess([], 1)
        manifest_path = run / "metadata/run_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["acceptance"] = "pass" if result.returncode == 0 else "fail"
        rd._atomic_write_json(manifest_path, manifest)
        return result.returncode
    return 1


if __name__ == "__main__":
    sys.exit(main())
