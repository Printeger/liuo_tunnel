# Adapted from IAP launch/_includes/run_directory.py @ fbccae168; Tunnel layout and lifecycle.
"""Resolve one collision-safe artifact directory for a canonical LIUO run."""

from __future__ import annotations

import atexit
import os
import re
import json
import shutil
import socket
import subprocess
import time
import uuid
import warnings
from datetime import datetime, timedelta, timezone
from pathlib import Path

import fcntl


_SAFE_COMPONENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_RUN_ID = re.compile(r"^\d{8}T\d{6}Z_\d{3}(?:_\d{2})?$")
_TERMINAL_LIFECYCLES = {"completed", "failed", "interrupted"}
_LIFECYCLES = {"active", *_TERMINAL_LIFECYCLES}
_SAFETY_OUTCOMES = {"executable", "hold", "not_applicable", "unknown"}
_ACTIVE_LOCKS: dict[Path, object] = {}


def _release_all_active_locks() -> None:
    for lock in list(_ACTIVE_LOCKS.values()):
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        lock.close()
    _ACTIVE_LOCKS.clear()


atexit.register(_release_all_active_locks)
_MODULES = {"rail_sim": ["odometry", "simulation", "observer"]}


def _validated_absolute(path: Path, label: str) -> Path:
    path = path.expanduser()
    if not path.is_absolute():
        raise RuntimeError(f"{label} must be an absolute path")
    path = path.resolve()
    if path == Path("/"):
        raise RuntimeError(f"{label} cannot be filesystem root")
    return path


def _component(value: str, label: str) -> str:
    value = str(value).strip()
    if not _SAFE_COMPONENT.fullmatch(value):
        raise RuntimeError(f"{label} is not a safe run-directory component: {value!r}")
    return value


def _default_run_root() -> Path:
    configured = os.environ.get("LIUO_RUN_ROOT", "").strip()
    if configured:
        return _validated_absolute(Path(configured), "LIUO_RUN_ROOT")

    # In a source or --symlink-install workspace, keep ordinary development
    # runs in the repository's ignored log tree. A copied package share also
    # has package.xml, so require source-only build files before selecting it.
    package_root = Path(__file__).resolve().parents[1]
    if (package_root / "packages").is_dir():
        return package_root / "log"

    state_root = os.environ.get("XDG_STATE_HOME", "").strip()
    state_base = Path(state_root).expanduser() if state_root else Path.home() / ".local/state"
    return _validated_absolute(state_base / "liuo_tunnel" / "log", "automatic run root")


def _new_run_id() -> str:
    now = datetime.now(timezone.utc)
    timestamp = now.strftime("%Y%m%dT%H%M%SZ")
    return f"{timestamp}_{now.microsecond // 1000:03d}"


def _parse_utc(value: object) -> datetime | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        return None
    return parsed.astimezone(timezone.utc)


def _retention_enabled(parent: Path) -> bool:
    configured = os.environ.get("LIUO_RETENTION_ENABLED")
    if configured is not None:
        return configured.strip().lower() in {"1", "true", "yes", "on"}
    package_root = Path(__file__).resolve().parents[1]
    return parent.resolve() == (package_root / "log").resolve()


def _read_run_manifest(run_dir: Path) -> dict | None:
    if run_dir.is_symlink() or not run_dir.is_dir() or not _RUN_ID.fullmatch(run_dir.name):
        return None
    manifest_path = run_dir / "metadata" / "run_manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        return None
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(manifest, dict):
        return None
    if (
        manifest.get("schema_version") != "liuo_tunnel_run_artifact_v1"
        or manifest.get("run_id") != run_dir.name
        or manifest.get("run_dir") != str(run_dir)
        or manifest.get("run_root") != str(run_dir.parent)
    ):
        return None
    return manifest


def _load_retention_manifest(run_dir: Path) -> tuple[dict, datetime] | None:
    manifest = _read_run_manifest(run_dir)
    if manifest is None:
        return None
    ended_at = _parse_utc(manifest.get("ended_at_utc"))
    if (
        manifest.get("lifecycle") not in _TERMINAL_LIFECYCLES
        or manifest.get("run_class") != "development"
        or manifest.get("retention_class") != "ordinary"
        or ended_at is None
    ):
        return None
    return manifest, ended_at


def _retained_input_run_ids(parent: Path) -> set[str]:
    """A protected campaign owner pins its evidence without rewriting old runs."""
    retained: set[str] = set()
    for child in parent.iterdir():
        manifest = _read_run_manifest(child)
        if manifest is None or manifest.get("retention_class") != "protected":
            continue
        references = manifest.get("retained_input_run_ids", [])
        if not isinstance(references, list):
            continue
        retained.update(name for name in references
                        if isinstance(name, str) and _RUN_ID.fullmatch(name))
    return retained


def _run_is_locked(run_dir: Path) -> bool:
    lock_path = run_dir / "metadata" / ".active.lock"
    if lock_path.is_symlink():
        return True
    if not lock_path.exists():
        return False
    if not lock_path.is_file():
        return True
    try:
        with lock_path.open("r+", encoding="utf-8") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
    except (OSError, BlockingIOError):
        return True
    return False


def prune_development_runs(
    parent: Path,
    *,
    now: datetime | None = None,
    keep_latest: int = 3,
    max_age: timedelta = timedelta(days=7),
) -> list[Path]:
    """Safely remove eligible old development runs directly below ``parent``."""

    parent = _validated_absolute(Path(parent), "retention root")
    if keep_latest < 0 or max_age < timedelta(0):
        raise ValueError("retention limits must be non-negative")
    if not parent.is_dir():
        return []
    current_time = (now or datetime.now(timezone.utc)).astimezone(timezone.utc)
    lock_path = parent / ".retention.lock"
    flags = os.O_RDWR | os.O_CREAT
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    removed: list[Path] = []
    try:
        lock_fd = os.open(lock_path, flags, 0o600)
    except OSError as error:
        warnings.warn(f"retention disabled: cannot safely open {lock_path}: {error}")
        return removed

    with os.fdopen(lock_fd, "r+", encoding="utf-8") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        records: list[tuple[Path, datetime]] = []
        for child in parent.iterdir():
            loaded = _load_retention_manifest(child)
            if loaded is not None:
                records.append((child, loaded[1]))
        records.sort(key=lambda item: item[0].name, reverse=True)
        protected = {path.name for path, _ in records[:keep_latest]} | _retained_input_run_ids(parent)
        cutoff = current_time - max_age

        for run_dir, ended_at in records[keep_latest:]:
            if run_dir.name in protected or ended_at >= cutoff or _run_is_locked(run_dir):
                continue
            # Re-read identity and policy under the root lock immediately before
            # the destructive transition.
            loaded = _load_retention_manifest(run_dir)
            if loaded is None or loaded[1] >= cutoff or run_dir.is_symlink():
                continue
            quarantine = parent / f".retention-trash.{run_dir.name}.{uuid.uuid4().hex}"
            try:
                os.replace(run_dir, quarantine)
                shutil.rmtree(quarantine)
                removed.append(run_dir)
            except OSError as error:
                warnings.warn(f"retention could not remove {run_dir}: {error}")
    return removed


def _update_latest(parent: Path, run_dir: Path) -> None:
    latest = parent / "latest"
    lock_path = parent / ".latest.lock"
    flags = os.O_RDWR | os.O_CREAT
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    lock_fd = os.open(lock_path, flags, 0o600)
    with os.fdopen(lock_fd, "r+", encoding="utf-8") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        if latest.exists() and not latest.is_symlink():
            raise RuntimeError(f"automatic run latest path is not a symlink: {latest}")
        if latest.is_symlink():
            current_name = Path(os.readlink(latest)).name
            if _RUN_ID.fullmatch(current_name) and current_name >= run_dir.name:
                return

        temporary = parent / f".latest.{os.getpid()}.{uuid.uuid4().hex}"
        try:
            temporary.symlink_to(run_dir.name, target_is_directory=True)
            os.replace(temporary, latest)
        finally:
            if temporary.is_symlink():
                temporary.unlink()


def _acquire_active_lock(run_dir: Path) -> None:
    lock_path = run_dir / "metadata" / ".active.lock"
    lock = lock_path.open("a", encoding="utf-8")
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BaseException:
        lock.close()
        raise
    _ACTIVE_LOCKS[run_dir] = lock


def resolve_run_directory(
    *,
    entrypoint: str,
    scenario: str = "",
) -> Path:
    """Atomically allocate one timestamped run below ``LIUO_RUN_ROOT``."""

    _component(entrypoint, "entrypoint")
    if str(scenario).strip():
        _component(scenario, "scenario")
    parent = _default_run_root()
    parent.mkdir(parents=True, exist_ok=True)
    if _retention_enabled(parent):
        try:
            prune_development_runs(parent)
        except Exception as error:
            warnings.warn(f"retention skipped after error: {error}")

    run_id = _new_run_id()
    for collision in range(100):
        suffix = "" if collision == 0 else f"_{collision:02d}"
        run_dir = parent / f"{run_id}{suffix}"
        try:
            run_dir.mkdir()
        except FileExistsError:
            continue
        run_dir = run_dir.resolve()
        _create_layout(run_dir)
        _write_run_manifest(run_dir, entrypoint=entrypoint, scenario=scenario)
        _update_latest(parent, run_dir)
        _acquire_active_lock(run_dir)
        return run_dir
    raise RuntimeError(f"could not allocate a unique run directory below {parent}")


def _create_layout(run_dir: Path) -> None:
    for category in ("runtime", "profiling", "export", "metadata"):
        (run_dir / category).mkdir(parents=False, exist_ok=True)
    (run_dir / "runtime" / "ros").mkdir(exist_ok=True)
    for namespace in (
        "odometry",
        "simulation",
        "capture",
        "analysis",
    ):
        (run_dir / "export" / namespace).mkdir(exist_ok=True)
    for namespace in ("config", "processes", "manifests"):
        (run_dir / "metadata" / namespace).mkdir(exist_ok=True)


def adopt_run_directory(requested_run_dir: str) -> Path:
    """Validate and adopt a run allocated by an outer canonical launch."""

    run_dir = _validated_absolute(Path(str(requested_run_dir).strip()), "run_dir")
    if not run_dir.is_dir():
        raise RuntimeError(f"run_dir must name an existing directory: {run_dir}")
    _create_layout(run_dir)
    return run_dir


def _git_value(*arguments: str) -> str:
    package_root = Path(__file__).resolve().parents[1]
    try:
        return subprocess.run(
            ["git", "-C", str(package_root), *arguments],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _atomic_write_json(path: Path, value: dict) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def _write_run_manifest(run_dir: Path, *, entrypoint: str, scenario: str) -> None:
    now = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    commit = _git_value("rev-parse", "HEAD")
    dirty = bool(_git_value("status", "--porcelain"))
    package_root = Path(__file__).resolve().parents[1]
    source_layout = (package_root / "packages").is_dir()
    manifest = {
        "schema_version": "liuo_tunnel_run_artifact_v1",
        "run_id": run_dir.name,
        "run_root": str(run_dir.parent),
        "run_dir": str(run_dir),
        "entrypoint": entrypoint,
        "scenario": str(scenario).strip() or None,
        "modules": _MODULES.get(entrypoint, []),
        "started_at_utc": now,
        "ended_at_utc": None,
        "lifecycle": "active",
        "safety_outcome": "not_applicable",
        "acceptance": "not_evaluated",
        "run_class": "development",
        "retention_class": "ordinary",
        "source": {"git_commit": commit or None, "git_worktree_clean": not dirty},
        "build": {
            "package_layout": "source" if source_layout else "installed_share",
            "package_root": str(package_root),
            "ros_distro": os.environ.get("ROS_DISTRO"),
        },
        "host": {"hostname": socket.gethostname()},
        "config_snapshots": [],
        "subordinate_manifests": [],
        "external_exports": [],
    }
    _atomic_write_json(run_dir / "metadata" / "run_manifest.json", manifest)


def finalize_run(
    run_dir: Path,
    *,
    lifecycle: str,
    safety_outcome: str | None = None,
) -> str:
    """Atomically record the run owner's terminal lifecycle decision."""

    if lifecycle not in _TERMINAL_LIFECYCLES:
        raise ValueError(
            f"lifecycle must be one of {sorted(_TERMINAL_LIFECYCLES)}, got {lifecycle!r}"
        )
    if safety_outcome is not None and safety_outcome not in _SAFETY_OUTCOMES:
        raise ValueError(
            f"safety_outcome must be one of {sorted(_SAFETY_OUTCOMES)}, "
            f"got {safety_outcome!r}"
        )
    run_dir = _validated_absolute(Path(run_dir), "run_dir")
    manifest_path = run_dir / "metadata" / "run_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("run_id") != run_dir.name or manifest.get("run_dir") != str(run_dir):
        raise RuntimeError(f"run manifest identity does not match {run_dir}")
    # A successful launch exit or later user interrupt cannot erase the
    # recorded loss of a required historical source. Only the owner finalizes.
    if (run_dir / "metadata/manifests/input_failure.json").exists():
        lifecycle = "failed"
    manifest["lifecycle"] = lifecycle
    manifest["ended_at_utc"] = datetime.now(timezone.utc).isoformat().replace(
        "+00:00", "Z"
    )
    if safety_outcome is not None:
        manifest["safety_outcome"] = safety_outcome
    _atomic_write_json(manifest_path, manifest)
    lock = _ACTIVE_LOCKS.pop(run_dir, None)
    if lock is not None:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        finally:
            lock.close()
    return lifecycle


def finalize_run_from_shutdown(run_dir: Path, event: object) -> None:
    """Map ROS launch shutdown semantics onto the run lifecycle enum."""

    reason = str(getattr(event, "reason", ""))
    if bool(getattr(event, "due_to_sigint", False)):
        lifecycle = "interrupted"
    elif reason.startswith("Caught exception in launch") or reason == "input_failure":
        lifecycle = "failed"
    else:
        lifecycle = "completed"
    try:
        finalize_run(run_dir, lifecycle=lifecycle)
    except Exception as error:
        warnings.warn(f"could not finalize run {run_dir}: {error}")


def write_subordinate_manifest(run_dir: Path, name: str, value: dict) -> Path:
    safe_name = _component(name, "manifest name")
    path = run_dir / "metadata" / "manifests" / f"{safe_name}.json"
    _atomic_write_json(path, value)
    register_subordinate_manifest(run_dir, path)
    return path


def register_subordinate_manifest(run_dir: Path, path: Path) -> None:
    manifest_path = run_dir / "metadata" / "run_manifest.json"
    if not manifest_path.is_file():
        return
    relative = path.resolve().relative_to(run_dir.resolve()).as_posix()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    references = manifest.setdefault("subordinate_manifests", [])
    if relative not in references:
        references.append(relative)
        references.sort()
        _atomic_write_json(manifest_path, manifest)


def register_config_snapshot(run_dir: Path, path: Path) -> None:
    manifest_path = run_dir / "metadata" / "run_manifest.json"
    relative = path.resolve().relative_to(run_dir.resolve()).as_posix()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    references = manifest.setdefault("config_snapshots", [])
    if relative not in references:
        references.append(relative)
        references.sort()
        _atomic_write_json(manifest_path, manifest)


