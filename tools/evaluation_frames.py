"""Evaluation-only fixed initial datum. Never imported by the estimator or replay publisher."""

import math


def _quaternion(row):
    return [float(row.get(k, 1 if k == "qw" else 0)) for k in ["qx", "qy", "qz", "qw"]]


def _transform(records, truth, inverse, shared_datum=False):
    starts = {}
    for row in truth:
        starts.setdefault(row["session"], row)
    starts = list(starts.values())
    sessions = list(dict.fromkeys(r["session"] for r in records))
    if shared_datum and len(starts) != 1:
        raise ValueError('shared map datum must be unique')
    if not shared_datum and len(sessions) > len(starts):
        raise ValueError("missing initial truth datum for evaluation session")
    output = []
    for row in records:
        initial = starts[0 if shared_datum else sessions.index(row["session"])]
        x, y, z, w = _quaternion(initial)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        angle = -yaw if inverse else yaw
        c, s = math.cos(angle), math.sin(angle)
        origin = [float(initial[k]) for k in ["x", "y", "z"]]
        result = dict(row)
        for fields, position in [(["x", "y", "z"], True), (["vx", "vy", "vz"], False)]:
            v = [float(row[k]) for k in fields]
            if position and inverse:
                v = [a - b for a, b in zip(v, origin)]
            v = [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]]
            if position and not inverse:
                v = [a + b for a, b in zip(v, origin)]
            result.update(zip(fields, v))
        x, y, z, w = _quaternion(row)
        c, s = math.cos(angle / 2), math.sin(angle / 2)
        result.update(
            zip(
                ["qx", "qy", "qz", "qw"],
                [c * x - s * y, c * y + s * x, c * z + s * w, c * w - s * z],
            )
        )
        output.append(result)
    return output


def world_estimates(trajectory, truth, fused):
    """OFF: initial translation/yaw only, no fitted transform or future samples."""
    return trajectory if fused else _transform(trajectory, truth, False)


def local_truth(truth, fused):
    """Reference in the estimator frame for the original joint covariance."""
    return truth if fused else _transform(truth, truth, True)


def map_evaluation_datum(run):
    """Offline only: use the original calibration run's declared simulation datum.

    Never use the reuse run's true start or fit any trajectory transformation.
    Requires retained provenance; missing source evidence is an error.
    """
    import json
    from pathlib import Path

    config = json.loads((run / "metadata/config/phase1.json").read_text())
    mode = config["uwb"]["mode"]
    if mode == "self_calibrate":
        source = run
    elif mode == "reuse_map":
        document = json.loads((run / "metadata/config/map.json").read_text())
        source = Path(document["source_run"])
    else:
        raise ValueError("map datum only applies to calibrated-map modes")
    original = json.loads((source / "metadata/config/phase1.json").read_text())
    if original["uwb"]["mode"] != "self_calibrate":
        raise ValueError("map source is not a self-calibration run")
    scenario = json.loads((source / "metadata/config/scenario.json").read_text())
    datum = scenario["simulation_start_xyz_yaw"]
    if len(datum) != 4 or not all(math.isfinite(v) for v in datum):
        raise ValueError("invalid declared first-run datum")
    x, y, z, yaw = datum
    return dict(session="datum", x=x, y=y, z=z, vx=0, vy=0, vz=0,
                qx=0, qy=0, qz=math.sin(yaw / 2), qw=math.cos(yaw / 2))


def run_world_estimates(run, trajectory, truth, mode):
    if mode in ("self_calibrate", "reuse_map"):
        return _transform(trajectory, [map_evaluation_datum(run)], False, shared_datum=(mode == "reuse_map"))
    return world_estimates(trajectory, truth, mode == "fuse")


def map_local_truth(run, truth):
    import json
    mode = json.loads((run / "metadata/config/phase1.json").read_text())["uwb"]["mode"]
    return _transform(truth, [map_evaluation_datum(run)], True, shared_datum=(mode == "reuse_map"))
