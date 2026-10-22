"""Evaluation-only fixed initial datum. Never imported by the estimator or replay publisher."""

import math


def _quaternion(row):
    return [float(row.get(k, 1 if k == "qw" else 0)) for k in ["qx", "qy", "qz", "qw"]]


def _transform(records, truth, inverse):
    starts = {}
    for row in truth:
        starts.setdefault(row["session"], row)
    starts = list(starts.values())
    sessions = list(dict.fromkeys(r["session"] for r in records))
    if len(sessions) > len(starts):
        raise ValueError("missing initial truth datum for evaluation session")
    output = []
    for row in records:
        initial = starts[sessions.index(row["session"])]
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
