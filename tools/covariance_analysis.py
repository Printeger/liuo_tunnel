"""Local linearized posterior diagnostics, never a noise-tuning objective."""

import csv
import math
import numpy as np


def summarize(run, trajectory, truth, covariance, fused):
    def keyed(records):
        sessions = list(dict.fromkeys(r["session"] for r in records))
        return {(sessions.index(r["session"]), round(float(r["stamp"]), 8)): r for r in records}

    truth = keyed(truth)
    covariance = keyed(covariance)
    diagnostics = []
    for key, state in keyed(trajectory).items():
        record = covariance[key]
        cov = np.array([[float(record[f"xv_{i}_{j}"]) for j in range(9)] for i in range(9)])
        if not np.isfinite(cov).all() or not np.allclose(cov, cov.T, rtol=1e-8, atol=1e-10):
            raise ValueError("invalid joint covariance")
        np.linalg.cholesky(cov)
        q = np.array([float(state[k]) for k in ["qx", "qy", "qz", "qw"]])
        q /= np.linalg.norm(q)
        if q[3] < 0:
            q *= -1
        x, y, z, w = q
        R = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        reference = truth[key]
        # Right-local Log(R_est^-1 R_truth); old captures had identity truth.
        qt = np.array(
            [float(reference.get(k, 1 if k == "qw" else 0)) for k in ["qx", "qy", "qz", "qw"]]
        )
        qt /= np.linalg.norm(qt)
        delta = np.r_[q[3] * qt[:3] - qt[3] * q[:3] - np.cross(q[:3], qt[:3]), q @ qt]
        if delta[3] < 0:
            delta *= -1
        norm = np.linalg.norm(delta[:3])
        rot = delta[:3] * (2 * math.atan2(norm, delta[3]) / norm if norm > 1e-12 else 2)
        p = np.array([float(reference[k]) - float(state[k]) for k in ["x", "y", "z"]])
        if not fused:
            p[2] -= 1.2
        position_error = R.T @ p
        a, b, c = rot
        skew = np.array([[0, -c, b], [c, 0, -a], [-b, a, 0]])
        angle = np.linalg.norm(rot)
        coefficient = (1 - angle / (2 * math.tan(angle / 2))) / angle**2 if angle > 1e-5 else 1 / 12
        rho = (np.eye(3) - 0.5 * skew + coefficient * skew @ skew) @ position_error
        velocity_error = np.array(
            [float(reference[k]) - float(state[k]) for k in ["vx", "vy", "vz"]]
        )
        error = np.r_[rot, rho, velocity_error]
        position_nees = float(position_error @ np.linalg.solve(cov[3:6, 3:6], position_error))
        joint_nees = float(error @ np.linalg.solve(cov, error))
        diagnostics.append(
            {
                "session": key[0] + 1,
                "stamp": float(state["stamp"]),
                "position_nees_3d": position_nees,
                "joint_nees_9d": joint_nees,
                "position_sigma_norm_m": float(np.sqrt(np.trace(cov[3:6, 3:6]))),
                "min_eigenvalue": float(np.linalg.eigvalsh(cov)[0]),
            }
        )
    if not diagnostics:
        raise ValueError("no covariance samples")
    (run / "export/analysis").mkdir(parents=True, exist_ok=True)
    with (run / "export/analysis/covariance.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(diagnostics[0]))
        writer.writeheader()
        writer.writerows(diagnostics)
    return {
        "samples": len(diagnostics),
        "position_nees_mean_3d": float(np.mean([d["position_nees_3d"] for d in diagnostics])),
        "joint_nees_mean_9d": float(np.mean([d["joint_nees_9d"] for d in diagnostics])),
        "position_95pct_ellipsoid_coverage": float(
            np.mean([d["position_nees_3d"] <= 7.814727903 for d in diagnostics])
        ),
        "interpretation": "Local linearized joint posterior; temporally correlated samples within one run. Report independent seeds separately; not a chi-square test on pooled frames. No noise retuning. Uses recorded truth orientation; legacy captures use identity.",
    }
