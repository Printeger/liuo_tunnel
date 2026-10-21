#!/usr/bin/env python3
"""OFFLINE geometry diagnostic with ideal truth-relative motion; never estimator input.

Fits only global translation/yaw to original raw TA observations from several
height initializations. This deliberately optimistic diagnostic separates layout
ambiguity from online trajectory estimation error. It is NOT an acceptance test.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


def analyze(run, start=3.0):
    scenario = json.loads((run / "metadata/config/scenario.json").read_text())
    if scenario["scene"] != "rail_tunnel_straight_v1":
        raise ValueError("diagnostic currently supports straight, identity-rotation truth only")
    config = json.loads((run / "metadata/config/phase1.json").read_text())
    map_data = json.loads((run / "metadata/config/map.json").read_text())
    if map_data["mode"] != "oracle_map":
        raise ValueError("ideal layout diagnostic requires declared oracle anchor fixture")
    with (run / "export/simulation/truth.csv").open() as stream:
        truth = list(csv.DictReader(stream))
    if len({row["session"] for row in truth}) != 1:
        raise ValueError("select a single-session source")
    with (run / "export/simulation/uwb.csv").open() as stream:
        observations = [
            row
            for row in csv.DictReader(stream)
            if row["a_type"] == "0" and start <= float(row["stamp"]) <= float(truth[-1]["stamp"])
        ]
    if not observations:
        raise ValueError("no original TA observations in requested interval")
    times = np.array([float(row["stamp"]) for row in truth])
    stamps = np.array([float(row["stamp"]) for row in observations])
    points = np.column_stack(
        [np.interp(stamps, times, [float(row[axis]) for row in truth]) for axis in ["x", "y", "z"]]
    ) + np.asarray(config["sensors"]["T_BU_translation"])
    by_id = {row["id"]: row["mean_m"] for row in map_data["anchors"]}
    anchors = np.array([by_id[int(row["b"])] for row in observations])
    measured = np.array([float(row["range"]) for row in observations])
    sigma = np.sqrt([float(row["variance"]) for row in observations])

    def residual(q):
        c, s = np.cos(q[3]), np.sin(q[3])
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return (np.linalg.norm(points @ rotation.T + q[:3] - anchors, axis=1) - measured) / sigma

    solutions = []
    for height in [-4, -1, 0, 1, 4]:
        fit = least_squares(residual, [0, 0, height, 0], max_nfev=500)
        solutions.append(
            {
                "initial_z_m": height,
                "translation_m_and_yaw_rad": fit.x.tolist(),
                "cost": fit.cost,
                "success": bool(fit.success),
                "optimality": fit.optimality,
            }
        )
    report = {
        "scope": "OFFLINE ideal truth-relative trajectory geometry diagnostic; NOT online acceptance or estimator input",
        "run_id": run.name,
        "map_id": map_data["map_id"],
        "samples": len(observations),
        "start_s": start,
        "solutions": solutions,
        "limitation": "Five numerical starts do not prove global uniqueness; relative trajectory is assumed exact here only.",
    }
    output = run / "export/analysis/ideal_rigid_range_diagnostic.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--start", type=float, default=3.0)
    args = parser.parse_args()
    print(json.dumps(analyze(args.run, args.start), indent=2))
