#!/usr/bin/env python3
"""Simulation-only oracle/survey fixtures. Never called by the estimator."""
import argparse
import json
import math
from pathlib import Path
import random


def generate(directory):
    directory.mkdir(parents=True, exist_ok=True)
    base = {
        "schema_version": "liuo_known_map_v1",
        "units": "m",
        "axis_convention": "ENU",
        "valid": True,
        "frame": "map",
        "tag_lever_m": [0.1, 0.15, 0.25],
        "coordinate_reference": "simulation world survey datum; no vehicle start pose constraint",
        "anchors": [
            {
                "id": i,
                "mean_m": [
                    200.0 * i,
                    -2.8 if i % 2 else 2.8,
                    (3.0 if i % 2 else 2.0) + 0.3 * math.sin(1.7 * i),
                ],
            }
            for i in range(13)
        ],
    }
    base.update(
        map_id="rail_oracle_v2_noncoplanar", mode="oracle_map", provenance="SIMULATION_TRUTH_ORACLE"
    )
    (directory / "oracle_map.json").write_text(json.dumps(base, indent=2) + "\n")
    # Survey error is independent of every raw sensor random stream, and contains
    # a common datum translation plus independent point measurement errors.
    rng = random.Random(67129)
    common = [rng.gauss(0, 0.01) for _ in range(3)]
    for anchor in base["anchors"]:
        anchor["mean_m"] = [
            v + common[k] + rng.gauss(0, 0.02) for k, v in enumerate(anchor["mean_m"])
        ]
    base.update(
        map_id="rail_synthetic_survey_v2_noncoplanar",
        mode="surveyed_map",
        provenance="SIMULATED_SURVEY_NOT_REAL_MEASUREMENTS",
        survey_seed=67129,
        common_datum_sigma_m=0.01,
        independent_sigma_m=0.02,
    )
    base["joint_covariance_m2"] = [
        [(0.01**2 if i % 3 == j % 3 else 0) + (0.02**2 if i == j else 0) for j in range(39)]
        for i in range(39)
    ]
    (directory / "surveyed_map.json").write_text(json.dumps(base, indent=2) + "\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("directory", type=Path)
    generate(p.parse_args().directory)
