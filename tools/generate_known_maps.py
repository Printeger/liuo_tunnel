#!/usr/bin/env python3
"""Simulation-only oracle/survey fixtures. Never called by the estimator."""
import argparse
import json
from pathlib import Path
import random


def generate(directory, survey_seed=67129):
    directory.mkdir(parents=True, exist_ok=True)
    base = {
        "schema_version": "liuo_known_map_v1",
        "units": "m",
        "axis_convention": "ENU",
        "valid": True,
        "frame": "map",
        "simulation_anchor_layout": "four_corner_50m_49_anchors_v3",
        "tag_lever_m": [0.1, 0.15, 0.25],
        "coordinate_reference": "simulation world survey datum; no vehicle start pose constraint",
        "anchors": [
            {
                "id": i,
                "mean_m": [
                    50.0 * i,
                    2.8 if i % 4 < 2 else -2.8,
                    0.4 if i % 4 in (0, 3) else 4.6,
                ],
            }
            for i in range(49)
        ],
    }
    base.update(
        map_id="rail_oracle_v3_four_corner_50m",
        mode="oracle_map",
        provenance="SIMULATION_TRUTH_ORACLE",
    )
    (directory / "oracle_map.json").write_text(json.dumps(base, indent=2) + "\n")
    # Survey error is independent of every raw sensor random stream, and contains
    # a common datum translation plus independent point measurement errors.
    rng = random.Random(survey_seed)
    common = [rng.gauss(0, 0.01) for _ in range(3)]
    for anchor in base["anchors"]:
        anchor["mean_m"] = [
            v + common[k] + rng.gauss(0, 0.02) for k, v in enumerate(anchor["mean_m"])
        ]
    base.update(
        map_id="rail_synthetic_survey_v3_four_corner_50m",
        mode="surveyed_map",
        provenance="SIMULATED_SURVEY_NOT_REAL_MEASUREMENTS",
        survey_seed=survey_seed,
        common_datum_sigma_m=0.01,
        independent_sigma_m=0.02,
    )
    if survey_seed != 67129:
        base["map_id"] += f"_survey_seed_{survey_seed}"
    base["joint_covariance_m2"] = [
        [
            (0.01**2 if i % 3 == j % 3 else 0) + (0.02**2 if i == j else 0)
            for j in range(3 * len(base["anchors"]))
        ]
        for i in range(3 * len(base["anchors"]))
    ]
    (directory / "surveyed_map.json").write_text(json.dumps(base, indent=2) + "\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("directory", type=Path)
    p.add_argument("--survey-seed", type=int, default=67129)
    args = p.parse_args()
    generate(args.directory, args.survey_seed)
