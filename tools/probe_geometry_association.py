#!/usr/bin/env python3
"""Bounded offline geometry-association experiment; never an estimator input.

Reads only recorded raw sensor messages, sensor extrinsics and the known map.
Does not import simulation code, read truth/scenario, or extrapolate fitted planes.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2


def fit_planes(points):
    """Finite RANSAC: 4 surfaces, 128 proposals each; descriptive fit only."""
    rng = np.random.default_rng(0)
    remaining = points[rng.choice(len(points), min(4096, len(points)), replace=False)]
    result = []
    for _ in range(4):
        if len(remaining) < 100:
            break
        best = None
        for _ in range(128):
            triple = remaining[rng.choice(len(remaining), 3, replace=False)]
            normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-8:
                continue
            normal /= norm
            offset = -normal @ triple[0]
            mask = np.abs(remaining @ normal + offset) < 0.06
            if best is None or mask.sum() > best.sum():
                best = mask
        if best is None or best.sum() < 100:
            break
        support = remaining[best]
        centroid = support.mean(axis=0)
        _, _, vectors = np.linalg.svd(support - centroid, full_matrices=False)
        normal = vectors[-1]
        if normal[np.argmax(np.abs(normal))] < 0:
            normal = -normal
        offset = -normal @ centroid
        result.append(
            dict(
                normal=normal.tolist(),
                offset_m=float(offset),
                support_points=len(support),
                rms_m=float(np.sqrt(np.mean((support @ normal + offset) ** 2))),
                support_min_m=support.min(axis=0).tolist(),
                support_max_m=support.max(axis=0).tolist(),
            )
        )
        remaining = remaining[~best]
    return result


def transformed_residual_error(points, planes, yaw, height):
    """Same measured scene in two frames: no independently registered map planes."""
    rotation = Rotation.from_euler("z", yaw).as_matrix()
    translation = np.array([0.0, 0.0, height])
    world = points @ rotation.T + translation
    errors = []
    for plane in planes:
        normal = np.array(plane["normal"])
        world_normal = rotation @ normal
        world_offset = plane["offset_m"] - world_normal @ translation
        errors.append(
            np.max(
                np.abs(
                    (world @ world_normal + world_offset)
                    - (points @ normal + plane["offset_m"])
                )
            )
        )
    return float(max(errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    config_path = args.source / "metadata/config/phase1.json"
    map_path = args.source / "metadata/config/map.json"
    config = json.loads(config_path.read_text())
    known_map = json.loads(map_path.read_text())
    reader = rosbag2_py.SequentialReader()
    bag = args.source / "export/capture/raw"
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    selected = ["/sensors/lidar/points", "/sensors/imu", "/liuo/uwb/observations"]
    types = {
        t.name: get_message(t.type)
        for t in reader.get_all_topics_and_types()
        if t.name in selected
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    stamps = [3.0, 50.0]
    clouds, ranges, acceleration = {}, {s: [] for s in stamps}, []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        message = deserialize_message(raw, types[topic])
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp > 50.2:
            break
        if topic == "/sensors/imu" and 1 <= stamp <= 3:
            acceleration.append(
                [getattr(message.linear_acceleration, k) for k in "xyz"]
            )
        for target in stamps:
            if topic == "/sensors/lidar/points" and abs(stamp - target) < 1e-6:
                cloud = point_cloud2.read_points_numpy(
                    message, field_names=("x", "y", "z"), skip_nans=True
                )
                clouds[target] = np.asarray(cloud, dtype=float).reshape(-1, 3)
            if topic == "/liuo/uwb/observations":
                registry = {n["logical_id"]: n for n in config["uwb"]["input"]["nodes"]}
                for o in message.observations:
                    source_stamp = o.source_frame_stamp.sec + o.source_frame_stamp.nanosec * 1e-9
                    if o.kind == 0 and o.time_basis in (1,2) and abs(source_stamp - target) <= .1 and o.node_b in registry:
                        ranges[target].append(dict(anchor_id=registry[o.node_b]["internal_id"], range_m=float(o.distance_m)))
    assert len(clouds) == 2 and len(acceleration) >= 390
    gravity = np.mean(acceleration, axis=0)
    # Gravity supplies a local vertical only, not a map height or yaw.
    leveling, _ = Rotation.align_vectors(
        [[0, 0, 1]], [gravity / np.linalg.norm(gravity)]
    )
    sensors = config["sensors"]
    extrinsic = Rotation.from_euler("z", sensors["T_BL_yaw"])
    translation = np.array(sensors["T_BL_translation"])
    results = []
    for stamp in stamps:
        points = leveling.apply(extrinsic.apply(clouds[stamp]) + translation)
        planes = fit_planes(points)
        assert planes, "NO_LOCAL_PLANES"
        invariance = [
            dict(
                yaw_rad=yaw,
                height_shift_m=height,
                max_residual_change_m=transformed_residual_error(
                    points, planes, yaw, height
                ),
            )
            for yaw in (0.0, np.pi / 2, np.pi)
            for height in (0.0, 7.0)
        ]
        assert max(r["max_residual_change_m"] for r in invariance) < 1e-10
        # Positive control: an independently fixed map plane must NOT follow
        # the candidate. A 7 m candidate shift changes its signed residual.
        horizontal = [p for p in planes if abs(p["normal"][2]) > 0.9]
        assert len(horizontal) >= 2, "NO_OBSERVED_VERTICAL_ENVELOPE"
        reference = horizontal[0]
        normal = np.array(reference["normal"])
        before = points @ normal + reference["offset_m"]
        after = (points + [0, 0, 7]) @ normal + reference["offset_m"]
        fixed_plane_delta = float(np.mean(after - before))
        assert abs(fixed_plane_delta) > 6.3, "REGISTERED_PLANE_CONTROL_FAILED"
        assert ranges[stamp], "NO_RAW_TA"
        results.append(
            dict(
                stamp=stamp,
                points=len(points),
                planes=planes,
                hypothetical_registered_plane_shift_residual_m=fixed_plane_delta,
                max_observed_lidar_range_m=float(
                    np.max(np.linalg.norm(clouds[stamp], axis=1))
                ),
                nearest_observed_ta_m=min(r["range_m"] for r in ranges[stamp]),
                observed_anchor_ids=sorted({r["anchor_id"] for r in ranges[stamp]}),
                frame_reexpression=invariance,
            )
        )
    paths = [Path(__file__), config_path, map_path, *sorted(bag.glob("*.mcap"))]
    digest = {}
    for path in paths:
        sha = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                sha.update(block)
        digest[str(path)] = sha.hexdigest()
    # Parameters are robot map z, local floor map z, local ceiling map z.
    # Relative height, headroom and measured separation have rank 2, not 3.
    # Even perfect horizontal signed anchor offsets add a zero vertical row.
    jacobian = np.array(
        [[1.0, -1.0, 0.0], [-1.0, 0.0, 1.0], [0.0, -1.0, 1.0], [0.0, 0.0, 0.0]]
    )
    null_vector = np.ones(3)
    assert np.linalg.matrix_rank(jacobian) == 2
    assert np.max(np.abs(jacobian @ null_vector)) == 0
    assert np.linalg.matrix_rank(np.vstack([jacobian, [0.0, 1.0, 0.0]])) == 3
    output = dict(
        vertical_geometry_rank=dict(
            parameters=["robot_map_z", "floor_map_z", "ceiling_map_z"],
            jacobian=jacobian.tolist(),
            rank=2,
            null_vector=null_vector.tolist(),
            rank_with_independent_floor_datum=3,
            excludes="Raw UWB and motion rows: this is NOT a full-system gauge claim",
        ),
        scope="Raw observed local surfaces and geometry-only frame ambiguity; NOT full sensor observability or initialization acceptance",
        source=str(args.source),
        map_id=known_map["map_id"],
        map_top_level_fields=sorted(known_map),
        anchor_fields=sorted(set().union(*(a.keys() for a in known_map["anchors"]))),
        gravity_samples=len(acceleration),
        initial_specific_force_mean=gravity.tolist(),
        observations=results,
        sha256=digest,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n")
    print(
        json.dumps({k: v for k, v in output.items() if k not in ["sha256"]}, indent=2)
    )


if __name__ == "__main__":
    main()
