#!/usr/bin/env python3
"""Replay a frozen source run at its original 1x delivery schedule; sole clock owner."""
import argparse
import json
import hashlib
import csv
import itertools
from delivery_schedule import DeliverySchedule
from raw_identity import hash_message, observation_identity
import os
from pathlib import Path
import resource
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uwb-load-fixture", action="store_true")
    parser.add_argument("--uwb-contract-fixture", action="store_true")
    parser.add_argument("--source-run", type=Path, required=True)
    parser.add_argument("--delivery-profile", type=Path)
    args = parser.parse_args()
    profile = json.loads(args.delivery_profile.read_text()) if args.delivery_profile else None
    source = args.source_run.resolve()
    run = Path(os.environ["LIUO_RUN_DIR"])
    duration = json.loads((source / "export/simulation/runtime.json").read_text())["simulation_s"]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source / "export/capture/raw"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    source_config = json.loads((source / "metadata/config/phase1.json").read_text())
    target_config = json.loads((run / "metadata/config/phase1.json").read_text())
    source_topic = source_config["uwb"].get("transport", {}).get("topic", "/liuo/uwb/observations")
    transport = target_config["uwb"].get("transport", {})
    target_topic = transport.get("topic", "/liuo/uwb/observations")
    allowed = {
        "/clock": "clock",
        "/sensors/imu": "imu",
        "/sensors/lidar/points": "cloud",
        source_topic: "uwb",
        "/sim/truth/odom": "truth",
    }
    rclpy.init()
    node = Node("liuo_frozen_replay")
    publishers = {}
    types = {}
    counts = {"imu": 0, "cloud": 0, "uwb": 0, "clock": 0}
    for topic in reader.get_all_topics_and_types():
        if topic.type == "liuo_tunnel_msgs/msg/UwbRange":
            raise RuntimeError("Legacy UwbRange bag: use historical LIUO revision; no implicit conversion")
        if topic.name not in allowed:
            raise RuntimeError("unsupported frozen topic: " + topic.name)
        types[topic.name] = get_message(topic.type)
        reliable = topic.name in ["/clock", "/sim/truth/odom"]
        qos = QoSProfile(
            depth=32 if allowed[topic.name] == "cloud" else (transport.get("depth", 100) if allowed[topic.name] == "uwb" else 2048),
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        )
        publishers[topic.name] = node.create_publisher(types[topic.name], target_topic if topic.name == source_topic else topic.name, qos)

    input_config = json.loads((run / "metadata/config/phase1.json").read_text())["uwb"]["input"]
    registry = {n["logical_id"]: (n["type"], n["internal_id"]) for n in input_config["nodes"]}
    def anchor_ids(o):
        return [registry[n][1] for n in (o.node_a, o.node_b) if n in registry and registry[n][0] == 1]
    scheduler = DeliverySchedule(profile, uwb_topic=source_topic)
    if set(publishers) != set(allowed):
        raise RuntimeError("frozen input lacks required topics")
    (run / "metadata/processes/simulation.json").write_text(
        json.dumps(
            {
                "process_name": "simulation",
                "role": "frozen_replay",
                "pid": os.getpid(),
                "run_directory": str(run),
                "source_run": str(source),
            },
            indent=2,
        )
        + "\n"
    )
    ready_deadline = time.monotonic() + 20
    while any(
        publishers[t].get_subscription_count() < 2
        for t in ["/sensors/imu", "/sensors/lidar/points", source_topic]
    ):
        if time.monotonic() > ready_deadline:
            raise RuntimeError("replay consumers not ready")
        rclpy.spin_once(node, timeout_sec=0.01)
    if not reader.has_next():
        raise RuntimeError("empty frozen input")
    hashes = {k: hashlib.sha256() for k in counts}
    duplicates = []

    def records():
        while reader.has_next():
            yield reader.read_next()

    # Load each type's first original payload before the delivery clock starts.
    # ROS Python type-support initialization can otherwise consume the first
    # deadline. No warm-up messages are published; replay preserves these bytes.
    prepare_ns = time.monotonic_ns()
    prefix = []
    prepared = set()
    prefix_bytes = 0
    while prepared != set(types):
        if not reader.has_next():
            raise RuntimeError("frozen input missing a required message type")
        record = reader.read_next()
        prefix.append(record)
        prefix_bytes += len(record[1])
        if len(prefix) > 256 or prefix_bytes > 16 * 1024 * 1024:
            raise RuntimeError("replay startup preparation bound exceeded")
        topic, data, _ = record
        if topic not in prepared:
            deserialize_message(data, types[topic])
            prepared.add(topic)
    input_prepare_s = (time.monotonic_ns() - prepare_ns) * 1e-9
    fixture = None
    if args.uwb_contract_fixture:
        from uwb_contract_fixture import cases
        fixture = iter(cases())
    stream = itertools.chain(prefix, records())
    if args.uwb_load_fixture:
        from uwb_load_fixture import LoadFixture
        stream = LoadFixture(source, run, source_topic).records(stream)
    first_stamp = None
    start_ns = None
    clock_session = 1
    last_clock = -1
    with (run / "export/simulation/wall_clock.csv").open("w") as clock_log, (
        run / "export/simulation/delivery.csv"
    ).open("w") as delivery_log:
        delivery_csv = csv.writer(delivery_log)
        delivery_csv.writerow(["network", "driver_session", "producer", "session", "sequence", "a_type", "a", "b_type", "b",
                               "measurement_stamp", "source_elapsed_s", "delivery_elapsed_s", "action", "base_range_m", "range_after_stress_m", "injected_bias_m"])
        clock_log.write("session,stamp,scheduled_wall_ns,published_wall_ns\n")
        for topic, data, record_ns, delivery_ns, action in scheduler.events(
            stream
        ):
            if first_stamp is None:
                first_stamp = record_ns
                start_ns = time.monotonic_ns()
            if delivery_ns - first_stamp >= round(duration * 1e9):
                raise RuntimeError(
                    "delivery profile extends beyond frozen duration; shorten its window"
                )
            deadline = start_ns + delivery_ns - first_stamp
            while True:
                remaining_ns = deadline - time.monotonic_ns()
                if remaining_ns <= 0:
                    break
                # Calculate once: a second clock read may cross the deadline.
                # Negative rclpy timeouts mean an unbounded wait, not zero.
                rclpy.spin_once(node, timeout_sec=min(0.001, remaining_ns * 1e-9))
            if time.monotonic_ns() - deadline > 250_000_000:
                raise RuntimeError("REPLAY_DEADLINE_MISSED: backlog >250ms")
            message = deserialize_message(data, types[topic])
            kind = allowed[topic]
            if kind == "uwb" and fixture is not None:
                case = next(fixture, None)
                if case is None:
                    continue
                message = case[1]
            if kind == "uwb" and fixture is None:
                kept = []
                elapsed = (record_ns - first_stamp) * 1e-9
                for o in message.observations:
                    identity = list(observation_identity(message, o, input_config["network_instance"]))
                    edge_action = "anchor_outage_drop" if scheduler.filter_observation(elapsed, anchor_ids(o)) else action
                    original, bias = scheduler.apply_range_stress(o, elapsed, anchor_ids(o))
                    row_identity = [input_config["network_instance"], message.driver_session_id, o.reporter_id, o.source_session_id, o.source_frame_seq, *registry[o.node_a], *registry[o.node_b]]
                    delivery_csv.writerow(row_identity + [o.source_frame_stamp.sec + o.source_frame_stamp.nanosec * 1e-9,
                        elapsed, (delivery_ns - first_stamp) * 1e-9, edge_action, original, o.distance_m, bias])
                    if edge_action not in ("outage_drop", "anchor_outage_drop"):
                        kept.append(o)
                        if action == "duplicate": duplicates.append(identity)
                if action == "outage_drop": continue
                if message.observations and not kept: continue
                message.observations = kept
            publishers[topic].publish(message)
            if time.monotonic_ns() - deadline > 250_000_000:
                raise RuntimeError(
                    "REPLAY_DEADLINE_MISSED: deserialize/publish exceeded 250ms on " + topic
                )
            if kind in counts:
                counts[kind] += 1
                hash_message(hashes[kind], kind, message)
            if kind == "clock":
                stamp = message.clock.sec + message.clock.nanosec * 1e-9
                if stamp < last_clock:
                    clock_session += 1
                last_clock = stamp
                clock_log.write(f"{clock_session},{stamp:.9f},{deadline},{time.monotonic_ns()}\n")
        end = start_ns + round(duration * 1e9)
        while time.monotonic_ns() < end:
            rclpy.spin_once(node, timeout_sec=0.001)
    wall = (time.monotonic_ns() - start_ns) * 1e-9
    result = {
        "simulation_s": duration,
        "input_prepare_s": input_prepare_s,
        "startup_buffer_records": len(prefix),
        "startup_buffer_bytes": prefix_bytes,
        "wall_s": wall,
        "real_time_factor": duration / wall,
        "peak_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "scan_model": "snapshot",
        "input_mode": "software_load_fixture" if args.uwb_load_fixture else ("software_contract_fixture" if fixture is not None else "frozen_replay"),
        "published_counts": counts,
        "delivered_sha256": {k: h.hexdigest() for k, h in hashes.items()},
        "duplicate_uwb": duplicates,
        "delivery_queue_peak": scheduler.peak,
    }
    (run / "export/simulation/runtime.json").write_text(json.dumps(result, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
