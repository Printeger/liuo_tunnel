#!/usr/bin/env python3
"""Replay a frozen source run at its original 1x delivery schedule; sole clock owner."""
import argparse
import json
import hashlib
import csv
import itertools
from delivery_schedule import DeliverySchedule
from raw_identity import hash_message
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
    parser.add_argument("--source-run", type=Path, required=True)
    parser.add_argument("--delivery-profile", type=Path)
    args = parser.parse_args()
    scheduler = DeliverySchedule(
        json.loads(args.delivery_profile.read_text()) if args.delivery_profile else None
    )
    source = args.source_run.resolve()
    run = Path(os.environ["LIUO_RUN_DIR"])
    duration = json.loads((source / "export/simulation/runtime.json").read_text())["simulation_s"]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source / "export/capture/raw"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    allowed = {
        "/clock": "clock",
        "/sensors/imu": "imu",
        "/sensors/lidar/points": "cloud",
        "/sensors/uwb/range": "uwb",
        "/sim/truth/odom": "truth",
    }
    rclpy.init()
    node = Node("liuo_frozen_replay")
    publishers = {}
    types = {}
    counts = {"imu": 0, "cloud": 0, "uwb": 0, "clock": 0}
    for topic in reader.get_all_topics_and_types():
        if topic.name not in allowed:
            raise RuntimeError("unsupported frozen topic: " + topic.name)
        types[topic.name] = get_message(topic.type)
        reliable = topic.name in ["/clock", "/sim/truth/odom"]
        qos = QoSProfile(
            depth=32 if allowed[topic.name] == "cloud" else 2048,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        )
        publishers[topic.name] = node.create_publisher(types[topic.name], topic.name, qos)
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
        for t in ["/sensors/imu", "/sensors/lidar/points", "/sensors/uwb/range"]
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
    first_stamp = None
    start_ns = None
    clock_session = 1
    last_clock = -1
    with (run / "export/simulation/wall_clock.csv").open("w") as clock_log, (
        run / "export/simulation/delivery.csv"
    ).open("w") as delivery_log:
        delivery_csv = csv.writer(delivery_log)
        delivery_csv.writerow(
            [
                "producer",
                "session",
                "sequence",
                "measurement_stamp",
                "source_elapsed_s",
                "delivery_elapsed_s",
                "action",
            ]
        )
        clock_log.write("session,stamp,scheduled_wall_ns,published_wall_ns\n")
        for topic, data, record_ns, delivery_ns, action in scheduler.events(
            itertools.chain(prefix, records())
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
            if kind == "uwb":
                identity = [message.producer_id, message.session_id, message.sequence]
                measurement_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                delivery_csv.writerow(
                    identity
                    + [
                        measurement_stamp,
                        (record_ns - first_stamp) * 1e-9,
                        (delivery_ns - first_stamp) * 1e-9,
                        action,
                    ]
                )
                if action == "outage_drop":
                    continue
                if action == "duplicate":
                    duplicates.append(identity)
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
        "input_mode": "frozen_replay",
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
