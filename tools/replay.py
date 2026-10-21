#!/usr/bin/env python3
"""Replay a frozen source run at its original 1x delivery schedule; sole clock owner."""
import argparse
import json
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
    args = parser.parse_args()
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
    first_stamp = None
    start_ns = time.monotonic_ns()
    clock_session = 1
    last_clock = -1
    with (run / "export/simulation/wall_clock.csv").open("w") as clock_log:
        clock_log.write("session,stamp,scheduled_wall_ns,published_wall_ns\n")
        while reader.has_next():
            topic, data, record_ns = reader.read_next()
            if first_stamp is None:
                first_stamp = record_ns
            deadline = start_ns + record_ns - first_stamp
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
            publishers[topic].publish(message)
            kind = allowed[topic]
            if kind in counts:
                counts[kind] += 1
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
        "wall_s": wall,
        "real_time_factor": duration / wall,
        "peak_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "scan_model": "snapshot",
        "input_mode": "frozen_replay",
        "published_counts": counts,
    }
    (run / "export/simulation/runtime.json").write_text(json.dumps(result, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
