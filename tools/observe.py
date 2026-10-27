#!/usr/bin/env python3
"""Independent runtime raw-message/graph audit; no estimator access to truth."""
import json
import csv
import numpy as np
import hashlib
from raw_identity import hash_message, observation_identity
import time
import os
from pathlib import Path
import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2
from liuo_uwb_msgs.msg import UwbObservationBatch
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry


class Observer(Node):
    def __init__(self):
        super().__init__("liuo_observer")
        self.result = {
            "counts": {"imu": 0, "cloud": 0, "uwb": 0, "clock": 0},
            "violations": [],
            "sessions": [],
            "clock_resets": 0,
            "imu_static_acc_z": [],
            "cloud_fields": None,
            "estimator_subscriptions": [],
            "clock_publishers": 0,
        }
        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=2048, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.hashes = {k: hashlib.sha256() for k in self.result["counts"]}
        self.first_wall_ns = None
        self.first_clock = None
        self.last_clock = 0
        self.identities = set()
        self.network = json.loads((Path(os.environ["LIUO_RUN_DIR"]) / "metadata/config/phase1.json").read_text())["uwb"]["input"]["network_instance"]
        self.result["duplicate_uwb"] = []
        self.imu_log = open(Path(os.environ["LIUO_RUN_DIR"]) / "export/capture/imu.csv", "w")
        self.imu_log.write("stamp,ax,ay,az,wx,wy,wz\n")
        self.create_subscription(Imu, "/sensors/imu", self.imu, self.sensor_qos)
        self.create_subscription(PointCloud2, "/sensors/lidar/points", self.cloud, self.sensor_qos)
        self.create_subscription(UwbObservationBatch, self.declare_parameter("uwb_topic", "/liuo/uwb/observations").value, self.uwb, QoSProfile(depth=self.declare_parameter("uwb_qos_depth", 100).value, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Clock, "/clock", self.clock, self.sensor_qos)
        self.result["odometry_count"] = 0
        self.odom_log = open(Path(os.environ["LIUO_RUN_DIR"]) / "export/capture/odometry.csv", "w")
        self.odom_csv = csv.writer(self.odom_log)
        self.odom_csv.writerow(
            ["stamp", "wx", "wy", "wz"] + [f"twist_{i}_{j}" for i in range(6) for j in range(6)]
        )
        self.create_subscription(Odometry, "/liuo/odom", self.odometry, 100)
        self.create_timer(1, self.graph)

    def hash_message(self, kind, m):
        hash_message(self.hashes[kind], kind, m)

    def reject(self, reason):
        if reason not in self.result["violations"]:
            self.result["violations"].append(reason)

    def odometry(self, m):
        self.result["odometry_count"] += 1
        if m.header.frame_id not in ["map", "odom"] or m.child_frame_id != "imu":
            self.reject("odometry frame contract")
        angular = [m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z]
        linear = [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z]
        if not all(math.isfinite(x) for x in angular + linear):
            self.reject("nonfinite odometry twist")
        for label, values in [("pose", m.pose.covariance), ("twist", m.twist.covariance)]:
            covariance = np.asarray(values).reshape(6, 6)
            if not np.isfinite(covariance).all():
                self.reject("nonfinite " + label + " covariance")
            elif (
                not np.allclose(covariance, covariance.T, atol=1e-10, rtol=1e-8)
                or np.linalg.eigvalsh(covariance).min() <= 0
            ):
                self.reject("nonpositive or asymmetric " + label + " covariance")
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom_csv.writerow([stamp] + angular + list(m.twist.covariance))

    def imu(self, m):
        self.hash_message("imu", m)
        self.result["counts"]["imu"] += 1
        if m.orientation_covariance[0] != -1 or m.header.frame_id != "imu":
            self.reject("IMU frame/orientation")
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.imu_log.write(
            f"{t},{m.linear_acceleration.x},{m.linear_acceleration.y},{m.linear_acceleration.z},{m.angular_velocity.x},{m.angular_velocity.y},{m.angular_velocity.z}\n"
        )
        if t < 5:
            self.result["imu_static_acc_z"].append(m.linear_acceleration.z)

    def cloud(self, m):
        self.hash_message("cloud", m)
        self.result["counts"]["cloud"] += 1
        if os.environ.get("LIUO_DIAGNOSTIC_CAPTURE") == "1" and self.result["counts"]["cloud"] in (
            30,
            60,
            90,
            120,
        ):
            from rclpy.serialization import serialize_message

            directory = Path(os.environ["LIUO_RUN_DIR"]) / "export/capture"
            (directory / f'cloud_{self.result["counts"]["cloud"]}.cdr').write_bytes(
                serialize_message(m)
            )
        fields = {f.name: f for f in m.fields}
        self.result["cloud_fields"] = list(fields)
        if not {"x", "y", "z", "intensity", "time"} <= fields.keys():
            self.reject("missing cloud fields")
            return
        if m.header.frame_id != "lidar":
            self.reject("cloud not lidar frame")
        if any(fields[k].datatype != 7 for k in fields):
            self.reject("cloud datatype")
            return
        if any(
            struct.unpack_from("<f", m.data, i * m.point_step + fields["time"].offset)[0] != 0
            for i in range(m.width * m.height)
        ):
            self.reject("snapshot time nonzero")

    def uwb(self, m):
        self.hash_message("uwb", m)
        self.result["counts"]["uwb"] += 1
        self.result.setdefault("uwb_observation_count", 0)
        self.result["uwb_observation_count"] += len(m.observations)
        for o in m.observations:
            identity = observation_identity(m, o, self.network)
            if identity in self.identities:
                self.result["duplicate_uwb"].append(list(identity))
            if identity not in self.identities and len(self.identities) >= 200000:
                raise RuntimeError("observer identity capacity exhausted; audit cannot forget old reports")
            if len(self.result["duplicate_uwb"]) > 4096:
                raise RuntimeError("observer duplicate evidence capacity exhausted")
            self.identities.add(identity)
            if m.driver_session_id not in self.result["sessions"]:
                if len(self.result["sessions"]) >= 128: raise RuntimeError("observer session capacity exhausted")
                self.result["sessions"].append(m.driver_session_id)
            if m.header.frame_id or not math.isfinite(o.distance_m) or o.distance_m <= 0:
                self.reject("UWB fields")
        self.result["uwb_unique_report_count"] = len(self.identities)

    def clock(self, m):
        self.hash_message("clock", m)
        self.result["counts"]["clock"] += 1
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if t < self.last_clock:
            self.result["clock_resets"] += 1
        if self.first_wall_ns is None:
            self.first_wall_ns = time.monotonic_ns()
            self.first_clock = t
        self.last_wall_ns = time.monotonic_ns()
        self.last_clock = t

    def graph(self):
        self.result["clock_publishers"] = self.count_publishers("/clock")
        if self.result["clock_publishers"] > 1:
            self.reject("multiple clock owners")
        try:
            topics = self.get_subscriber_names_and_types_by_node("liuo_tunnel_node", "/")
            self.result["estimator_subscriptions"] = [t[0] for t in topics]
            if any("truth" in t[0] for t in topics):
                self.reject("estimator truth subscription")
        except Exception:
            pass


def main():
    rclpy.init()
    node = Observer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.result["raw_sha256"] = {k: h.hexdigest() for k, h in node.hashes.items()}
        node.result["observed_wall_duration_s"] = (
            (node.last_wall_ns - node.first_wall_ns) * 1e-9
            if node.first_wall_ns is not None
            else None
        )
        values = node.result.pop("imu_static_acc_z")
        node.result["imu_static_acc_z_mean"] = sum(values) / len(values) if values else None
        path = Path(os.environ["LIUO_RUN_DIR"]) / "metadata/manifests/observation_audit.json"
        path.write_text(json.dumps(node.result, indent=2) + "\n")
        node.imu_log.close()
        node.odom_log.close()
        node.destroy_node()


if __name__ == "__main__":
    main()
