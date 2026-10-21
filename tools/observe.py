#!/usr/bin/env python3
"""Independent runtime raw-message/graph audit; no estimator access to truth."""
import json
import os
from pathlib import Path
import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from liuo_tunnel_msgs.msg import UwbRange
from rosgraph_msgs.msg import Clock


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
        self.last_clock = 0
        self.identities = set()
        self.imu_log = open(Path(os.environ["LIUO_RUN_DIR"]) / "export/capture/imu.csv", "w")
        self.imu_log.write("stamp,ax,ay,az,wx,wy,wz\n")
        self.create_subscription(Imu, "/sensors/imu", self.imu, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, "/sensors/lidar/points", self.cloud, qos_profile_sensor_data
        )
        self.create_subscription(UwbRange, "/sensors/uwb/range", self.uwb, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.clock, qos_profile_sensor_data)
        self.create_timer(1, self.graph)

    def reject(self, reason):
        if reason not in self.result["violations"]:
            self.result["violations"].append(reason)

    def imu(self, m):
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
        self.result["counts"]["uwb"] += 1
        identity = (m.producer_id, m.session_id, m.sequence)
        if identity in self.identities:
            self.reject("duplicate UWB identity")
        self.identities.add(identity)
        if m.session_id not in self.result["sessions"]:
            self.result["sessions"].append(m.session_id)
        if (
            m.header.frame_id
            or abs(m.variance_m2 - 0.01) > 1e-10
            or not math.isfinite(m.range_m)
            or m.range_m <= 0
            or m.propagation_status != 0
        ):
            self.reject("UWB fields")

    def clock(self, m):
        self.result["counts"]["clock"] += 1
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if t < self.last_clock:
            self.result["clock_resets"] += 1
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
        values = node.result.pop("imu_static_acc_z")
        node.result["imu_static_acc_z_mean"] = sum(values) / len(values) if values else None
        path = Path(os.environ["LIUO_RUN_DIR"]) / "metadata/manifests/observation_audit.json"
        path.write_text(json.dumps(node.result, indent=2) + "\n")
        node.imu_log.close()
        node.destroy_node()


if __name__ == "__main__":
    main()
