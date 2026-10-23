#!/usr/bin/env python3
"""Display-only bounded cloud accumulation and raw range annotations. No truth input."""
import json
import os
from collections import OrderedDict, deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from liuo_tunnel_msgs.msg import UwbRange


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Display(Node):
    def __init__(self):
        super().__init__('liuo_display')
        self.cells = OrderedDict()
        self.anchors = {}
        self.poses = deque(maxlen=800)
        self.ranges = {}
        self.header = None
        self.last_stamp = -1
        self.last_clock = -1
        self.stats = dict(clouds=0, map_publications=0, received_ranges=0, drawn_links=0, max_voxels=0, max_anchors=0, resets=0)
        self.health = 'INITIALIZING: no certified pose'
        self.tag = json.loads(Path(os.environ['LIUO_VIS_CONFIG']).read_text())['sensors']['T_BU_translation']
        self.map_pub = self.create_publisher(PointCloud2, '/liuo/visual/cloud_map', 1)
        self.range_pub = self.create_publisher(MarkerArray, '/liuo/visual/ranges', 2)
        self.create_subscription(PointCloud2, '/liuo/visual/current_cloud', self.cloud, 2)
        self.create_subscription(MarkerArray, '/liuo/visual/anchors', self.anchor, 2)
        self.create_subscription(Odometry, '/liuo/odom', self.pose, 100)
        self.create_subscription(UwbRange, '/sensors/uwb/range', self.range, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/liuo/diagnostics', self.status, 10)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish)

    def clock(self, msg):
        stamp = seconds(msg.clock)
        if stamp < self.last_clock:
            self.stats['resets'] += 1
            self.cells.clear()
            self.poses.clear()
            self.anchors.clear()
            self.ranges.clear()
            self.header = Header(frame_id='map', stamp=msg.clock)
            self.last_stamp = -1
            self.health = 'INITIALIZING: time reset'
            self.publish()
        self.last_clock = stamp

    def status(self, msg):
        self.health = ' | '.join(x.message for x in msg.status)

    def anchor(self, msg):
        self.anchors = {m.id: m.pose.position for m in msg.markers if m.ns == 'estimated_anchor'}
        self.stats['max_anchors'] = max(self.stats['max_anchors'], len(self.anchors))

    def pose(self, msg):
        self.poses.append(msg)

    def range(self, msg):
        self.stats['received_ranges'] += 1
        # Last raw observation per link; unknown anchors remain numeric-only.
        self.ranges[(msg.node_a_type, msg.node_a_id, msg.node_b_type, msg.node_b_id)] = msg
        if len(self.ranges) > 256:
            self.ranges.pop(next(iter(self.ranges)))

    def cloud(self, msg):
        self.stats['clouds'] += 1
        stamp = seconds(msg.header.stamp)
        if stamp < self.last_stamp:
            self.cells.clear()
            self.poses.clear()
            self.anchors.clear()
            self.ranges.clear()
        self.last_stamp = stamp
        self.header = msg.header
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        # 0.4 m display voxels, oldest eviction at 150k. Never estimator feedback.
        for p in xyz:
            key = tuple(np.floor(p / .4).astype(int))
            self.cells[key] = tuple(float(v) for v in p)
            self.cells.move_to_end(key)
        while len(self.cells) > 150000:
            self.cells.popitem(last=False)
        self.stats['max_voxels'] = max(self.stats['max_voxels'], len(self.cells))

    def endpoint(self, kind, ident, stamp):
        if kind == UwbRange.NODE_ANCHOR:
            return self.anchors.get(ident)
        if not self.poses:
            return None
        pose = min(self.poses, key=lambda p: abs(seconds(p.header.stamp) - stamp))
        if abs(seconds(pose.header.stamp) - stamp) > .06:
            return None
        q = pose.pose.pose.orientation
        v = np.array(self.tag)
        u = np.array([q.x, q.y, q.z])
        v = v + 2 * np.cross(u, np.cross(u, v) + q.w * v)
        p = pose.pose.pose.position
        return Point(x=p.x + v[0], y=p.y + v[1], z=p.z + v[2])

    def publish(self):
        if self.header is None:
            self.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(self.header, list(self.cells.values())))
        self.stats['map_publications'] += 1
        out = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        out.markers.append(clear)
        now = self.get_clock().now().nanoseconds * 1e-9
        lines = []
        for key, r in self.ranges.items():
            stamp = seconds(r.header.stamp)
            if not 0 <= now - stamp <= 2:
                continue
            label = f'{"T" if key[0] == 0 else "A"}{key[1]}-A{key[3]} {r.range_m:.2f} m (status {r.status})'
            lines.append(label)
            a = self.endpoint(key[0], key[1], stamp)
            b = self.endpoint(key[2], key[3], stamp)
            if a is None or b is None or r.status != UwbRange.STATUS_OK:
                continue
            marker = Marker(header=self.header, ns='raw_range_links', id=len(out.markers), type=Marker.LINE_LIST)
            marker.pose.orientation.w = 1.
            marker.points = [a, b]
            marker.scale.x = .04
            marker.color.g = marker.color.b = marker.color.a = 1.
            marker.lifetime.sec = 2
            out.markers.append(marker)
            self.stats['drawn_links'] += 1
        label = Marker(header=self.header, ns='raw_ranges_and_health', id=0, type=Marker.TEXT_VIEW_FACING)
        label.pose.orientation.w = 1.
        if self.poses:
            label.pose.position = self.poses[-1].pose.pose.position
            label.pose.position = Point(x=label.pose.position.x, y=label.pose.position.y, z=label.pose.position.z + 5.)
        label.scale.z = .5
        label.color.r = label.color.g = label.color.b = label.color.a = 1.
        label.lifetime.sec = 2
        label.text = self.health + '\nRAW received (not acceptance):\n' + '\n'.join(lines)
        out.markers.append(label)
        self.range_pub.publish(out)


def main():
    rclpy.init()
    node = Display()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        (Path(os.environ['LIUO_RUN_DIR']) / 'runtime/visualization_summary.json').write_text(json.dumps(node.stats, indent=2) + '\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
