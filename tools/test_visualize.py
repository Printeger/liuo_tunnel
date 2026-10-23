"""Run with the project ROS environment sourced: python3 tools/test_visualize.py."""
import os
from pathlib import Path
import unittest
import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from visualize import Display


class DisplayTest(unittest.TestCase):
    def test_frames_timestamps_reset(self):
        os.environ['LIUO_VIS_CONFIG'] = str(Path(__file__).resolve().parents[1] / 'packages/liuo_tunnel/config/phase3_self_calibrate.json')
        rclpy.init()
        node = Display()
        try:
            header = Header(frame_id='map')
            header.stamp.sec = 10
            node.cloud(create_cloud_xyz32(header, [(1., 2., 3.), (1.01, 2.01, 3.01)]))
            self.assertEqual(len(node.cells), 1)
            pose = Odometry(header=header)
            pose.pose.pose.orientation.w = 1.
            pose.pose.pose.position.x = 4.
            node.pose(pose)
            endpoint = node.endpoint(0, 0, 10)
            self.assertAlmostEqual(endpoint.x, 4.1)
            self.assertAlmostEqual(endpoint.z, .25)
            self.assertIsNone(node.endpoint(0, 0, 9))
            self.assertIsNone(node.endpoint(1, 99, 10))
            node.publish()
            self.assertEqual(pose.pose.pose.position.z, 0)
            clock = Clock()
            clock.clock.sec = 10
            node.clock(clock)
            clock.clock.sec = 1
            node.clock(clock)
            self.assertFalse(node.cells)
            self.assertFalse(node.poses)
            self.assertEqual(node.stats['resets'], 1)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
