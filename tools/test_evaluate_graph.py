import unittest
from evaluate import raw_lidar_graph_present


class RawGraphContractTest(unittest.TestCase):
    def test_phase2_requires_three_pose_raw_graph_not_just_pairwise_output(self):
        config = dict(schema_version="liuo_phase2_v1")
        graph = [
            dict(X="1", lidar_factors="0", imu_factors="0"),
            dict(X="2", lidar_factors="1", imu_factors="1"),
            dict(X="3", lidar_factors="2", imu_factors="1"),
        ]
        self.assertTrue(raw_lidar_graph_present(graph, config))
        self.assertFalse(raw_lidar_graph_present(graph[:2], config))
        self.assertFalse(
            raw_lidar_graph_present(
                graph + [dict(X="4", lidar_factors="0", imu_factors="1")], config
            )
        )
        self.assertFalse(
            raw_lidar_graph_present(
                graph + [dict(X="4", lidar_factors="2", imu_factors="0")], config
            )
        )
        self.assertFalse(raw_lidar_graph_present([], config))

    def test_phase1_legacy_edge_count_is_preserved(self):
        graph = [dict(X="3", lidar_factors="2", imu_factors="1")]
        self.assertFalse(raw_lidar_graph_present(graph, {}))
        graph.append(dict(X="4", lidar_factors="3", imu_factors="1"))
        self.assertTrue(raw_lidar_graph_present(graph, {}))
