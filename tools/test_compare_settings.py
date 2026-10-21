import copy
import unittest
from compare_runs import estimator_settings


class PairedSettingsTest(unittest.TestCase):
    def test_uwb_toggle_does_not_hide_different_lidar_configuration(self):
        baseline = {
            "scan_model": "snapshot",
            "sensors": {"imu_rate_hz": 200},
            "odometry": {"connection_window": 1},
            "uwb": {"mode": "fuse"},
        }
        off = copy.deepcopy(baseline)
        off["uwb"]["mode"] = "record_only"
        self.assertEqual(estimator_settings(baseline), estimator_settings(off))
        off["odometry"]["connection_window"] = 3
        self.assertNotEqual(estimator_settings(baseline), estimator_settings(off))

    def test_documented_defaults_match_only_equivalent_settings(self):
        baseline = {"scan_model": "snapshot", "sensors": {}, "odometry": {}}
        explicit = copy.deepcopy(baseline)
        explicit["odometry"] = {
            "max_keyframes": 15,
            "relinearize_threshold": 0.05,
            "use_dogleg": False,
            "factorization": "CHOLESKY",
        }
        self.assertEqual(estimator_settings(baseline), estimator_settings(explicit))
        explicit["odometry"]["use_dogleg"] = True
        self.assertNotEqual(estimator_settings(baseline), estimator_settings(explicit))
