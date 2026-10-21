import unittest
from run import watchdog_failure


class RunWatchdogTest(unittest.TestCase):
    def test_total_timeout_even_when_both_outputs_progress(self):
        self.assertIn(
            "WALL_TIMEOUT", watchdog_failure(121, 120, {"simulation": 0, "odometry": 0}, 30)
        )

    def test_each_module_must_progress_independently(self):
        self.assertIn("odometry", watchdog_failure(40, 120, {"simulation": 0, "odometry": 31}, 30))
        self.assertIn(
            "simulation", watchdog_failure(40, 120, {"simulation": 31, "odometry": 0}, 30)
        )
        self.assertIsNone(watchdog_failure(40, 120, {"simulation": 1, "odometry": 1}, 30))
