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

    def test_completed_calibration_candidate_is_progress_but_clock_is_not(self):
        from run import output_progress_token
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            odometry = root / 'odometry.csv'
            candidates = root / 'calibration_branches.csv'
            odometry.write_text('header\n')
            before = output_progress_token(odometry, candidates)
            (root / 'clock.csv').write_text('more sensor input\n')
            self.assertEqual(before, output_progress_token(odometry, candidates))
            candidates.write_text('header\n1,17,19,0,0,1,2,3,10,SOLVED\n')
            self.assertNotEqual(before, output_progress_token(odometry, candidates))
            self.assertEqual(output_progress_token(odometry), (odometry.stat().st_size, 0))
