import math
from pathlib import Path
import tempfile
import unittest
from covariance_analysis import summarize


class CovarianceAnalysis(unittest.TestCase):
    def test_joint_units_and_known_rotation(self):
        state = dict(
            session="1",
            stamp="2",
            x="0",
            y="0",
            z="0",
            vx="0",
            vy="0",
            vz="0",
            qx="0",
            qy="0",
            qz=str(math.sin(math.pi / 4)),
            qw=str(math.cos(math.pi / 4)),
        )
        truth = dict(session="99", stamp="2", x="0", y="0", z="0", vx="2", vy="0", vz="0")
        cov = {f"xv_{i}_{j}": str(float(i == j)) for i in range(9) for j in range(9)}
        cov.update(session="1", stamp="2")
        with tempfile.TemporaryDirectory() as directory:
            result = summarize(Path(directory), [state], [truth], [cov], True)
            self.assertAlmostEqual(result["joint_nees_mean_9d"], math.pi**2 / 4 + 4)
            self.assertAlmostEqual(result["position_nees_mean_3d"], 0)
            cov["xv_0_0"] = "-1"
            with self.assertRaises(Exception):
                summarize(Path(directory), [state], [truth], [cov], True)
