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
            truth.update({k: state[k] for k in ["qx", "qy", "qz", "qw"]})
            result = summarize(Path(directory), [state], [truth], [cov], True)
            self.assertAlmostEqual(result["joint_nees_mean_9d"], 4)
            cov["xv_0_0"] = "-1"
            with self.assertRaises(Exception):
                summarize(Path(directory), [state], [truth], [cov], True)

    def test_off_initial_translation_and_yaw_reference(self):
        base = dict(
            session="99",
            stamp="1",
            x=200.0,
            y=0.8,
            z=2.0,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            qx=0.0,
            qy=0.0,
            qz=math.sqrt(0.5),
            qw=math.sqrt(0.5),
        )
        reference = dict(base, stamp="2", y=3.8, vy=2.0)
        state = dict(
            session="1",
            stamp="2",
            x=3.0,
            y=0.0,
            z=0.0,
            vx=2.0,
            vy=0.0,
            vz=0.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
        )
        cov = {f"xv_{i}_{j}": float(i == j) for i in range(9) for j in range(9)}
        cov.update(session="1", stamp="2")
        with tempfile.TemporaryDirectory() as directory:
            result = summarize(Path(directory), [state], [base, reference], [cov], False)
            self.assertAlmostEqual(result["position_nees_mean_3d"], 0.0)
            self.assertAlmostEqual(result["joint_nees_mean_9d"], 0.0)
