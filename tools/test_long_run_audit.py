import math
from long_run_audit import attitude_distributions
import unittest
from long_run_audit import continuity, coverage


def row(stamp, x=0, vx=0, state="TRACKING"):
    return {
        "session": "1",
        "stamp": str(stamp),
        "x": str(x),
        "y": "0",
        "z": "1.2",
        "vx": str(vx),
        "vy": "0",
        "vz": "0",
        "qx": "0",
        "qy": "0",
        "qz": "0",
        "qw": "1",
        "wx": "0",
        "wy": "0",
        "wz": "0",
        "cov_valid": "1",
        "state": state,
    }


class LongRunAuditTest(unittest.TestCase):
    def setUp(self):
        self.config = {
            "uwb": {"mode": "fuse", "bootstrap_s": 8},
            "odometry": {"initialization_s": 2},
        }
        self.truth = [row(1 + i * 0.1) for i in range(87)]
        self.trajectory = [row(9 + i * 0.1) for i in range(7)]

    def test_declared_initialization_denominator_cannot_hide_late_start(self):
        full = continuity(self.trajectory, self.truth, self.config)
        self.assertEqual(full["expected_epochs"], 7)
        self.assertEqual(full["availability"], 1)
        late = continuity(self.trajectory[2:], self.truth, self.config)
        self.assertEqual(late["expected_epochs"], 7)
        self.assertEqual(late["states"]["MISSING"], 2)
        self.assertLess(late["availability"], 0.99)

    def test_degraded_is_reported_separately_lost_and_nonfinite_are_invalid(self):
        self.trajectory[1]["state"] = "DEGRADED"
        self.trajectory[2]["state"] = "LOST"
        self.trajectory[3]["qw"] = "nan"
        result = continuity(self.trajectory, self.truth, self.config)
        self.assertEqual(result["valid_epochs"], 5)
        self.assertEqual(result["states"]["DEGRADED"], 1)
        self.assertEqual(result["states"]["LOST"], 1)
        self.assertEqual(result["states"]["INVALID"], 1)

    def test_duplicate_and_unexpected_epochs_are_not_extra_availability(self):
        records = self.trajectory + [self.trajectory[0], row(100)]
        result = continuity(records, self.truth, self.config)
        self.assertEqual(result["availability"], 1)
        self.assertEqual(result["duplicate_epochs"], 1)
        self.assertEqual(result["unexpected_epochs"], 1)

    def test_full_route_and_physical_reverse_are_required(self):
        outbound = [row(i, x) for i, x in enumerate([0, 600, 1200, 1800, 2350])]
        self.assertFalse(coverage(outbound[:3], False)["outbound_complete"])
        self.assertTrue(coverage(outbound, False)["outbound_complete"])
        self.assertFalse(coverage(outbound + [row(100, 0)], True)["return_complete"])
        returning = outbound + [row(90, 2000, -5), row(100, 0)]
        self.assertTrue(coverage(returning, True)["return_complete"])
        self.assertFalse(coverage(returning, False)["return_complete"])

    def test_signed_attitude_axes_and_antipodal_quaternion(self):
        for axis, key in [("roll", "qx"), ("pitch", "qy"), ("yaw", "qz")]:
            state = row(1)
            state.update(qx="0", qy="0", qz="0", qw=str(math.cos(math.pi / 12)))
            state[key] = str(-math.sin(math.pi / 12))
            antipodal = dict(state)
            for k in ["qx", "qy", "qz", "qw"]:
                antipodal[k] = str(-float(state[k]))
            result = attitude_distributions([state, antipodal])
            self.assertAlmostEqual(result["axes"][axis]["mean_deg"], -30)
            self.assertAlmostEqual(result["axes"][axis]["absolute_p95_deg"], 30)
            for other in set(["roll", "pitch", "yaw"]) - {axis}:
                self.assertAlmostEqual(result["axes"][other]["rmse_deg"], 0)
        with self.assertRaises(ValueError):
            attitude_distributions([dict(state, qw="nan")])
