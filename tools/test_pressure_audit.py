import unittest
from pressure_audit import interval, sustained_return, recovery_bins


class PressureEvidenceTest(unittest.TestCase):
    def test_large_outage_errors_are_retained_not_averaged_with_recovery(self):
        data = [
            dict(
                stamp=i * 0.1,
                error_m=10 if i < 10 else 0.1,
                axial_m=9 if i < 10 else 0.05,
                state="DEGRADED" if i < 10 else "TRACKING",
            )
            for i in range(30)
        ]
        result = interval(data, 0, 1)
        self.assertEqual(result["position_rmse_m"], 10)
        self.assertEqual(result["axial_max_m"], 9)
        self.assertEqual(result["states"], {"DEGRADED": 10})
        self.assertEqual(sustained_return(data, 0), 1)

    def test_short_lucky_return_or_publication_gap_is_not_sustained_recovery(self):
        short = [dict(stamp=i * 0.1, error_m=0.1) for i in range(10)]
        self.assertIsNone(sustained_return(short, 0))
        gapped = short + [dict(stamp=2, error_m=0.1)]
        self.assertIsNone(sustained_return(gapped, 0))
        short.append(dict(stamp=1, error_m=0.4))
        self.assertIsNone(sustained_return(short, 0))

    def test_small_residual_does_not_hide_large_position_error(self):
        trajectory = [dict(stamp=10.1, error_m=5, axial_m=4, state="TRACKING")]
        decisions = [
            dict(
                measurement_stamp="10.1",
                reason="ACCEPT_TA",
                residual_class="PRE_OPT_INNOVATION",
                whitened="1",
            )
        ]
        result = recovery_bins(trajectory, decisions, 10, 12)
        self.assertEqual(result[0]["measurement_sigma_normalized_innovation_rms"], 1)
        self.assertEqual(result[0]["navigation"]["position_rmse_m"], 5)
        decisions[0]["reason"] = "DUPLICATE"
        self.assertIsNone(
            recovery_bins(trajectory, decisions, 10, 12)[0][
                "measurement_sigma_normalized_innovation_rms"
            ]
        )
        decisions[0]["reason"] = "ACCEPT_TA"
        decisions[0]["whitened"] = "nan"
        with self.assertRaises(ValueError):
            recovery_bins(trajectory, decisions, 10, 12)
