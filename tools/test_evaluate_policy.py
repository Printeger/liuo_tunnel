import unittest

from evaluate import acceptance_policy
from pressure_audit import accuracy_comparison


class RepresentativeAcceptanceTest(unittest.TestCase):
    def test_accuracy_exceedances_do_not_block_or_certify_stage(self):
        failures = [
            "known-map position RMSE >0.30m",
            "known-map axial P95 >0.50m",
            "known-map axial max >1.0m",
            "known-map attitude P95 >2deg",
            "entrance tracking sanity RMSE > 1 m",
        ]
        report = acceptance_policy(failures, True)
        self.assertEqual(report["result"], "PASS")
        self.assertEqual(report["failures"], [])
        self.assertEqual(report["accuracy_reference_exceedances"], failures)
        self.assertEqual(report["stage_acceptance"], "NOT_ASSESSED")
        # Pressure summaries must not silently report that accuracy passed.
        comparison = accuracy_comparison(report)
        self.assertEqual(comparison["result"], "FAIL")
        self.assertEqual(set(comparison["exceedances"]), set(failures))

    def test_real_failures_are_not_removed_by_accuracy_policy(self):
        for failure in ["nonfinite trajectory", "raw topics missing", "static anchors not retained", "processing P95 >=100ms"]:
            with self.subTest(failure=failure):
                report = acceptance_policy(["known-map position RMSE >0.30m", failure], True)
                self.assertEqual(report["result"], "FAIL")
                self.assertEqual(report["failures"], [failure])

    def test_phase1_and_historical_input_are_unchanged(self):
        failures = ["entrance tracking sanity RMSE > 1 m"]
        report = acceptance_policy(failures, False)
        self.assertEqual(report["result"], "FAIL")
        self.assertEqual(report["accuracy_reference_exceedances"], [])
        acceptance_policy(failures, True)
        self.assertEqual(failures, ["entrance tracking sanity RMSE > 1 m"])
        legacy = {"failures": ["known-map position RMSE >0.30m"]}
        self.assertEqual(accuracy_comparison(legacy)["exceedances"], legacy["failures"])
