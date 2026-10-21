import unittest
from campaign_audit import MODES, coverage_failures


class CampaignCoverageTest(unittest.TestCase):
    def setUp(self):
        self.entries = [dict(seed=seed, mode=mode) for seed in range(10) for mode in MODES]

    def test_requires_every_seed_and_both_maps_plus_off(self):
        self.assertEqual(coverage_failures(self.entries), [])
        failures = coverage_failures(self.entries[:-1])
        self.assertEqual(len(failures), 1)
        self.assertIn("seed 9 surveyed_map", failures[0])

    def test_duplicate_cannot_replace_a_missing_seed(self):
        entries = self.entries[:-1] + [self.entries[0]]
        self.assertEqual(len(coverage_failures(entries)), 2)

    def test_unknown_mode_or_extra_seed_rejected(self):
        self.assertTrue(coverage_failures(self.entries + [dict(seed=10, mode="oracle_map")]))
        self.assertTrue(coverage_failures(self.entries + [dict(seed=0, mode="oracle")]))


class IndependentSummaryTest(unittest.TestCase):
    def test_equal_run_weight_retains_bad_run_and_missing_values(self):
        from campaign_audit import independent_run_summary

        entries = [
            dict(
                run_id="good",
                seed=0,
                mode="surveyed_map",
                metrics=dict(position_rmse_m=0.1, covariance_diagnostics=dict(samples=10000)),
            ),
            dict(
                run_id="bad",
                seed=1,
                mode="surveyed_map",
                metrics=dict(position_rmse_m=0.9, covariance_diagnostics=dict(samples=10)),
            ),
            dict(
                run_id="invalid",
                seed=2,
                mode="surveyed_map",
                metrics=dict(position_rmse_m=float("nan")),
            ),
        ]
        stats = independent_run_summary(entries)["modes"]["surveyed_map"]["metrics"][
            "position_rmse_m"
        ]
        self.assertEqual(stats["mean"], 0.5)
        self.assertEqual(stats["max"], 0.9)
        self.assertEqual(stats["n_runs"], 2)
        self.assertEqual(stats["unavailable_runs"], ["invalid"])
        self.assertEqual(stats["observations"][1]["run_id"], "bad")
        self.assertTrue(coverage_failures(entries))

    def test_single_run_has_no_sample_spread_and_empty_mode_no_mean(self):
        from campaign_audit import independent_run_summary

        summary = independent_run_summary(
            [dict(run_id="only", seed=0, mode="oracle_map", metrics=dict(position_rmse_m=0.2))]
        )["modes"]
        self.assertIsNone(summary["oracle_map"]["metrics"]["position_rmse_m"]["sample_sd"])
        self.assertIsNone(summary["record_only"]["metrics"]["position_rmse_m"]["mean"])
