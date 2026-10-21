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
