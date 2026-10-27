#!/usr/bin/env python3
import json
from pathlib import Path
import tempfile
import unittest
from analyze_candidate_branches import analyze


class CandidateDampingTest(unittest.TestCase):
    def report(self, events):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run/'metadata/config').mkdir(parents=True)
            (run/'metadata/config/phase1.json').write_text(json.dumps({'uwb': {}}))
            (run/'profiling').mkdir()
            (run/'profiling/anchor_extension.csv').write_text(
                'session,stamp,anchor,event,value_a,value_b\n'
                '1,1,3,CANDIDATES_STARTED,0,10\n'+events)
            return analyze(run)['damping_diagnostic']

    def test_old_logs_are_unavailable(self):
        self.assertFalse(self.report('')['available'])

    def test_ceiling_separate_from_numerical_cost_stop(self):
        report = self.report('1,2,3,CANDIDATE_DAMPING,100000,100000\n'
                             '1,3,3,CANDIDATE_DAMPING,1,100000\n')
        self.assertEqual(report['logged_candidates'], 2)
        self.assertEqual(report['at_or_above_upper_bound'], 1)

    def test_invalid_measurement_rejected(self):
        with self.assertRaises(ValueError):
            self.report('1,2,3,CANDIDATE_DAMPING,nan,100000\n')


if __name__ == '__main__':
    unittest.main()
