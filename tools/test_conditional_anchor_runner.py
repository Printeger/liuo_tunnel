#!/usr/bin/env python3
"""Analytic data fixture for D1 serialization/solver boundary; not scene evidence."""
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np


class ConditionalRunnerTest(unittest.TestCase):
    def test_analytic_noiseless_ranges_without_reading_anchor_file(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source, joint = root/'source', root/'joint'
            simulation = source/'export/simulation'
            simulation.mkdir(parents=True)
            (joint/'metadata/manifests').mkdir(parents=True)
            (joint/'profiling').mkdir()
            # Invalid on purpose: the conditional solver must never read it.
            (simulation/'anchors.json').write_text('NOT A VALID ANCHOR DOCUMENT')
            points = np.array([[i, np.sin(i), np.cos(i)] for i in range(7)])
            anchors = [[3., 2., 1.], [5., -1., 2.]]
            with (simulation/'truth.csv').open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(['stamp','x','y','z','qx','qy','qz','qw'])
                for i, point in enumerate(points):
                    writer.writerow([i+1, *point, 0, 0, 0, 1])
            ta = {}
            for anchor, point in enumerate(anchors):
                for i, tag in enumerate(points):
                    ta[f'{anchor}-{i}'] = dict(measurement=dict(endpoints=[0,anchor], acquisition_ns=(i+1)*10**9,
                        range_m=float(np.linalg.norm(tag-point)), variance_m2=.01))
            aa = dict(edge=dict(measurement=dict(endpoints=[0,1], acquisition_ns=10**9,
                range_m=float(np.linalg.norm(np.asarray(anchors[0])-anchors[1])), variance_m2=.01)))
            saved = dict(source_run=str(source), raw_sha256='analytic_fixture', seconds=6,
                         measurement_set_sha256='analytic_fixture', sensors=dict(T_BU_translation=[0,0,0]),
                         samples=dict(ta=ta, aa=aa))
            observations = root/'observations.json'
            observations.write_text(json.dumps(saved))
            (joint/'metadata/manifests/input_identity.json').write_text(json.dumps(dict(raw_sha256='analytic_fixture',seconds=6)))
            for name in ('local_geometry.jsonl','candidate_inputs.jsonl'):
                (joint/'profiling'/name).write_text('')
            output = root/'result'
            environment = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
            subprocess.run([sys.executable, str(Path(__file__).with_name('solve_conditional_anchors.py')),
                            '--observations',str(observations),'--joint-run',str(joint),
                            '--truth-format','straight_csv','--output',str(output)], check=True, env=environment, timeout=30,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            report = json.loads((output/'report.json').read_text())
            self.assertEqual(len(report['candidates']),32)
            self.assertGreater(report['converged_feasible_count'],0)
            best = min(report['candidates'],key=lambda r:r['cost'])
            self.assertLess(best['cost'],1e-8)
            np.testing.assert_allclose(best['anchors'],anchors,atol=1e-5)
            self.assertEqual(len(best['information_eigenvalues']),6)


if __name__ == '__main__':
    unittest.main()
