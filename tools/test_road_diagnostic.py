import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation


class RoadDiagnosticTest(unittest.TestCase):
    def test_fixed_initial_gauge_does_not_fit_away_later_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            truth = root / 'truth/export/simulation'; truth.mkdir(parents=True)
            run = root / 'estimate/export/odometry'; run.mkdir(parents=True)
            t = np.arange(0, 12.01, .1)
            positions = np.column_stack([10*t, 0*t, 0*t])
            rotations = Rotation.from_euler('z', .01*t)
            np.savez(truth / 'trajectory.npz', t=t, p=positions, R=rotations.as_matrix())
            gauge = Rotation.from_euler('z', .7)
            # Deterministic growing error must remain after initial gauge fixing.
            drift = np.column_stack([.1*t*t, 0*t, 0*t])
            estimated = gauge.inv().apply(positions + drift - [3, 4, 5])
            quaternions = (gauge.inv() * rotations).as_quat()
            with (run / 'trajectory.csv').open('w') as f:
                writer = csv.writer(f); writer.writerow(['stamp','x','y','z','qx','qy','qz','qw'])
                writer.writerows([[time+1, *p, *q] for time,p,q in zip(t,estimated,quaternions)])
            subprocess.run([sys.executable, str(Path(__file__).with_name('analyze_road_diagnostic.py')),
                '--run', str(root / 'estimate'), '--truth-run', str(root / 'truth')], check=True, stdout=subprocess.DEVNULL)
            result = json.loads((root / 'estimate/export/analysis/road_fixed_start_diagnostic.json').read_text())
            self.assertAlmostEqual(result['position_rmse_m'], np.sqrt(np.mean((.1*t*t)**2)), places=8)
            self.assertAlmostEqual(result['maximum_position_error_m'], 14.4, places=8)
            self.assertLess(result['attitude_rmse_deg'], 1e-8)
            expected = .1*((t[t <= 11]+1)**2-t[t <= 11]**2)
            self.assertAlmostEqual(result['rpe']['1.0']['translation_rmse_m'], np.sqrt(np.mean(expected**2)), places=8)


if __name__ == '__main__':
    unittest.main()
