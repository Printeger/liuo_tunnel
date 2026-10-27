#!/usr/bin/env python3
"""Regression: REUSE error must survive fixed CAL gauge evaluation."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from analyze_road_diagnostic import route_error_components


class FixedMapGauge(unittest.TestCase):
    def test_route_components_keep_error_norm_and_use_motion_not_body_heading(self):
        t = np.arange(4.)
        velocity = np.array([[0, 0, 0], [0, 2, .5], [3, 0, .5], [0, 0, 0]])
        error = np.tile([2., 3., 4.], (4, 1))
        actual = route_error_components(t, error, t, velocity)
        np.testing.assert_allclose(actual, [[3, -2, 4], [3, -2, 4], [2, 3, 4], [2, 3, 4]])
        np.testing.assert_allclose(np.linalg.norm(actual, axis=1), np.linalg.norm(error, axis=1))
        with self.assertRaises(ValueError):
            route_error_components(t, error, t, np.zeros((4, 3)))

    def test_reuse_start_offset_is_not_aligned_away(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cal, reuse, truth = [root / name for name in ['cal','reuse','truth']]
            for run in [cal,reuse,truth]:
                for category in ['export/analysis','export/odometry','export/simulation','metadata/config']:
                    (run/category).mkdir(parents=True)
            (cal/'export/analysis/road_fixed_start_diagnostic.json').write_text(json.dumps(dict(
                gauge='first_output_yaw_and_translation_only_no_route_fit',
                yaw_rotation_matrix=np.eye(3).tolist(),translation=[10,0,0],reference_stamp=3.1)))
            (reuse/'metadata/config/reuse_map.json').write_text(json.dumps(dict(source_run=str(cal),map_id='old-map')))
            t=np.arange(0,3.1,.1)
            p=np.column_stack([t,np.zeros_like(t),np.zeros_like(t)])
            np.savez(truth/'export/simulation/trajectory.npz',t=t,p=p,
                     v=np.tile([1.,0.,0.],(len(t),1)),R=np.tile(np.eye(3),(len(t),1,1)))
            with (truth/'export/simulation/truth.csv').open('w') as f:
                w=csv.writer(f);w.writerow(['stamp','x','y','z','vx','vy','vz','qx','qy','qz','qw'])
                for value in t:
                    w.writerow([value+1,value,0,0,1,0,0,0,0,0,1])
            with (reuse/'export/odometry/trajectory.csv').open('w') as f:
                w=csv.writer(f);w.writerow(['stamp','x','y','z','qx','qy','qz','qw','map_id'])
                for i,value in enumerate(t):
                    w.writerow([value+1,value+5,0,0,0,0,0,1,'pending' if i==0 else 'old-map'])
            for format_name in ['road_npz','straight_csv']:
                with self.subTest(truth_format=format_name):
                    subprocess.run([sys.executable,str(Path(__file__).with_name('analyze_road_diagnostic.py')),
                                    '--run',str(reuse),'--truth-run',str(truth),'--map-reference-run',str(cal),
                                    '--truth-format',format_name],check=True,stdout=subprocess.DEVNULL)
                    result=json.loads((reuse/'export/analysis/road_fixed_map_diagnostic.json').read_text())
                    self.assertAlmostEqual(result['position_rmse_m'],15)
                    np.testing.assert_allclose(result['route_frame']['along_cross_height_rmse_m'], [15,0,0])
                    self.assertEqual(result['pre_map_output_samples'],1)
                    self.assertEqual(result['samples'],len(t)-1)
                    self.assertEqual(result['reference_stamp'],3.1)
                    self.assertEqual(result['gauge'],'inherited_CAL_gauge_no_REUSE_alignment')

if __name__=='__main__': unittest.main()
