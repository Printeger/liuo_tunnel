import unittest
import json
import tempfile
from pathlib import Path
from collections import Counter
import numpy as np
from analyze_anchor_map import distance_variance, topology, truth_inputs


class MapDiagnostic(unittest.TestCase):
    def test_straight_truth_checks_layout_range_and_delivery(self):
        with tempfile.TemporaryDirectory() as root:
            run = Path(root)
            (run/'metadata/config').mkdir(parents=True)
            (run/'export/simulation').mkdir(parents=True)
            scenario = run/'metadata/config/scenario.json'
            scenario.write_text(json.dumps(dict(scene='empty_tunnel_straight_v2',
                anchor_layout='high_first_four_corner_400m_7_anchors_v5')))
            physical = run/'export/simulation/physical_uwb.csv'
            header = 'a_type,a,b,true_range,delivered\n'
            distance = np.hypot(400, 5.6)
            physical.write_text(header+f'1,0,1,{distance},1\n1,1,0,{distance},0\n')
            truth, measured, delivered, provenance = truth_inputs(run, 'straight_csv')
            self.assertEqual(len(truth), 7)
            self.assertEqual(measured, Counter({(0,1):2}))
            self.assertEqual(delivered, Counter({(0,1):1}))
            self.assertEqual(provenance['format'], 'straight_csv')
            physical.write_text(header+'1,0,1,1,1\n')
            with self.assertRaisesRegex(ValueError, 'disagrees'):
                truth_inputs(run, 'straight_csv')
            scenario.write_text(json.dumps(dict(scene='unknown', anchor_layout='unknown')))
            with self.assertRaisesRegex(ValueError, 'does not match'):
                truth_inputs(run, 'straight_csv')

    def test_distance_uncertainty_preserves_common_translation_correlation(self):
        points=np.array([[0.,0.,0.],[3.,4.,0.]])
        covariance=np.kron(np.ones((2,2)),100*np.eye(3))+.01*np.eye(6)
        self.assertAlmostEqual(distance_variance(covariance,points,0,1),.02,places=10)
        angle=.8; R=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        Q=np.kron(np.eye(2),R)
        self.assertAlmostEqual(distance_variance(Q@covariance@Q.T,points@R.T,0,1),.02,places=10)

    def test_graph_distinguishes_isolates_and_common_gauge(self):
        positions={9:np.array([0.,0.,0.]),2:np.array([1.,0.,0.]),17:np.array([0.,1.,0.]),8:np.array([2.,2.,2.])}
        result=topology(positions,Counter({(2,9):10,(9,17):20,(2,17):30}))
        self.assertEqual(result['rigidity_rank'],3)
        self.assertEqual(result['common_rigid_gauge_rank'],6)
        self.assertEqual(result['nullity_after_common_rigid_gauge'],3)
        self.assertIn([8],result['components'])
        empty=topology(positions,Counter())
        self.assertEqual(empty['rigidity_rank'],0)
        self.assertEqual(len(empty['components']),4)


if __name__=='__main__': unittest.main()
