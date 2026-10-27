import unittest
from collections import Counter
import numpy as np
from analyze_anchor_map import distance_variance, topology


class MapDiagnostic(unittest.TestCase):
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
