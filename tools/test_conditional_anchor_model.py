#!/usr/bin/env python3
import unittest
import numpy as np
from conditional_anchor_model import AnchorProblem


class ConditionalAnchorModelTest(unittest.TestCase):
    def test_probe_cannot_spend_final_domain_tolerance_to_claim_descent(self):
        class BoundaryProblem(AnchorProblem):
            def __init__(self):
                pass
            def domain(self, flat):
                return np.array([flat[0]+.0003*flat[1]])
            def residual_jacobian(self, flat):
                return np.sqrt(20000)*(1+self.domain(flat)), np.sqrt(20000)*np.array([[1.,.0003]])
        result = BoundaryProblem().feasible_probe([0.,0.])
        # A -0.1 mm y step leaves the domain by 3e-8 m; the old 1e-7
        # acceptance would falsely report a roughly 0.0006 cost decrease.
        self.assertEqual(min(p['minimum_cost_change'] for p in result['probes']),0.)

    def test_independent_finite_difference_gradient_and_raw_information(self):
        ta = [dict(anchor=4, tag=[x, 0., 0.], range=3.+x, variance=.01) for x in (0., 1., 2.)]
        aa = [dict(first=4, second=9, range=5., variance=.04)]
        problem = AnchorProblem([4, 9], ta, aa)
        state = np.array([2., 3., 1., 7., 1., 2.])
        residual, jacobian = problem.residual_jacobian(state)
        step = 1e-5
        numeric = np.column_stack([(problem.residual_jacobian(state+np.eye(6)[i]*step)[0]
                                    -problem.residual_jacobian(state-np.eye(6)[i]*step)[0])/(2*step) for i in range(6)])
        np.testing.assert_allclose(jacobian, numeric, rtol=1e-8, atol=1e-8)
        cost, gradient = problem.objective(state)
        self.assertAlmostEqual(cost, sum(float(r*r)/2 for r in residual))
        np.testing.assert_allclose(gradient, numeric.T@residual, rtol=1e-8, atol=1e-7)
        # AA joint block has off-diagonal information, not two independent priors.
        information = jacobian.T@jacobian
        self.assertGreater(np.linalg.norm(information[:3, 3:]), 1.)
        self.assertLess(min(p['minimum_cost_change'] for p in problem.feasible_probe(state)['probes']), -1e-6)

    def test_straight_ranges_have_exact_rotational_ambiguity(self):
        ta = [dict(anchor=1, tag=[x, 0., 0.], range=np.hypot(x-4, 3), variance=.01) for x in range(9)]
        problem = AnchorProblem([1], ta, [])
        for point in ([4., 3., 0.], [4., 0., -3.], [4., -3., 0.]):
            residual, jacobian = problem.residual_jacobian(point)
            self.assertLess(np.linalg.norm(residual), 1e-12)
            self.assertLess(np.linalg.eigvalsh(jacobian.T@jacobian)[0], 1e-10)
        self.assertEqual(problem.range_seeds(1).shape, (8, 3))

    def test_measured_patch_domain_and_plane_do_not_add_height_residual(self):
        floor = dict(normal=[0,0,1], center=[0,0,-1], along=[1,0,0], across=[0,1,0],
                     longitudinal_axis=[1,0,0], variance=.0001, along_spread=10., across_spread=10.,
                     minimum_longitudinal=-2., maximum_longitudinal=2.)
        geometry = dict(normal=[0,1,0], tangent=[1,0,0], up=[0,0,1], center_offset=0., sigma=.1,
                        minimum_longitudinal=-3., maximum_longitudinal=3., maximum_height=4., floors=[floor,floor])
        patch = dict(anchor=1, p=np.zeros(3), R=np.eye(3), offset=2., geometry=geometry)
        problem = AnchorProblem([1], [], [], [patch])
        for point in ([0,2,0], [0,2,3]):
            self.assertEqual(problem.objective(point)[0], 0.)
            self.assertGreaterEqual(min(problem.domain(point)), 0.)
        self.assertLess(min(problem.domain([2.1,2,0])), 0.)  # inside wall, outside actual floor support
        self.assertLess(min(problem.domain([0,2,-2])), 0.)
        self.assertLess(min(problem.domain([0,2,4.3])), 0.)


if __name__ == '__main__':
    unittest.main()
