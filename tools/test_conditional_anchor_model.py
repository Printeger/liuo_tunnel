#!/usr/bin/env python3
import unittest
import numpy as np
from conditional_anchor_model import AnchorProblem


class ConditionalAnchorModelTest(unittest.TestCase):
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

    def test_straight_ranges_have_exact_rotational_ambiguity(self):
        ta = [dict(anchor=1, tag=[x, 0., 0.], range=np.hypot(x-4, 3), variance=.01) for x in range(9)]
        problem = AnchorProblem([1], ta, [])
        for point in ([4., 3., 0.], [4., 0., -3.], [4., -3., 0.]):
            residual, jacobian = problem.residual_jacobian(point)
            self.assertLess(np.linalg.norm(residual), 1e-12)
            self.assertLess(np.linalg.eigvalsh(jacobian.T@jacobian)[0], 1e-10)
        self.assertEqual(problem.range_seeds(1).shape, (8, 3))


if __name__ == '__main__':
    unittest.main()
