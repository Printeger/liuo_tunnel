import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate_ape import rigid_fit, associate


class ApeTest(unittest.TestCase):
    def test_rigid_alignment_no_scale(self):
        estimated=np.array([[0.,0.,0.],[1.,0.,0.],[0.,2.,0.],[0.,0.,3.],[1.,2.,3.]])
        known=Rotation.from_euler('xyz',[.2,-.3,.5]).as_matrix()
        truth=(known@estimated.T).T+[10.,-2.,3.]
        r,t,_=rigid_fit(estimated,truth)
        np.testing.assert_allclose((r@estimated.T).T+t,truth,atol=1e-12)
        r,t,_=rigid_fit(estimated,truth*2)
        self.assertGreater(np.linalg.norm((r@estimated.T).T+t-truth*2),1.)
        self.assertAlmostEqual(np.linalg.det(r),1.)

    def test_no_gap_interpolation(self):
        values=np.zeros((3,8));values[:,0]=[61.,61.1,65.]
        indices,valid=associate(values,np.array([61.,61.1,62.,65.]))
        self.assertEqual(valid.tolist(),[True,True,False,True])


if __name__=='__main__':
    unittest.main()
