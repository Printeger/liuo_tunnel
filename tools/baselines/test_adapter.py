import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from run_external import Adapter


class AdapterTest(unittest.TestCase):
    def test_raw_samples_and_zero_scan_time(self):
        adapter = Adapter(Rotation.identity(), np.zeros(3))
        header = Header(frame_id='lidar')
        elevations = np.deg2rad(np.linspace(-25, 25, 40))
        values = [(float(4*np.cos(a)), 0., float(4*np.sin(a)), 7., 0.) for a in elevations]
        fields = [PointField(name=k, offset=4*i, datatype=PointField.FLOAT32, count=1) for i,k in enumerate(('x','y','z','intensity','time'))]
        source = point_cloud2.create_cloud(header, fields, values)
        converted = adapter.cloud(source)
        for key in ('x','y','z','intensity','time'):
            np.testing.assert_array_equal(point_cloud2.read_points_numpy(source,field_names=(key,)), point_cloud2.read_points_numpy(converted,field_names=(key,)))
        np.testing.assert_array_equal(point_cloud2.read_points_numpy(converted,field_names=('ring',)).reshape(-1),np.arange(40))
        m=Imu();m.linear_acceleration.z=9.80665;m.angular_velocity.z=.01;m.header.stamp.sec=1
        adapter.imu(m)
        self.assertEqual(m.linear_acceleration.z,9.80665)
        self.assertEqual(m.angular_velocity.z,.01)
        self.assertEqual(m.orientation_covariance[0],-1)
        self.assertEqual(m.header.stamp.sec,1)


if __name__=='__main__':
    unittest.main()
