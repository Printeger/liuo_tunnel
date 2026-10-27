import copy
import unittest
from check_baseline_shared_input import check_hashes


class SharedInput(unittest.TestCase):
    def test_changed_raw_or_lost_adapted_sample_is_not_a_shared_input_pass(self):
        counts=dict(imu=4,cloud=2,clock=4);raw=dict(imu='imu',cloud='cloud')
        result=dict(counts=counts.copy(),raw_semantic_sha256=raw.copy(),observed_counts=dict(imu=4,cloud=2),
                    observed_semantic_sha256=dict(imu='converted-imu',cloud='converted-cloud'),
                    adapted_semantic_sha256=dict(imu='converted-imu',cloud='converted-cloud'))
        check_hashes(result,counts,raw)
        for section,key,value in [('raw_semantic_sha256','imu','other'),('counts','cloud',3),
                                  ('observed_counts','imu',3),('observed_semantic_sha256','cloud','lost'),
                                  ('counts','clock',3)]:
            with self.subTest(section=section,key=key):
                bad=copy.deepcopy(result);bad[section][key]=value
                with self.assertRaises(ValueError): check_hashes(bad,counts,raw)


if __name__=='__main__': unittest.main()
