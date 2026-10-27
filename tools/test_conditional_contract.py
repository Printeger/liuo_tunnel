#!/usr/bin/env python3
import copy
import unittest
from conditional_contract import validate_oracle_map, vehicle_condition_config


class ConditionalContractTest(unittest.TestCase):
    def setUp(self):
        self.base = dict(sensors=dict(T_BU_translation=[.1, .2, .3]),
                         odometry=dict(backend='gpu', lag_s=10),
                         uwb=dict(mode='self_calibrate_free', use_aa=True, local_installation=True,
                                  static_edges=dict(topic='/static'),
                                  input=dict(nodes=[dict(type=1, internal_id=7)])))
        self.map = dict(schema_version='liuo_known_map_v1', mode='oracle_map',
                        diagnostic_role='D2_ORACLE_ANCHORS_NOT_DEPLOYABLE', input_raw_sha256='raw',
                        anchors=[dict(id=7, mean_m=[1., 2., 3.])], tag_lever_m=[.1, .2, .3])

    def test_changes_only_declared_condition_and_preserves_base(self):
        original = copy.deepcopy(self.base)
        derived = vehicle_condition_config(self.base)
        self.assertEqual(self.base, original)
        self.assertEqual(derived['sensors'], original['sensors'])
        self.assertEqual(derived['odometry'], original['odometry'])
        self.assertEqual(derived['uwb']['input'], original['uwb']['input'])
        self.assertFalse(derived['uwb']['use_aa'])
        self.assertEqual(derived['uwb']['initial_position_guess'], [0., 0., 0.])
        self.assertEqual(derived['uwb']['bootstrap_s'], 8.)
        self.assertNotIn('static_edges', derived['uwb'])
        validate_oracle_map(self.map, derived, 'raw')

    def test_rejects_unlabelled_mismatched_or_vehicle_truth_input(self):
        for field, value in [('diagnostic_role', 'production'), ('input_raw_sha256', 'other'),
                             ('mode', 'surveyed_map'), ('vehicle_pose', [0, 0, 0]),
                             ('trajectory', []), ('tag_lever_m', [0, 0, 0]),
                             ('anchors', [dict(id=8, mean_m=[1, 2, 3])])]:
            bad = copy.deepcopy(self.map)
            bad[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_oracle_map(bad, self.base, 'raw')

    def test_preserves_explicit_diagnostic_collection_interval(self):
        self.base['uwb']['bootstrap_s'] = 30.
        derived = vehicle_condition_config(self.base)
        self.assertEqual(derived['uwb']['bootstrap_s'], 30.)


if __name__ == '__main__':
    unittest.main()
