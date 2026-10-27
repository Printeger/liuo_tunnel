#!/usr/bin/env python3
"""Decoded equality must ignore padding, never physical or delivery fields."""
import copy
import unittest
from rclpy.serialization import serialize_message
from liuo_tunnel_msgs.msg import StaticAnchorRecord, StaticRangeLeaf
from compare_ta_ablation import static_messages


class StaticEqualityTest(unittest.TestCase):
    def setUp(self):
        self.message = StaticAnchorRecord()
        self.message.network = 'n'
        self.message.record_id = 'r'
        self.message.revision = 1
        self.message.leaves = [StaticRangeLeaf(physical_id='p', range_m=4., variance_m2=.01)]

    def rows(self, message):
        return [dict(cdr_hex=serialize_message(message).hex())]

    def test_cdr_padding_is_not_a_field(self):
        data = bytearray(serialize_message(self.message))
        # CDR encapsulation + two int32 stamp fields + empty frame_id string:
        # its terminator is byte 16, followed by alignment to byte 20.
        self.assertEqual(data[12:17], b'\x01\x00\x00\x00\x00')
        original = static_messages([dict(cdr_hex=data.hex())])
        data[17:20] = b'\x91\x82\x73'
        self.assertEqual(original, static_messages([dict(cdr_hex=data.hex())]))

    def test_all_fields_and_multiplicity(self):
        baseline = static_messages(self.rows(self.message))
        for field in ('header', 'identity', 'revision', 'range', 'variance', 'acquisition'):
            message = copy.deepcopy(self.message)
            if field == 'header':
                message.header.stamp.nanosec = 1
            elif field == 'identity':
                message.leaves[0].physical_id = 'other'
            elif field == 'revision':
                message.revision += 1
            elif field == 'range':
                message.leaves[0].range_m += .001
            elif field == 'variance':
                message.leaves[0].variance_m2 += .001
            else:
                message.leaves[0].acquisition_stamp.sec = 1
            with self.subTest(field=field):
                self.assertNotEqual(baseline, static_messages(self.rows(message)))
        self.assertNotEqual(baseline, static_messages(self.rows(self.message) * 2))
        self.assertEqual(baseline, static_messages(self.rows(copy.deepcopy(self.message))))


if __name__ == '__main__':
    unittest.main()
