#!/usr/bin/env python3
"""Regression: a valid later leaf cannot replace an earlier checkpoint leaf."""
import unittest
from checkpoint_ledger_audit import check_checkpoint


class CheckpointAuditTest(unittest.TestCase):
    def setUp(self):
        self.audit = [
            dict(event='input', reason='QUEUED', leaves=[('old', (1, 2)), ('early', (1, 2)), ('pending', (2, 3))]),
            dict(event='graph_update', stamp=10., fused_leaves=1),
            dict(event='input', reason='QUEUED', leaves=[('later', (1, 2))]),
            dict(event='graph_update', stamp=11., fused_leaves=3)]

    def check(self, stamp, ready, actual):
        return check_checkpoint(self.audit, stamp, ready, {'old'}, actual, lambda row: row['leaves'])

    def test_checkpoint_and_later_endpoint_initialization(self):
        self.check(10., {1, 2}, {'early'})
        self.check(11., {1, 2, 3}, {'early', 'later', 'pending'})

    def test_same_count_future_leaf_replacement(self):
        with self.assertRaisesRegex(AssertionError, 'identities'):
            self.check(10., {1, 2}, {'later'})

    def test_nonexistent_or_duplicate_checkpoint(self):
        with self.assertRaisesRegex(AssertionError, 'exactly one'):
            self.check(10.5, {1, 2}, {'early'})
        self.audit.append(dict(event='graph_update', stamp=10., fused_leaves=1))
        with self.assertRaisesRegex(AssertionError, 'exactly one'):
            self.check(10., {1, 2}, {'early'})

    def test_rejected_input_cannot_be_assumed_fused(self):
        self.audit.insert(1, dict(event='rejected_before_graph'))
        with self.assertRaisesRegex(AssertionError, 'rejected'):
            self.check(10., {1, 2}, {'early'})


if __name__ == '__main__':
    unittest.main()
