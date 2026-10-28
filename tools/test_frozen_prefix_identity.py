#!/usr/bin/env python3
import csv
from pathlib import Path
import tempfile
import unittest
from frozen_prefix_identity import write_truth_prefix


class TruthPrefixTest(unittest.TestCase):
    def test_last_included_scan_and_original_source_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder)/'source.csv'
            target = Path(folder)/'target.csv'
            original = 'stamp,x\n1,2\n85.90000000000001,3\n86,4\n360.9,5\n'
            source.write_text(original)
            write_truth_prefix(source,target,85.9)
            with target.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row['x'] for row in rows],['2','3'])
            self.assertEqual(source.read_text(),original)


if __name__=='__main__':
    unittest.main()
