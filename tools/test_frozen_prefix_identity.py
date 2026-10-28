#!/usr/bin/env python3
import csv
from pathlib import Path
import tempfile
import unittest
from frozen_prefix_identity import write_truth_prefix, write_boundary_truth


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

    def test_boundary_seek_keeps_physical_values_but_two_sessions(self):
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'source.csv'; target=Path(folder)/'target.csv'
            original='session,stamp,x\n7,1,10\n7,25.9,20\n7,26,30\n7,35.90000000000001,40\n7,36,50\n7,70.9,60\n'
            source.write_text(original)
            write_boundary_truth(source,target,70)
            with target.open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual([r['x'] for r in rows],['10','20','30','40']*2)
            self.assertEqual([r['session'] for r in rows],['7']*4+['1000000007']*4)
            self.assertEqual(rows[3]['stamp'],rows[7]['stamp'])
            write_boundary_truth(source,target,60)
            with target.open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual([r['x'] for r in rows],['10','20','30','40','10','20'])
            self.assertEqual(source.read_text(),original)
            with self.assertRaises(ValueError):
                write_boundary_truth(source,target,71)


if __name__=='__main__':
    unittest.main()
