import concurrent.futures
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import run_directory as rd


class RunContract(unittest.TestCase):
    def test_concurrent_allocation_and_failure(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(
            os.environ, {"LIUO_RUN_ROOT": tmp, "LIUO_RETENTION_ENABLED": "0"}
        ):
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
                runs = list(
                    pool.map(lambda _: rd.resolve_run_directory(entrypoint="rail_sim"), range(8))
                )
            self.assertEqual(len(set(runs)), 8)
            for run in runs:
                self.assertEqual(run.parent, Path(tmp))
                self.assertEqual(rd.adopt_run_directory(str(run)), run)
                rd.write_subordinate_manifest(run, "input_failure", {"reason": "injected"})
                self.assertEqual(rd.finalize_run(run, lifecycle="completed"), "failed")
                m = json.loads((run / "metadata/run_manifest.json").read_text())
                self.assertEqual(m["lifecycle"], "failed")
                self.assertIsNotNone(m["ended_at_utc"])
                self.assertEqual(m["acceptance"], "not_evaluated")

    def test_non_run_latest_does_not_hide_new_run(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {"LIUO_RUN_ROOT": tmp}):
            (Path(tmp) / "latest").symlink_to("latest_test-result")
            run = rd.resolve_run_directory(entrypoint="rail_sim")
            self.assertEqual((Path(tmp) / "latest").resolve(), run)
            rd.finalize_run(run, lifecycle="completed")

    def test_absolute_root_and_unknown_latest_protected(self):
        with patch.dict(os.environ, {"LIUO_RUN_ROOT": "relative"}):
            with self.assertRaises(RuntimeError):
                rd.resolve_run_directory(entrypoint="rail_sim")
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {"LIUO_RUN_ROOT": tmp}):
            latest = Path(tmp) / "latest"
            latest.write_text("user data")
            with self.assertRaises(RuntimeError):
                rd.resolve_run_directory(entrypoint="rail_sim")
            self.assertEqual(latest.read_text(), "user data")


if __name__ == "__main__":
    unittest.main()
