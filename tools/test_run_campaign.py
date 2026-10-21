import signal
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from run_campaign import DISK_RESERVE_BYTES, validate_resume_groups, wait_owned_run


class CampaignControlTest(unittest.TestCase):
    def test_completed_owner_is_not_signaled(self):
        owner = Mock()
        owner.wait.return_value = 0
        self.assertEqual(wait_owned_run(owner), (0, None))
        owner.send_signal.assert_not_called()

    def test_low_disk_only_signals_owned_process_and_waits_for_finalization(self):
        owner = Mock()
        owner.wait.side_effect = [subprocess.TimeoutExpired("owned", 5), 1]
        with patch("run_campaign.shutil.disk_usage", return_value=SimpleNamespace(free=1)):
            code, reason = wait_owned_run(owner)
        self.assertEqual(code, 1)
        self.assertIn("DISK_RESERVE_BELOW_2_GIB", reason)
        owner.send_signal.assert_called_once_with(signal.SIGINT)
        self.assertEqual(owner.wait.call_args.kwargs, {"timeout": 60})

    def test_sufficient_space_continues_same_owner(self):
        owner = Mock()
        owner.wait.side_effect = [subprocess.TimeoutExpired("owned", 5), 0]
        with patch(
            "run_campaign.shutil.disk_usage", return_value=SimpleNamespace(free=DISK_RESERVE_BYTES)
        ):
            self.assertEqual(wait_owned_run(owner), (0, None))
        owner.send_signal.assert_not_called()

    def test_resume_requires_explicit_complete_nonduplicated_triplets(self):
        entries = [dict(seed=0, mode=m) for m in ["record_only", "oracle_map", "surveyed_map"]]
        self.assertEqual(set(validate_resume_groups(entries, range(10))), {0})
        for invalid in [
            entries[:2],
            entries + [entries[0]],
            entries + [dict(seed=10, mode="record_only")],
        ]:
            with self.subTest(entries=invalid), self.assertRaises(ValueError):
                validate_resume_groups(invalid, range(10))


class CaptureReleaseTest(unittest.TestCase):
    def test_release_preserves_results_and_records_hash_and_is_idempotent(self):
        import hashlib
        import json
        import tempfile
        from pathlib import Path
        from run_campaign import release_raw_capture

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "log" / "test_run"
            raw = run / "export/capture/raw"
            raw.mkdir(parents=True)
            (run / "metadata/manifests").mkdir(parents=True)
            manifest = run / "metadata/run_manifest.json"
            manifest.write_text(json.dumps(dict(run_id=run.name, lifecycle="failed")))
            original = manifest.read_bytes()
            (raw / "raw_0.mcap").write_bytes(b"frozen input")
            (raw / "metadata.yaml").write_text("retained metadata")
            with patch("run_campaign.ROOT", root):
                receipt = release_raw_capture(run, "explicit user authorization")
                self.assertEqual(
                    receipt["files"][0]["sha256"], hashlib.sha256(b"frozen input").hexdigest()
                )
                self.assertFalse((raw / "raw_0.mcap").exists())
                self.assertTrue((raw / "metadata.yaml").exists())
                self.assertEqual(manifest.read_bytes(), original)
                self.assertEqual(release_raw_capture(run, "retry"), receipt)

    def test_active_external_and_symlinked_captures_are_refused(self):
        import json
        import tempfile
        from pathlib import Path
        from run_campaign import release_raw_capture

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "log" / "test_run"
            raw = run / "export/capture/raw"
            raw.mkdir(parents=True)
            (run / "metadata/manifests").mkdir(parents=True)
            manifest = run / "metadata/run_manifest.json"
            target = root / "keep.mcap"
            target.write_bytes(b"keep")
            (raw / "raw_0.mcap").symlink_to(target)
            with patch("run_campaign.ROOT", root):
                for state in ["active", "completed"]:
                    manifest.write_text(json.dumps(dict(run_id=run.name, lifecycle=state)))
                    with self.assertRaises(ValueError):
                        release_raw_capture(run, "test")
                with self.assertRaises(ValueError):
                    release_raw_capture(root, "test")
            self.assertEqual(target.read_bytes(), b"keep")
