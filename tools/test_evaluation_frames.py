import math
import unittest
from evaluation_frames import world_estimates, local_truth


class EvaluationFrames(unittest.TestCase):
    def test_internal_start_heading_and_velocity_roundtrip(self):
        for yaw in [0, math.pi / 2, math.pi]:
            base = dict(
                session="99",
                stamp="1",
                x=200.0,
                y=0.8,
                z=2.0,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                qx=0.0,
                qy=0.0,
                qz=math.sin(yaw / 2),
                qw=math.cos(yaw / 2),
            )
            later = dict(
                base,
                stamp="2",
                x=200 + 3 * math.cos(yaw),
                y=0.8 + 3 * math.sin(yaw),
                vx=2 * math.cos(yaw),
                vy=2 * math.sin(yaw),
            )
            local = local_truth([base, later], False)
            self.assertAlmostEqual(local[1]["x"], 3)
            self.assertAlmostEqual(local[1]["y"], 0)
            self.assertAlmostEqual(local[1]["z"], 0)
            self.assertAlmostEqual(local[1]["vx"], 2)
            self.assertAlmostEqual(local[1]["qw"], 1)
            for row in local:
                row["session"] = "1"
            world = world_estimates(local, [base, later], False)
            for expected, actual in zip([base, later], world):
                for key in ["x", "y", "z", "vx", "vy", "vz", "qx", "qy", "qz", "qw"]:
                    self.assertAlmostEqual(expected[key], actual[key])
            self.assertIs(world_estimates(local, [base, later], True), local)

    def test_legacy_identity_and_independent_session_datums(self):
        rows = [
            dict(session="a", x=0.0, y=0.0, z=1.2, vx=0.0, vy=0.0, vz=0.0),
            dict(session="b", x=1000.0, y=0.8, z=2.0, vx=0.0, vy=0.0, vz=0.0),
        ]
        for row in local_truth(rows, False):
            self.assertEqual([row[k] for k in ["x", "y", "z"]], [0, 0, 0])

class CalibratedMapFrames(unittest.TestCase):
    def test_reuse_uses_first_run_datum_not_second_start(self):
        import json
        import tempfile
        from pathlib import Path
        from evaluation_frames import run_world_estimates, map_local_truth
        with tempfile.TemporaryDirectory() as directory:
            first, second = Path(directory) / "first", Path(directory) / "second"
            for run in (first, second):
                (run / "metadata/config").mkdir(parents=True)
            (first / "metadata/config/phase1.json").write_text(json.dumps({"uwb": {"mode": "self_calibrate"}}))
            (second / "metadata/config/phase1.json").write_text(json.dumps({"uwb": {"mode": "reuse_map"}}))
            (first / "metadata/config/scenario.json").write_text(json.dumps({"simulation_start_xyz_yaw": [200, 3, 1.2, math.pi / 2]}))
            (second / "metadata/config/scenario.json").write_text(json.dumps({"simulation_start_xyz_yaw": [999, 8, 40, 0]}))
            (second / "metadata/config/map.json").write_text(json.dumps({"source_run": str(first)}))
            row = dict(session="1", stamp=4, x=20, y=2, z=.4, vx=1, vy=0, vz=0, qx=0, qy=0, qz=0, qw=1)
            world = run_world_estimates(second, [row], [dict(row, x=999)], "reuse_map")[0]
            self.assertAlmostEqual(world["x"], 198)
            self.assertAlmostEqual(world["y"], 23)
            self.assertAlmostEqual(world["z"], 1.6)
            self.assertAlmostEqual(world["vy"], 1)
            restored = map_local_truth(second, [world])[0]
            for key in ("x", "y", "z", "vx", "vy", "vz", "qx", "qy", "qz", "qw"):
                self.assertAlmostEqual(restored[key], row[key])
            (first / "metadata/config/scenario.json").unlink()
            with self.assertRaises(FileNotFoundError):
                run_world_estimates(second, [row], [dict(row, x=999)], "reuse_map")
