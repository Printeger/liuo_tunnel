import json
from pathlib import Path
import tempfile
import unittest
from generate_known_maps import generate


class SurveyFixtureTest(unittest.TestCase):
    def test_default_fixture_preserved_and_survey_stream_independent(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first, second = root / "first", root / "second"
            generate(first)
            generate(second, survey_seed=68129)
            bundled = Path(__file__).resolve().parents[1] / "packages/liuo_tunnel/config/maps"
            for name in ["oracle_map.json", "surveyed_map.json"]:
                self.assertEqual((first / name).read_bytes(), (bundled / name).read_bytes())
            self.assertEqual(
                (first / "oracle_map.json").read_bytes(), (second / "oracle_map.json").read_bytes()
            )
            a = json.loads((first / "surveyed_map.json").read_text())
            b = json.loads((second / "surveyed_map.json").read_text())
            self.assertNotEqual(a["anchors"], b["anchors"])
            self.assertNotEqual(a["map_id"], b["map_id"])
            self.assertEqual(a["joint_covariance_m2"], b["joint_covariance_m2"])
            self.assertEqual(b["survey_seed"], 68129)
            self.assertEqual(b["provenance"], "SIMULATED_SURVEY_NOT_REAL_MEASUREMENTS")
            self.assertGreater(a["joint_covariance_m2"][0][3], 0)
            previous = (second / "surveyed_map.json").read_bytes()
            generate(second, survey_seed=68129)
            self.assertEqual(previous, (second / "surveyed_map.json").read_bytes())

    def test_closed_kilometre_density_including_unsorted_and_boundary(self):
        from generate_known_maps import validate_deployment

        def anchors(xs):
            return [dict(mean_m=[x, 2.8, 0.4]) for x in xs]

        validate_deployment(anchors([2400, 0, 400, 800, 1200, 1600, 2000]))
        validate_deployment(anchors([0, 300, 600, 1000.001]))
        for xs in [[0, 300, 600, 1000], [1000, 600, 0, 300], [0, 0, 0, 0], [float("nan")]]:
            with self.assertRaises(ValueError):
                validate_deployment(anchors(xs))
