import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
COMPETITION_CODE = ROOT / "competition_code"
if str(COMPETITION_CODE) not in sys.path:
    sys.path.insert(0, str(COMPETITION_CODE))

from shadow.longitudinal import ShadowLongitudinalPlanner
from experiments.hybrid.export_shadow_profile import export_profile


def point(x, speed, acceleration=2.0, deceleration=10.0, constraint="acceleration"):
    return {
        "x": float(x), "y": 0.0, "s_m": float(x), "ds_m": 5.0,
        "target_speed_kmh": float(speed),
        "acceleration_mps2": float(acceleration),
        "deceleration_mps2": float(deceleration),
        "active_constraint": constraint, "confidence": "high", "support": 20,
    }


class ShadowLongitudinalPlannerTests(unittest.TestCase):
    def test_observe_is_shadow_only_and_does_not_mutate_applied_control(self):
        planner = ShadowLongitudinalPlanner({
            "schema_version": 1,
            "source_result_sha256": "abc",
            "points": [point(0, 120), point(5, 110), point(10, 115)],
        })
        control = {"throttle": np.float64(1.0), "brake": np.float64(0.0)}
        before = dict(control)
        result = planner.observe(np.array([0.1, 0.0, 0.0]), 125.0, control, 10)
        self.assertEqual(control, before)
        self.assertTrue(result["available"])
        self.assertEqual(result["phase"], "braking")
        self.assertEqual(result["command"], "brake")
        self.assertGreater(result["brake"], 0.0)
        self.assertEqual(result["distance_to_release_m"], 5.0)
        self.assertEqual(result["source_result_sha256"], "abc")

    def test_nearest_profile_search_tracks_forward_and_recovers_globally(self):
        planner = ShadowLongitudinalPlanner({
            "points": [point(index * 5, 100 + index) for index in range(30)],
        })
        self.assertEqual(planner.observe([0, 0], 100, {}, 0)["profile_index"], 0)
        self.assertEqual(planner.observe([20, 0], 104, {}, 4)["profile_index"], 4)
        self.assertEqual(planner.observe([125, 0], 125, {}, 25)["profile_index"], 25)

    def test_export_keeps_only_runtime_profile_fields_and_source_hash(self):
        source_result = {
            "distance_profile": [{
                "x": 1.0, "y": 2.0, "s_m": 3.0, "ds_m": 5.0,
                "stability_capped_planned_speed_kmh": 200.0,
                "observed_acceleration_envelope_mps2": 2.5,
                "observed_deceleration_envelope_mps2": 12.0,
                "active_constraint": "braking", "confidence": "high",
                "longitudinal_envelope_support": 42,
                "unused": "discard me",
            }]
        }
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "source.json"
            destination = Path(temporary) / "profile.json"
            source.write_text(json.dumps(source_result), encoding="utf-8")
            exported = export_profile(source, destination)
            loaded = json.loads(destination.read_text(encoding="utf-8"))
        self.assertEqual(exported, loaded)
        self.assertEqual(loaded["point_count"], 1)
        self.assertNotIn("unused", loaded["points"][0])
        self.assertEqual(len(loaded["source_result_sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
