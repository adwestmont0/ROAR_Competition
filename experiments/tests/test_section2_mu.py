import json
import tempfile
import unittest
from pathlib import Path

from experiments.harness.core import apply_parameters, expand_config


ROOT = Path(__file__).resolve().parents[2]


class Section2MuTests(unittest.TestCase):
    def test_mu340_changes_exactly_runtime_and_debug_section2_values(self):
        registry = json.loads((ROOT / "experiments/parameters.json").read_text())
        source = (ROOT / "competition_code/ThrottleController.py").read_text()
        old = "            2: 3.35,\n"
        new = "            2: 3.4,\n"
        self.assertEqual(source.count(old), 2)
        with tempfile.TemporaryDirectory() as temporary:
            checkout = Path(temporary)
            destination = checkout / "competition_code/ThrottleController.py"
            destination.parent.mkdir(parents=True)
            destination.write_text(source)
            diff = apply_parameters(checkout, {"throttle.section_mu.2": 3.40}, registry)
            changed = destination.read_text()
        self.assertEqual(changed.count(new), 2)
        self.assertEqual(changed.replace(new, old), source)
        self.assertEqual(diff.count("-            2: 3.35,"), 2)
        self.assertEqual(diff.count("+            2: 3.4,"), 2)

    def test_initial_schedule_is_ctctc_and_retains_known_good(self):
        config = json.loads((ROOT / "experiments/configs/qualifying_brake_section2_mu340_initial.json").read_text())
        schedule = expand_config(config)
        self.assertEqual([item["is_control"] for item in schedule], [True, False, True, False, True])
        for item in schedule:
            parameters = item["parameters"]
            self.assertEqual(parameters["qualifying_brake.two_event_combination"], 1)
            self.assertEqual(parameters["qualifying_brake.wp794_lap3_final_tick"], 1)
            self.assertEqual(parameters["qualifying_brake.wp2501_left_foot_percent"], 10)
            if item["is_control"]:
                self.assertNotIn("throttle.section_mu.2", parameters)
            else:
                self.assertEqual(parameters["throttle.section_mu.2"], 3.40)


if __name__ == "__main__":
    unittest.main()
