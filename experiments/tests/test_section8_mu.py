import json
import tempfile
import unittest
from pathlib import Path

from experiments.harness.core import apply_parameters, expand_config


ROOT = Path(__file__).resolve().parents[2]


class Section8MuTests(unittest.TestCase):
    def test_mu310_changes_exactly_runtime_and_debug_section8_values(self):
        registry = json.loads((ROOT / "experiments/parameters.json").read_text())
        source = (ROOT / "competition_code/ThrottleController.py").read_text()
        old = "            8: 2.75,\n"
        new = "            8: 3.1,\n"
        self.assertEqual(source.count(old), 2)

        with tempfile.TemporaryDirectory() as temporary:
            checkout = Path(temporary)
            destination = checkout / "competition_code/ThrottleController.py"
            destination.parent.mkdir(parents=True)
            destination.write_text(source)
            diff = apply_parameters(
                checkout, {"throttle.section_mu.8": 3.10}, registry
            )
            changed = destination.read_text()

        self.assertEqual(changed.count(new), 2)
        self.assertEqual(changed.replace(new, old), source)
        self.assertEqual(diff.count("-            8: 2.75,"), 2)
        self.assertEqual(diff.count("+            8: 3.1,"), 2)

    def test_initial_campaign_is_exact_ctctc_with_known_good_fixed(self):
        path = ROOT / "experiments/configs/qualifying_brake_section8_mu310_initial.json"
        schedule = expand_config(json.loads(path.read_text()))
        self.assertEqual(
            [run["is_control"] for run in schedule],
            [True, False, True, False, True],
        )
        for run in schedule:
            parameters = run["parameters"]
            self.assertEqual(parameters["qualifying_brake.two_event_combination"], 1)
            self.assertEqual(parameters["qualifying_brake.wp794_lap3_final_tick"], 1)
            self.assertEqual(parameters["qualifying_brake.wp2501_left_foot_percent"], 10)
            if run["is_control"]:
                self.assertNotIn("throttle.section_mu.8", parameters)
            else:
                self.assertEqual(parameters["throttle.section_mu.8"], 3.10)


if __name__ == "__main__":
    unittest.main()
