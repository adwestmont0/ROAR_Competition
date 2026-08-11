import sys
import unittest
from pathlib import Path

import numpy as np


COMPETITION_CODE = Path(__file__).resolve().parents[2] / "competition_code"
if str(COMPETITION_CODE) not in sys.path:
    sys.path.insert(0, str(COMPETITION_CODE))

from SpeedData import SpeedData
from ThrottleController import ThrottleController
from telemetry import RaceTelemetry


class LongitudinalInstrumentationTests(unittest.TestCase):
    def test_debug_branch_mirrors_controller_decisions(self):
        cases = (
            # current, recommended, previous, counter, branch, throttle, brake
            (220.0, 218.0, 223.0, 0, "throttle_maintain_over", None, 0),
            (222.3, 218.4, 225.7, 0, "brake_initiate", -1, 1),
            (250.0, 235.0, 252.0, 2, "brake_counter", -1, 1),
            (200.0, 250.0, 199.0, 0, "throttle_full", 1, 0),
        )
        for current, recommended, previous, counter, branch, throttle, brake in cases:
            with self.subTest(branch=branch):
                controller = ThrottleController()
                controller.previous_speed = previous
                controller.brake_ticks = counter
                speed_data = SpeedData(3.0, current, 190.0, recommended)
                self.assertEqual(controller._debug_decision_branch(speed_data), branch)
                actual_throttle, actual_brake = controller.speed_data_to_throttle_and_brake(speed_data)
                self.assertEqual(actual_brake, brake)
                if throttle is not None:
                    self.assertEqual(actual_throttle, throttle)

    def test_debug_branch_does_not_change_controller_state(self):
        controller = ThrottleController()
        controller.previous_speed = 250.0
        controller.brake_ticks = 2
        speed_data = SpeedData(3.0, 248.0, 200.0, 235.0)
        before = (controller.previous_speed, controller.brake_ticks)
        controller._debug_decision_branch(speed_data)
        self.assertEqual((controller.previous_speed, controller.brake_ticks), before)

    def test_race_telemetry_records_raw_decision_and_steering_change(self):
        controller = type("Controller", (), {"last_longitudinal_debug": {
            "raw_throttle": -1, "raw_brake": 1,
            "decision_branch": "brake_initiate",
            "recommended_speed_kmh": 218.4,
            "brake_ticks_before": 0,
            "brake_ticks_after_decision": 1,
            "brake_ticks_after_run": 0,
        }})()
        solution = type("Solution", (), {
            "throttle_controller": controller,
            "current_waypoint_idx": 1803,
            "current_section": 4,
            "lapNum": 1,
        })()
        telemetry = RaceTelemetry("unused")
        common = dict(
            location=np.zeros(3), velocity=np.zeros(3),
            official_waypoint_index=1, solution=solution,
            collision_impulse=0.0,
        )
        telemetry.record_tick(
            tick=1, sim_time_seconds=0.0,
            control={"throttle": 0.0, "brake": 0.0, "steer": 0.1, "target_gear": 4},
            **common,
        )
        telemetry.record_tick(
            tick=2, sim_time_seconds=0.05,
            control={"throttle": 0.0, "brake": 1.0, "steer": 0.15, "target_gear": -1},
            **common,
        )
        first, second = telemetry.samples
        self.assertEqual(first["controller_raw_brake"], 1)
        self.assertEqual(first["controller_decision_branch"], "brake_initiate")
        self.assertEqual(first["controller_recommended_speed_kmh"], 218.4)
        self.assertEqual(first["steer_change"], 0.0)
        self.assertAlmostEqual(second["steer_change"], 0.05)


if __name__ == "__main__":
    unittest.main()
