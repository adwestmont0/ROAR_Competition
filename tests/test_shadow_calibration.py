import unittest

from experiments.analysis.shadow_calibration import decision


class ShadowCalibrationTest(unittest.TestCase):
    def test_original_brakes_from_gradient_below_target(self):
        command, _, cause = decision(169, 172, -.5, 10, "original")
        self.assertEqual(command, "brake")
        self.assertEqual(cause, "profile_gradient")

    def test_actual_target_guard_releases_below_target(self):
        command, brake, cause = decision(169, 172, -.5, 10, "actual_target_guard")
        self.assertEqual((command, brake, cause), ("not_brake", 0.0, "release"))

    def test_guard_preserves_braking_near_target(self):
        self.assertEqual(decision(172, 172, -.5, 10, "actual_target_guard")[0], "brake")

    def test_speed_error_still_brakes(self):
        self.assertEqual(decision(175, 172, .1, 10, "actual_target_guard")[2], "speed_error")


if __name__ == "__main__":
    unittest.main()
