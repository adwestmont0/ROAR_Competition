import unittest

from experiments.qualifying_brake_wp1232_conditional_rebrake import (
    apply_wp1232_conditional_rebrake,
)


class DummyThrottleController:
    def __init__(self, branch="throttle_maintain", raw_brake=0):
        self.last_longitudinal_debug = {
            "decision_branch": branch,
            "raw_brake": raw_brake,
            "override_applied_brake": raw_brake,
            "override_tick_suppressed": False,
            "override_suppression_reason": "",
        }


class DummyController:
    def __init__(self, waypoint=1274, tick=101, suppression_tick=100,
                 branch="brake_initiate", raw_brake=1):
        self.current_waypoint_idx = waypoint
        self.num_ticks = tick
        self._qualifying_last_release_suppression_tick = suppression_tick
        self.throttle_controller = DummyThrottleController(branch, raw_brake)


class ConditionalRebrakeTests(unittest.TestCase):
    def test_clean_release_is_unchanged(self):
        controller = DummyController(branch="throttle_maintain", raw_brake=0)
        before_debug = dict(controller.throttle_controller.last_longitudinal_debug)
        result = apply_wp1232_conditional_rebrake(controller, 1.102, 0.0, 3, 176.0)
        self.assertEqual(result, (1.102, 0.0, 3))
        self.assertEqual(controller.throttle_controller.last_longitudinal_debug, before_debug)

    def test_only_immediate_fresh_rebrake_is_suppressed(self):
        controller = DummyController()
        result = apply_wp1232_conditional_rebrake(controller, -1.0, 1.0, -1, 179.0)
        self.assertEqual(result, (0.0, 0.0, 2))
        debug = controller.throttle_controller.last_longitudinal_debug
        self.assertEqual(debug["override_suppression_reason"], "conditional-rebrake")
        self.assertTrue(debug["override_tick_suppressed"])

    def test_no_delayed_or_non_wp1232_suppression(self):
        for waypoint, tick in ((1274, 102), (1400, 101)):
            controller = DummyController(waypoint=waypoint, tick=tick)
            self.assertEqual(
                apply_wp1232_conditional_rebrake(controller, -1.0, 1.0, -1, 179.0),
                (-1.0, 1.0, -1),
            )

    def test_at_most_one_suppression_per_traversal(self):
        controller = DummyController()
        apply_wp1232_conditional_rebrake(controller, -1.0, 1.0, -1, 179.0)
        self.assertEqual(
            apply_wp1232_conditional_rebrake(controller, -1.0, 1.0, -1, 179.0),
            (-1.0, 1.0, -1),
        )


if __name__ == "__main__":
    unittest.main()
