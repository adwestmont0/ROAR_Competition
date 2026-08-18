import unittest

from experiments.qualifying_brake_wp2501_left_foot import (
    apply_wp2501_left_foot_braking,
)


class DummyController:
    def __init__(self, event=1, ordinal=35):
        debug = {
            "override_selected_event": event,
            "override_brake_ordinal": ordinal,
        }
        self.throttle_controller = type(
            "Throttle", (), {"last_longitudinal_debug": debug}
        )()


class WP2501LeftFootTests(unittest.TestCase):
    def test_final_five_applied_ticks_overlap(self):
        for ordinal in range(35, 40):
            controller = DummyController(ordinal=ordinal)
            result = apply_wp2501_left_foot_braking(
                controller, -1.0, 1.0, -1, 0.15
            )
            self.assertEqual(result, (0.15, 1.0, -1))
            self.assertTrue(
                controller.throttle_controller.last_longitudinal_debug["lfb_active"]
            )

    def test_brake_and_gear_are_byte_identical(self):
        controller = DummyController(ordinal=37)
        result = apply_wp2501_left_foot_braking(
            controller, -1.0, 0.73, -1, 0.10
        )
        self.assertEqual(result[1:], (0.73, -1))

    def test_outside_window_or_event_is_unchanged(self):
        for event, ordinal in ((1, 34), (1, 40), (2, 37)):
            controller = DummyController(event=event, ordinal=ordinal)
            before = dict(controller.throttle_controller.last_longitudinal_debug)
            result = apply_wp2501_left_foot_braking(
                controller, -1.0, 1.0, -1, 0.20
            )
            self.assertEqual(result, (-1.0, 1.0, -1))
            self.assertEqual(
                controller.throttle_controller.last_longitudinal_debug, before
            )

    def test_no_overlap_without_applied_brake(self):
        controller = DummyController(ordinal=37)
        self.assertEqual(
            apply_wp2501_left_foot_braking(
                controller, 0.0, 0.0, 4, 0.20
            ),
            (0.0, 0.0, 4),
        )


if __name__ == "__main__":
    unittest.main()
