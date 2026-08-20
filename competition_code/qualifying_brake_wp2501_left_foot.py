"""Late-window WP2501 throttle/brake overlap experiment."""


WINDOW_FIRST_ORDINAL = 35
WINDOW_LAST_ORDINAL = 39


def apply_wp2501_left_foot_braking(
    controller, throttle, brake, gear, overlap_throttle
):
    """Change only throttle on the final five normally applied brake ticks."""
    debug = controller.throttle_controller.last_longitudinal_debug
    event_id = debug.get("override_selected_event")
    ordinal = debug.get("override_brake_ordinal")
    active = (
        event_id == 1
        and ordinal is not None
        and WINDOW_FIRST_ORDINAL <= int(ordinal) <= WINDOW_LAST_ORDINAL
        and float(brake) > 0
    )
    if not active:
        return throttle, brake, gear

    preserved_brake = float(brake)
    throttle = max(0.0, min(1.0, float(overlap_throttle)))
    debug.update({
        "lfb_active": True,
        "lfb_applied_throttle": throttle,
        "lfb_preserved_brake": preserved_brake,
        "lfb_window_first_ordinal": WINDOW_FIRST_ORDINAL,
        "lfb_window_last_ordinal": WINDOW_LAST_ORDINAL,
    })
    return throttle, brake, gear
