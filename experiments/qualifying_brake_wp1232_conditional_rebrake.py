"""Suppress only the immediate fresh WP1232 re-brake after release-1."""


def apply_wp1232_conditional_rebrake(controller, throttle, brake, gear, speed_kmh):
    in_event = 1210 <= controller.current_waypoint_idx <= 1300
    if not in_event:
        controller._wp1232_conditional_consumed = False
        return throttle, brake, gear

    debug = controller.throttle_controller.last_longitudinal_debug
    suppression_tick = getattr(
        controller, "_qualifying_last_release_suppression_tick", None
    )
    immediate_tick = (
        suppression_tick is not None
        and controller.num_ticks == suppression_tick + 1
    )
    fresh_rebrake = (
        float(brake) > 0
        and debug.get("raw_brake", 0) > 0
        and debug.get("decision_branch") == "brake_initiate"
    )
    already_consumed = getattr(controller, "_wp1232_conditional_consumed", False)

    if not (immediate_tick and fresh_rebrake and not already_consumed):
        return throttle, brake, gear

    controller._wp1232_conditional_consumed = True
    # Suppress braking without fabricating positive throttle. This mirrors the
    # existing release override's neutral command and preserves a forward gear.
    throttle = 0.0
    brake = 0.0
    gear = max(1, int(speed_kmh / 60))
    debug.update({
        "override_applied_brake": 0.0,
        "override_tick_suppressed": True,
        "override_suppression_reason": "conditional-rebrake",
        "override_event_state": "conditional-rebrake-suppressed",
        "override_brake_rerequest": True,
        "override_released_ticks_before_rerequest": 0,
    })
    return throttle, brake, gear
