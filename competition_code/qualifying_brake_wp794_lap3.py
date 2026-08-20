"""Single-tick WP794 experiment layered on the validated two-event setup."""


def apply_wp794_lap3_override(controller, throttle, brake, gear, speed_kmh):
    in_event = controller.lapNum == 3 and 780 <= controller.current_waypoint_idx <= 820
    if not in_event:
        controller._wp794_lap3_brake_ordinal = 0
        controller._wp794_lap3_suppression_tick = None
        return throttle, brake, gear

    raw_brake = float(brake)
    suppressed = False
    rerequest = False
    released_ticks = None
    state = "armed"
    if raw_brake > 0:
        controller._wp794_lap3_brake_ordinal = (
            getattr(controller, "_wp794_lap3_brake_ordinal", 0) + 1
        )
        ordinal = controller._wp794_lap3_brake_ordinal
        suppression_tick = getattr(controller, "_wp794_lap3_suppression_tick", None)
        if suppression_tick is not None:
            rerequest = True
            released_ticks = max(0, controller.num_ticks - suppression_tick - 1)
        if ordinal == 6:
            suppressed = True
            state = "early-release-suppressed"
            controller._wp794_lap3_suppression_tick = controller.num_ticks
            throttle = 0.0
            brake = 0.0
            gear = max(1, int(speed_kmh / 60))
        elif rerequest:
            state = "brake-rerequest-immediate" if released_ticks == 0 else "brake-rerequest-delayed"
        else:
            state = "braking"
    else:
        ordinal = getattr(controller, "_wp794_lap3_brake_ordinal", 0)
        if getattr(controller, "_wp794_lap3_suppression_tick", None) is not None:
            state = "released"

    if controller.throttle_controller.last_longitudinal_debug:
        controller.throttle_controller.last_longitudinal_debug.update({
            "override_selected_event": 4,
            "override_onset_delay_ticks": 0,
            "override_release_early_ticks": 1,
            "override_raw_winner_brake": raw_brake,
            "override_applied_brake": float(brake),
            "override_tick_suppressed": suppressed,
            "override_suppression_reason": "early-release" if suppressed else "",
            "override_brake_ordinal": ordinal,
            "override_event_state": state,
            "override_brake_rerequest": rerequest,
            "override_released_ticks_before_rerequest": released_ticks,
        })
    return throttle, brake, gear
