"""Disposable post-controller override for the validated two-event experiment."""


EVENTS = (
    (2, 1210, 1300, 24, 1),
    (1, 2475, 2585, 41, 2),
)


def apply_combined_brake_override(controller, throttle, brake, gear, speed_kmh):
    waypoint = controller.current_waypoint_idx
    selected = next(
        (event for event in EVENTS if event[1] <= waypoint <= event[2]), None
    )
    raw_brake = float(brake)
    suppressed = False
    reason = ""
    rerequest = False
    released_ticks = None

    if selected is None:
        controller._qualifying_combined_event = None
        controller._qualifying_brake_ordinal = 0
        controller._qualifying_last_release_suppression_tick = None
        event_id = 0
        ordinal = 0
        release_early = 0
        state = "outside"
    else:
        event_id, _low, _high, baseline_ticks, release_early = selected
        if getattr(controller, "_qualifying_combined_event", None) != event_id:
            controller._qualifying_combined_event = event_id
            controller._qualifying_brake_ordinal = 0
            controller._qualifying_last_release_suppression_tick = None
        state = "armed"
        if raw_brake > 0:
            controller._qualifying_brake_ordinal = (
                getattr(controller, "_qualifying_brake_ordinal", 0) + 1
            )
            ordinal = controller._qualifying_brake_ordinal
            last_suppression = getattr(
                controller, "_qualifying_last_release_suppression_tick", None
            )
            if last_suppression is not None:
                rerequest = True
                released_ticks = max(0, controller.num_ticks - last_suppression - 1)
            if baseline_ticks - release_early < ordinal <= baseline_ticks:
                suppressed = True
                reason = "early-release"
                state = "early-release-suppressed"
                if rerequest:
                    state += "-rerequest-immediate" if released_ticks == 0 else "-rerequest-delayed"
                controller._qualifying_last_release_suppression_tick = controller.num_ticks
            elif rerequest:
                state = "brake-rerequest-immediate" if released_ticks == 0 else "brake-rerequest-delayed"
            else:
                state = "braking"
            if suppressed:
                throttle = 0.0
                brake = 0.0
                gear = max(1, int(speed_kmh / 60))
        else:
            ordinal = getattr(controller, "_qualifying_brake_ordinal", 0)
            if getattr(controller, "_qualifying_last_release_suppression_tick", None) is not None:
                state = "released"

    if controller.throttle_controller.last_longitudinal_debug:
        controller.throttle_controller.last_longitudinal_debug.update({
            "override_selected_event": event_id,
            "override_onset_delay_ticks": 0,
            "override_release_early_ticks": release_early,
            "override_raw_winner_brake": raw_brake,
            "override_applied_brake": float(brake),
            "override_tick_suppressed": suppressed,
            "override_suppression_reason": reason,
            "override_brake_ordinal": ordinal,
            "override_event_state": state,
            "override_brake_rerequest": rerequest,
            "override_released_ticks_before_rerequest": released_ticks,
        })
    return throttle, brake, gear
