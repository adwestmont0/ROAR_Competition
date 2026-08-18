"""Offline forensics for the WP1232 release-1 clean-release/re-brake split."""

import csv
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from experiments.harness.core import load_ledger, write_json


CAMPAIGNS = {
    "qualifying-brake-timing-initial",
    "qualifying-brake-timing-validation",
    "qualifying-brake-two-event-combination",
    "qualifying-brake-wp794-lap3-validation-rerun2",
}
OFFSETS = (-3, -2, -1, 0, 1, 2)
HORIZONS = (25, 50, 100, 200)


def number(row, field, default=None):
    value = row.get(field, "")
    return default if value in (None, "") else float(value)


def integer(row, field, default=None):
    value = number(row, field, default)
    return default if value is None else int(value)


def enabled(item):
    p = item.get("parameters", {})
    return bool(p.get("qualifying_brake.two_event_combination")) or (
        p.get("qualifying_brake.event") == 2
        and p.get("qualifying_brake.onset_delay_ticks") == 0
        and p.get("qualifying_brake.release_early_ticks") == 1
    )


def path_steps(root):
    points = np.load(root / "competition_code/waypoints/waypointsPrimary.npz")["locations"][35:, :2]
    return np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)


def forward_distance(steps, start, end):
    if end >= start:
        return float(np.sum(steps[start:end]))
    return float(np.sum(steps[start:]) + np.sum(steps[:end]))


def raw_brake(row):
    return number(row, "override_raw_winner_brake", number(row, "controller_raw_brake", 0)) or 0


def suppressed_wp1232(row):
    return (
        integer(row, "override_selected_event") == 2
        and str(row.get("override_tick_suppressed", "")).lower() == "true"
        and row.get("override_suppression_reason") == "early-release"
    )


def feature_row(row, prior):
    dt = number(row, "sim_time_seconds", 0) - number(prior, "sim_time_seconds", 0)
    speed_delta = number(row, "speed_kmh", 0) - number(prior, "speed_kmh", 0)
    return {
        "tick": integer(row, "tick"),
        "waypoint": integer(row, "custom_waypoint_index"),
        "speed_kmh": number(row, "speed_kmh"),
        "longitudinal_acceleration_mps2": None if dt <= 0 else speed_delta / 3.6 / dt,
        "steer": number(row, "steer"),
        "steer_change": number(row, "steer_change"),
        "lookahead_waypoint": number(row, "controller_lookahead_waypoint_index"),
        "lookahead_count": number(row, "controller_lookahead_count"),
        "target_speed_kmh": number(row, "controller_target_speed_kmh"),
        "recommended_speed_kmh": number(row, "controller_recommended_speed_kmh"),
        "speed_error_kmh": number(row, "controller_speed_error_kmh"),
        "speed_ratio": number(row, "controller_speed_ratio"),
        "brake_threshold_ratio": number(row, "controller_brake_threshold_ratio"),
        "speed_change_kmh": number(row, "controller_speed_change_kmh"),
        "raw_throttle": number(row, "controller_raw_throttle"),
        "raw_brake": number(row, "controller_raw_brake"),
        "applied_throttle": number(row, "throttle"),
        "applied_brake": number(row, "brake"),
        "decision_branch": row.get("controller_decision_branch"),
        "selected_constraint": row.get("controller_selected_constraint"),
        "brake_ticks_before": number(row, "controller_brake_ticks_before"),
        "brake_ticks_after_decision": number(row, "controller_brake_ticks_after_decision"),
        "brake_ticks_after_run": number(row, "controller_brake_ticks_after_run"),
        "section": integer(row, "section"),
    }


def first_at_distance(rows, start, steps, distance):
    start_wp = integer(rows[start], "custom_waypoint_index")
    return next((i for i in range(start, len(rows))
                 if forward_distance(steps, start_wp, integer(rows[i], "custom_waypoint_index")) >= distance), None)


def extract_traversal(rows, suppression, steps, metadata):
    lap = integer(rows[suppression], "lap")
    after = suppression + 1
    if after >= len(rows) or integer(rows[after], "lap") != lap:
        return None
    if raw_brake(rows[after]) > 0:
        behavior = "RE-BRAKE"
    elif number(rows[after], "controller_raw_throttle", -1) > 0:
        behavior = "CLEAN RELEASE"
    else:
        return None

    wp1781_onset = next((i for i in range(after, len(rows))
                         if integer(rows[i], "lap") == lap
                         and 1760 <= integer(rows[i], "custom_waypoint_index") <= 1825
                         and raw_brake(rows[i]) > 0), None)
    if wp1781_onset is None:
        return None
    wp1781_end = wp1781_onset
    while (wp1781_end + 1 < len(rows)
           and integer(rows[wp1781_end + 1], "lap") == lap
           and integer(rows[wp1781_end + 1], "custom_waypoint_index") <= 1825):
        wp1781_end += 1
    wp1781_raw = [i for i in range(wp1781_onset, wp1781_end + 1) if raw_brake(rows[i]) > 0]

    event_start = next(i for i in range(0, suppression + 1)
                       if integer(rows[i], "lap") == lap
                       and 1210 <= integer(rows[i], "custom_waypoint_index") <= 1300
                       and raw_brake(rows[i]) > 0)
    event_end = suppression
    while (event_end + 1 < len(rows)
           and integer(rows[event_end + 1], "lap") == lap
           and integer(rows[event_end + 1], "custom_waypoint_index") <= 1300):
        event_end += 1
    applied = [i for i in range(event_start, event_end + 1) if number(rows[i], "brake", 0) > 0]
    actual_release = min(applied[-1] + 1, len(rows) - 1)
    first_throttle = next((i for i in range(suppression + 1, event_end + 1)
                           if number(rows[i], "controller_raw_throttle", -1) > 0), None)

    horizons = {}
    for distance in HORIZONS:
        index = first_at_distance(rows, suppression, steps, distance)
        horizons[str(distance)] = None if index is None else {
            "seconds_from_suppression": number(rows[index], "sim_time_seconds") - number(rows[suppression], "sim_time_seconds"),
            "speed_kmh": number(rows[index], "speed_kmh"),
            "waypoint": integer(rows[index], "custom_waypoint_index"),
        }
    anchors = {}
    for waypoint in (1700, 1750, 1770, 1779):
        index = next((i for i in range(suppression, wp1781_onset + 2)
                      if integer(rows[i], "custom_waypoint_index") >= waypoint), None)
        if index is not None:
            anchors[str(waypoint)] = {
                "seconds_from_suppression": number(rows[index], "sim_time_seconds") - number(rows[suppression], "sim_time_seconds"),
                **feature_row(rows[index], rows[max(0, index - 1)]),
            }
    next_brake = next((i for i in range(event_end + 1, len(rows))
                       if integer(rows[i], "lap") == lap and raw_brake(rows[i]) > 0), None)
    physical_anchor_times = {}
    reference = next((i for i in range(event_start - 20, event_start + 1)
                      if i >= 0 and integer(rows[i], "lap") == lap
                      and integer(rows[i], "custom_waypoint_index") >= 1210), event_start)
    for waypoint in (1232, 1276, 1300, 1350, 1400, 1700, 1750, 1770, 1779):
        index = next((i for i in range(reference, wp1781_onset + 2)
                      if integer(rows[i], "custom_waypoint_index") >= waypoint), None)
        if index is not None:
            physical_anchor_times[str(waypoint)] = (
                number(rows[index], "sim_time_seconds") - number(rows[reference], "sim_time_seconds")
            )
    common_horizon_times = {}
    for distance in HORIZONS:
        index = next((i for i in range(reference, wp1781_onset + 2)
                      if integer(rows[i], "custom_waypoint_index") >= 1274
                      and forward_distance(steps, 1274, integer(rows[i], "custom_waypoint_index")) >= distance), None)
        if index is not None:
            common_horizon_times[str(distance)] = (
                number(rows[index], "sim_time_seconds") - number(rows[reference], "sim_time_seconds")
            )

    wp794_onset = next((i for i in range(reference - 1, -1, -1)
                        if integer(rows[i], "lap") == lap
                        and 780 <= integer(rows[i], "custom_waypoint_index") <= 820
                        and raw_brake(rows[i]) > 0
                        and (i == 0 or raw_brake(rows[i - 1]) <= 0
                             or integer(rows[i - 1], "custom_waypoint_index") < 780)), None)
    wp794 = None
    if wp794_onset is not None:
        wp794_end = wp794_onset
        while (wp794_end + 1 < reference
               and integer(rows[wp794_end + 1], "lap") == lap
               and integer(rows[wp794_end + 1], "custom_waypoint_index") <= 820):
            wp794_end += 1
        wp794_brakes = [i for i in range(wp794_onset, wp794_end + 1) if raw_brake(rows[i]) > 0]
        wp794_applied = [i for i in range(wp794_onset, wp794_end + 1) if number(rows[i], "brake", 0) > 0]
        wp794_release = min(wp794_applied[-1] + 1, len(rows) - 1)
        wp794 = {
            "raw_brake_ticks": len(wp794_brakes),
            "applied_brake_ticks": len(wp794_applied),
            "onset_waypoint": integer(rows[wp794_onset], "custom_waypoint_index"),
            "release_waypoint": integer(rows[wp794_release], "custom_waypoint_index"),
            "release_speed_kmh": number(rows[wp794_release], "speed_kmh"),
            "seconds_release_to_wp1232_onset": number(rows[event_start], "sim_time_seconds") - number(rows[wp794_release], "sim_time_seconds"),
        }
    output = dict(metadata)
    output.update({
        "lap": lap,
        "behavior": behavior,
        "suppression_tick": integer(rows[suppression], "tick"),
        "suppression_waypoint": integer(rows[suppression], "custom_waypoint_index"),
        "features": {str(offset): feature_row(rows[suppression + offset], rows[suppression + offset - 1])
                     for offset in OFFSETS},
        "wp1232_event_onset_time": number(rows[event_start], "sim_time_seconds"),
        "wp1232_event_onset": feature_row(rows[event_start], rows[event_start - 1]),
        "wp1232_minimum_speed_kmh": min(number(rows[i], "speed_kmh") for i in range(event_start, event_end + 1)),
        "wp1232_release_speed_kmh": number(rows[actual_release], "speed_kmh"),
        "wp1232_exit_speed_kmh": number(rows[first_throttle], "speed_kmh") if first_throttle is not None else None,
        "wp1232_actual_release_waypoint": integer(rows[actual_release], "custom_waypoint_index"),
        "time_to_next_brake_seconds": None if next_brake is None else number(rows[next_brake], "sim_time_seconds") - number(rows[suppression], "sim_time_seconds"),
        "horizons": horizons,
        "anchors": anchors,
        "physical_anchor_times_from_wp1210": physical_anchor_times,
        "common_horizon_times_from_wp1210": common_horizon_times,
        "preceding_wp794": wp794,
        "wp1781_regime": len(wp1781_raw),
        "wp1781_onset_waypoint": integer(rows[wp1781_onset], "custom_waypoint_index"),
        "wp1781_arrival_seconds_from_suppression": number(rows[wp1781_onset], "sim_time_seconds") - number(rows[suppression], "sim_time_seconds"),
        "wp1781_entry_speed_kmh": number(rows[wp1781_onset], "speed_kmh"),
        "wp1781_lookahead_waypoint": number(rows[wp1781_onset], "controller_lookahead_waypoint_index"),
        "wp1781_target_speed_kmh": number(rows[wp1781_onset], "controller_target_speed_kmh"),
    })
    return output


def stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {"n": len(values), "mean": statistics.mean(values), "min": min(values),
            "max": max(values), "stdev": statistics.stdev(values) if len(values) > 1 else 0}


def group_summary(samples):
    return {
        "count": len(samples),
        "attempts": len({x["attempt_id"] for x in samples}),
        "campaigns": dict(Counter(x["experiment_name"] for x in samples)),
        "laps": dict(Counter(x["lap"] for x in samples)),
        "release_speed_kmh": stats([x["wp1232_release_speed_kmh"] for x in samples]),
        "minimum_speed_kmh": stats([x["wp1232_minimum_speed_kmh"] for x in samples]),
        "exit_speed_kmh": stats([x["wp1232_exit_speed_kmh"] for x in samples]),
        "time_to_next_brake_seconds": stats([x["time_to_next_brake_seconds"] for x in samples]),
        "horizon_times": {str(h): stats([x["horizons"][str(h)]["seconds_from_suppression"] for x in samples]) for h in HORIZONS},
        "physical_anchor_times_from_wp1210": {
            wp: stats([x["physical_anchor_times_from_wp1210"].get(wp) for x in samples])
            for wp in ("1232", "1276", "1300", "1350", "1400", "1700", "1750", "1770", "1779")
        },
        "common_horizon_times_from_wp1210": {
            str(h): stats([x["common_horizon_times_from_wp1210"].get(str(h)) for x in samples])
            for h in HORIZONS
        },
        "wp1232_onset_waypoint": stats([x["wp1232_event_onset"]["waypoint"] for x in samples]),
        "wp1232_onset_speed_kmh": stats([x["wp1232_event_onset"]["speed_kmh"] for x in samples]),
        "wp794_release_speed_kmh": stats([x["preceding_wp794"]["release_speed_kmh"] for x in samples if x["preceding_wp794"]]),
        "wp794_release_to_wp1232_seconds": stats([x["preceding_wp794"]["seconds_release_to_wp1232_onset"] for x in samples if x["preceding_wp794"]]),
        "wp1781_regimes": dict(Counter(x["wp1781_regime"] for x in samples)),
        "wp1781_14_frequency": sum(x["wp1781_regime"] == 14 for x in samples) / len(samples),
    }


def best_threshold(samples, offset, field):
    pairs = [(x["features"][str(offset)].get(field), x["behavior"] == "RE-BRAKE") for x in samples]
    pairs = [(v, y) for v, y in pairs if isinstance(v, (int, float))]
    if len(pairs) != len(samples) or len({v for v, _ in pairs}) < 2:
        return None
    values = sorted({v for v, _ in pairs})
    candidates = [(a + b) / 2 for a, b in zip(values, values[1:])]
    best = None
    for threshold in candidates:
        for direction in ("above", "below"):
            correct = sum(((v > threshold) if direction == "above" else (v < threshold)) == y for v, y in pairs)
            result = (correct / len(pairs), threshold, direction)
            if best is None or result[0] > best[0]:
                best = result
    clean = [v for v, y in pairs if not y]
    rebrake = [v for v, y in pairs if y]
    overlap = max(0.0, min(max(clean), max(rebrake)) - max(min(clean), min(rebrake)))
    return {"accuracy": best[0], "threshold": best[1], "rebrake_direction": best[2],
            "clean": stats(clean), "rebrake": stats(rebrake), "range_overlap": overlap}


def main(root=Path.cwd()):
    ledger = [x for x in load_ledger(root / "experiment_results/ledger.jsonl")
              if x.get("experiment_name") in CAMPAIGNS and x.get("outcome") == "finished"
              and x.get("telemetry_path") and enabled(x)]
    steps = path_steps(root)
    samples = []
    for item in ledger:
        path = root / item["telemetry_path"]
        with (path.parent / "ticks.csv").open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        metadata = {"attempt_id": item["attempt_id"], "experiment_name": item["experiment_name"],
                    "parameters": item.get("parameters", {})}
        for i, row in enumerate(rows):
            if suppressed_wp1232(row):
                sample = extract_traversal(rows, i, steps, metadata)
                if sample is not None:
                    samples.append(sample)

    groups = {name: [x for x in samples if x["behavior"] == name]
              for name in ("CLEAN RELEASE", "RE-BRAKE")}
    numeric_fields = [k for k, v in samples[0]["features"]["0"].items() if isinstance(v, (int, float))]
    discriminators = []
    for offset in OFFSETS:
        for field in numeric_fields:
            result = best_threshold(samples, offset, field)
            if result:
                discriminators.append({"offset": offset, "field": field, **result})
    discriminators.sort(key=lambda x: (-x["accuracy"], x["range_overlap"], abs(x["offset"])))

    contingency = defaultdict(Counter)
    for sample in samples:
        contingency[sample["behavior"]][sample["wp1781_regime"]] += 1
    branch_split = {name: dict(Counter(x["features"]["1"]["decision_branch"] for x in group))
                    for name, group in groups.items()}
    anchors = {name: {wp: {field: stats([x["anchors"].get(wp, {}).get(field) for x in group])
                                  for field in ("seconds_from_suppression", "speed_kmh",
                                                "longitudinal_acceleration_mps2", "waypoint",
                                                "lookahead_waypoint", "target_speed_kmh")}
                       for wp in ("1700", "1750", "1770", "1779")}
               for name, group in groups.items()}
    output = {
        "campaigns": sorted(CAMPAIGNS), "attempt_count": len(ledger), "sample_count": len(samples),
        "group_summaries": {name: group_summary(group) for name, group in groups.items()},
        "branch_at_first_post_suppression_tick": branch_split,
        "contingency": {name: dict(counts) for name, counts in contingency.items()},
        "anchor_summaries": anchors,
        "top_discriminators": discriminators[:30], "samples": samples,
    }
    destination = root / "experiment_results/analysis/wp1232_rebrake_forensics"
    destination.mkdir(parents=True, exist_ok=True)
    write_json(destination / "result.json", output)
    print(json.dumps({k: output[k] for k in ("attempt_count", "sample_count", "group_summaries",
                                             "branch_at_first_post_suppression_tick", "contingency",
                                             "top_discriminators")}, indent=2))


if __name__ == "__main__":
    main()
