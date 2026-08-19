"""Rank Spring 2026 longitudinal changes by open-loop command binding.

Replays the current winner on recorded known-good trajectories.  This is a
first-order causal screen: changed commands are real decision-boundary hits,
but downstream vehicle response still requires CARLA validation.
"""

import csv
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np


BASE_MU = {0: 2.75, 1: 3.00, 2: 3.35, 3: 3.4, 4: 2.95,
           5: 2.75, 6: 3.3, 7: 2.75, 8: 2.75, 9: 2.1}
SCALARS = {
    "s2_mu_3.40": {"mu": {2: 3.40}},
    "s4_mu_3.05": {"mu": {4: 3.05}},
    "full_throttle_threshold_0.95": {"speed_up_threshold": 0.95},
    "throttle_up_multiplier_1.35": {"throttle_increase": 1.35},
    "brake_initiation_multiplier_1.05": {"brake_threshold": 1.05},
    "max_speed_305": {"max_speed": 305.0},
}
STRUCTURAL = {
    "brake_counter_div3_cap8": {"brake_divisor": 3.0, "brake_cap": 8},
    "lighter_braking_branches": {"lighter_braking": True},
}


def f(row, key):
    return float(row[key])


def radius(points):
    sides = [round(math.dist(points[0], points[1]), 3),
             round(math.dist(points[1], points[2]), 3),
             round(math.dist(points[0], points[2]), 3)]
    if min(sides) < 2:
        return 10000.0
    semi = sum(sides) / 2
    area2 = semi
    for side in sides:
        area2 *= semi - side
    if area2 < 2:
        return 10000.0
    return math.prod(sides) / (4 * math.sqrt(area2))


def target_speed(rad, mu, maximum):
    if rad >= 10000:
        return maximum
    return max(20.0, min(math.sqrt(mu * 9.81 * rad) * 3.6, maximum))


def recommendation(row, locations, change):
    section = int(row["section"])
    maximum = change.get("max_speed", 300.0)
    mu = change.get("mu", {}).get(section, BASE_MU[section])
    start = np.array([f(row, "x"), f(row, "y")])
    lookahead = int(row["controller_lookahead_waypoint_index"])
    intended = [0, 30, 60, 90, 120, 140, 170]
    points = [start]
    distances = [0.0] * 7
    cumulative = 0.0
    previous = start
    for offset in range(300):
        point = locations[(lookahead + offset) % len(locations)]
        cumulative += float(np.linalg.norm(point - previous))
        if cumulative > intended[len(points)]:
            distances[len(points)] = cumulative
            points.append(point)
        previous = point
        if len(points) >= 7:
            break
    definitions = [
        ("close", [0, 1, 2], distances[0] + 3),
        ("mid", [1, 2, 3], distances[1]),
        ("far", [2, 3, 4], distances[2]),
    ]
    if f(row, "speed_kmh") > 100:
        if section != 9:
            definitions.append(("wide-mid", [1, 3, 5], distances[0] + 3))
        definitions.append(("wide-far", [0, 3, 6], distances[0] + 3))
    candidates = []
    for name, indices, distance in definitions:
        rad = radius([points[index] for index in indices])
        target = target_speed(rad, mu, maximum)
        recommended = math.sqrt(825 * ((target * target) / 675 + distance))
        candidates.append((recommended, name, rad, target, distance))
    return min(candidates)


def decision(row, recommended, counter, change):
    speed = f(row, "speed_kmh")
    previous = f(row, "controller_previous_speed_kmh")
    speed_change = round(speed - previous, 3)
    percent_change = (speed - previous) / (previous + 0.0001)
    tick_change = round(2.4 / (speed + 0.001), 5)
    ratio = speed / recommended
    brake_threshold = change.get("brake_threshold", 1.0)
    speed_up = change.get("speed_up_threshold", 0.90)
    throttle_increase = change.get("throttle_increase", 1.25)
    branch = ""
    if ratio > 1:
        if ratio > 1 + brake_threshold * tick_change:
            if counter > 0:
                throttle, brake, branch = -1.0, 1.0, "brake_counter"
            elif speed_change < 2.5:
                counter = round((speed - recommended) / change.get("brake_divisor", 7.0))
                if "brake_cap" in change:
                    counter = min(change["brake_cap"], counter)
                throttle, brake, branch = -1.0, 1.0, "brake_initiate"
            else:
                counter = 0
                throttle, brake, branch = 1.0, 0.0, "throttle_early1"
        elif speed_change >= 2.5:
            counter = 0
            throttle, brake, branch = 1.0, 0.0, "throttle_early2"
        else:
            maintain = 0.75 + speed / 500
            if ratio > 1.02 or percent_change > -tick_change / 2:
                if change.get("lighter_braking"):
                    throttle, brake = 1.0, 0.6
                else:
                    throttle, brake = maintain * 0.7, 0.0
                branch = "throttle_down"
            else:
                if change.get("lighter_braking"):
                    throttle, brake = 1.0, 0.1
                else:
                    throttle, brake = maintain, 0.0
                branch = "throttle_maintain_over"
    else:
        counter = 0
        if speed_change >= 2.5:
            throttle, brake, branch = 1.0, 0.0, "throttle_full_speed_drop"
        elif ratio < speed_up:
            throttle, brake, branch = 1.0, 0.0, "throttle_full"
        else:
            maintain = 0.75 + speed / 500
            if ratio < 0.98 or tick_change < -0.01:
                throttle, brake, branch = maintain * throttle_increase, 0.0, "throttle_up"
            else:
                throttle, brake, branch = maintain, 0.0, "throttle_maintain"
    if counter > 0 and brake > 0:
        counter -= 1
    return min(1.0, max(0.0, throttle)), min(1.0, max(0.0, brake)), counter, branch


def cohort(root):
    selected = []
    with (root / "experiment_results/ledger.jsonl").open() as stream:
        for line in stream:
            item = json.loads(line)
            parameters = item.get("parameters", {})
            if not (item.get("outcome") == "finished" and item.get("telemetry_path")):
                continue
            if not (parameters.get("qualifying_brake.two_event_combination") == 1
                    and parameters.get("qualifying_brake.wp794_lap3_final_tick") == 1
                    and parameters.get("qualifying_brake.wp2501_left_foot_percent") == 10):
                continue
            # S8=3.1 is admitted: its validation proved zero command changes.
            extras = set(parameters) - {
                "qualifying_brake.two_event_combination",
                "qualifying_brake.wp794_lap3_final_tick",
                "qualifying_brake.wp2501_left_foot_percent",
                "runtime.low_bandwidth_sensor_mode", "throttle.section_mu.8",
            }
            if not extras:
                selected.append(item)
    return selected


def clusters(changes):
    counts = Counter((int(row["lap"]), int(row["custom_waypoint_index"]) // 25 * 25)
                     for row in changes)
    return [{"lap": lap, "waypoints": [start, start + 24], "ticks": count}
            for (lap, start), count in counts.most_common(12)]


def evaluate(root, items, locations, name, change):
    changed, raw_changed, baseline_errors, branch_changes = [], 0, 0, Counter()
    state_changed = []
    section_counts, zone_counts, run_counts = Counter(), Counter(), Counter()
    reconstruction_errors = []
    zones = [("WP391", 370, 425), ("WP427", 426, 455), ("WP627", 610, 650),
             ("WP794", 780, 820), ("WP1232", 1210, 1300),
             ("WP1781", 1760, 1825), ("WP2501", 2475, 2585)]
    for item in items:
        path = root / item["telemetry_path"]
        rows = list(csv.DictReader((path.parent / "ticks.csv").open()))
        counter = 0
        for row in rows:
            reconstruct = "mu" in change or "max_speed" in change
            if reconstruct:
                base_rec = recommendation(row, locations, {})[0]
                reconstruction_errors.append(
                    abs(base_rec - f(row, "controller_recommended_speed_kmh"))
                )
                rec = recommendation(row, locations, change)[0]
            else:
                rec = f(row, "controller_recommended_speed_kmh")
            throttle, brake, counter, branch = decision(row, rec, counter, change)
            logged_t = min(1.0, max(0.0, f(row, "controller_raw_throttle")))
            logged_b = min(1.0, max(0.0, f(row, "controller_raw_brake")))
            if abs(throttle - logged_t) > 1e-8 or abs(brake - logged_b) > 1e-8:
                changed.append(row)
                run_counts[item["attempt_id"]] += 1
                section_counts[int(row["section"])] += 1
                matched = False
                waypoint = int(row["custom_waypoint_index"])
                for zone, low, high in zones:
                    if low <= waypoint <= high:
                        zone_counts[zone] += 1
                        matched = True
                if not matched:
                    zone_counts["other"] += 1
                branch_changes[(row["controller_decision_branch"], branch)] += 1
            if (branch != row["controller_decision_branch"]
                    or counter != int(f(row, "controller_brake_ticks_after_run"))):
                state_changed.append(row)
            # raw magnitude differences hidden by clipping
            if change.get("throttle_increase") and row["controller_decision_branch"] == "throttle_up":
                raw_changed += 1
            # Keep counter synchronized for scalar changes that cannot alter it.
            if name in {"full_throttle_threshold_0.95", "throttle_up_multiplier_1.35", "lighter_braking_branches"}:
                counter = int(f(row, "controller_brake_ticks_after_run"))
        # A replay mismatch for the baseline state is measured independently below.
    return {
        "name": name, "category": "structural" if name in STRUCTURAL else "scalar",
        "changed_applied_command_ticks": len(changed),
        "changed_controller_state_ticks": len(state_changed),
        "changed_raw_magnitude_ticks": raw_changed,
        "runs_affected": len(run_counts), "per_run": dict(run_counts),
        "sections": dict(sorted(section_counts.items())), "zones": dict(zone_counts),
        "top_clusters": clusters(changed),
        "top_state_change_clusters": clusters(state_changed),
        "branch_transitions": [{"from": a, "to": b, "ticks": n}
                               for (a, b), n in branch_changes.most_common()],
        "constraint_reconstruction_error_kmh": (None if not reconstruction_errors else {
            "median": float(np.median(reconstruction_errors)),
            "p99": float(np.percentile(reconstruction_errors, 99)),
            "maximum": max(reconstruction_errors),
        }),
    }


def main(root=Path.cwd()):
    items = cohort(root)
    locations = np.load(root / "competition_code/waypoints/waypointsPrimary.npz")["locations"][35:, :2]
    results = [evaluate(root, items, locations, name, change)
               for name, change in {**SCALARS, **STRUCTURAL}.items()]
    result = {
        "method": "fixed-trajectory sequential brake-counter replay",
        "runs": len(items), "run_ids": [item["attempt_id"] for item in items],
        "ticks": sum(json.loads((root / item["telemetry_path"]).read_text())["recorded_ticks"] for item in items),
        "results": results,
        "caveat": "Changed-command counts are first-order binding signals, not closed-loop lap-time predictions.",
    }
    output = root / "experiment_results/analysis/spring_command_binding"
    output.mkdir(parents=True, exist_ok=True)
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
