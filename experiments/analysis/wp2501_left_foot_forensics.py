"""Offline full-run forensics for WP2501 0.10 left-foot braking validation."""

import csv
import datetime as dt
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from experiments.harness.core import load_ledger, write_json


CAMPAIGN = "qualifying-brake-wp2501-left-foot-validation"
ZONES = ((391, 415), (427, 441), (627, 633), (794, 801),
         (1232, 1276), (1781, 1804), (2501, 2561))
EVENT_WINDOWS = {
    (391, 415): (370, 425),
    (427, 441): (426, 470),
    (627, 633): (610, 660),
    (794, 801): (780, 820),
    (1232, 1276): (1210, 1300),
    (1781, 1804): (1760, 1825),
    (2501, 2561): (2475, 2585),
}


def number(row, field, default=None):
    value = row.get(field, "")
    return default if value in (None, "") else float(value)


def raw_brake(row):
    return number(row, "override_raw_winner_brake",
                  number(row, "controller_raw_brake", number(row, "brake", 0.0)))


def load_run(root, item):
    path = root / item["telemetry_path"]
    with (path.parent / "ticks.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    progress = np.maximum.accumulate(
        np.asarray([number(row, "official_waypoint_index", 0.0) for row in rows])
    )
    times = np.asarray([number(row, "sim_time_seconds", 0.0) for row in rows])
    return {"item": item, "rows": rows, "progress": progress, "times": times}


def crossing(run, progress):
    index = int(np.searchsorted(run["progress"], progress, side="left"))
    index = min(max(index, 1), len(run["rows"]) - 1)
    p0, p1 = run["progress"][index - 1:index + 1]
    t0, t1 = run["times"][index - 1:index + 1]
    fraction = 0.0 if p1 <= p0 else (progress - p0) / (p1 - p0)
    return float(t0 + fraction * (t1 - t0))


def started(run):
    return dt.datetime.fromisoformat(run["item"]["started_utc"])


def bracket(treatment, controls):
    before = [run for run in controls if started(run) < started(treatment)]
    after = [run for run in controls if started(run) > started(treatment)]
    return before[-1], after[0]


def control_weights(treatment, pair, method):
    before, after = pair
    if method == "preceding":
        return ((before, 1.0),)
    if method == "following":
        return ((after, 1.0),)
    if method == "nearest":
        chosen = min(pair, key=lambda run: abs((started(run) - started(treatment)).total_seconds()))
        return ((chosen, 1.0),)
    if method == "time_interpolated":
        span = (started(after) - started(before)).total_seconds()
        fraction = (started(treatment) - started(before)).total_seconds() / span
        return ((before, 1.0 - fraction), (after, fraction))
    return ((before, 0.5), (after, 0.5))


def reference_time(weighted, progress):
    return sum(weight * crossing(run, progress) for run, weight in weighted)


def delta_at(run, weighted, progress):
    return crossing(run, progress) - reference_time(weighted, progress)


def groups_in_zone(run, low, high):
    candidates = [(index, row) for index, row in enumerate(run["rows"])
                  if low <= int(row["custom_waypoint_index"]) <= high]
    groups, current, previous = [], [], None
    for entry in candidates:
        if previous is not None and entry[0] > previous + 1:
            groups.append(current)
            current = []
        current.append(entry)
        previous = entry[0]
    if current:
        groups.append(current)
    return groups[:3]


def event_metrics(run, zone, lap):
    groups = groups_in_zone(run, *EVENT_WINDOWS[zone])
    if lap >= len(groups):
        return None
    group = groups[lap]
    raw = [(index, row) for index, row in group if raw_brake(row) > 0]
    applied = [(index, row) for index, row in group if number(row, "brake", 0.0) > 0]
    if not raw:
        return None
    onset_i, onset = raw[0]
    release_i = min(applied[-1][0] + 1, len(run["rows"]) - 1)
    release = run["rows"][release_i]
    full_i = next((i for i in range(release_i, len(run["rows"]))
                   if number(run["rows"][i], "throttle", 0.0) >= 0.99
                   and number(run["rows"][i], "brake", 0.0) <= 0.0), None)
    next_i = next((i for i in range(group[-1][0] + 1, len(run["rows"]))
                   if raw_brake(run["rows"][i]) > 0), None)
    suppression = next((i for i, row in group
                        if str(row.get("override_tick_suppressed", "")).lower() == "true"), None)
    post_suppression = None if suppression is None or suppression + 1 >= len(run["rows"]) else run["rows"][suppression + 1]
    return {
        "lap": lap + 1,
        "entry_progress": float(run["progress"][group[0][0]]),
        "exit_progress": float(run["progress"][group[-1][0]]),
        "entry_speed": number(group[0][1], "speed_kmh"),
        "onset_waypoint": int(onset["custom_waypoint_index"]),
        "onset_progress": float(run["progress"][onset_i]),
        "raw_ticks": len(raw),
        "applied_ticks": len(applied),
        "release_waypoint": int(release["custom_waypoint_index"]),
        "release_progress": float(run["progress"][release_i]),
        "minimum_speed": min(number(row, "speed_kmh") for _, row in group),
        "exit_speed": number(release, "speed_kmh"),
        "lookahead_waypoint": number(onset, "controller_lookahead_waypoint_index"),
        "target_speed": number(onset, "controller_target_speed_kmh"),
        "recommended_speed": number(onset, "controller_recommended_speed_kmh"),
        "first_full_throttle_progress": None if full_i is None else float(run["progress"][full_i]),
        "first_full_throttle_waypoint": None if full_i is None else int(run["rows"][full_i]["custom_waypoint_index"]),
        "segment_time": None if next_i is None else float(run["times"][next_i] - run["times"][group[0][0]]),
        "wp1232_behavior": None if zone != (1232, 1276) or post_suppression is None else (
            "re_brake" if raw_brake(post_suppression) > 0 else "clean_release"
        ),
    }


def regime(event, zone):
    if event is None:
        return "missing"
    base = f"{event['raw_ticks']}/{event['applied_ticks']}"
    if zone == (1232, 1276) and event["wp1232_behavior"]:
        base += ":" + event["wp1232_behavior"]
    return base


def weighted_mean_events(events, weights, field):
    values = [(event[field], weight) for event, weight in zip(events, weights)
              if event is not None and event[field] is not None]
    total = sum(weight for _, weight in values)
    return None if not total else sum(value * weight for value, weight in values) / total


def summarize_zone(treatment, weighted, zone, lap):
    tm = event_metrics(treatment, zone, lap)
    controls = [event_metrics(run, zone, lap) for run, _ in weighted]
    weights = [weight for _, weight in weighted]
    if tm is None or any(event is None for event in controls):
        return None
    fields = ("entry_speed", "onset_waypoint", "raw_ticks", "applied_ticks",
              "release_waypoint", "minimum_speed", "exit_speed", "lookahead_waypoint",
              "target_speed", "recommended_speed", "first_full_throttle_waypoint", "segment_time")
    deltas = {}
    for field in fields:
        reference = weighted_mean_events(controls, weights, field)
        deltas[field] = None if tm[field] is None or reference is None else tm[field] - reference
    return {
        "treatment": tm,
        "control_regimes": [regime(event, zone) for event in controls],
        "treatment_regime": regime(tm, zone),
        "deltas": deltas,
        "cumulative_entering": delta_at(treatment, weighted, tm["entry_progress"]),
        "cumulative_leaving": delta_at(treatment, weighted, tm["exit_progress"]),
    }


def find_changes(grid, mean_trace, individual, threshold=0.025):
    # Aggregate monotone excursions until the trace reverses or crosses threshold.
    smooth = np.convolve(mean_trace, np.ones(5) / 5.0, mode="same")
    derivative = np.diff(smooth)
    events, start, direction = [], 0, 0
    for index, value in enumerate(derivative, 1):
        sign = 1 if value > 0.0005 else (-1 if value < -0.0005 else direction)
        if direction and sign != direction:
            change = smooth[index - 1] - smooth[start]
            if abs(change) >= threshold:
                samples = individual[:, index - 1] - individual[:, start]
                events.append({"start": float(grid[start]), "end": float(grid[index - 1]),
                               "change": float(change),
                               "repeatability": int(np.sum(np.sign(samples) == np.sign(change))),
                               "samples": samples.tolist()})
            start = index - 1
        direction = sign
    return events


def main(root=Path.cwd()):
    ledger = [item for item in load_ledger(root / "experiment_results/ledger.jsonl")
              if item.get("experiment_name") == CAMPAIGN and item.get("attempt_id")
              and item.get("outcome") == "finished" and item.get("telemetry_path")]
    runs = sorted([load_run(root, item) for item in ledger], key=started)
    controls = [run for run in runs if run["item"]["is_control"]]
    treatments = [run for run in runs if not run["item"]["is_control"]]
    end = min(run["progress"][-1] for run in runs)
    grid = np.arange(0.0, end + 0.001, 2.0)
    methods = ("midpoint", "time_interpolated", "preceding", "following", "nearest")
    traces = {}
    lap_deltas = {}
    for method in methods:
        matrix, finish = [], []
        for treatment in treatments:
            weighted = control_weights(treatment, bracket(treatment, controls), method)
            baseline = delta_at(treatment, weighted, 0.0)
            matrix.append([delta_at(treatment, weighted, point) - baseline for point in grid])
            finish.append(treatment["item"]["elapsed_time_seconds"] - sum(
                weight * run["item"]["elapsed_time_seconds"] for run, weight in weighted))
        traces[method] = {"matrix": np.asarray(matrix), "finish_deltas": finish}
        lap_deltas[method] = {
            "mean": statistics.mean(finish), "median": statistics.median(finish),
            "stddev": statistics.stdev(finish), "minimum": min(finish), "maximum": max(finish),
        }
    zone_samples = defaultdict(list)
    frequencies = defaultdict(lambda: {"control": Counter(), "treatment": Counter()})
    for run in controls:
        for zone in ZONES:
            for lap in range(3):
                frequencies[(zone, lap + 1)]["control"][regime(event_metrics(run, zone, lap), zone)] += 1
    for treatment in treatments:
        weighted = control_weights(treatment, bracket(treatment, controls), "midpoint")
        for zone in ZONES:
            for lap in range(3):
                sample = summarize_zone(treatment, weighted, zone, lap)
                zone_samples[(zone, lap + 1)].append(sample)
                frequencies[(zone, lap + 1)]["treatment"][regime(event_metrics(treatment, zone, lap), zone)] += 1
    matrix = traces["midpoint"]["matrix"]
    result = {
        "campaign": CAMPAIGN,
        "alignment": "interpolated monotonic official race-progress index",
        "grid_step_official_waypoints": 2,
        "run_ids": {"controls": [r["item"]["attempt_id"] for r in controls],
                    "treatments": [r["item"]["attempt_id"] for r in treatments]},
        "alternative_control_effects": lap_deltas,
        "regime_frequencies": {
            f"WP{zone[0]}-{zone[1]}:lap{lap}": {
                mode: dict(counter) for mode, counter in modes.items()
            } for (zone, lap), modes in frequencies.items()
        },
        "zone_samples": {
            f"WP{zone[0]}-{zone[1]}:lap{lap}": samples
            for (zone, lap), samples in zone_samples.items()
        },
        "material_trace_changes": find_changes(grid, np.mean(matrix, axis=0), matrix),
    }
    output = root / "experiment_results/analysis/wp2501_left_foot_forensics"
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "result.json", result)
    with (output / "cumulative_time_delta.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["official_progress", "mean_delta", "median_delta", "stddev"] +
                        [r["item"]["attempt_id"] for r in treatments])
        for index, point in enumerate(grid):
            values = matrix[:, index]
            writer.writerow([point, np.mean(values), np.median(values), np.std(values, ddof=1), *values])
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
