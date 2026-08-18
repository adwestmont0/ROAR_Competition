"""Analyze the initial bracketed WP2501 left-foot-braking sweep."""

import csv
import json
import statistics
from pathlib import Path

import numpy as np

from experiments.harness.core import load_ledger, write_json


CAMPAIGN = "qualifying-brake-wp2501-left-foot-initial"
HORIZONS = (25, 50, 100, 200)


def num(row, field, default=0.0):
    value = row.get(field, "")
    return default if value in (None, "") else float(value)


def path_steps(root):
    points = np.load(root / "competition_code/waypoints/waypointsPrimary.npz")["locations"][35:, :2]
    return np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)


def distance(steps, start, end):
    if end >= start:
        return float(np.sum(steps[start:end]))
    return float(np.sum(steps[start:]) + np.sum(steps[:end]))


def raw_brake(row):
    return num(row, "override_raw_winner_brake", num(row, "controller_raw_brake"))


def extract_event(rows, onset, steps):
    lap = int(rows[onset]["lap"])
    end = onset
    while (end + 1 < len(rows) and int(rows[end + 1]["lap"]) == lap
           and int(rows[end + 1]["custom_waypoint_index"]) <= 2585):
        end += 1
    event = rows[onset:end + 1]
    applied = [i for i in range(onset, end + 1) if num(rows[i], "brake") > 0]
    raw = [i for i in range(onset, end + 1) if raw_brake(rows[i]) > 0]
    overlap = [i for i in applied if num(rows[i], "throttle") > 0]
    release = min(applied[-1] + 1, len(rows) - 1)
    full_throttle = next((i for i in range(release, len(rows))
                          if num(rows[i], "throttle") >= 0.99 and num(rows[i], "brake") <= 0), None)
    next_brake = next((i for i in range(end + 1, len(rows)) if raw_brake(rows[i]) > 0), None)
    release_wp = int(rows[release]["custom_waypoint_index"])
    horizons = {}
    for horizon in HORIZONS:
        index = next((i for i in range(release, len(rows))
                      if distance(steps, release_wp, int(rows[i]["custom_waypoint_index"])) >= horizon), None)
        horizons[str(horizon)] = None if index is None else {
            "seconds_from_onset": num(rows[index], "sim_time_seconds") - num(rows[onset], "sim_time_seconds"),
            "speed_kmh": num(rows[index], "speed_kmh"),
        }
    def acceleration(index):
        if index <= 0:
            return None
        dt = num(rows[index], "sim_time_seconds") - num(rows[index - 1], "sim_time_seconds")
        return None if dt <= 0 else (num(rows[index], "speed_kmh") - num(rows[index - 1], "speed_kmh")) / 3.6 / dt
    return {
        "lap": lap,
        "onset_waypoint": int(rows[onset]["custom_waypoint_index"]),
        "entry_speed_kmh": num(rows[onset], "speed_kmh"),
        "minimum_speed_kmh": min(num(row, "speed_kmh") for row in event),
        "raw_brake_ticks": len(raw),
        "applied_brake_ticks": len(applied),
        "overlap_ticks": len(overlap),
        "overlap_throttle": None if not overlap else statistics.mean(num(rows[i], "throttle") for i in overlap),
        "overlap_ordinals": [int(num(rows[i], "override_brake_ordinal")) for i in overlap],
        "release_waypoint": release_wp,
        "release_speed_kmh": num(rows[release], "speed_kmh"),
        "release_gear": int(num(rows[release], "target_gear")),
        "acceleration_before_release_mps2": acceleration(release),
        "acceleration_after_release_mps2": acceleration(min(release + 1, len(rows) - 1)),
        "first_full_throttle_seconds_from_onset": None if full_throttle is None else num(rows[full_throttle], "sim_time_seconds") - num(rows[onset], "sim_time_seconds"),
        "first_full_throttle_waypoint": None if full_throttle is None else int(rows[full_throttle]["custom_waypoint_index"]),
        "time_to_next_brake_seconds": None if next_brake is None else num(rows[next_brake], "sim_time_seconds") - num(rows[onset], "sim_time_seconds"),
        "horizons": horizons,
    }


def load_run(root, item, steps):
    ticks = root / item["telemetry_path"]
    with (ticks.parent / "ticks.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    events = []
    seen = set()
    for index, row in enumerate(rows):
        lap = int(row["lap"])
        waypoint = int(row["custom_waypoint_index"])
        if lap in seen or not 2475 <= waypoint <= 2585 or raw_brake(row) <= 0:
            continue
        events.append(extract_event(rows, index, steps))
        seen.add(lap)
    return {
        "attempt_id": item["attempt_id"], "started_utc": item["started_utc"],
        "is_control": item["is_control"], "outcome": item["outcome"],
        "elapsed_time_seconds": item.get("elapsed_time_seconds"),
        "collision_waypoint": item.get("custom_waypoint_index"),
        "throttle_percent": item.get("parameters", {}).get("qualifying_brake.wp2501_left_foot_percent", 0),
        "events": events,
    }


def mean(values):
    values = [value for value in values if value is not None]
    return statistics.mean(values) if values else None


def event_mean(run, getter):
    return mean([getter(event) for event in run["events"]])


def adjacent_controls(runs, index):
    before = next((run for run in reversed(runs[:index]) if run["is_control"] and run["outcome"] == "finished"), None)
    after = next((run for run in runs[index + 1:] if run["is_control"] and run["outcome"] == "finished"), None)
    return [run for run in (before, after) if run is not None]


def control_reference(controls, lap, getter):
    values = []
    for run in controls:
        event = next((event for event in run["events"] if event["lap"] == lap), None)
        if event is not None:
            values.append(getter(event))
    return mean(values)


def main(root=Path.cwd()):
    ledger = [item for item in load_ledger(root / "experiment_results/ledger.jsonl")
              if item.get("experiment_name") == CAMPAIGN and item.get("attempt_id")
              and item.get("outcome") in {"finished", "collision"} and item.get("telemetry_path")]
    steps = path_steps(root)
    runs = [load_run(root, item, steps) for item in ledger]
    runs.sort(key=lambda run: run["started_utc"])
    treatments = []
    for index, run in enumerate(runs):
        if run["is_control"]:
            continue
        controls = adjacent_controls(runs, index)
        deltas = {str(h): [] for h in HORIZONS}
        speed_deltas = {str(h): [] for h in HORIZONS}
        release_deltas = []
        minimum_deltas = []
        acceleration_before_deltas = []
        acceleration_after_deltas = []
        full_throttle_deltas = []
        segment_deltas = []
        for event in run["events"]:
            lap = event["lap"]
            for horizon in HORIZONS:
                treatment = event["horizons"][str(horizon)]
                reference = control_reference(
                    controls, lap, lambda candidate: (
                        None if candidate["horizons"][str(horizon)] is None
                        else candidate["horizons"][str(horizon)]["seconds_from_onset"]
                    )
                )
                if treatment is not None and reference is not None:
                    deltas[str(horizon)].append(treatment["seconds_from_onset"] - reference)
                speed_reference = control_reference(
                    controls, lap, lambda candidate: (
                        None if candidate["horizons"][str(horizon)] is None
                        else candidate["horizons"][str(horizon)]["speed_kmh"]
                    )
                )
                if treatment is not None and speed_reference is not None:
                    speed_deltas[str(horizon)].append(treatment["speed_kmh"] - speed_reference)
            release_ref = control_reference(controls, lap, lambda candidate: candidate["release_speed_kmh"])
            minimum_ref = control_reference(controls, lap, lambda candidate: candidate["minimum_speed_kmh"])
            accel_before_ref = control_reference(controls, lap, lambda candidate: candidate["acceleration_before_release_mps2"])
            accel_after_ref = control_reference(controls, lap, lambda candidate: candidate["acceleration_after_release_mps2"])
            full_throttle_ref = control_reference(controls, lap, lambda candidate: candidate["first_full_throttle_seconds_from_onset"])
            segment_ref = control_reference(controls, lap, lambda candidate: candidate["time_to_next_brake_seconds"])
            if release_ref is not None:
                release_deltas.append(event["release_speed_kmh"] - release_ref)
            if minimum_ref is not None:
                minimum_deltas.append(event["minimum_speed_kmh"] - minimum_ref)
            if accel_before_ref is not None:
                acceleration_before_deltas.append(event["acceleration_before_release_mps2"] - accel_before_ref)
            if accel_after_ref is not None:
                acceleration_after_deltas.append(event["acceleration_after_release_mps2"] - accel_after_ref)
            if full_throttle_ref is not None:
                full_throttle_deltas.append(event["first_full_throttle_seconds_from_onset"] - full_throttle_ref)
            if segment_ref is not None and event["time_to_next_brake_seconds"] is not None:
                segment_deltas.append(event["time_to_next_brake_seconds"] - segment_ref)
        control_laps = [event for control in controls for event in control["events"]]
        raw_reference = mean([control["elapsed_time_seconds"] for control in controls])
        treatments.append({
            **run,
            "nearby_control_attempts": [control["attempt_id"] for control in controls],
            "nearby_control_lap_time_seconds": raw_reference,
            "adjusted_lap_delta_seconds": None if run["outcome"] != "finished" else run["elapsed_time_seconds"] - raw_reference,
            "local_segment_delta_seconds": mean(segment_deltas),
            "horizon_time_deltas_seconds": {key: mean(values) for key, values in deltas.items()},
            "horizon_speed_deltas_kmh": {key: mean(values) for key, values in speed_deltas.items()},
            "release_speed_delta_kmh": mean(release_deltas),
            "minimum_speed_delta_kmh": mean(minimum_deltas),
            "acceleration_before_release_delta_mps2": mean(acceleration_before_deltas),
            "acceleration_after_release_delta_mps2": mean(acceleration_after_deltas),
            "first_full_throttle_delta_seconds": mean(full_throttle_deltas),
            "mean_raw_brake_ticks": event_mean(run, lambda event: event["raw_brake_ticks"]),
            "mean_applied_brake_ticks": event_mean(run, lambda event: event["applied_brake_ticks"]),
            "mean_overlap_ticks": event_mean(run, lambda event: event["overlap_ticks"]),
            "control_mean_applied_brake_ticks": mean([event["applied_brake_ticks"] for event in control_laps]),
            "release_acceleration_before_mps2": event_mean(run, lambda event: event["acceleration_before_release_mps2"]),
            "release_acceleration_after_mps2": event_mean(run, lambda event: event["acceleration_after_release_mps2"]),
        })
    result = {
        "campaign": CAMPAIGN,
        "window": {"first_ordinal": 35, "last_ordinal": 39, "applied_ticks": 5},
        "controls": [run for run in runs if run["is_control"]],
        "treatments": treatments,
        "infrastructure_errors_excluded": sum(
            item.get("experiment_name") == CAMPAIGN and item.get("outcome") == "infra_error"
            for item in load_ledger(root / "experiment_results/ledger.jsonl")
        ),
    }
    output = root / "experiment_results/analysis/wp2501_left_foot_initial"
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "result.json", result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
