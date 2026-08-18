"""Mine naturally selected raw winner brake-count regimes from retained telemetry."""

import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path

import numpy as np

from experiments.harness.core import load_ledger, write_json


ZONES = {
    "WP391-415": (391, 415, 427, 441),
    "WP427-441": (427, 441, 627, 633),
    "WP627-633": (627, 633, 794, 801),
    "WP794-801": (780, 820, 1210, 1300),
    "WP1232-1276": (1210, 1300, 1760, 1825),
    "WP1781-1804": (1760, 1825, 2475, 2585),
    "WP2501-2561": (2475, 2585, 370, 450),
}
CAMPAIGNS = {
    "qualifying-brake-timing-initial",
    "qualifying-brake-timing-validation",
    "qualifying-brake-two-event-combination",
    "qualifying-brake-wp794-lap3-validation-rerun2",
}
HORIZONS = (25, 50, 100, 200)


def number(row, field, default=None):
    value = row.get(field, "")
    return default if value in (None, "") else float(value)


def raw_brake(row):
    return number(row, "override_raw_winner_brake",
                  number(row, "controller_raw_brake", number(row, "brake", 0))) or 0.0


def classify(parameters):
    if parameters.get("qualifying_brake.two_event_combination"):
        return "combined+wp794" if parameters.get("qualifying_brake.wp794_lap3_final_tick") else "combined"
    event = parameters.get("qualifying_brake.event")
    onset = parameters.get("qualifying_brake.onset_delay_ticks")
    release = parameters.get("qualifying_brake.release_early_ticks")
    if event == 1 and onset == 0 and release == 2:
        return "wp2501-only"
    if event == 2 and onset == 0 and release == 1:
        return "wp1232-only"
    if event == 3:
        return "wp1781-direct"
    if event is None:
        return "original-control"
    return "excluded"


def naturally_observed(zone, configuration, lap):
    if configuration == "excluded" or configuration == "wp1781-direct":
        return False
    if zone == "WP1232-1276" and configuration in {"wp1232-only", "combined", "combined+wp794"}:
        return False
    if zone == "WP2501-2561" and configuration in {"wp2501-only", "combined", "combined+wp794"}:
        return False
    if zone == "WP794-801" and configuration == "combined+wp794" and lap == 3:
        return False
    return True


def path_data(root):
    points = np.load(root / "competition_code/waypoints/waypointsPrimary.npz")["locations"][35:, :2]
    steps = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    return steps


def forward_distance(steps, start, end):
    if end >= start:
        return float(np.sum(steps[start:end]))
    return float(np.sum(steps[start:]) + np.sum(steps[:end]))


def traversal(rows, onset, end, next_low, next_high, steps, metadata):
    lap = int(rows[onset]["lap"])
    event_rows = rows[onset:end + 1]
    raw_indices = [i for i in range(onset, end + 1) if raw_brake(rows[i]) > 0]
    applied = [i for i in range(onset, end + 1) if number(rows[i], "brake", 0) > 0]
    release = min((applied[-1] + 1) if applied else onset, len(rows) - 1)
    throttle = next((i for i in range(release, len(rows)) if number(rows[i], "throttle", 0) > 0), None)
    next_onset = next((i for i in range(end + 1, len(rows))
                       if next_low <= int(rows[i]["custom_waypoint_index"]) <= next_high
                       and raw_brake(rows[i]) > 0), None)
    previous_raw = [i for i in range(0, onset) if raw_brake(rows[i]) > 0]
    previous_release = (previous_raw[-1] + 1) if previous_raw else None
    pre = max(0, onset - 3)
    dt = float(rows[onset]["sim_time_seconds"]) - float(rows[pre]["sim_time_seconds"])
    acceleration = 0.0 if dt <= 0 else (
        float(rows[onset]["speed_kmh"]) - float(rows[pre]["speed_kmh"])
    ) / 3.6 / dt
    release_wp = int(rows[release]["custom_waypoint_index"])
    horizons = {}
    for horizon in HORIZONS:
        index = next((i for i in range(release, len(rows))
                      if forward_distance(steps, release_wp, int(rows[i]["custom_waypoint_index"])) >= horizon), None)
        horizons[str(horizon)] = None if index is None else {
            "time_from_onset_seconds": float(rows[index]["sim_time_seconds"]) - float(rows[onset]["sim_time_seconds"]),
            "speed_kmh": float(rows[index]["speed_kmh"]),
        }
    result = dict(metadata)
    result.update({
        "lap": lap,
        "raw_brake_ticks": len(raw_indices),
        "applied_brake_ticks": len(applied),
        "entry_speed_kmh": float(rows[onset]["speed_kmh"]),
        "entry_acceleration_mps2": acceleration,
        "brake_onset_tick": int(rows[onset]["tick"]),
        "brake_onset_waypoint": int(rows[onset]["custom_waypoint_index"]),
        "target_speed_kmh": number(rows[onset], "controller_target_speed_kmh"),
        "recommended_speed_kmh": number(rows[onset], "controller_recommended_speed_kmh"),
        "lookahead_waypoint": number(rows[onset], "controller_lookahead_waypoint_index"),
        "lookahead_count": number(rows[onset], "controller_lookahead_count"),
        "steer": number(rows[onset], "steer"),
        "throttle": number(rows[onset], "throttle"),
        "minimum_speed_kmh": min(float(row["speed_kmh"]) for row in event_rows),
        "release_waypoint": release_wp,
        "release_seconds_from_onset": float(rows[release]["sim_time_seconds"]) - float(rows[onset]["sim_time_seconds"]),
        "exit_speed_kmh": float(rows[release]["speed_kmh"]),
        "throttle_reapplication_waypoint": None if throttle is None else int(rows[throttle]["custom_waypoint_index"]),
        "throttle_reapplication_seconds": None if throttle is None else float(rows[throttle]["sim_time_seconds"]) - float(rows[onset]["sim_time_seconds"]),
        "time_to_next_brake_seconds": None if next_onset is None else float(rows[next_onset]["sim_time_seconds"]) - float(rows[onset]["sim_time_seconds"]),
        "previous_event_exit_speed_kmh": None if previous_release is None else float(rows[previous_release]["speed_kmh"]),
        "time_since_previous_brake_seconds": None if previous_release is None else float(rows[onset]["sim_time_seconds"]) - float(rows[previous_release]["sim_time_seconds"]),
        "decision_branch": rows[onset].get("controller_decision_branch"),
        "selected_constraint": rows[onset].get("controller_selected_constraint"),
        "horizons": horizons,
    })
    return result


def extract(root, item, steps):
    ticks = root / item["telemetry_path"]
    with (ticks.parent / "ticks.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    configuration = classify(item.get("parameters", {}))
    metadata = {
        "attempt_id": item["attempt_id"], "experiment_name": item["experiment_name"],
        "configuration": configuration, "outcome": item["outcome"],
        "collision_impulse": item.get("collision_impulse"),
    }
    output = []
    for zone, (low, high, next_low, next_high) in ZONES.items():
        seen_laps = set()
        for index, row in enumerate(rows):
            lap = int(row["lap"])
            waypoint = int(row["custom_waypoint_index"])
            if lap in seen_laps or not low <= waypoint <= high or raw_brake(row) <= 0:
                continue
            end = index
            while end + 1 < len(rows) and int(rows[end + 1]["lap"]) == lap and low <= int(rows[end + 1]["custom_waypoint_index"]) <= high:
                end += 1
            sample = traversal(rows, index, end, next_low, next_high, steps, metadata)
            sample["zone"] = zone
            sample["natural_eligible"] = naturally_observed(zone, configuration, lap)
            output.append(sample)
            seen_laps.add(lap)
    return output


def mean(samples, field):
    values = [sample[field] for sample in samples if sample.get(field) is not None]
    return statistics.mean(values) if values else None


def summarize(samples):
    fields = ("entry_speed_kmh", "entry_acceleration_mps2", "brake_onset_waypoint",
              "target_speed_kmh", "lookahead_waypoint", "lookahead_count",
              "minimum_speed_kmh", "release_waypoint", "release_seconds_from_onset",
              "exit_speed_kmh", "time_to_next_brake_seconds",
              "previous_event_exit_speed_kmh", "time_since_previous_brake_seconds")
    result = {field: mean(samples, field) for field in fields}
    result["traversals"] = len(samples)
    result["attempts"] = len({sample["attempt_id"] for sample in samples})
    result["colliding_attempt_traversals"] = sum(sample["outcome"] == "collision" for sample in samples)
    result["configurations"] = dict((name, sum(sample["configuration"] == name for sample in samples))
                                    for name in sorted({sample["configuration"] for sample in samples}))
    result["horizons"] = {
        str(horizon): {
            "time_from_onset_seconds": statistics.mean(
                sample["horizons"][str(horizon)]["time_from_onset_seconds"]
                for sample in samples if sample["horizons"][str(horizon)] is not None
            ),
            "speed_kmh": statistics.mean(
                sample["horizons"][str(horizon)]["speed_kmh"]
                for sample in samples if sample["horizons"][str(horizon)] is not None
            ),
        } for horizon in HORIZONS
    }
    return result


def main(root=Path.cwd()):
    ledger = [item for item in load_ledger(root / "experiment_results/ledger.jsonl")
              if item.get("experiment_name") in CAMPAIGNS
              and item.get("outcome") in {"finished", "collision"}
              and item.get("telemetry_path")]
    steps = path_data(root)
    samples = [sample for item in ledger for sample in extract(root, item, steps)]
    regimes = defaultdict(lambda: defaultdict(list))
    for sample in samples:
        if sample["natural_eligible"]:
            regimes[sample["zone"]][sample["raw_brake_ticks"]].append(sample)
    summaries = {
        zone: {str(count): summarize(group) for count, group in sorted(counts.items())}
        for zone, counts in regimes.items()
    }
    output = root / "experiment_results/analysis/natural_brake_regimes"
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "result.json", {
        "campaigns": sorted(CAMPAIGNS), "regime_summaries": summaries,
        "samples": samples,
    })
    print(output / "result.json")


if __name__ == "__main__":
    main()
