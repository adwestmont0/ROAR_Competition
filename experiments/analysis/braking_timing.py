import csv
import json
import statistics
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from experiments.harness.core import load_ledger, read_json, write_json


EVENTS = {
    1: {"name": "WP2501-2561", "window": (2475, 2585), "next_window": (370, 450)},
    2: {"name": "WP1232-1276", "window": (1210, 1300), "next_window": (1760, 1825)},
    3: {"name": "WP1781-1804", "window": (1760, 1825), "next_window": (2475, 2585)},
}
HORIZONS_M = (25, 50, 100, 200)


def _float(row: Dict[str, str], field: str) -> Optional[float]:
    value = row.get(field, "")
    return None if value in (None, "") else float(value)


def _truthy(value: Optional[str]) -> bool:
    return str(value).lower() in {"1", "true", "yes"}


def _raw_brake(row: Dict[str, str]) -> float:
    value = _float(row, "override_raw_winner_brake")
    if value is None:
        value = _float(row, "controller_raw_brake")
    if value is None:
        value = _float(row, "brake")
    return value or 0.0


def _mean(values: List[Optional[float]]) -> Optional[float]:
    usable = [float(value) for value in values if value is not None]
    return statistics.mean(usable) if usable else None


def _path_distances(root: Path) -> np.ndarray:
    locations = np.load(
        root / "competition_code/waypoints/waypointsPrimary.npz"
    )["locations"][35:, :2]
    return np.linalg.norm(np.roll(locations, -1, axis=0) - locations, axis=1)


def _forward_distance(steps: np.ndarray, start: int, end: int) -> float:
    if end >= start:
        return float(np.sum(steps[start:end]))
    return float(np.sum(steps[start:]) + np.sum(steps[:end]))


def _traversal(
    rows: List[Dict[str, str]], start_index: int, event_id: int, steps: np.ndarray
) -> Dict[str, Any]:
    event = EVENTS[event_id]
    lap = rows[start_index]["lap"]
    end_index = start_index
    while end_index + 1 < len(rows):
        following = rows[end_index + 1]
        waypoint = int(following["custom_waypoint_index"])
        if following["lap"] != lap or not event["window"][0] <= waypoint <= event["window"][1]:
            break
        end_index += 1
    event_rows = rows[start_index:end_index + 1]
    applied_indices = [
        start_index + offset for offset, row in enumerate(event_rows)
        if float(row["brake"]) > 0
    ]
    suppressed_indices = [
        start_index + offset for offset, row in enumerate(event_rows)
        if _truthy(row.get("override_tick_suppressed"))
    ]
    release_index = min((applied_indices[-1] + 1) if applied_indices else start_index, len(rows) - 1)
    throttle_index = next(
        (index for index in range(release_index, len(rows)) if float(rows[index]["throttle"]) > 0),
        None,
    )
    next_onset = next(
        (
            index for index in range(end_index + 1, len(rows))
            if event["next_window"][0] <= int(rows[index]["custom_waypoint_index"]) <= event["next_window"][1]
            and _raw_brake(rows[index]) > 0
        ),
        None,
    )
    release_wp = int(rows[release_index]["custom_waypoint_index"])
    horizons: Dict[str, Any] = {}
    for horizon in HORIZONS_M:
        horizon_index = next(
            (
                index for index in range(release_index, len(rows))
                if _forward_distance(
                    steps, release_wp, int(rows[index]["custom_waypoint_index"])
                ) >= horizon
            ),
            None,
        )
        horizons[str(horizon)] = None if horizon_index is None else {
            "speed_kmh": float(rows[horizon_index]["speed_kmh"]),
            "time_from_entry_seconds": (
                float(rows[horizon_index]["sim_time_seconds"])
                - float(rows[start_index]["sim_time_seconds"])
            ),
            "time_from_release_seconds": (
                float(rows[horizon_index]["sim_time_seconds"])
                - float(rows[release_index]["sim_time_seconds"])
            ),
            "waypoint": int(rows[horizon_index]["custom_waypoint_index"]),
        }
    early_suppressions = [
        index for index in suppressed_indices
        if rows[index].get("override_suppression_reason") == "early-release"
    ]
    rerequests = []
    for suppressed in early_suppressions:
        following = next(
            (
                index for index in range(suppressed + 1, end_index + 1)
                if _raw_brake(rows[index]) > 0
            ),
            None,
        )
        if following is not None:
            rerequests.append({
                "suppressed_tick": int(rows[suppressed]["tick"]),
                "requested_again_tick": int(rows[following]["tick"]),
                "released_ticks_between": following - suppressed - 1,
                "immediate": following == suppressed + 1,
            })
    return {
        "lap": int(lap),
        "event_entry_sim_time_seconds": float(rows[start_index]["sim_time_seconds"]),
        "entry_waypoint": int(rows[start_index]["custom_waypoint_index"]),
        "entry_speed_kmh": float(rows[start_index]["speed_kmh"]),
        "minimum_speed_kmh": min(float(row["speed_kmh"]) for row in event_rows),
        "raw_winner_brake_ticks": sum(_raw_brake(row) > 0 for row in event_rows),
        "applied_brake_ticks": len(applied_indices),
        "suppressed_brake_ticks": len(suppressed_indices),
        "suppression_reasons": [rows[index].get("override_suppression_reason") for index in suppressed_indices],
        "release_waypoint": release_wp,
        "release_speed_kmh": float(rows[release_index]["speed_kmh"]),
        "throttle_reapplication_waypoint": (
            None if throttle_index is None else int(rows[throttle_index]["custom_waypoint_index"])
        ),
        "throttle_reapplication_seconds_from_entry": (
            None if throttle_index is None else
            float(rows[throttle_index]["sim_time_seconds"])
            - float(rows[start_index]["sim_time_seconds"])
        ),
        "local_time_to_next_brake_seconds": (
            None if next_onset is None else
            float(rows[next_onset]["sim_time_seconds"])
            - float(rows[start_index]["sim_time_seconds"])
        ),
        "winner_rerequests": rerequests,
        "downstream": horizons,
    }


def _attempt(root: Path, item: Dict[str, Any], event_id: int, steps: np.ndarray) -> Dict[str, Any]:
    result = {
        "attempt_id": item["attempt_id"],
        "started_utc": item.get("started_utc"),
        "outcome": item["outcome"],
        "full_lap_time_seconds": item.get("elapsed_time_seconds"),
        "terminal_waypoint": item.get("custom_waypoint_index"),
        "terminal_location": item.get("terminal_location"),
        "collision_impulse": item.get("collision_impulse"),
        "parameters": item.get("parameters", {}),
        "is_control": item.get("is_control", False),
        "event_id": event_id,
        "event_name": EVENTS[event_id]["name"],
        "traversals": [],
    }
    telemetry = item.get("telemetry_path")
    if not telemetry:
        return result
    ticks = (root / telemetry).parent / "ticks.csv"
    with ticks.open(encoding="utf-8", newline="") as infile:
        rows = list(csv.DictReader(infile))
    low, high = EVENTS[event_id]["window"]
    starts = []
    seen_laps = set()
    for index, row in enumerate(rows):
        in_window = low <= int(row["custom_waypoint_index"]) <= high
        if in_window and _raw_brake(row) > 0 and row["lap"] not in seen_laps:
            starts.append(index)
            seen_laps.add(row["lap"])
    result["traversals"] = [_traversal(rows, index, event_id, steps) for index in starts]
    return result


def _aggregate(traversals: List[Dict[str, Any]]) -> Dict[str, Any]:
    fields = (
        "event_entry_sim_time_seconds", "entry_speed_kmh", "minimum_speed_kmh", "raw_winner_brake_ticks",
        "applied_brake_ticks", "suppressed_brake_ticks", "release_waypoint",
        "release_speed_kmh", "throttle_reapplication_waypoint",
        "throttle_reapplication_seconds_from_entry", "local_time_to_next_brake_seconds",
    )
    result = {field: _mean([row.get(field) for row in traversals]) for field in fields}
    result["winner_rerequested_after_suppression"] = any(
        row["winner_rerequests"] for row in traversals
    )
    gaps = [
        request["released_ticks_between"] for row in traversals
        for request in row["winner_rerequests"]
    ]
    result["rerequest_released_tick_gaps"] = gaps
    result["downstream"] = {}
    for horizon in HORIZONS_M:
        samples = [row["downstream"][str(horizon)] for row in traversals if row["downstream"][str(horizon)]]
        result["downstream"][str(horizon)] = {
            field: _mean([sample[field] for sample in samples])
            for field in ("speed_kmh", "time_from_entry_seconds", "time_from_release_seconds")
        }
    return result


def analyze(root: Path, config_path: Path) -> Dict[str, Any]:
    config = read_json(config_path)
    fixed_parameters = config.get("fixed_parameters", {})
    ledger = [
        item for item in load_ledger(root / config.get("results_dir", "experiment_results") / "ledger.jsonl")
        if item.get("experiment_name") == config["name"]
        and item.get("outcome") in {"finished", "collision", "timeout", "exception", "controller_hang"}
        and all(item.get("parameters", {}).get(name) == value for name, value in fixed_parameters.items())
    ]
    steps = _path_distances(root)
    attempts = []
    for item in ledger:
        parameters = item.get("parameters", {})
        selected = parameters.get("qualifying_brake.event")
        combined = parameters.get("qualifying_brake.two_event_combination")
        event_ids = (1, 2) if combined else (EVENTS if selected is None else (int(selected),))
        for event_id in event_ids:
            measured = _attempt(root, item, event_id, steps)
            measured["aggregate"] = _aggregate(measured["traversals"])
            attempts.append(measured)
    controls = [item for item in attempts if item["is_control"]]
    treatments = [item for item in attempts if not item["is_control"]]
    result = {
        "schema_version": 1,
        "experiment_name": config["name"],
        "controls": controls,
        "treatments": treatments,
    }
    output = (
        root / config.get("results_dir", "experiment_results")
        / "analysis" / config["name"].replace("-", "_")
    )
    output.mkdir(parents=True, exist_ok=True)
    result_path = output / "result.json"
    write_json(result_path, result)
    result["result_path"] = str(result_path.relative_to(root))
    return result
