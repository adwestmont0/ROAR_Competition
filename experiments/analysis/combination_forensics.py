"""Offline, position-aligned forensic analysis for the two-event campaign."""

import csv
import datetime as dt
import json
import statistics
from pathlib import Path

import numpy as np

from experiments.harness.core import load_ledger, write_json


EXPERIMENT = "qualifying-brake-two-event-combination"
VALIDATION_EXPERIMENT = "qualifying-brake-timing-validation"
ZONES = ((391, 415), (427, 441), (627, 633), (794, 801),
         (1232, 1276), (1781, 1804), (2501, 2561))


def _time(item):
    return dt.datetime.fromisoformat(item["started_utc"])


def _number(row, name, default=None):
    value = row.get(name, "")
    return default if value in (None, "") else float(value)


def _truth(value):
    return str(value).lower() in {"1", "true", "yes"}


def load_run(root, item, waypoint_s, lap_length):
    ticks = root / item["telemetry_path"]
    ticks = ticks.parent / "ticks.csv"
    with ticks.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    count = len(waypoint_s) - 1
    wrap = 0
    previous = int(rows[0]["custom_waypoint_index"])
    start_s = waypoint_s[previous]
    progress = []
    for row in rows:
        waypoint = int(row["custom_waypoint_index"])
        if previous > count * 0.8 and waypoint < count * 0.2:
            wrap += 1
        elif previous < count * 0.2 and waypoint > count * 0.8:
            wrap -= 1
        unwrapped_s = wrap * lap_length + waypoint_s[waypoint] - start_s
        progress.append(max(unwrapped_s, progress[-1] if progress else 0.0))
        previous = waypoint
    return {"item": item, "rows": rows, "progress": np.asarray(progress)}


def crossing(run, distance):
    index = int(np.searchsorted(run["progress"], distance, side="left"))
    index = min(max(index, 1), len(run["rows"]) - 1)
    p0, p1 = run["progress"][index - 1:index + 1]
    t0 = float(run["rows"][index - 1]["sim_time_seconds"])
    t1 = float(run["rows"][index]["sim_time_seconds"])
    fraction = 0.0 if p1 <= p0 else (distance - p0) / (p1 - p0)
    return t0 + fraction * (t1 - t0)


def pair_controls(treatment, controls):
    before = [run for run in controls if _time(run["item"]) < _time(treatment["item"])]
    after = [run for run in controls if _time(run["item"]) > _time(treatment["item"])]
    return [before[-1], after[0]]


def trace(treatment, controls, grid):
    treatment_times = np.asarray([crossing(treatment, point) for point in grid])
    control_times = np.mean(
        [[crossing(control, point) for point in grid] for control in controls], axis=0
    )
    delta = treatment_times - control_times
    delta -= delta[0]
    return delta


def rows_in_pass(run, low, high, pass_number):
    count = len(run["progress"])
    candidates = [
        (index, row) for index, row in enumerate(run["rows"])
        if low <= int(row["custom_waypoint_index"]) <= high
    ]
    groups = []
    current = []
    last = None
    for entry in candidates:
        if last is not None and entry[0] > last + 1:
            groups.append(current)
            current = []
        current.append(entry)
        last = entry[0]
    if current:
        groups.append(current)
    return groups[pass_number] if pass_number < len(groups) else []


def raw_brake(row):
    return _number(row, "override_raw_winner_brake",
                   _number(row, "controller_raw_brake", _number(row, "brake", 0)))


def zone_metrics(run, zone, pass_number):
    group = rows_in_pass(run, *zone, pass_number)
    if not group:
        return None
    raw = [(i, row) for i, row in group if raw_brake(row) > 0]
    applied = [(i, row) for i, row in group if _number(row, "brake", 0) > 0]
    if not raw:
        return None
    onset_i, onset = raw[0]
    release_i = min((applied[-1][0] + 1) if applied else onset_i, len(run["rows"]) - 1)
    release = run["rows"][release_i]
    throttle_i = next((i for i in range(release_i, len(run["rows"]))
                       if _number(run["rows"][i], "throttle", 0) > 0), None)
    next_brake_i = next((i for i in range(group[-1][0] + 1, len(run["rows"]))
                        if raw_brake(run["rows"][i]) > 0), None)
    return {
        "entry_time": float(group[0][1]["sim_time_seconds"]),
        "entry_speed": float(group[0][1]["speed_kmh"]),
        "onset_time": float(onset["sim_time_seconds"]),
        "onset_waypoint": int(onset["custom_waypoint_index"]),
        "raw_brake_ticks": len(raw),
        "applied_brake_ticks": len(applied),
        "release_time": float(release["sim_time_seconds"]),
        "release_waypoint": int(release["custom_waypoint_index"]),
        "minimum_speed": min(float(row["speed_kmh"]) for _, row in group),
        "exit_speed": float(release["speed_kmh"]),
        "throttle_time": None if throttle_i is None else float(run["rows"][throttle_i]["sim_time_seconds"]),
        "throttle_waypoint": None if throttle_i is None else int(run["rows"][throttle_i]["custom_waypoint_index"]),
        "segment_time": None if next_brake_i is None else float(run["rows"][next_brake_i]["sim_time_seconds"]) - float(group[0][1]["sim_time_seconds"]),
        "section": int(float(onset["section"])),
        "target_speed": _number(onset, "controller_target_speed_kmh"),
        "lookahead_waypoint": _number(onset, "controller_lookahead_waypoint_index"),
        "lookahead_count": _number(onset, "controller_lookahead_count"),
        "decision_branch": onset.get("controller_decision_branch"),
    }


def delta_metrics(treatment, controls, zone, pass_number):
    tm = zone_metrics(treatment, zone, pass_number)
    cm = [zone_metrics(control, zone, pass_number) for control in controls]
    if tm is None or any(item is None for item in cm):
        return None
    result = {}
    for field in ("entry_speed", "onset_time", "onset_waypoint", "raw_brake_ticks",
                  "applied_brake_ticks", "release_time", "release_waypoint",
                  "minimum_speed", "exit_speed", "throttle_time", "throttle_waypoint",
                  "segment_time", "target_speed", "lookahead_waypoint", "lookahead_count"):
        if tm[field] is not None and all(item[field] is not None for item in cm):
            result[field + "_delta"] = tm[field] - statistics.mean(item[field] for item in cm)
    result.update({"treatment": tm, "control": {
        field: statistics.mean(item[field] for item in cm)
        for field in tm if isinstance(tm[field], (int, float)) and tm[field] is not None
    }})
    return result


def main(root=Path.cwd()):
    all_ledger = load_ledger(root / "experiment_results" / "ledger.jsonl")
    ledger = [item for item in all_ledger
              if item.get("experiment_name") == EXPERIMENT and item.get("outcome") == "finished"]
    points = np.load(root / "competition_code/waypoints/waypointsPrimary.npz")["locations"][35:, :2]
    steps = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    waypoint_s = np.r_[0.0, np.cumsum(steps)]
    lap_length = float(np.sum(steps))
    runs = [load_run(root, item, waypoint_s, lap_length) for item in ledger]
    controls = [run for run in runs if run["item"]["is_control"]]
    treatments = [run for run in runs if not run["item"]["is_control"]]
    end = min(run["progress"][-1] for run in runs)
    grid = np.arange(0.0, end, 2.0)
    traces = []
    zone_results = []
    for treatment in treatments:
        paired = pair_controls(treatment, controls)
        values = trace(treatment, paired, grid)
        traces.append(values)
        per_zone = {}
        for zone in ZONES:
            samples = [delta_metrics(treatment, paired, zone, lap) for lap in range(3)]
            per_zone[f"WP{zone[0]}-{zone[1]}"] = samples
        zone_results.append({"attempt_id": treatment["item"]["attempt_id"], "zones": per_zone})
    trace_matrix = np.asarray(traces)
    validation_items = [item for item in all_ledger
                        if item.get("experiment_name") == VALIDATION_EXPERIMENT
                        and item.get("outcome") == "finished"]
    validation_runs = [load_run(root, item, waypoint_s, lap_length) for item in validation_items]
    validation_controls = [run for run in validation_runs if run["item"]["is_control"]]
    independent = {}
    for label, event, release in (("wp1232", 2, 1), ("wp2501", 1, 2)):
        selected = [run for run in validation_runs if not run["item"]["is_control"]
                    and run["item"]["parameters"].get("qualifying_brake.event") == event
                    and run["item"]["parameters"].get("qualifying_brake.onset_delay_ticks") == 0
                    and run["item"]["parameters"].get("qualifying_brake.release_early_ticks") == release]
        independent[label] = np.asarray([
            trace(run, pair_controls(run, validation_controls), grid) for run in selected
        ])
    output_dir = root / "experiment_results/analysis/qualifying_brake_two_event_forensics"
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "cumulative_time_delta.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["distance_m", "mean_delta_s", "median_delta_s", "stddev_s"] +
                        [run["item"]["attempt_id"] for run in treatments])
        for index, distance in enumerate(grid):
            samples = trace_matrix[:, index]
            writer.writerow([distance, np.mean(samples), np.median(samples), np.std(samples, ddof=1), *samples])
    expected = np.mean(independent["wp1232"], axis=0) + np.mean(independent["wp2501"], axis=0)
    interaction = np.mean(trace_matrix, axis=0) - expected
    uncertainty = np.sqrt(
        np.var(trace_matrix, axis=0, ddof=1) / len(trace_matrix)
        + np.var(independent["wp1232"], axis=0, ddof=1) / len(independent["wp1232"])
        + np.var(independent["wp2501"], axis=0, ddof=1) / len(independent["wp2501"])
    )
    with (output_dir / "interaction_trace.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["distance_m", "combined_delta_s", "wp1232_delta_s",
                         "wp2501_delta_s", "expected_additive_delta_s",
                         "interaction_s", "interaction_standard_error_s"])
        for index, distance in enumerate(grid):
            writer.writerow([distance, np.mean(trace_matrix[:, index]),
                             np.mean(independent["wp1232"][:, index]),
                             np.mean(independent["wp2501"][:, index]), expected[index],
                             interaction[index], uncertainty[index]])
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    for values in trace_matrix:
        axes[0].plot(grid, values, color="#8aa6c1", alpha=0.35, linewidth=0.7)
    axes[0].plot(grid, np.mean(trace_matrix, axis=0), color="#005a9c", linewidth=2,
                 label="Combined mean")
    axes[0].axhline(0, color="black", linewidth=0.6)
    axes[0].set_ylabel("Cumulative delta t (s)")
    axes[0].legend()
    axes[1].plot(grid, interaction, color="#a33a2b", linewidth=2,
                 label="Observed interaction")
    axes[1].fill_between(grid, interaction - 1.96 * uncertainty,
                         interaction + 1.96 * uncertainty, color="#a33a2b", alpha=0.15,
                         label="Approx. 95% interval")
    axes[1].axhline(0, color="black", linewidth=0.6)
    axes[1].set_ylabel("Interaction (s)")
    axes[1].set_xlabel("Unwrapped track distance (m; three laps)")
    axes[1].legend()
    for axis in axes:
        for lap in (1, 2):
            axis.axvline(lap * lap_length, color="black", linestyle=":", linewidth=0.8)
        axis.grid(alpha=0.2)
    figure.tight_layout()
    figure.savefig(output_dir / "time_delta_traces.svg")
    plt.close(figure)
    write_json(output_dir / "zone_comparisons.json", {
        "lap_length_m": lap_length,
        "treatment_attempts": [run["item"]["attempt_id"] for run in treatments],
        "control_attempts": [run["item"]["attempt_id"] for run in controls],
        "zones": zone_results,
    })
    print(output_dir)


if __name__ == "__main__":
    main()
