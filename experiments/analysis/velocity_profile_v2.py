"""Physics-informed, offline-only velocity planner for the Monza racing line."""

import csv
import datetime as dt
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

from experiments.harness.core import write_json
from .velocity_profile import _ledger, _ticks, _verified_baseline


SCHEMA_VERSION = 2


def _closed_line(root: Path, spacing_m: float) -> Dict[str, Any]:
    raw = np.load(str(root / "competition_code/waypoints/waypointsPrimary.npz"))["locations"][35:, :2]
    segment = np.linalg.norm(np.diff(raw, axis=0), axis=1)
    raw_s = np.r_[0.0, np.cumsum(segment)]
    lap_length = float(raw_s[-1] + np.linalg.norm(raw[0] - raw[-1]))
    count = max(3, int(round(lap_length / spacing_m)))
    ds = lap_length / count
    s = np.arange(count, dtype=float) * ds
    extended_s = np.r_[raw_s, lap_length]
    extended = np.vstack([raw, raw[0]])
    xy = np.column_stack([
        np.interp(s, extended_s, extended[:, 0]),
        np.interp(s, extended_s, extended[:, 1]),
    ])
    # A 10 m centered stencil suppresses waypoint quantization while retaining
    # the chicane geometry relevant to lateral acceleration.
    half_window = max(1, int(round(10.0 / ds)))
    previous = np.roll(xy, half_window, axis=0)
    following = np.roll(xy, -half_window, axis=0)
    first = (following - previous) / (2.0 * half_window * ds)
    second = (following - 2.0 * xy + previous) / ((half_window * ds) ** 2)
    denominator = np.maximum(np.linalg.norm(first, axis=1) ** 3, 1e-9)
    curvature = (first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]) / denominator
    original_to_bin = np.minimum(count - 1, np.floor(raw_s / ds).astype(int))
    bin_to_waypoint = np.searchsorted(raw_s, s, side="left")
    bin_to_waypoint = np.minimum(len(raw) - 1, bin_to_waypoint)
    return {
        "raw": raw, "raw_s": raw_s, "lap_length": lap_length,
        "count": count, "ds": ds, "s": s, "xy": xy,
        "curvature": curvature, "original_to_bin": original_to_bin,
        "bin_to_waypoint": bin_to_waypoint,
    }


def _fill(values: np.ndarray, fallback: float) -> np.ndarray:
    result = values.astype(float).copy()
    valid = np.flatnonzero(np.isfinite(result))
    if not len(valid):
        result[:] = fallback
    elif len(valid) == 1:
        result[:] = result[valid[0]]
    else:
        missing = np.flatnonzero(~np.isfinite(result))
        result[missing] = np.interp(missing, valid, result[valid])
    return result


def _speed_stats(
    root: Path, runs: List[Dict[str, Any]], line: Dict[str, Any]
) -> Tuple[Dict[str, np.ndarray], List[Dict[str, Any]]]:
    count = line["count"]
    by_bin: List[List[float]] = [[] for _ in range(count)]
    control_by_bin: List[List[Tuple[float, float]]] = [[] for _ in range(count)]
    cached = []
    for run in runs:
        rows = _ticks(root, run)
        cached.append({"run": run, "rows": rows})
        per_bin: Dict[int, List[Tuple[float, float, float]]] = defaultdict(list)
        for row in rows:
            original = int(row["custom_waypoint_index"]) % len(line["raw"])
            bucket = int(line["original_to_bin"][original])
            per_bin[bucket].append((float(row["speed_kmh"]), float(row["throttle"]), float(row["brake"])))
        for bucket, values in per_bin.items():
            array = np.asarray(values, dtype=float)
            by_bin[bucket].append(float(np.mean(array[:, 0])))
            control_by_bin[bucket].append((float(np.mean(array[:, 1])), float(np.mean(array[:, 2] > 0.0))))
    stats: Dict[str, np.ndarray] = {}
    for name, quantile in (("median", .50), ("p75", .75), ("p90", .90)):
        stats[name] = _fill(np.asarray([
            np.quantile(values, quantile) if values else np.nan for values in by_bin
        ]), 20.0)
    stats["support"] = np.asarray([len(values) for values in by_bin], dtype=int)
    stats["throttle"] = _fill(np.asarray([
        np.mean([value[0] for value in values]) if values else np.nan
        for values in control_by_bin
    ]), 0.0)
    stats["brake_fraction"] = _fill(np.asarray([
        np.mean([value[1] for value in values]) if values else np.nan
        for values in control_by_bin
    ]), 0.0)
    return stats, cached


def _smoothed_acceleration(rows: List[Dict[str, str]], half_window: int = 3) -> Iterable[Tuple[int, float, float, float, float]]:
    if len(rows) < 2 * half_window + 1:
        return []
    time = np.asarray([float(row["sim_time_seconds"]) for row in rows])
    speed = np.asarray([float(row["speed_kmh"]) / 3.6 for row in rows])
    output = []
    for index in range(half_window, len(rows) - half_window):
        dt_seconds = time[index + half_window] - time[index - half_window]
        if dt_seconds <= 0 or dt_seconds > 1.0:
            continue
        acceleration = (speed[index + half_window] - speed[index - half_window]) / dt_seconds
        output.append((
            index, speed[index] * 3.6, acceleration,
            float(rows[index]["throttle"]), float(rows[index]["brake"]),
        ))
    return output


def _longitudinal_envelopes(
    cached_runs: List[Dict[str, Any]], line: Dict[str, Any], speed_step_kmh: float = 10.0
) -> Dict[str, Any]:
    centers = np.arange(5.0, 305.0, speed_step_kmh)
    acceleration: List[List[float]] = [[] for _ in centers]
    acceleration_low_lat: List[List[float]] = [[] for _ in centers]
    acceleration_high_lat: List[List[float]] = [[] for _ in centers]
    deceleration: List[List[float]] = [[] for _ in centers]
    lateral_values: List[float] = []
    transitions = {"throttle_to_brake": [], "brake_to_throttle": []}
    for cached in cached_runs:
        rows = cached["rows"]
        prior_mode = None
        for index, speed_kmh, longitudinal, throttle, brake in _smoothed_acceleration(rows):
            row = rows[index]
            original = int(row["custom_waypoint_index"]) % len(line["raw"])
            bucket = int(line["original_to_bin"][original])
            lateral = (speed_kmh / 3.6) ** 2 * abs(float(line["curvature"][bucket]))
            lateral_values.append(lateral)
            speed_bucket = int(np.clip(math.floor(speed_kmh / speed_step_kmh), 0, len(centers) - 1))
            if throttle >= .9 and brake <= .01 and longitudinal > 0:
                acceleration[speed_bucket].append(longitudinal)
                (acceleration_high_lat if lateral >= 8.0 else acceleration_low_lat)[speed_bucket].append(longitudinal)
            if brake >= .5 and longitudinal < 0:
                deceleration[speed_bucket].append(-longitudinal)
            mode = "brake" if brake > .5 else ("throttle" if throttle > .8 else "coast")
            if prior_mode == "throttle" and mode == "brake":
                transitions["throttle_to_brake"].append({"speed_kmh": speed_kmh, "custom_waypoint_index": original})
            elif prior_mode == "brake" and mode == "throttle":
                transitions["brake_to_throttle"].append({"speed_kmh": speed_kmh, "custom_waypoint_index": original})
            prior_mode = mode

    def envelope(samples: List[List[float]], quantile: float, fallback: float, low: float, high: float) -> Tuple[np.ndarray, np.ndarray]:
        raw = np.asarray([np.quantile(values, quantile) if len(values) >= 10 else np.nan for values in samples])
        support = np.asarray([len(values) for values in samples], dtype=int)
        return np.clip(_fill(raw, fallback), low, high), support

    accel, accel_support = envelope(acceleration, .90, 1.0, .15, 8.0)
    accel_low, accel_low_support = envelope(acceleration_low_lat, .90, 1.0, .15, 8.0)
    accel_high, accel_high_support = envelope(acceleration_high_lat, .90, .7, .10, 8.0)
    decel, decel_support = envelope(deceleration, .90, 8.0, 1.0, 20.0)
    lateral = np.asarray(lateral_values)
    lateral_limit = float(np.quantile(lateral, .99))
    return {
        "speed_centers_kmh": centers, "acceleration_mps2": accel,
        "acceleration_low_lateral_mps2": accel_low,
        "acceleration_high_lateral_mps2": accel_high,
        "deceleration_mps2": decel, "acceleration_support": accel_support,
        "acceleration_low_lateral_support": accel_low_support,
        "acceleration_high_lateral_support": accel_high_support,
        "deceleration_support": decel_support,
        "lateral_acceleration_limit_mps2": lateral_limit,
        "lateral_acceleration_p95_mps2": float(np.quantile(lateral, .95)),
        "lateral_acceleration_p99_mps2": float(np.quantile(lateral, .99)),
        "lateral_samples": int(len(lateral)), "transitions": transitions,
    }


def _lookup(speed_kmh: float, centers: np.ndarray, values: np.ndarray) -> float:
    return float(np.interp(speed_kmh, centers, values, left=values[0], right=values[-1]))


def _acceleration_at(speed_kmh: float, lateral_mps2: float, envelope: Dict[str, Any]) -> float:
    values = (
        envelope["acceleration_high_lateral_mps2"]
        if lateral_mps2 >= 8.0
        else envelope["acceleration_low_lateral_mps2"]
    )
    return _lookup(speed_kmh, envelope["speed_centers_kmh"], values)


def _solve_closed(
    limit_kmh: np.ndarray, curvature: np.ndarray, ds: float,
    envelope: Dict[str, Any], tolerance_kmh: float = .001, maximum_iterations: int = 1000,
) -> Tuple[np.ndarray, int, float]:
    velocity = np.maximum(1.0, limit_kmh / 3.6)
    count = len(velocity)
    final_change = math.inf
    for iteration in range(1, maximum_iterations + 1):
        before = velocity.copy()
        # Backward braking feasibility: velocity at i must be slow enough to
        # reach the already-constrained velocity at i+1.
        for offset in range(count):
            index = count - 1 - offset
            following = (index + 1) % count
            decel = _lookup(velocity[index] * 3.6, envelope["speed_centers_kmh"], envelope["deceleration_mps2"])
            velocity[index] = min(velocity[index], math.sqrt(max(0.0, velocity[following] ** 2 + 2.0 * decel * ds)))
        # Forward propulsion feasibility.
        for index in range(count):
            following = (index + 1) % count
            lateral = velocity[index] ** 2 * abs(curvature[index])
            accel = _acceleration_at(velocity[index] * 3.6, lateral, envelope)
            velocity[following] = min(velocity[following], math.sqrt(max(0.0, velocity[index] ** 2 + 2.0 * accel * ds)))
        final_change = float(np.max(np.abs(velocity - before)) * 3.6)
        if final_change <= tolerance_kmh:
            return velocity * 3.6, iteration, final_change
    return velocity * 3.6, maximum_iterations, final_change


def _stability_caps(
    root: Path, failures: List[Dict[str, Any]], line: Dict[str, Any],
    baseline: Dict[str, np.ndarray], lateral_limit_kmh: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    samples: List[List[float]] = [[] for _ in range(line["count"])]
    for failure in failures:
        rows = _ticks(root, failure)[-160:]
        per_bin: Dict[int, List[float]] = defaultdict(list)
        for row in rows:
            original = int(row["custom_waypoint_index"]) % len(line["raw"])
            per_bin[int(line["original_to_bin"][original])].append(float(row["speed_kmh"]))
        for bucket, values in per_bin.items():
            samples[bucket].append(float(np.mean(values)))
    cap = lateral_limit_kmh.copy()
    support = np.asarray([len(values) for values in samples], dtype=int)
    boundary = np.asarray([np.quantile(values, .25) if values else np.nan for values in samples])
    for index, values in enumerate(samples):
        if not values:
            continue
        failure_q25 = float(np.quantile(values, .25))
        # Only declare a stability boundary when failures overlap the observed
        # baseline operating range. Extremely high-speed experimental failures
        # remain evidence but do not suppress unrelated feasible speeds.
        if failure_q25 <= baseline["p90"][index] + 5.0:
            cap[index] = min(cap[index], max(baseline["median"][index], failure_q25 - 1.0))
    return cap, support, boundary


def _time_seconds(speed_kmh: np.ndarray, ds: float) -> float:
    return float(np.sum(ds / np.maximum(speed_kmh / 3.6, .1)))


def _constraint_reason(
    index: int, planned: np.ndarray, uncapped: np.ndarray, lateral_limit: np.ndarray,
    stability_limit: np.ndarray, curvature: np.ndarray, ds: float, envelope: Dict[str, Any],
) -> str:
    if stability_limit[index] + .05 < uncapped[index] and abs(planned[index] - stability_limit[index]) < .25:
        return "empirical_stability"
    if abs(planned[index] - lateral_limit[index]) < .25:
        return "lateral"
    previous = (index - 1) % len(planned)
    lateral = (planned[previous] / 3.6) ** 2 * abs(curvature[previous])
    accel = _acceleration_at(planned[previous], lateral, envelope)
    forward = 3.6 * math.sqrt((planned[previous] / 3.6) ** 2 + 2.0 * accel * ds)
    following = (index + 1) % len(planned)
    decel = _lookup(planned[index], envelope["speed_centers_kmh"], envelope["deceleration_mps2"])
    backward = 3.6 * math.sqrt((planned[following] / 3.6) ** 2 + 2.0 * decel * ds)
    if abs(planned[index] - forward) <= abs(planned[index] - backward):
        return "acceleration"
    return "braking"


def _candidate_regions(intervals: List[Dict[str, Any]], adjacency_tolerance_m: float) -> List[Dict[str, Any]]:
    eligible = sorted(
        (
            dict(item) for item in intervals
            if item["stability_capped_predicted_gain_seconds"] > 0
            and item["active_constraint"] != "empirical_stability"
            and item["confidence"] in {"medium", "high"}
        ),
        key=lambda item: item["start_m"],
    )
    regions: List[Dict[str, Any]] = []
    for item in eligible:
        if (
            regions and item["active_constraint"] == regions[-1]["active_constraint"]
            and item["start_m"] - regions[-1]["end_m"] <= adjacency_tolerance_m
        ):
            region = regions[-1]
            old_length = region["end_m"] - region["start_m"]
            new_length = item["end_m"] - item["start_m"]
            total_length = old_length + new_length
            region["end_m"] = item["end_m"]
            region["custom_waypoint_end"] = item["custom_waypoint_end"]
            for key in (
                "empirical_opportunity_seconds", "physics_predicted_gain_seconds",
                "stability_capped_predicted_gain_seconds", "failure_boundary_support",
            ):
                region[key] += item[key]
            for key in ("mean_baseline_speed_kmh", "mean_planned_speed_kmh"):
                region[key] = (region[key] * old_length + item[key] * new_length) / total_length
            region["minimum_successful_run_support"] = min(region["minimum_successful_run_support"], item["minimum_successful_run_support"])
            region["minimum_longitudinal_envelope_support"] = min(region["minimum_longitudinal_envelope_support"], item["minimum_longitudinal_envelope_support"])
            if item["confidence"] == "medium":
                region["confidence"] = "medium"
            for key, value in item["constraint_counts"].items():
                region["constraint_counts"][key] = region["constraint_counts"].get(key, 0) + value
        else:
            regions.append(item)
    return sorted(regions, key=lambda item: item["stability_capped_predicted_gain_seconds"], reverse=True)


def analyze(
    root: Path, results_dir: str = "experiment_results", spacing_m: float = 5.0,
    interval_m: float = 100.0,
) -> Dict[str, Any]:
    results_root = root / results_dir
    attempts = _ledger(results_root / "ledger.jsonl")
    successes = [item for item in attempts if _verified_baseline(root, item)]
    failures = [item for item in attempts if item.get("outcome") == "collision" and item.get("telemetry_path")]
    line = _closed_line(root, spacing_m)
    baseline, cached = _speed_stats(root, successes, line)
    envelope = _longitudinal_envelopes(cached, line)
    curvature = line["curvature"]
    lateral_limit = np.minimum(
        300.0,
        3.6 * np.sqrt(envelope["lateral_acceleration_limit_mps2"] / np.maximum(np.abs(curvature), 1e-7)),
    )
    physics, physics_iterations, physics_error = _solve_closed(
        lateral_limit, curvature, line["ds"], envelope
    )
    stability_limit, failure_support, failure_boundary = _stability_caps(
        root, failures, line, baseline, lateral_limit
    )
    capped, capped_iterations, capped_error = _solve_closed(
        stability_limit, curvature, line["ds"], envelope
    )

    records = []
    for index in range(line["count"]):
        speed = capped[index]
        lateral = (speed / 3.6) ** 2 * abs(curvature[index])
        accel = _acceleration_at(speed, lateral, envelope)
        decel = _lookup(speed, envelope["speed_centers_kmh"], envelope["deceleration_mps2"])
        empirical_gain = max(0.0, line["ds"] * 3.6 * (1.0 / baseline["median"][index] - 1.0 / baseline["p90"][index]))
        physics_gain = line["ds"] * 3.6 * (1.0 / baseline["median"][index] - 1.0 / physics[index])
        capped_gain = line["ds"] * 3.6 * (1.0 / baseline["median"][index] - 1.0 / capped[index])
        reason = _constraint_reason(index, capped, physics, lateral_limit, stability_limit, curvature, line["ds"], envelope)
        support = int(baseline["support"][index])
        speed_bin = int(np.argmin(np.abs(envelope["speed_centers_kmh"] - speed)))
        acceleration_support_values = (
            envelope["acceleration_high_lateral_support"]
            if lateral >= 8.0 else envelope["acceleration_low_lateral_support"]
        )
        acceleration_support = int(acceleration_support_values[speed_bin])
        deceleration_support = int(envelope["deceleration_support"][speed_bin])
        if reason == "acceleration":
            envelope_support = acceleration_support
        elif reason == "braking":
            envelope_support = deceleration_support
        else:
            envelope_support = max(acceleration_support, deceleration_support)
        confidence = "high" if support >= 20 and envelope_support >= 50 else ("medium" if support >= 10 and envelope_support >= 10 else "low")
        records.append({
            "bin": index, "s_m": float(line["s"][index]), "ds_m": line["ds"],
            "x": float(line["xy"][index, 0]), "y": float(line["xy"][index, 1]),
            "custom_waypoint_index": int(line["bin_to_waypoint"][index]),
            "curvature_1pm": float(curvature[index]),
            "baseline_median_speed_kmh": float(baseline["median"][index]),
            "baseline_p75_speed_kmh": float(baseline["p75"][index]),
            "baseline_p90_speed_kmh": float(baseline["p90"][index]),
            "physics_planned_speed_kmh": float(physics[index]),
            "stability_capped_planned_speed_kmh": float(capped[index]),
            "lateral_speed_limit_kmh": float(lateral_limit[index]),
            "stability_speed_limit_kmh": float(stability_limit[index]),
            "estimated_lateral_acceleration_mps2": float(lateral),
            "observed_acceleration_envelope_mps2": float(accel),
            "observed_deceleration_envelope_mps2": float(decel),
            "empirical_opportunity_seconds": float(empirical_gain),
            "physics_predicted_gain_seconds": float(physics_gain),
            "stability_capped_predicted_gain_seconds": float(capped_gain),
            "successful_run_support": support,
            "longitudinal_envelope_support": envelope_support,
            "acceleration_envelope_support": acceleration_support,
            "deceleration_envelope_support": deceleration_support,
            "failure_boundary_support": int(failure_support[index]),
            "failure_boundary_q25_speed_kmh": None if not np.isfinite(failure_boundary[index]) else float(failure_boundary[index]),
            "confidence": confidence, "active_constraint": reason,
            "mean_baseline_throttle": float(baseline["throttle"][index]),
            "baseline_braking_run_fraction": float(baseline["brake_fraction"][index]),
        })

    group = max(1, int(round(interval_m / line["ds"])))
    intervals = []
    for start in range(0, len(records), group):
        members = records[start:start + group]
        if not members:
            continue
        reasons = defaultdict(int)
        for member in members:
            reasons[member["active_constraint"]] += 1
        active = max(reasons, key=reasons.get)
        support = min(member["successful_run_support"] for member in members)
        envelope_support = min(member["longitudinal_envelope_support"] for member in members)
        confidence = "high" if support >= 20 and envelope_support >= 50 else ("medium" if support >= 10 and envelope_support >= 10 else "low")
        intervals.append({
            "start_m": members[0]["s_m"], "end_m": min(line["lap_length"], members[-1]["s_m"] + line["ds"]),
            "custom_waypoint_start": members[0]["custom_waypoint_index"],
            "custom_waypoint_end": members[-1]["custom_waypoint_index"],
            "empirical_opportunity_seconds": float(sum(x["empirical_opportunity_seconds"] for x in members)),
            "physics_predicted_gain_seconds": float(sum(x["physics_predicted_gain_seconds"] for x in members)),
            "stability_capped_predicted_gain_seconds": float(sum(x["stability_capped_predicted_gain_seconds"] for x in members)),
            "active_constraint": active, "constraint_counts": dict(reasons),
            "confidence": confidence, "minimum_successful_run_support": support,
            "minimum_longitudinal_envelope_support": envelope_support,
            "failure_boundary_support": int(sum(x["failure_boundary_support"] for x in members)),
            "mean_baseline_speed_kmh": float(statistics.mean(x["baseline_median_speed_kmh"] for x in members)),
            "mean_planned_speed_kmh": float(statistics.mean(x["stability_capped_planned_speed_kmh"] for x in members)),
        })
    ranked = sorted(intervals, key=lambda item: item["stability_capped_predicted_gain_seconds"], reverse=True)
    recommended = _candidate_regions(intervals, 2.0 * line["ds"])[:5]
    baseline_time = _time_seconds(baseline["median"], line["ds"])
    physics_time = _time_seconds(physics, line["ds"])
    capped_time = _time_seconds(capped, line["ds"])
    support_mask = np.asarray([record["confidence"] in {"medium", "high"} for record in records])
    support_qualified_speed = np.where(support_mask, capped, baseline["median"])
    support_qualified_time = _time_seconds(support_qualified_speed, line["ds"])
    envelope_json = {
        key: value.tolist() if isinstance(value, np.ndarray) else value
        for key, value in envelope.items() if key != "transitions"
    }
    envelope_json["transitions"] = {
        key: {"count": len(values), "median_speed_kmh": statistics.median(x["speed_kmh"] for x in values) if values else None}
        for key, values in envelope["transitions"].items()
    }
    result = {
        "schema_version": SCHEMA_VERSION, "event_type": "physics_informed_velocity_profile",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "population": {"verified_successful_baseline_runs": len(successes), "collision_runs": len(failures)},
        "line": {"lap_length_m": line["lap_length"], "spacing_m": line["ds"], "bins": line["count"], "curvature_stencil_m": 10.0},
        "longitudinal_envelopes": envelope_json,
        "solver": {
            "physics_iterations": physics_iterations, "physics_final_change_kmh": physics_error,
            "stability_capped_iterations": capped_iterations, "stability_capped_final_change_kmh": capped_error,
            "closed_lap_periodic": True,
        },
        "integrated_lap_time": {
            "baseline_median_seconds": baseline_time,
            "physics_planned_seconds": physics_time,
            "stability_capped_planned_seconds": capped_time,
            "support_qualified_planned_seconds": support_qualified_time,
            "physics_theoretical_gain_seconds": baseline_time - physics_time,
            "stability_capped_theoretical_gain_seconds": baseline_time - capped_time,
            "support_qualified_theoretical_gain_seconds": baseline_time - support_qualified_time,
            "low_confidence_distance_m": float(np.sum(~support_mask) * line["ds"]),
        },
        "top_5_candidate_intervals": recommended,
        "ranked_intervals": ranked,
        "distance_profile": records,
        "v1_reference": {
            "result_path": str((results_root / "analysis/velocity_profile/latest.json").relative_to(root)),
            "report_path": str((results_root / "analysis/velocity_profile/latest.md").relative_to(root)),
        },
    }
    output = results_root / "analysis" / "velocity_profile_v2"
    result_path, report_path = output / "latest.json", output / "latest.md"
    result["result_path"] = str(result_path.relative_to(root)); result["human_report_path"] = str(report_path.relative_to(root))
    write_json(result_path, result)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(render_report(result), encoding="utf-8")
    return result


def render_report(result: Dict[str, Any]) -> str:
    timing = result["integrated_lap_time"]
    envelope = result["longitudinal_envelopes"]
    lines = [
        "# Monza physics-informed velocity profile (v2)", "",
        "This is an offline/shadow planner. It does not modify or apply vehicle control.", "",
        "## Whole-lap integration", "",
        "- Baseline median trajectory: %.3f s" % timing["baseline_median_seconds"],
        "- Physics-only plan: %.3f s (%+.3f s versus baseline)" % (timing["physics_planned_seconds"], timing["physics_theoretical_gain_seconds"]),
        "- Stability-capped plan: %.3f s (%+.3f s versus baseline)" % (timing["stability_capped_planned_seconds"], timing["stability_capped_theoretical_gain_seconds"]),
        "- Support-qualified plan: %.3f s (%+.3f s; baseline retained in low-confidence bins)" % (timing["support_qualified_planned_seconds"], timing["support_qualified_theoretical_gain_seconds"]),
        "- Low-confidence distance excluded from the support-qualified gain: %.1f m" % timing["low_confidence_distance_m"], "",
        "## Empirical capability calibration", "",
        "- Lateral acceleration P95/P99/planner limit: %.2f / %.2f / %.2f m/s²" % (
            envelope["lateral_acceleration_p95_mps2"], envelope["lateral_acceleration_p99_mps2"], envelope["lateral_acceleration_limit_mps2"]),
        "- Lateral samples: %d" % envelope["lateral_samples"],
        "- Acceleration and deceleration envelopes use the 90th percentile of smoothed, state-filtered observations in 10 km/h speed bins.",
        "- Acceleration is conditioned on low versus high lateral acceleration (threshold 8 m/s²).", "",
        "### Speed-dependent longitudinal envelopes", "",
        "| Speed | Acceleration P90 | Accel support | Deceleration P90 | Brake support |",
        "|---:|---:|---:|---:|---:|",
    ]
    centers = envelope["speed_centers_kmh"]
    for index in range(len(centers)):
        if 125 <= centers[index] <= 255 and index % 2 == 0:
            lines.append("| %.0f km/h | %.2f m/s² | %d | %.2f m/s² | %d |" % (
                centers[index], envelope["acceleration_mps2"][index], envelope["acceleration_support"][index],
                envelope["deceleration_mps2"][index], envelope["deceleration_support"][index],
            ))
    transitions = envelope["transitions"]
    lines += [
        "", "### Observed control transitions", "",
        "- Throttle→brake transitions: %d; median transition speed: %.1f km/h" % (
            transitions["throttle_to_brake"]["count"], transitions["throttle_to_brake"]["median_speed_kmh"]),
        "- Brake→throttle transitions: %d; median transition speed: %.1f km/h" % (
            transitions["brake_to_throttle"]["count"], transitions["brake_to_throttle"]["median_speed_kmh"]), "",
        "## Top 5 telemetry-supported candidate intervals", "",
        "| Rank | Distance | Custom WP | Baseline → plan | Physics gain | Capped gain | Empirical opportunity | Active constraint | Confidence | Why supported |",
        "|---:|---:|---:|---:|---:|---:|---:|---|---|---|",
    ]
    for rank, item in enumerate(result["top_5_candidate_intervals"], 1):
        why = "%d baseline runs; longitudinal envelope n≥%d" % (
            item["minimum_successful_run_support"], item["minimum_longitudinal_envelope_support"]
        )
        if item["failure_boundary_support"]:
            why += "; %d collision-boundary samples applied" % item["failure_boundary_support"]
        lines.append(
            "| %d | %.0f–%.0f m | %d–%d | %.1f→%.1f km/h | %.3f s | %.3f s | %.3f s | %s | %s | %s |" % (
                rank, item["start_m"], item["end_m"], item["custom_waypoint_start"], item["custom_waypoint_end"],
                item["mean_baseline_speed_kmh"], item["mean_planned_speed_kmh"],
                item["physics_predicted_gain_seconds"], item["stability_capped_predicted_gain_seconds"],
                item["empirical_opportunity_seconds"], item["active_constraint"], item["confidence"], why,
            )
        )
    lines += [
        "", "## Interpretation guardrails", "",
        "- Physics-predicted opportunity extrapolates from observed acceleration, braking, and lateral envelopes.",
        "- Empirical opportunity is the smaller baseline-median-to-P90 comparison and is reported separately.",
        "- Collision evidence is retained as a stability ceiling and propagated upstream by the braking pass.",
        "- Positive theoretical gain is not an online experiment proposal; controller realizability and repeated shadow validation come first.", "",
        "The JSON result contains the full per-bin line geometry, constraints, supports, and envelopes.", "",
    ]
    return "\n".join(lines)
