"""Causal, distance-domain interpretation of v2 velocity opportunities."""

import datetime as dt
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

from experiments.harness.core import read_json, write_json
from .velocity_profile import _ledger, _ticks, _verified_baseline
from .velocity_profile_v2 import _closed_line, _smoothed_acceleration, _lookup


SCHEMA_VERSION = 1
CAUSES = {
    "target-speed limitation", "braking onset", "braking magnitude",
    "braking release", "throttle onset", "acceleration capability",
    "preceding-corner exit speed", "racing-line/curvature limitation",
    "unsupported model extrapolation",
}


def _median_or_none(values: Iterable[float]) -> Optional[float]:
    materialized = list(values)
    return float(statistics.median(materialized)) if materialized else None


def _circular_distance(a: float, b: float, lap: float) -> float:
    difference = abs(a - b) % lap
    return min(difference, lap - difference)


def _baseline_fields(
    root: Path, runs: List[Dict[str, Any]], line: Dict[str, Any]
) -> Dict[str, np.ndarray]:
    count = line["count"]
    names = ("acceleration", "throttle", "brake", "steer", "line_error", "section")
    per_name: Dict[str, List[List[float]]] = {name: [[] for _ in range(count)] for name in names}
    speed_by_run: List[List[float]] = [[] for _ in range(count)]
    for run in runs:
        rows = _ticks(root, run)
        accel_by_tick = {index: acceleration for index, _, acceleration, _, _ in _smoothed_acceleration(rows)}
        per_bin: Dict[int, Dict[str, List[float]]] = defaultdict(lambda: defaultdict(list))
        for row_index, row in enumerate(rows):
            original = int(row["custom_waypoint_index"]) % len(line["raw"])
            bucket = int(line["original_to_bin"][original])
            tangent = line["xy"][(bucket + 1) % count] - line["xy"][(bucket - 1) % count]
            tangent /= max(float(np.linalg.norm(tangent)), 1e-9)
            displacement = np.asarray([float(row["x"]), float(row["y"])]) - line["xy"][bucket]
            signed_error = tangent[0] * displacement[1] - tangent[1] * displacement[0]
            values = {
                "throttle": float(row["throttle"]), "brake": float(row["brake"]),
                "steer": float(row["steer"]), "line_error": float(signed_error),
                "section": float(row["section"]), "speed": float(row["speed_kmh"]),
            }
            if row_index in accel_by_tick:
                values["acceleration"] = float(accel_by_tick[row_index])
            for name, value in values.items():
                per_bin[bucket][name].append(value)
        for bucket, values in per_bin.items():
            for name in names:
                if values.get(name):
                    if name == "section":
                        per_name[name][bucket].append(float(Counter(int(x) for x in values[name]).most_common(1)[0][0]))
                    else:
                        per_name[name][bucket].append(float(np.mean(values[name])))
            speed_by_run[bucket].append(float(np.mean(values["speed"])))
    output = {
        name: np.asarray([_median_or_none(values) if values else np.nan for values in bins], dtype=float)
        for name, bins in per_name.items()
    }
    output["speed_std"] = np.asarray([
        float(np.std(values, ddof=1)) if len(values) > 1 else np.nan for values in speed_by_run
    ])
    output["support"] = np.asarray([len(values) for values in speed_by_run], dtype=int)
    return output


def _lookahead(speed_kmh: float) -> int:
    for upper, points in ((70, 8), (90, 9), (110, 11), (130, 14), (160, 18), (180, 22), (200, 26), (250, 30), (300, 35)):
        if speed_kmh < upper:
            return points
    return 8


def _radius(points: np.ndarray, maximum: float = 10000.0) -> float:
    first, second, third = points
    side1 = round(float(np.linalg.norm(second - first)), 3)
    side2 = round(float(np.linalg.norm(third - second)), 3)
    side3 = round(float(np.linalg.norm(third - first)), 3)
    if min(side1, side2, side3) < 2:
        return maximum
    semiperimeter = (side1 + side2 + side3) / 2.0
    area_squared = semiperimeter * (semiperimeter - side1) * (semiperimeter - side2) * (semiperimeter - side3)
    if area_squared < 2:
        return maximum
    return side1 * side2 * side3 / (4.0 * math.sqrt(area_squared))


def _target_speed(radius: float, section: int, maximum_radius: float = 10000.0) -> float:
    if radius >= maximum_radius:
        return 300.0
    mu = {1: 3.00, 2: 3.35, 3: 3.4, 4: 2.95, 6: 3.3, 7: 2.75, 8: 2.75, 9: 2.1}.get(section, 2.75)
    return max(20.0, min(math.sqrt(mu * 9.81 * radius) * 3.6, 300.0))


def _recommended_speed(
    current_index: int, location: np.ndarray, speed_kmh: float, section: int,
    raw: np.ndarray,
) -> float:
    start_index = (current_index + _lookahead(speed_kmh)) % len(raw)
    intended = [0, 30, 60, 90, 120, 140, 170]
    target_distances = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0]
    points = [location]
    accumulated = 0.0
    start = location
    for offset in range(300):
        end = raw[(start_index + offset) % len(raw)]
        accumulated += float(np.linalg.norm(end - start))
        if accumulated > intended[len(points)]:
            target_distances[len(points)] = accumulated
            points.append(end)
        start = end
        if len(points) >= len(target_distances):
            break
    if len(points) < 7:
        return float("nan")
    point_array = np.asarray(points)
    candidates = []
    configurations = [((0, 1, 2), target_distances[0] + 3.0), ((1, 2, 3), target_distances[1]), ((2, 3, 4), target_distances[2])]
    if speed_kmh > 100:
        if section != 9:
            configurations.append(((1, 3, 5), target_distances[0] + 3.0))
        configurations.append(((0, 3, 6), target_distances[0] + 3.0))
    for indices, distance in configurations:
        target = _target_speed(_radius(point_array[list(indices)]), section)
        candidates.append(math.sqrt(825.0 * ((target ** 2) / 675.0 + distance)))
    return min(candidates)


def _material_divergence(
    window_indices: List[int], profile: List[Dict[str, Any]], baseline: Dict[str, np.ndarray],
    candidate_start_position: int, candidate_end_position: int,
) -> Optional[int]:
    flags = []
    for index in window_indices:
        difference = profile[index]["stability_capped_planned_speed_kmh"] - profile[index]["baseline_median_speed_kmh"]
        noise = baseline["speed_std"][index]
        threshold = max(2.0, 2.0 * noise if np.isfinite(noise) else 2.0)
        flags.append(abs(difference) >= threshold and profile[index]["confidence"] != "low")
    runs = []
    offset = 0
    while offset < len(flags):
        if not flags[offset]:
            offset += 1
            continue
        end = offset + 1
        while end < len(flags) and flags[end]:
            end += 1
        if end - offset >= 3:
            runs.append((offset, end))
        offset = end
    overlapping = [
        run for run in runs
        if run[0] < candidate_end_position and run[1] > candidate_start_position
    ]
    if overlapping:
        return window_indices[overlapping[-1][0]]
    return None


def _classify(
    divergence: Optional[int], window_indices: List[int], details: List[Dict[str, Any]],
    at_window_start: bool,
) -> Tuple[str, str]:
    if divergence is None:
        return "unsupported model extrapolation", "No sustained, support-qualified speed separation was found in the extended window."
    detail_by_index = {item["bin"]: item for item in details}
    point = detail_by_index[divergence]
    if point["confidence"] == "low":
        return "unsupported model extrapolation", "The first separation lies in a low-support state bin."
    if at_window_start:
        return "preceding-corner exit speed", "The planned velocity advantage already exists at the upstream edge of the 300 m extension."
    target_gap = point["planned"]["speed_kmh"] - point["baseline"]["controller_recommended_speed_kmh"]
    if point["active_physics_constraint"] == "lateral":
        return "racing-line/curvature limitation", "Separation begins where the lateral/curvature envelope is active."
    if target_gap > 3.0 and point["baseline"]["speed_kmh"] <= point["baseline"]["controller_recommended_speed_kmh"] + 3.0:
        return "target-speed limitation", "The existing controller recommendation sits materially below the physics-planned speed."
    baseline_brake = point["baseline"]["brake"]
    planned_brake = point["planned"]["required_brake_fraction"]
    baseline_accel = point["baseline"]["acceleration_mps2"]
    planned_accel = point["planned"]["acceleration_mps2"]
    baseline_throttle = point["baseline"]["throttle"]
    if baseline_brake is None or baseline_accel is None or baseline_throttle is None:
        return "unsupported model extrapolation", "The initiating bin lacks measured baseline control or acceleration support."
    if baseline_brake > .25 and planned_brake < .1:
        if baseline_accel < -.5 and planned_accel >= -.25:
            return "braking release", "Baseline remains in braking while the feasible plan has released it."
        return "braking onset", "Baseline braking is active before the feasible trajectory requires comparable braking."
    if baseline_brake > .25 and planned_brake > .1 and baseline_accel < planned_accel - .5:
        return "braking magnitude", "Both trajectories brake, but baseline deceleration is materially stronger."
    if point["planned"]["required_throttle_fraction"] > .1 and baseline_throttle < .5:
        return "throttle onset", "The feasible trajectory calls for propulsion before baseline throttle is established."
    if (
        baseline_throttle >= .5
        and point["planned"]["required_throttle_fraction"] > .1
        and planned_accel > baseline_accel + .25
    ):
        return "acceleration capability", "Both trajectories accelerate; the planned envelope sustains a larger velocity build."
    return "preceding-corner exit speed", "The separation is best explained by velocity carried from the immediately preceding corner."


def _gain_attribution(
    indices_from_divergence: List[int], candidate_indices: set, profile: List[Dict[str, Any]],
    ds: float,
) -> Dict[str, Any]:
    if not indices_from_divergence:
        return {"total_gain_seconds": 0.0, "initial_velocity_advantage_seconds": 0.0, "continued_acceleration_difference_seconds": 0.0}
    first = indices_from_divergence[0]
    initial_energy_advantage = (
        (profile[first]["stability_capped_planned_speed_kmh"] / 3.6) ** 2
        - (profile[first]["baseline_median_speed_kmh"] / 3.6) ** 2
    )
    total = initial = 0.0
    for index in indices_from_divergence:
        baseline_speed = profile[index]["baseline_median_speed_kmh"] / 3.6
        planned_speed = profile[index]["stability_capped_planned_speed_kmh"] / 3.6
        # Under identical acceleration versus distance, v^2 differs by the
        # initial kinetic-energy offset.  This isolates carry-in velocity from
        # any continuing difference in acceleration after the initiating bin.
        initial_only = math.sqrt(max(.01, baseline_speed ** 2 + initial_energy_advantage))
        if index in candidate_indices:
            total += ds * (1.0 / max(baseline_speed, .1) - 1.0 / max(planned_speed, .1))
            initial += ds * (1.0 / max(baseline_speed, .1) - 1.0 / max(initial_only, .1))
    continued = total - initial
    return {
        "total_gain_seconds": float(total),
        "initial_velocity_advantage_seconds": float(initial),
        "continued_acceleration_difference_seconds": float(continued),
        "initial_velocity_fraction": None if abs(total) < 1e-9 else float(initial / total),
    }


def analyze(
    root: Path, results_dir: str = "experiment_results", extension_m: float = 300.0,
) -> Dict[str, Any]:
    results_root = root / results_dir
    v2 = read_json(results_root / "analysis/velocity_profile_v2/latest.json")
    profile = v2["distance_profile"]
    line = _closed_line(root, float(v2["line"]["spacing_m"]))
    attempts = _ledger(results_root / "ledger.jsonl")
    successes = [item for item in attempts if _verified_baseline(root, item)]
    baseline = _baseline_fields(root, successes, line)
    envelope = v2["longitudinal_envelopes"]
    centers = np.asarray(envelope["speed_centers_kmh"], dtype=float)
    accel_low = np.asarray(envelope["acceleration_low_lateral_mps2"], dtype=float)
    accel_high = np.asarray(envelope["acceleration_high_lateral_mps2"], dtype=float)
    decel_envelope = np.asarray(envelope["deceleration_mps2"], dtype=float)
    count, ds, lap = len(profile), float(line["ds"]), float(line["lap_length"])
    top = v2["top_5_candidate_intervals"]
    candidates = []
    for rank, candidate in enumerate(top, 1):
        upstream_bins = int(math.ceil(extension_m / ds))
        start_bin = int(round(candidate["start_m"] / ds)) % count
        candidate_length_bins = max(1, int(round((candidate["end_m"] - candidate["start_m"]) / ds)))
        candidate_indices_ordered = [(start_bin + offset) % count for offset in range(candidate_length_bins)]
        window_indices = [(start_bin - upstream_bins + offset) % count for offset in range(candidate_length_bins + 2 * upstream_bins)]
        divergence = _material_divergence(
            window_indices, profile, baseline, upstream_bins,
            upstream_bins + candidate_length_bins,
        )
        detail_rows = []
        for unwrapped_offset, index in enumerate(window_indices):
            base_speed = profile[index]["baseline_median_speed_kmh"]
            planned_speed = profile[index]["stability_capped_planned_speed_kmh"]
            section = int(round(baseline["section"][index])) if np.isfinite(baseline["section"][index]) else 0
            base_recommended = _recommended_speed(profile[index]["custom_waypoint_index"], line["xy"][index], base_speed, section, line["raw"])
            planned_recommended = _recommended_speed(profile[index]["custom_waypoint_index"], line["xy"][index], planned_speed, section, line["raw"])
            previous = (index - 1) % count; following = (index + 1) % count
            planned_acceleration = (((profile[following]["stability_capped_planned_speed_kmh"] / 3.6) ** 2 - (profile[previous]["stability_capped_planned_speed_kmh"] / 3.6) ** 2) / (4.0 * ds))
            lateral = (planned_speed / 3.6) ** 2 * abs(profile[index]["curvature_1pm"])
            accel_cap = _lookup(planned_speed, centers, accel_high if lateral >= 8.0 else accel_low)
            decel_cap = _lookup(planned_speed, centers, decel_envelope)
            required_throttle = max(0.0, min(1.0, planned_acceleration / max(accel_cap, .01)))
            required_brake = max(0.0, min(1.0, -planned_acceleration / max(decel_cap, .01)))
            base_steer = baseline["steer"][index]
            planned_steer = float(np.clip(base_steer * planned_speed / max(base_speed, 1.0), -1.0, 1.0)) if np.isfinite(base_steer) else None
            detail_rows.append({
                "bin": index,
                "unwrapped_distance_from_candidate_start_m": float((unwrapped_offset - upstream_bins) * ds),
                "s_m": profile[index]["s_m"], "custom_waypoint_index": profile[index]["custom_waypoint_index"],
                "x": profile[index]["x"], "y": profile[index]["y"], "curvature_1pm": profile[index]["curvature_1pm"],
                "active_physics_constraint": profile[index]["active_constraint"], "confidence": profile[index]["confidence"],
                "baseline": {
                    "speed_kmh": base_speed, "acceleration_mps2": None if not np.isfinite(baseline["acceleration"][index]) else float(baseline["acceleration"][index]),
                    "controller_recommended_speed_kmh": base_recommended,
                    "throttle": None if not np.isfinite(baseline["throttle"][index]) else float(baseline["throttle"][index]),
                    "brake": None if not np.isfinite(baseline["brake"][index]) else float(baseline["brake"][index]),
                    "lateral_acceleration_mps2": (base_speed / 3.6) ** 2 * abs(profile[index]["curvature_1pm"]),
                    "steering": None if not np.isfinite(base_steer) else float(base_steer),
                    "racing_line_error_m": None if not np.isfinite(baseline["line_error"][index]) else float(baseline["line_error"][index]),
                    "support": int(baseline["support"][index]),
                },
                "planned": {
                    "speed_kmh": planned_speed, "acceleration_mps2": float(planned_acceleration),
                    "controller_recommended_speed_kmh": planned_recommended,
                    "required_throttle_fraction": float(required_throttle), "required_brake_fraction": float(required_brake),
                    "lateral_acceleration_mps2": float(lateral),
                    "steering_proxy": planned_steer, "racing_line_error_m": 0.0,
                },
            })
        at_start = divergence == window_indices[0] if divergence is not None else False
        cause, explanation = _classify(divergence, window_indices, detail_rows, at_start)
        if cause not in CAUSES:
            raise RuntimeError("Unknown causal classification: " + cause)
        if divergence is None:
            downstream = []
        else:
            divergence_position = window_indices.index(divergence)
            downstream = window_indices[divergence_position:]
        attribution = _gain_attribution(
            downstream, set(candidate_indices_ordered), profile, ds
        )
        candidates.append({
            "source_rank": rank, "candidate": candidate,
            "extension_m": extension_m,
            "earliest_material_divergence": None if divergence is None else {
                "s_m": profile[divergence]["s_m"], "custom_waypoint_index": profile[divergence]["custom_waypoint_index"],
                "distance_before_candidate_start_m": float((window_indices.index(divergence) - upstream_bins) * ds),
                "baseline_speed_kmh": profile[divergence]["baseline_median_speed_kmh"],
                "planned_speed_kmh": profile[divergence]["stability_capped_planned_speed_kmh"],
            },
            "initiating_cause": cause, "causal_explanation": explanation,
            "gain_attribution": attribution, "distance_profile": detail_rows,
        })

    # Merge only when candidate regions are adjacent on the closed lap and the
    # statistically detected initiating events are spatially the same.
    parent = list(range(len(candidates)))
    def find(value: int) -> int:
        while parent[value] != value:
            parent[value] = parent[parent[value]]; value = parent[value]
        return value
    def union(first: int, second: int) -> None:
        a, b = find(first), find(second)
        if a != b: parent[b] = a
    for first in range(len(candidates)):
        for second in range(first + 1, len(candidates)):
            a, b = candidates[first], candidates[second]
            ad, bd = a["earliest_material_divergence"], b["earliest_material_divergence"]
            if ad is None or bd is None:
                continue
            region_a, region_b = a["candidate"], b["candidate"]
            gaps = [
                _circular_distance(region_a["end_m"], region_b["start_m"], lap),
                _circular_distance(region_b["end_m"], region_a["start_m"], lap),
            ]
            same_event = _circular_distance(ad["s_m"], bd["s_m"], lap) <= 150.0
            if min(gaps) <= extension_m and same_event:
                union(first, second)
    groups: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    for index, candidate in enumerate(candidates): groups[find(index)].append(candidate)
    interventions = []
    for members in groups.values():
        earliest = min(
            (member["earliest_material_divergence"] for member in members if member["earliest_material_divergence"] is not None),
            key=lambda value: min(value["distance_before_candidate_start_m"], 0.0),
            default=None,
        )
        cause_counts = Counter(member["initiating_cause"] for member in members)
        cause = cause_counts.most_common(1)[0][0]
        total_gain = sum(member["gain_attribution"]["total_gain_seconds"] for member in members)
        initial = sum(member["gain_attribution"]["initial_velocity_advantage_seconds"] for member in members)
        continued = total_gain - initial
        interventions.append({
            "source_candidate_ranks": [member["source_rank"] for member in members],
            "source_custom_waypoint_regions": [[member["candidate"]["custom_waypoint_start"], member["candidate"]["custom_waypoint_end"]] for member in members],
            "initiating_cause": cause, "intervention_point": earliest,
            "predicted_gain_seconds": float(total_gain),
            "initial_velocity_advantage_seconds": float(initial),
            "continued_acceleration_difference_seconds": float(continued),
            "initial_velocity_fraction": None if abs(total_gain) < 1e-9 else float(initial / total_gain),
            "why_merged": "shared initiating event within 150 m and overlapping 300 m causal windows" if len(members) > 1 else "single causal region",
        })
    interventions.sort(key=lambda value: value["predicted_gain_seconds"], reverse=True)
    result = {
        "schema_version": SCHEMA_VERSION, "event_type": "counterfactual_opportunity_analysis",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "source_v2_result": v2["result_path"], "population": {"verified_successful_baseline_runs": len(successes)},
        "semantics": {
            "planned_throttle_brake": "required effort normalized by empirical acceleration/deceleration envelopes; not an applied command",
            "planned_steering": "baseline applied steering scaled by planned/baseline speed; proxy only",
            "planned_racing_line_error": "zero by construction because the physics plan follows the existing custom line",
            "controller_recommended_speed": "offline replay of the winner's lookahead, radius, section-mu, and SpeedData recommendation",
            "material_divergence": "three consecutive 5 m bins exceeding max(2 km/h, two baseline run standard deviations)",
        },
        "candidate_analyses": candidates, "ranked_causal_intervention_points": interventions,
    }
    output = results_root / "analysis" / "counterfactual_opportunity"
    result_path, report_path = output / "latest.json", output / "latest.md"
    result["result_path"] = str(result_path.relative_to(root)); result["human_report_path"] = str(report_path.relative_to(root))
    write_json(result_path, result); report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(render_report(result), encoding="utf-8")
    return result


def render_report(result: Dict[str, Any]) -> str:
    lines = [
        "# Counterfactual opportunity analysis", "",
        "Offline/shadow analysis only. No applied controller is changed.", "",
        "## Ranked causal intervention points", "",
        "| Rank | Source regions | Intervention WP | Initiating cause | Gain | Initial velocity | Continued acceleration | Merge conclusion |",
        "|---:|---|---:|---|---:|---:|---:|---|",
    ]
    for rank, item in enumerate(result["ranked_causal_intervention_points"], 1):
        point = item["intervention_point"]
        lines.append("| %d | %s | %s | %s | %.3f s | %.3f s | %.3f s | %s |" % (
            rank, ", ".join("WP %d–%d" % tuple(region) for region in item["source_custom_waypoint_regions"]),
            "unresolved" if point is None else str(point["custom_waypoint_index"]), item["initiating_cause"],
            item["predicted_gain_seconds"], item["initial_velocity_advantage_seconds"],
            item["continued_acceleration_difference_seconds"], item["why_merged"],
        ))
    lines += ["", "## Candidate diagnoses", ""]
    for item in result["candidate_analyses"]:
        candidate = item["candidate"]; divergence = item["earliest_material_divergence"]; attribution = item["gain_attribution"]
        divergence_detail = None
        if divergence is not None:
            divergence_detail = next(
                row for row in item["distance_profile"]
                if row["custom_waypoint_index"] == divergence["custom_waypoint_index"]
                and abs(row["s_m"] - divergence["s_m"]) < 1e-6
            )
        lines += [
            "### Candidate %d: WP %d–%d" % (item["source_rank"], candidate["custom_waypoint_start"], candidate["custom_waypoint_end"]), "",
            "- Earliest material separation: %s" % ("unresolved" if divergence is None else "WP %d, %+.1f m from candidate start (%.1f → %.1f km/h)" % (divergence["custom_waypoint_index"], divergence["distance_before_candidate_start_m"], divergence["baseline_speed_kmh"], divergence["planned_speed_kmh"])),
            "- Initiating cause: **%s**" % item["initiating_cause"],
            "- Diagnosis: %s" % item["causal_explanation"],
            "- Candidate-region gain: %.3f s" % attribution["total_gain_seconds"],
            "- Initial-velocity contribution: %.3f s" % attribution["initial_velocity_advantage_seconds"],
            "- Continued-acceleration contribution: %.3f s" % attribution["continued_acceleration_difference_seconds"], "",
        ]
        if divergence_detail is not None:
            base, plan = divergence_detail["baseline"], divergence_detail["planned"]
            lines += [
                "Initiating-bin state:", "",
                "| Quantity | Baseline | Planned |", "|---|---:|---:|",
                "| Speed | %.1f km/h | %.1f km/h |" % (base["speed_kmh"], plan["speed_kmh"]),
                "| Acceleration | %s | %.2f m/s² |" % ("unavailable" if base["acceleration_mps2"] is None else "%.2f m/s²" % base["acceleration_mps2"], plan["acceleration_mps2"]),
                "| Controller recommended speed | %.1f km/h | %.1f km/h |" % (base["controller_recommended_speed_kmh"], plan["controller_recommended_speed_kmh"]),
                "| Throttle / required throttle | %s | %.2f |" % ("unavailable" if base["throttle"] is None else "%.2f" % base["throttle"], plan["required_throttle_fraction"]),
                "| Brake / required brake | %s | %.2f |" % ("unavailable" if base["brake"] is None else "%.2f" % base["brake"], plan["required_brake_fraction"]),
                "| Lateral acceleration | %.2f m/s² | %.2f m/s² |" % (base["lateral_acceleration_mps2"], plan["lateral_acceleration_mps2"]),
                "| Steering / steering proxy | %s | %s |" % ("unavailable" if base["steering"] is None else "%.3f" % base["steering"], "unavailable" if plan["steering_proxy"] is None else "%.3f" % plan["steering_proxy"]),
                "| Racing-line error | %s | %.2f m |" % ("unavailable" if base["racing_line_error_m"] is None else "%.2f m" % base["racing_line_error_m"], plan["racing_line_error_m"]),
                "", "Curvature: %.5f 1/m. Active physics constraint: **%s**. Baseline support: %d runs." % (
                    divergence_detail["curvature_1pm"], divergence_detail["active_physics_constraint"], base["support"]
                ), "",
            ]
    lines += [
        "## Interpretation", "",
        "The ranked items are causal investigation points, not controller-change proposals. Full 5 m baseline/planned comparisons—including reconstructed controller recommendation, control-effort proxies, curvature, lateral acceleration, steering proxy, racing-line error, and active constraint—are in the JSON result.", "",
        "Initial-velocity attribution preserves the initiating kinetic-energy offset while following the baseline speed history. The residual is attributed to continued acceleration/deceleration differences; a negative residual means the planned trajectory subsequently gives back part of its carry-in advantage.", "",
    ]
    return "\n".join(lines)
