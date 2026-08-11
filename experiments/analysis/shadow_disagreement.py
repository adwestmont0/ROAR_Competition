"""Offline event analysis of winner versus H1 shadow longitudinal decisions."""

import csv
import datetime as dt
import math
import statistics
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from experiments.harness.core import read_json, write_json
from .velocity_profile import _ledger


SCHEMA_VERSION = 1
PROFILE_SHA = "3cb9e133fc72028d1098fba2319650c6b8dfe40b2b399f09a96d5bcdeb83d208"
BRAKE_ACTIVE = .05
BRAKE_DELTA = .20
TARGET_DELTA_KMH = 5.0
DECEL_DELTA = 1.5


def _number(row: Dict[str, str], name: str, default: float = 0.0) -> float:
    try:
        return float(row.get(name, ""))
    except (TypeError, ValueError):
        return default


def _median(values: Iterable[float]) -> Optional[float]:
    values = [float(x) for x in values if x is not None and math.isfinite(float(x))]
    return float(statistics.median(values)) if values else None


def _winner_required_decel(row: Dict[str, str]) -> float:
    speed = _number(row, "speed_kmh") / 3.6
    target = _number(row, "controller_target_speed_kmh", _number(row, "controller_recommended_speed_kmh")) / 3.6
    distance = max(_number(row, "controller_selected_target_distance_m", 3.0), 1.0)
    return max(0.0, (speed * speed - target * target) / (2.0 * distance))


def classify_tick(row: Dict[str, str]) -> Tuple[bool, List[str], float]:
    """Apply noise-resistant disagreement thresholds to one telemetry tick."""
    wb, sb = _number(row, "brake"), _number(row, "shadow_brake")
    wt = _number(row, "controller_recommended_speed_kmh")
    st = _number(row, "shadow_target_speed_kmh")
    wd, sd = _winner_required_decel(row), max(0.0, -_number(row, "shadow_required_acceleration_mps2"))
    kinds = []
    if wb > BRAKE_ACTIVE and sb <= BRAKE_ACTIVE:
        kinds.append("winner_brakes_shadow_does_not")
    elif sb > BRAKE_ACTIVE and wb <= BRAKE_ACTIVE:
        kinds.append("shadow_brakes_winner_does_not")
    elif max(wb, sb) > BRAKE_ACTIVE and abs(wb - sb) >= BRAKE_DELTA:
        kinds.append("brake_magnitude")
    # A target gap alone is meaningful near a control transition, or when very large.
    if abs(st - wt) >= TARGET_DELTA_KMH and max(wb, sb) > BRAKE_ACTIVE:
        kinds.append("target_speed")
    if abs(sd - wd) >= DECEL_DELTA and max(wb, sb, wd, sd) > BRAKE_ACTIVE:
        kinds.append("required_deceleration")
    magnitude = abs(wb - sb) + abs(st - wt) / 15.0 + abs(sd - wd) / 3.0
    return bool(kinds), kinds, magnitude


def group_flags(flags: List[bool], gap: int = 1) -> List[Tuple[int, int]]:
    marked = [i for i, value in enumerate(flags) if value]
    if not marked:
        return []
    groups, start, previous = [], marked[0], marked[0]
    for index in marked[1:]:
        if index - previous > gap + 1:
            groups.append((start, previous + 1)); start = index
        previous = index
    groups.append((start, previous + 1))
    return groups


def _local_groups(rows: List[Dict[str, str]], flags: List[bool], max_span_m: float = 120.0) -> List[Tuple[int, int]]:
    """Bound open-loop shadow persistence to locally testable decision windows."""
    output = []
    for start, end in group_flags(flags):
        piece_start = start
        for index in range(start + 1, end):
            span = _number(rows[index], "shadow_profile_s_m") - _number(rows[piece_start], "shadow_profile_s_m")
            wrapped = span < -1000.0
            section_change = rows[index]["section"] != rows[index - 1]["section"]
            if wrapped or section_change or span > max_span_m:
                output.append((piece_start, index)); piece_start = index
        output.append((piece_start, end))
    return [item for item in output if item[1] > item[0]]


def _first(rows: List[Dict[str, str]], predicate) -> Optional[Dict[str, str]]:
    return next((row for row in rows if predicate(row)), None)


def _behavior(rows: List[Dict[str, str]], prefix: str) -> Dict[str, Any]:
    brake_name, throttle_name = (("brake", "throttle") if prefix == "winner" else ("shadow_brake", "shadow_throttle"))
    active = [row for row in rows if _number(row, brake_name) > BRAKE_ACTIVE]
    release = None
    if active:
        last_tick = int(active[-1]["tick"])
        release = _first(rows, lambda x: int(x["tick"]) > last_tick and _number(x, brake_name) <= BRAKE_ACTIVE)
    throttle = _first(rows, lambda x: active and int(x["tick"]) > int(active[-1]["tick"]) and _number(x, throttle_name) > .05)
    return {
        "brake_onset_waypoint": int(active[0]["custom_waypoint_index"]) if active else None,
        "brake_duration_ticks": len(active),
        "peak_brake": max((_number(x, brake_name) for x in active), default=0.0),
        "mean_brake": float(statistics.mean(_number(x, brake_name) for x in active)) if active else 0.0,
        "brake_release_waypoint": int(release["custom_waypoint_index"]) if release else None,
        "throttle_reapplication_waypoint": int(throttle["custom_waypoint_index"]) if throttle else None,
    }


def _profile_context(profile: List[Dict[str, Any]], index: int) -> Dict[str, Any]:
    point = profile[index % len(profile)]
    return {
        "curvature_1pm": point.get("curvature_1pm"),
        "lateral_acceleration_mps2": point.get("estimated_lateral_acceleration_mps2"),
        "physics_constraint": point.get("active_constraint"),
        "profile_confidence": point.get("confidence"),
        "profile_support": point.get("support"),
    }


def _sample_after(rows: List[Dict[str, str]], end: int, horizon: float) -> Optional[Dict[str, Any]]:
    origin = _number(rows[end - 1], "shadow_profile_s_m")
    lap = max(_number(x, "shadow_profile_s_m") for x in rows) + 5.0
    for row in rows[end:]:
        distance = (_number(row, "shadow_profile_s_m") - origin) % lap
        if distance >= horizon:
            return {
                "actual_speed_kmh": _number(row, "speed_kmh"),
                "shadow_target_speed_kmh": _number(row, "shadow_target_speed_kmh"),
                "predicted_speed_delta_kmh": _number(row, "shadow_target_speed_kmh") - _number(row, "speed_kmh"),
                "custom_waypoint_index": int(row["custom_waypoint_index"]),
            }
    return None


def _predicted_gain(rows: List[Dict[str, str]], start: int, end: int, horizon: float = 200.0) -> float:
    gain, travelled = 0.0, 0.0
    previous = rows[start]
    for row in rows[start + 1:]:
        ds = _number(row, "shadow_profile_s_m") - _number(previous, "shadow_profile_s_m")
        if ds < -1000:  # lap wrap
            ds += 5606.256107483448
        if 0 < ds < 25:
            actual = max(_number(previous, "speed_kmh") / 3.6, 1.0)
            planned = max(_number(previous, "shadow_target_speed_kmh") / 3.6, 1.0)
            gain += ds / actual - ds / planned
            travelled += ds
        if travelled >= horizon:
            break
        previous = row
    return gain


def _event(rows: List[Dict[str, str]], start: int, end: int, profile: List[Dict[str, Any]], attempt: str, lap: int) -> Dict[str, Any]:
    # Include context to capture onset/release, while disagreement bounds remain exact.
    context = rows[max(0, start - 12):min(len(rows), end + 20)]
    event_rows = rows[start:end]
    kinds = Counter(kind for row in event_rows for kind in classify_tick(row)[1])
    winner, shadow = _behavior(context, "winner"), _behavior(context, "shadow")
    if winner["brake_duration_ticks"] and not shadow["brake_duration_ticks"]:
        primary = "reduced braking"
    elif shadow["brake_duration_ticks"] and not winner["brake_duration_ticks"]:
        primary = "increased/earlier braking"
    elif winner["brake_release_waypoint"] and shadow["brake_release_waypoint"]:
        delta = shadow["brake_duration_ticks"] - winner["brake_duration_ticks"]
        primary = "earlier release" if delta < 0 else ("later release" if delta > 0 else "braking/profile disagreement")
    else:
        primary = "target-speed/profile disagreement"
    indices = [int(x["shadow_profile_index"]) for x in event_rows]
    target_deltas = [_number(x, "shadow_target_speed_kmh") - _number(x, "controller_recommended_speed_kmh") for x in event_rows]
    alignments = [_number(x, "shadow_profile_distance_error_m") for x in event_rows]
    steer = [abs(_number(x, "steer")) for x in event_rows]
    details = [_profile_context(profile, i) for i in indices]
    final_profile_index = indices[-1]
    future_indices = [(final_profile_index + offset) % len(profile) for offset in range(1, 61)]
    apex_index = max(future_indices, key=lambda index: abs(profile[index].get("curvature_1pm") or 0.0))
    apex_offset = future_indices.index(apex_index) + 1
    exit_index = next((index for index in future_indices[apex_offset:] if abs(profile[index].get("curvature_1pm") or 0.0) < .003), None)
    next_brake = _first(rows[end:], lambda x: _number(x, "brake") > BRAKE_ACTIVE)
    winner_brake_integral = sum(_number(x, "brake") for x in event_rows)
    shadow_brake_integral = sum(_number(x, "shadow_brake") for x in event_rows)
    return {
        "attempt_id": attempt, "lap": lap,
        "start_tick": int(event_rows[0]["tick"]), "end_tick": int(event_rows[-1]["tick"]),
        "start_waypoint": int(event_rows[0]["custom_waypoint_index"]), "end_waypoint": int(event_rows[-1]["custom_waypoint_index"]),
        "start_s_m": _number(event_rows[0], "shadow_profile_s_m"), "end_s_m": _number(event_rows[-1], "shadow_profile_s_m"),
        "section": Counter(int(x["section"]) for x in event_rows).most_common(1)[0][0],
        "duration_ticks": len(event_rows), "types": dict(kinds), "primary_disagreement": primary,
        "winner": {**winner, "entry_speed_kmh": _number(event_rows[0], "speed_kmh"), "minimum_speed_kmh": min(_number(x, "speed_kmh") for x in context), "exit_speed_kmh": _number(event_rows[-1], "speed_kmh")},
        "shadow": {**shadow, "target_speed_min_kmh": min(_number(x, "shadow_target_speed_kmh") for x in context), "target_speed_mean_kmh": statistics.mean(_number(x, "shadow_target_speed_kmh") for x in context)},
        "target_speed_delta_mean_kmh": statistics.mean(target_deltas),
        "target_speed_delta_peak_abs_kmh": max(abs(x) for x in target_deltas),
        "required_decel_delta_peak_abs_mps2": max(abs(max(0.0, -_number(x, "shadow_required_acceleration_mps2")) - _winner_required_decel(x)) for x in event_rows),
        "winner_brake_command_integral": winner_brake_integral,
        "shadow_brake_command_integral": shadow_brake_integral,
        "winner_minus_shadow_brake_integral": winner_brake_integral - shadow_brake_integral,
        "profile_alignment_error_mean_m": statistics.mean(alignments), "profile_alignment_error_max_m": max(alignments),
        "max_abs_steering": max(steer), "max_lateral_acceleration_mps2": max((x.get("lateral_acceleration_mps2") or 0.0) for x in details),
        "active_constraints": dict(Counter(x.get("physics_constraint") or "unknown" for x in details)),
        "downstream": {**{str(h): _sample_after(rows, end, h) for h in (25, 50, 100, 200)},
                       "next_apex": {"profile_index": apex_index, "custom_waypoint_index": profile[apex_index]["custom_waypoint_index"], "distance_m": apex_offset * profile[apex_index]["ds_m"]},
                       "corner_exit": {"profile_index": exit_index, "custom_waypoint_index": profile[exit_index]["custom_waypoint_index"]} if exit_index is not None else None,
                       "next_winner_braking": {"custom_waypoint_index": int(next_brake["custom_waypoint_index"]), "tick": int(next_brake["tick"])} if next_brake else None},
        "predicted_time_opportunity_200m_s": _predicted_gain(rows, start, end),
        "magnitude": statistics.mean(classify_tick(x)[2] for x in event_rows),
    }


def _circular_distance(a: float, b: float, lap: float) -> float:
    difference = abs(a - b) % lap
    return min(difference, lap - difference)


def cluster_events(events: List[Dict[str, Any]], traversal_count: int, lap_length: float) -> List[Dict[str, Any]]:
    clusters: List[List[Dict[str, Any]]] = []
    for event in sorted(events, key=lambda x: (x["start_s_m"] + x["end_s_m"]) / 2):
        center = (event["start_s_m"] + event["end_s_m"]) / 2
        broad = "winner_more" if event["winner"]["mean_brake"] > event["shadow"]["mean_brake"] else "shadow_more"
        match = None
        for cluster in clusters:
            representative = cluster[0]
            other = (representative["start_s_m"] + representative["end_s_m"]) / 2
            other_broad = "winner_more" if representative["winner"]["mean_brake"] > representative["shadow"]["mean_brake"] else "shadow_more"
            if broad == other_broad and _circular_distance(center, other, lap_length) <= 35.0:
                match = cluster; break
        if match is None:
            clusters.append([event])
        else:
            match.append(event)
    output = []
    for cluster in clusters:
        traversals = {(x["attempt_id"], x["lap"]) for x in cluster}
        repeats = len(traversals) / max(traversal_count, 1)
        gain = _median(x["predicted_time_opportunity_200m_s"] for x in cluster) or 0.0
        align = _median(x["profile_alignment_error_max_m"] for x in cluster) or 0.0
        steer = _median(x["max_abs_steering"] for x in cluster) or 0.0
        lateral = _median(x["max_lateral_acceleration_mps2"] for x in cluster) or 0.0
        centers = [(x["start_s_m"] + x["end_s_m"]) / 2 for x in cluster]
        center_spread = statistics.pstdev(centers) if len(centers) > 1 else 0.0
        collision_zone = any(1240 <= x["start_waypoint"] <= 1420 or 1240 <= x["end_waypoint"] <= 1420 for x in cluster)
        threshold_risk = len({x["winner"]["brake_duration_ticks"] for x in cluster}) > 1
        risk = 1.0 + 2.0 * collision_zone + max(0.0, steer - .35) * 2 + max(0.0, align - 2.0) / 2 + (0.75 if lateral > 10 else 0) + (0.75 if threshold_risk else 0) + min(center_spread / 30.0, 1.0)
        locality = 1.0 / (1.0 + (_median(x["duration_ticks"] for x in cluster) or 0) / 20.0)
        magnitude = _median(x["magnitude"] for x in cluster) or 0.0
        brake_relief = _median(x["winner_minus_shadow_brake_integral"] for x in cluster) or 0.0
        favorable = brake_relief >= 0.5 and gain > 0.0
        representative = min(cluster, key=lambda x: abs(((x["start_s_m"] + x["end_s_m"]) / 2) - statistics.median(centers)))
        output.append({
            "event_id": None, "start_waypoint": int(round(_median(x["start_waypoint"] for x in cluster) or 0)),
            "end_waypoint": int(round(_median(x["end_waypoint"] for x in cluster) or 0)),
            "start_s_m": _median(x["start_s_m"] for x in cluster), "end_s_m": _median(x["end_s_m"] for x in cluster),
            "section": Counter(x["section"] for x in cluster).most_common(1)[0][0],
            "primary_disagreement": Counter(x["primary_disagreement"] for x in cluster).most_common(1)[0][0],
            "appearances": len(traversals), "eligible_traversals": traversal_count, "repeatability": repeats,
            "location_std_m": center_spread, "predicted_time_opportunity_200m_s": gain,
            "empirical_evidence": "repeated shadow proposal only; no shadow commands were applied",
            "profile_alignment_error_max_m_median": align, "max_abs_steering_median": steer,
            "max_lateral_acceleration_mps2_median": lateral, "discrete_threshold_indicator": threshold_risk,
            "risk_score": risk, "magnitude_score": magnitude,
            "brake_relief_integral_median": brake_relief,
            "favorable_local_intervention": favorable,
            "promising_score": max(0.0, gain) * repeats * locality / risk if favorable else 0.0,
            "winner": representative["winner"], "shadow": representative["shadow"],
            "downstream": representative["downstream"], "active_constraints": representative["active_constraints"],
            "recommended_experiment": "Locally substitute only the shadow brake timing/release decision, retain winner target/lateral logic, and run an interleaved forced-clean 5+5 A/B." if favorable else "Do not apply: the shadow does not reduce integrated braking in this local event; retain as a disagreement/risk diagnostic.",
        })
    for index, item in enumerate(sorted(output, key=lambda x: x["start_s_m"]), 1):
        item["event_id"] = index
    return output


def _report(result: Dict[str, Any]) -> str:
    lines = ["# Winner versus H1 shadow disagreement analysis", "", result["interpretation"], "",
             "## A. Most promising experiments", "", "| Rank | Event | WP | Sec | Primary disagreement | Predicted 200 m value | Repeatability | Risk |", "|---:|---:|---:|---:|---|---:|---:|---:|"]
    for rank, x in enumerate(result["rankings"]["most_promising"][:10], 1):
        lines.append("| %d | #%d | %d–%d | %d | %s | %+.3f s | %.0f%% | %.2f |" % (rank, x["event_id"], x["start_waypoint"], x["end_waypoint"], x["section"], x["primary_disagreement"], x["predicted_time_opportunity_200m_s"], 100*x["repeatability"], x["risk_score"]))
    if not result["rankings"]["most_promising"]:
        lines.append("| — | — | — | — | No event passed the favorable local-intervention gate | — | — | — |")
    lines += ["", "## B. Largest controller disagreements", "", "| Rank | Event | WP | Primary disagreement | Magnitude | Repeatability | Risk |", "|---:|---:|---:|---|---:|---:|---:|"]
    for rank, x in enumerate(result["rankings"]["largest_disagreements"][:10], 1):
        lines.append("| %d | #%d | %d–%d | %s | %.2f | %.0f%% | %.2f |" % (rank, x["event_id"], x["start_waypoint"], x["end_waypoint"], x["primary_disagreement"], x["magnitude_score"], 100*x["repeatability"], x["risk_score"]))
    lines += ["", "## Candidate details", ""]
    for x in result["rankings"]["candidate_diagnostics"][:10]:
        downstream = x["downstream"].get("200") or {}
        lines += ["### Event #%d" % x["event_id"], "", "- Location: WP %d–%d; section %d" % (x["start_waypoint"], x["end_waypoint"], x["section"]), "- Winner: brakes at %s, releases at %s, duration %s ticks" % (x["winner"]["brake_onset_waypoint"], x["winner"]["brake_release_waypoint"], x["winner"]["brake_duration_ticks"]), "- Shadow: brakes at %s, releases at %s, duration %s ticks" % (x["shadow"]["brake_onset_waypoint"], x["shadow"]["brake_release_waypoint"], x["shadow"]["brake_duration_ticks"]), "- Primary disagreement: %s" % x["primary_disagreement"], "- Predicted benefit: %+.3f s through +200 m" % x["predicted_time_opportunity_200m_s"], "- Observed evidence: %s" % x["empirical_evidence"], "- Downstream +200 m predicted speed delta: %s km/h" % ("%.2f" % downstream.get("predicted_speed_delta_kmh") if downstream else "unavailable"), "- Profile alignment error: %.2f m; repeatability %.0f%%; risk %.2f" % (x["profile_alignment_error_max_m_median"], 100*x["repeatability"], x["risk_score"]), "- Recommended experiment: %s" % x["recommended_experiment"], ""]
    lines += ["Predicted opportunity compares the logged winner trajectory with the unapplied shadow target trajectory; it is not empirical proof of a time gain. Applied commands were unchanged.", ""]
    return "\n".join(lines)


def analyze(root: Path, results_dir: str = "experiment_results") -> Dict[str, Any]:
    results_root = root / results_dir
    profile_result = read_json(results_root / "analysis/velocity_profile_v2/latest.json")
    profile = profile_result["distance_profile"]
    lap_length = float(profile[-1]["s_m"] + profile[-1]["ds_m"])
    eligible, rejected, events, traversals, tick_comparisons = [], [], [], set(), []
    for item in _ledger(results_root / "ledger.jsonl"):
        if item.get("outcome") not in ("finished", "collision") or not item.get("telemetry_path"):
            continue
        path = (root / item["telemetry_path"]).parent / "ticks.csv"
        if not path.exists():
            continue
        with path.open(encoding="utf-8", newline="") as infile:
            rows = list(csv.DictReader(infile))
        populated = [x for x in rows if x.get("shadow_available") == "True" and x.get("shadow_profile_source_sha256") == PROFILE_SHA]
        if not populated:
            continue
        if len(populated) != len(rows):
            rejected.append({"attempt_id": item["attempt_id"], "reason": "partial shadow telemetry"}); continue
        eligible.append(item["attempt_id"])
        for row in rows:
            flag, kinds, magnitude = classify_tick(row)
            profile_index = int(row["shadow_profile_index"])
            context = _profile_context(profile, profile_index)
            tick_comparisons.append({
                "attempt_id": item["attempt_id"], "lap": row["lap"], "tick": row["tick"],
                "sim_time_seconds": row["sim_time_seconds"], "custom_waypoint_index": row["custom_waypoint_index"],
                "official_waypoint_index": row["official_waypoint_index"], "distance_s_m": row["shadow_profile_s_m"],
                "section": row["section"], "speed_kmh": row["speed_kmh"],
                "winner_target_speed_kmh": row["controller_recommended_speed_kmh"], "shadow_target_speed_kmh": row["shadow_target_speed_kmh"],
                "winner_throttle": row["throttle"], "shadow_throttle": row["shadow_throttle"],
                "winner_brake": row["brake"], "shadow_brake": row["shadow_brake"],
                "winner_phase": row["controller_decision_branch"], "shadow_phase": row["shadow_phase"],
                "steering": row["steer"], "curvature_1pm": context["curvature_1pm"],
                "winner_projected_required_decel_mps2": _winner_required_decel(row),
                "shadow_projected_required_decel_mps2": max(0.0, -_number(row, "shadow_required_acceleration_mps2")),
                "profile_alignment_error_m": row["shadow_profile_distance_error_m"],
                "meaningful_disagreement": flag, "disagreement_types": ";".join(kinds), "disagreement_magnitude": magnitude,
            })
        by_lap: Dict[int, List[Dict[str, str]]] = {}
        for row in rows:
            by_lap.setdefault(int(row["lap"]), []).append(row)
        for lap, lap_rows in by_lap.items():
            traversals.add((item["attempt_id"], lap))
            flags = [classify_tick(row)[0] for row in lap_rows]
            for start, end in _local_groups(lap_rows, flags):
                events.append(_event(lap_rows, start, end, profile, item["attempt_id"], lap))
    if not eligible:
        raise ValueError("No complete, profile-compatible H1 shadow telemetry was found")
    clusters = cluster_events(events, len(traversals), lap_length)
    promising = sorted([x for x in clusters if x["favorable_local_intervention"]], key=lambda x: (x["promising_score"], x["repeatability"]), reverse=True)
    diagnostics = sorted(clusters, key=lambda x: (max(0.0, x["predicted_time_opportunity_200m_s"]) * x["repeatability"] / x["risk_score"]), reverse=True)
    largest = sorted(clusters, key=lambda x: x["magnitude_score"], reverse=True)
    result = {
        "schema_version": SCHEMA_VERSION, "event_type": "shadow_longitudinal_disagreement_analysis",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "method": {"thresholds": {"brake_active": BRAKE_ACTIVE, "brake_magnitude_delta": BRAKE_DELTA, "target_speed_delta_kmh": TARGET_DELTA_KMH, "required_deceleration_delta_mps2": DECEL_DELTA}, "event_gap_tolerance_ticks": 1, "spatial_cluster_radius_m": 35.0, "downstream_horizons_m": [25, 50, 100, 200]},
        "population": {"eligible_attempts": eligible, "eligible_attempt_count": len(eligible), "eligible_traversals": len(traversals), "tick_events": len(events), "spatial_events": len(clusters), "rejected_attempts": rejected, "profile_sha256": PROFILE_SHA},
        "interpretation": "This is shadow-only, one-step counterfactual screening. Winner commands remained applied. A speed hypothesis is admitted to the promising ranking only when shadow also reduces local integrated braking; target-profile gains alone are not treated as executable closed-loop evidence.",
        "rankings": {"most_promising": promising, "candidate_diagnostics": diagnostics, "largest_disagreements": largest},
    }
    output = results_root / "analysis/shadow_disagreement"
    result_path, report_path, ticks_path = output / "latest.json", output / "latest.md", output / "tick_comparisons.csv"
    output.mkdir(parents=True, exist_ok=True)
    with ticks_path.open("w", encoding="utf-8", newline="") as outfile:
        writer = csv.DictWriter(outfile, fieldnames=list(tick_comparisons[0]))
        writer.writeheader(); writer.writerows(tick_comparisons)
    result["result_path"] = str(result_path.relative_to(root)); result["human_report_path"] = str(report_path.relative_to(root))
    result["tick_comparisons_path"] = str(ticks_path.relative_to(root))
    write_json(result_path, result)
    report_path.parent.mkdir(parents=True, exist_ok=True); report_path.write_text(_report(result), encoding="utf-8")
    return result
