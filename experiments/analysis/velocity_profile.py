import csv
import datetime as dt
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

from experiments.harness.core import read_json, write_json


SCHEMA_VERSION = 1


def _ledger(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    by_attempt = {}
    with path.open(encoding="utf-8") as infile:
        for line in infile:
            item = json.loads(line)
            if item.get("attempt_id") and item.get("outcome"):
                by_attempt[item["attempt_id"]] = item
    return list(by_attempt.values())


def _verified_baseline(root: Path, item: Dict[str, Any]) -> bool:
    if item.get("outcome") != "finished" or not item.get("telemetry_path"):
        return False
    diff = item.get("source_diff_path")
    return bool(diff) and (root / diff).exists() and not (root / diff).read_text(encoding="utf-8").strip()


def _ticks(root: Path, item: Dict[str, Any]) -> List[Dict[str, str]]:
    path = (root / item["telemetry_path"]).parent / "ticks.csv"
    with path.open(encoding="utf-8", newline="") as infile:
        return list(csv.DictReader(infile))


def _line(root: Path) -> Tuple[np.ndarray, np.ndarray, float]:
    points = np.load(str(root / "competition_code/waypoints/waypointsPrimary.npz"))["locations"][35:, :2]
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    closing = float(np.linalg.norm(points[0] - points[-1]))
    arc = np.r_[0.0, np.cumsum(segment)]
    return points, arc, float(arc[-1] + closing)


def _quantile(values: List[float], q: float) -> float:
    return float(np.quantile(np.asarray(values, dtype=float), q))


def analyze(root: Path, results_dir: str = "experiment_results", bin_m: float = 20.0, interval_m: float = 100.0) -> Dict[str, Any]:
    results_root = root / results_dir
    attempts = _ledger(results_root / "ledger.jsonl")
    successes = [x for x in attempts if _verified_baseline(root, x)]
    failures = [x for x in attempts if x.get("outcome") == "collision" and x.get("telemetry_path")]
    points, arc, lap_length = _line(root)
    bins = int(math.ceil(lap_length / bin_m))
    success_by_bin: Dict[int, List[Tuple[str, float, float, float]]] = defaultdict(list)
    failure_by_bin: Dict[int, List[Tuple[str, float]]] = defaultdict(list)

    for item in successes:
        per_bin: Dict[int, List[Tuple[float, float, float]]] = defaultdict(list)
        for row in _ticks(root, item):
            index = int(row["custom_waypoint_index"]) % len(points)
            bucket = min(bins - 1, int(arc[index] / bin_m))
            per_bin[bucket].append((float(row["speed_kmh"]), float(row["throttle"]), float(row["brake"])))
        for bucket, values in per_bin.items():
            array = np.asarray(values)
            success_by_bin[bucket].append((item["attempt_id"], float(array[:, 0].mean()), float(array[:, 1].mean()), float((array[:, 2] > 0).mean())))

    for item in failures:
        rows = _ticks(root, item)
        terminal = rows[-160:]  # about 8 seconds / typically 300–400 m of boundary evidence
        per_bin: Dict[int, List[float]] = defaultdict(list)
        for row in terminal:
            index = int(row["custom_waypoint_index"]) % len(points)
            per_bin[min(bins - 1, int(arc[index] / bin_m))].append(float(row["speed_kmh"]))
        for bucket, values in per_bin.items():
            failure_by_bin[bucket].append((item["attempt_id"], float(np.mean(values))))

    profiles = []
    for bucket in range(bins):
        samples = success_by_bin.get(bucket, [])
        if not samples:
            continue
        speeds = [x[1] for x in samples]
        actual = _quantile(speeds, 0.5)
        fastest_reliable = _quantile(speeds, 0.75)
        boundary = [x[1] for x in failure_by_bin.get(bucket, [])]
        feasible = fastest_reliable
        stability_limited = False
        if boundary:
            lower_failure = _quantile(boundary, 0.25)
            if lower_failure <= fastest_reliable + 2.0:
                feasible = min(feasible, max(actual, lower_failure - 1.0))
                stability_limited = True
        ds = min(bin_m, lap_length - bucket * bin_m)
        opportunity = max(0.0, 3.6 * ds * (1.0 / actual - 1.0 / feasible)) if feasible > actual else 0.0
        profiles.append({
            "bin": bucket, "start_m": bucket * bin_m, "end_m": min(lap_length, (bucket + 1) * bin_m),
            "custom_waypoint_start": int(np.searchsorted(arc, bucket * bin_m, side="left")),
            "successful_runs": len(samples), "actual_speed_kmh": actual,
            "feasible_speed_kmh": feasible, "successful_speed_q25_kmh": _quantile(speeds, .25),
            "successful_speed_q75_kmh": fastest_reliable,
            "failure_boundary_count": len(boundary),
            "failure_speed_q25_kmh": _quantile(boundary, .25) if boundary else None,
            "estimated_opportunity_seconds": opportunity,
            "mean_throttle": float(np.mean([x[2] for x in samples])),
            "braking_run_fraction": float(np.mean([x[3] > .05 for x in samples])),
            "stability_limited": stability_limited,
        })

    group_bins = max(1, int(round(interval_m / bin_m)))
    intervals = []
    for start in range(0, bins, group_bins):
        members = [x for x in profiles if start <= x["bin"] < start + group_bins]
        if not members:
            continue
        opportunity = sum(x["estimated_opportunity_seconds"] for x in members)
        failures_here = sum(x["failure_boundary_count"] for x in members)
        min_runs = min(x["successful_runs"] for x in members)
        spread = float(np.mean([x["successful_speed_q75_kmh"] - x["successful_speed_q25_kmh"] for x in members]))
        stable = any(x["stability_limited"] for x in members)
        brake = float(np.mean([x["braking_run_fraction"] for x in members]))
        throttle = float(np.mean([x["mean_throttle"] for x in members]))
        if stable:
            limitation = "stability constrained"
        elif brake > .35:
            limitation = "brake timing/release"
        elif throttle > .75:
            limitation = "acceleration/throttle recovery"
        else:
            limitation = "corner speed or transient balance"
        confidence = "high" if min_runs >= 20 and spread <= 2.0 else ("medium" if min_runs >= 10 else "low")
        intervals.append({
            "start_m": start * bin_m, "end_m": min(lap_length, (start + group_bins) * bin_m),
            "custom_waypoint_start": members[0]["custom_waypoint_start"],
            "custom_waypoint_end": members[-1]["custom_waypoint_start"],
            "estimated_opportunity_seconds": opportunity, "confidence": confidence,
            "limitation": limitation, "failure_boundary_samples": failures_here,
            "minimum_successful_runs": min_runs,
        })
    intervals.sort(key=lambda x: x["estimated_opportunity_seconds"], reverse=True)
    result = {
        "schema_version": SCHEMA_VERSION, "event_type": "offline_velocity_profile",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "method": {
            "actual_speed": "median across provenance-verified successful baseline runs",
            "feasible_speed": "successful-run 75th percentile, constrained by the lower quartile of nearby collision speeds minus 1 km/h",
            "opportunity_formula": "3.6 * ds * (1/v_actual_kmh - 1/v_feasible_kmh)",
            "failure_boundary_window": "final 160 telemetry ticks before each collision",
            "bin_m": bin_m, "interval_m": interval_m,
        },
        "population": {"successful_baseline_runs": len(successes), "collision_runs": len(failures)},
        "lap_length_m": lap_length, "ranked_intervals": intervals, "distance_profile": profiles,
    }
    output_dir = results_root / "analysis" / "velocity_profile"
    result_path, report_path = output_dir / "latest.json", output_dir / "latest.md"
    result["result_path"] = str(result_path.relative_to(root)); result["human_report_path"] = str(report_path.relative_to(root))
    write_json(result_path, result)
    lines = ["# Offline Monza velocity-profile opportunity ranking", "", "Population: %d successful baseline runs; %d collision runs." % (len(successes), len(failures)), "", "| Rank | Distance | Custom WP | Opportunity | Confidence | Limitation |", "|---:|---:|---:|---:|---|---|"]
    for rank, item in enumerate(intervals[:20], 1):
        lines.append("| %d | %.0f–%.0f m | %d–%d | %.3f s | %s | %s |" % (rank, item["start_m"], item["end_m"], item["custom_waypoint_start"], item["custom_waypoint_end"], item["estimated_opportunity_seconds"], item["confidence"], item["limitation"]))
    lines += ["", "The opportunity estimate is a screening statistic, not a claim that all gains can be combined. Collision-adjacent intervals are conservatively capped and should be treated as constraints, not optimization targets.", ""]
    report_path.parent.mkdir(parents=True, exist_ok=True); report_path.write_text("\n".join(lines), encoding="utf-8")
    return result
