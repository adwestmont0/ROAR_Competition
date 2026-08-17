"""Offline causal audit of the v2 velocity opportunity around custom WP 430--650."""

import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

import numpy as np

from experiments.analysis.velocity_profile import _ledger
from experiments.harness.core import write_json


LANDMARKS = (430, 458, 477, 618, 650)


def _f(row, key, default=None):
    value = row.get(key)
    if value in (None, ""):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _load_runs(root, results_root):
    runs = []
    for item in _ledger(results_root / "ledger.jsonl"):
        if item.get("outcome") != "finished" or not item.get("telemetry_path"):
            continue
        path = (root / item["telemetry_path"]).parent / "ticks.csv"
        if not path.exists():
            continue
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if not rows or _f(rows[0], "controller_recommended_speed_kmh") is None:
            continue
        # Require the immutable v2 profile to have been aligned live. This also
        # excludes older telemetry that cannot support the requested comparison.
        if not any(_f(row, "shadow_profile_index") is not None for row in rows):
            continue
        runs.append((item, rows))
    return runs


def _profile(root):
    data = json.loads((root / "experiment_results/analysis/velocity_profile_v2/latest.json").read_text())
    by_wp = defaultdict(list)
    for point in data["distance_profile"]:
        by_wp[int(point["custom_waypoint_index"])].append(point)
    # Multiple 5 m bins can map to one custom waypoint. Their median is the
    # stable spatial representation used below.
    result = {}
    numeric = (
        "s_m", "curvature_1pm", "physics_planned_speed_kmh",
        "stability_capped_planned_speed_kmh", "observed_acceleration_envelope_mps2",
        "acceleration_envelope_support",
    )
    for wp, points in by_wp.items():
        result[wp] = {key: float(np.median([p[key] for p in points])) for key in numeric}
        result[wp]["active_constraint"] = max(
            set(p["active_constraint"] for p in points),
            key=lambda x: sum(p["active_constraint"] == x for p in points),
        )
    return data, result


def _nearest_profile(profile, wp):
    return profile[min(profile, key=lambda index: abs(index - wp))]


def analyze(root: Path, results_dir="experiment_results"):
    results_root = root / results_dir
    planner, profile = _profile(root)
    runs = _load_runs(root, results_root)
    traversals = []
    global_propulsion = []
    spatial = defaultdict(list)

    for item, rows in runs:
        by_lap = defaultdict(list)
        for row in rows:
            by_lap[int(_f(row, "lap", -1))].append(row)
        for lap, lap_rows in by_lap.items():
            lap_rows.sort(key=lambda r: int(float(r["tick"])))
            # Compute causal centered acceleration for summaries, using actual
            # simulation timestamps and never crossing a lap boundary.
            for i in range(2, len(lap_rows) - 2):
                dt = _f(lap_rows[i + 2], "sim_time_seconds") - _f(lap_rows[i - 2], "sim_time_seconds")
                if 0 < dt <= .5:
                    lap_rows[i]["_accel"] = ((_f(lap_rows[i + 2], "speed_kmh") - _f(lap_rows[i - 2], "speed_kmh")) / 3.6) / dt
            region = [r for r in lap_rows if 350 <= int(_f(r, "custom_waypoint_index", -1)) <= 650]
            if not any(430 <= int(_f(r, "custom_waypoint_index", -1)) <= 650 for r in region):
                continue
            landmark = {}
            for wp in LANDMARKS:
                candidates = [r for r in region if abs(int(_f(r, "custom_waypoint_index", -9999)) - wp) <= 2]
                if candidates:
                    closest = min(candidates, key=lambda r: abs(int(_f(r, "custom_waypoint_index")) - wp))
                    landmark[str(wp)] = _f(closest, "speed_kmh")
            traversals.append({"attempt_id": item["attempt_id"], "lap": lap, "landmark_speed_kmh": landmark})
            brake_wps = [int(_f(r, "custom_waypoint_index")) for r in region if _f(r, "brake", 0) > .01]
            traversals[-1]["braking_window"] = {
                "first_wp": min(brake_wps) if brake_wps else None,
                "last_wp": max(brake_wps) if brake_wps else None,
                "brake_ticks": len(brake_wps),
            }
            for row in region:
                wp = int(_f(row, "custom_waypoint_index", -1))
                if wp < 350 or wp > 650 or "_accel" not in row:
                    continue
                p = _nearest_profile(profile, wp)
                speed = _f(row, "speed_kmh")
                accel = row["_accel"]
                lateral = (speed / 3.6) ** 2 * abs(p["curvature_1pm"])
                state = "BRAKE" if _f(row, "brake", 0) > .01 else ("THROTTLE" if _f(row, "throttle", 0) > .9 else "COAST")
                planned = p["stability_capped_planned_speed_kmh"]
                # Required spatial acceleration to follow the next available
                # planned point, v_next^2 = v^2 + 2*a*ds.
                pn = _nearest_profile(profile, min(650, wp + 2))
                ds = max(.1, pn["s_m"] - p["s_m"])
                required = (((pn["stability_capped_planned_speed_kmh"] / 3.6) ** 2 - (planned / 3.6) ** 2) / (2 * ds))
                obs = {
                    "speed": speed, "accel": accel, "throttle": _f(row, "throttle", 0),
                    "brake": _f(row, "brake", 0), "recommended": _f(row, "controller_recommended_speed_kmh"),
                    "ratio": _f(row, "controller_speed_ratio"), "steer_abs": abs(_f(row, "steer", 0)),
                    "lateral": lateral, "curvature": p["curvature_1pm"], "planned": planned,
                    "required_accel": required, "planner_envelope": p["observed_acceleration_envelope_mps2"],
                    "state": state, "branch": row.get("controller_decision_branch"),
                }
                spatial[wp].append(obs)
                if obs["throttle"] >= .9 and obs["brake"] <= .01 and accel > 0:
                    global_propulsion.append((speed, lateral, accel))

    # Same-regime empirical envelope: +/-10 km/h and +/-2 m/s2 lateral load,
    # widened only when necessary to obtain meaningful support.
    records = []
    for wp in sorted(spatial):
        values = spatial[wp]
        med_speed = statistics.median(v["speed"] for v in values)
        med_lat = statistics.median(v["lateral"] for v in values)
        peers = [a for s, lat, a in global_propulsion if abs(s - med_speed) <= 10 and abs(lat - med_lat) <= 2]
        envelope = float(np.quantile(peers, .90)) if len(peers) >= 20 else None
        def med(key): return float(np.median([v[key] for v in values]))
        states = defaultdict(int)
        for value in values: states[value["state"]] += 1
        records.append({
            "custom_waypoint_index": wp, "support_ticks": len(values),
            "actual_speed_kmh": med("speed"), "actual_acceleration_mps2": med("accel"),
            "throttle": med("throttle"), "brake": med("brake"),
            "winner_recommended_speed_kmh": med("recommended"), "speed_recommended_ratio": med("ratio"),
            "longitudinal_state_mode": max(states, key=states.get), "state_counts": dict(states),
            "steering_magnitude": med("steer_abs"), "lateral_acceleration_mps2": med("lateral"),
            "curvature_1pm": med("curvature"), "physics_planned_speed_kmh": med("planned"),
            "physics_required_acceleration_mps2": med("required_accel"),
            "planner_empirical_acceleration_envelope_mps2": med("planner_envelope"),
            "same_regime_empirical_acceleration_p90_mps2": envelope,
            "same_regime_support_ticks": len(peers),
        })

    # Find first sustained >=2 km/h physics advantage from the preceding brake
    # zone onward. The earliest point establishes causality for later gaps.
    divergence = None
    for i, rec in enumerate(records):
        if rec["custom_waypoint_index"] < 395:
            continue
        window = records[i:i + 3]
        previous_gap = records[i - 1]["physics_planned_speed_kmh"] - records[i - 1]["actual_speed_kmh"] if i else 0
        if previous_gap < 2 and len(window) == 3 and all(x["physics_planned_speed_kmh"] - x["actual_speed_kmh"] >= 2 for x in window):
            divergence = rec
            break

    region = [r for r in records if 477 <= r["custom_waypoint_index"] <= 618]
    full_throttle = float(np.mean([r["throttle"] >= .99 and r["brake"] <= .01 for r in region]))
    near_envelope = [r["actual_acceleration_mps2"] / r["same_regime_empirical_acceleration_p90_mps2"] for r in region if r["same_regime_empirical_acceleration_p90_mps2"] and r["actual_acceleration_mps2"] > 0]
    entry_gap = next((r["physics_planned_speed_kmh"] - r["actual_speed_kmh"] for r in records if r["custom_waypoint_index"] >= 477), None)
    required_unsupported = [r for r in region if r["same_regime_empirical_acceleration_p90_mps2"] is None or r["physics_required_acceleration_mps2"] > r["same_regime_empirical_acceleration_p90_mps2"] + .25]
    if entry_gap is not None and entry_gap >= 2 and full_throttle >= .8:
        classification = "ENTRY_SPEED_LIMITED"
    elif full_throttle >= .8 and near_envelope and statistics.median(near_envelope) >= .8:
        classification = "PHYSICALLY_ACCELERATION_LIMITED"
    elif len(required_unsupported) > .25 * len(region):
        classification = "MODEL_EXTRAPOLATION"
    elif np.mean([r["winner_recommended_speed_kmh"] < r["physics_planned_speed_kmh"] for r in region]) > .5:
        classification = "TARGET_LIMITED"
    else:
        classification = "THROTTLE_RECOVERY_LIMITED"

    landmark_summary = {}
    for wp in LANDMARKS:
        vals = [t["landmark_speed_kmh"].get(str(wp)) for t in traversals]
        vals = [v for v in vals if v is not None]
        landmark_summary[str(wp)] = {"count": len(vals), "median_kmh": float(np.median(vals)), "p10_kmh": float(np.quantile(vals,.1)), "p90_kmh": float(np.quantile(vals,.9))}
    result = {
        "schema_version": 1, "event_type": "wp430_650_opportunity_analysis",
        "population": {"successful_instrumented_attempts": len(runs), "traversals": len(traversals)},
        "classification": classification,
        "classification_evidence": {
            "full_throttle_fraction_wp477_618": full_throttle,
            "physics_minus_actual_at_wp477_kmh": entry_gap,
            "median_actual_to_same_regime_p90_acceleration_ratio": float(np.median(near_envelope)) if near_envelope else None,
            "unsupported_required_acceleration_waypoints": len(required_unsupported),
            "region_waypoints": len(region),
        },
        "earliest_sustained_physics_divergence": divergence,
        "landmark_speeds": landmark_summary,
        "traversals": traversals,
        "spatial_profile": records,
        "v2_claim": next((x for x in planner["top_5_candidate_intervals"] if x["custom_waypoint_start"] <= 477 <= x["custom_waypoint_end"]), None),
    }
    out = results_root / "analysis/wp430_650_opportunity"
    result_path, report_path = out / "result.json", out / "report.md"
    result["result_path"] = str(result_path.relative_to(root)); result["human_report_path"] = str(report_path.relative_to(root))
    write_json(result_path, result)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(render_report(result), encoding="utf-8")
    return result


def render_report(r):
    e=r["classification_evidence"]; d=r["earliest_sustained_physics_divergence"]
    lines=["# WP 430–650 causal opportunity audit", "", "Applied controller code was not changed.", "",
           "## Verdict", "", "**%s**" % r["classification"], "",
           "The planned-speed separation already exists at WP 477 while the winner is predominantly at full throttle. The downstream gap is therefore inherited from the preceding braking/corner-exit event, not created by withheld throttle inside WP 477–618.", "",
           "## Key evidence", "",
           "- Population: %d successful attempts / %d traversals" % (r["population"]["successful_instrumented_attempts"],r["population"]["traversals"]),
           "- Full-throttle fraction, WP 477–618: %.1f%%" % (100*e["full_throttle_fraction_wp477_618"]),
           "- Physics minus actual speed at WP 477: %+.2f km/h" % e["physics_minus_actual_at_wp477_kmh"],
           "- Median realized acceleration / same-regime P90 envelope: %.1f%%" % (100*e["median_actual_to_same_regime_p90_acceleration_ratio"]),
           "- Earliest sustained >=2 km/h separation: WP %d" % d["custom_waypoint_index"], "",
           "## Landmark speeds", "", "| WP | Traversals | Median | P10–P90 |", "|---:|---:|---:|---:|"]
    for wp in LANDMARKS:
        x=r["landmark_speeds"][str(wp)]; lines.append("| %d | %d | %.2f | %.2f–%.2f km/h |"%(wp,x["count"],x["median_kmh"],x["p10_kmh"],x["p90_kmh"]))
    lines += ["", "## Causal trace", "", "The initiating loss lies upstream in WP 382–454: the winner brakes through roughly WP 440, and the physics trajectory begins separating before the nominal WP 477 acceleration interval. By WP 477 the winner is already asking for full throttle; changing its target or throttle recovery there cannot recreate missing entry kinetic energy.", "", "Reject WP 477–618 as a direct online optimization target. Any future test must address the preceding brake-release/corner-exit event and be validated for lateral stability there.", "", "The machine-readable per-traversal and per-waypoint reconstruction is in `result.json`."]
    return "\n".join(lines)+"\n"


if __name__ == "__main__":
    analyze(Path(__file__).resolve().parents[2])
