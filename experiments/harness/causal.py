import datetime as dt
import csv
import hashlib
import json
import statistics
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from .core import (
    append_jsonl, dry_run, execute_with_recovery, git_output, load_ledger, read_json,
    run_recovery, write_json,
)
from .evaluation import CONTROLLER_OUTCOMES, evaluation_lock, summarize_measurements


SCHEMA_VERSION = 2


def causal_schedule(repetitions: int, include_controls: bool = True) -> List[str]:
    if repetitions < 1:
        raise ValueError("causal repetitions must be positive")
    if include_controls:
        return [cohort for _ in range(repetitions) for cohort in ("control", "restricted")]
    return ["restricted"] * repetitions


def _key(config: Dict[str, Any], commit: str) -> str:
    causal = config["causal"]
    value = {
        "schema_version": SCHEMA_VERSION,
        "baseline_commit": commit,
        "repetitions": int(causal.get("repetitions", 5)),
        "control_parameters": causal.get("control_parameters", {}),
        "candidate_parameters": causal["candidate_parameters"],
        "include_controls": bool(causal.get("include_controls", True)),
        "schedule": "forced_clean_control_restricted_interleaved",
    }
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()[:20]


def _terminal(attempts: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    return next((x for x in reversed(attempts) if x.get("outcome") in CONTROLLER_OUTCOMES), None)


def dry_run_causal(config: Dict[str, Any], registry: Dict[str, Any], root: Path) -> Dict[str, Any]:
    causal = config["causal"]
    output = {}
    for cohort, parameters in (
        ("control", causal.get("control_parameters", {})),
        ("restricted", causal["candidate_parameters"]),
    ):
        concrete = dict(config)
        concrete["parameters"] = dict(parameters)
        output[cohort] = dry_run(concrete, registry, root)[0]
    repetitions = int(causal.get("repetitions", 5))
    schedule = causal_schedule(repetitions, bool(causal.get("include_controls", True)))
    return {"schedule": schedule, "total_scheduled_attempts": len(schedule), "cohorts": output}


def _write_report(path: Path, result: Dict[str, Any]) -> None:
    control = result["cohorts"]["control"]
    restricted = result["cohorts"]["restricted"]
    title = result.get("experiment_name", "causal campaign").replace("-", " ").title().replace("Wp", "WP")
    lines = [
        "# %s" % title, "",
        "Every measurement used a forced-clean CARLA start.", "",
    ]
    for label, summary in (("Baseline control", control), ("Treatment", restricted)):
        elapsed = summary.get("finished_elapsed_seconds")
        lines += [
            "## " + label, "",
            "- Completion rate: %s" % ("unavailable" if summary["completion_rate"] is None else "%.1f%%" % (100 * summary["completion_rate"])),
            "- Collision rate: %s" % ("unavailable" if summary["collision_rate"] is None else "%.1f%%" % (100 * summary["collision_rate"])),
            "- Finished mean: %s" % ("unavailable" if elapsed is None else "%.3f s" % elapsed["mean"]), "",
        ]
    lines += ["This campaign tests causal stability, not lap-time acceptance. See result.json for the authoritative record.", ""]
    focused = result.get("focused_event_analysis")
    arrival = result.get("arrival_analysis")
    if arrival and not focused:
        baseline = arrival["baseline"]
        restricted_arrival = arrival["restricted"]
        lines += [
            "## Causal intermediate variable", "",
            "- Arrival window: custom WP %d–%d" % tuple(arrival["custom_waypoint_window"]),
            "- Baseline traversals: %d; mean arrival speed: %s" % (baseline["traversals"], "unavailable" if baseline["mean_speed_kmh"] is None else "%.3f km/h" % baseline["mean_speed_kmh"]),
            "- Restricted traversals: %d; mean arrival speed: %s" % (restricted_arrival["traversals"], "unavailable" if restricted_arrival["mean_speed_kmh"] is None else "%.3f km/h" % restricted_arrival["mean_speed_kmh"]),
            "- Mean shift: %s" % ("unavailable" if arrival["restricted_minus_baseline_mean_kmh"] is None else "%+.3f km/h" % arrival["restricted_minus_baseline_mean_kmh"]),
            "- Restricted collisions in WP 1380–1420: %d" % arrival["restricted_zone_collisions"], "",
        ]
    if focused:
        control = focused["cohorts"]["control"]["aggregate"]
        treatment = focused["cohorts"]["restricted"]["aggregate"]
        adjusted = focused["restricted_minus_control"]
        labels = (
            ("Entry speed (km/h)", "entry_speed_kmh"),
            ("Exit speed (km/h)", "exit_speed_kmh"),
            ("Minimum speed (km/h)", "minimum_speed_kmh"),
            ("Brake onset (WP)", "braking_onset_waypoint"),
            ("Brake release (WP)", "braking_release_waypoint"),
            ("Brake ticks", "total_brake_ticks"),
            ("Throttle reapplied (WP)", "throttle_reapplication_waypoint"),
            ("Section time (s)", "section_time_seconds"),
            ("Total race time (s)", "total_race_time_seconds"),
            ("Collision rate", "collision_rate"),
        )
        lines += [
            "## Focused braking-event measurements", "",
            "Aggregation status: **%s**." % focused.get("status", "complete").upper(),
            "Missing cohorts: %s." % (", ".join(focused.get("missing_cohorts", [])) or "none"), "",
            "Incomplete cohorts: %s." % (", ".join(focused.get("incomplete_cohorts", [])) or "none"), "",
            "| Metric | Control | Treatment | Treatment − control |", "|---|---:|---:|---:|",
        ]
        def display(value: Optional[float]) -> str:
            return "unavailable" if value is None else "%.3f" % value
        for label, field in labels:
            lines.append("| %s | %s | %s | %s |" % (
                label, display(control[field]), display(treatment[field]), display(adjusted[field]),
            ))
        for waypoint in focused["specification"]["downstream_waypoints"]:
            key = str(waypoint)
            lines.append("| Speed at WP %d (km/h) | %s | %s | %s |" % (
                waypoint, display(control["downstream_speed_kmh"][key]),
                display(treatment["downstream_speed_kmh"][key]),
                display(adjusted["downstream_speed_kmh"][key]),
            ))
        lines += [
            "", "- Actual total-race gain: %s" % display(focused["actual_total_race_gain_seconds"]),
            "- Offline prediction at observed modest exit-speed delta: %s" % display(focused["offline_predicted_gain_for_observed_exit_delta_seconds"]),
            "- Actual minus offline prediction: %s" % display(focused["actual_minus_offline_predicted_gain_seconds"]), "",
        ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def _arrival_values(root: Path, measurement: Dict[str, Any], low: int = 1402, high: int = 1409) -> List[float]:
    telemetry = measurement.get("telemetry_path")
    if not telemetry:
        return []
    ticks = (root / telemetry).parent / "ticks.csv"
    by_lap: Dict[str, List[float]] = {}
    with ticks.open(encoding="utf-8", newline="") as infile:
        for row in csv.DictReader(infile):
            if low <= int(row["custom_waypoint_index"]) <= high:
                by_lap.setdefault(row["lap"], []).append(float(row["speed_kmh"]))
    return [statistics.mean(values) for values in by_lap.values() if values]


def _arrival_analysis(root: Path, results_root: Path, restricted: List[Optional[Dict[str, Any]]]) -> Dict[str, Any]:
    ledger = load_ledger(results_root / "ledger.jsonl")
    attempts = {item["attempt_id"]: item for item in ledger if item.get("attempt_id") and item.get("outcome")}
    baseline_values: List[float] = []
    baseline_runs = 0
    for item in attempts.values():
        diff_value = item.get("source_diff_path")
        diff = root / diff_value if diff_value else None
        if item.get("outcome") == "finished" and diff is not None and diff.exists() and not diff.read_text(encoding="utf-8").strip():
            baseline_runs += 1
            baseline_values.extend(_arrival_values(root, item))
    usable = [item for item in restricted if item is not None]
    restricted_values = [value for item in usable for value in _arrival_values(root, item)]
    def summary(values: List[float], runs: int) -> Dict[str, Any]:
        return {
            "runs": runs, "traversals": len(values),
            "mean_speed_kmh": statistics.mean(values) if values else None,
            "median_speed_kmh": statistics.median(values) if values else None,
            "minimum_speed_kmh": min(values) if values else None,
            "maximum_speed_kmh": max(values) if values else None,
        }
    baseline = summary(baseline_values, baseline_runs)
    treatment = summary(restricted_values, len(usable))
    shift = None
    if treatment["mean_speed_kmh"] is not None and baseline["mean_speed_kmh"] is not None:
        shift = treatment["mean_speed_kmh"] - baseline["mean_speed_kmh"]
    return {
        "custom_waypoint_window": [1402, 1409], "baseline": baseline, "restricted": treatment,
        "restricted_minus_baseline_mean_kmh": shift,
        "restricted_zone_collisions": sum(
            item.get("outcome") == "collision" and 1380 <= int(item.get("custom_waypoint_index") or -1) <= 1420
            for item in usable
        ),
    }


def _mean_or_none(values: List[float]) -> Optional[float]:
    return statistics.mean(values) if values else None


def _focused_traversals(
    root: Path, measurement: Dict[str, Any], specification: Dict[str, Any],
) -> List[Dict[str, Any]]:
    telemetry = measurement.get("telemetry_path")
    if not telemetry:
        return []
    ticks_path = (root / telemetry).parent / "ticks.csv"
    if not ticks_path.exists():
        return []
    rows = list(csv.DictReader(ticks_path.open(encoding="utf-8", newline="")))
    intervention_low, intervention_high = map(int, specification["intervention_window"])
    minimum_low, minimum_high = map(int, specification["minimum_speed_window"])
    tolerance = int(specification.get("downstream_tolerance_waypoints", 2))
    downstream = [int(value) for value in specification["downstream_waypoints"]]
    traversals = []
    for lap in sorted({int(row["lap"]) for row in rows}):
        lap_rows = [row for row in rows if int(row["lap"]) == lap]
        intervention = [
            row for row in lap_rows
            if intervention_low <= int(row["custom_waypoint_index"]) <= intervention_high
        ]
        minimum_window = [
            row for row in lap_rows
            if minimum_low <= int(row["custom_waypoint_index"]) <= minimum_high
        ]
        if not intervention or not minimum_window:
            continue
        braking = [row for row in minimum_window if float(row["brake"]) > .5]
        onset_tick = int(braking[0]["tick"]) if braking else None
        after_onset = [
            row for row in minimum_window
            if onset_tick is not None and int(row["tick"]) > onset_tick
        ]
        release = next((row for row in after_onset if float(row["brake"]) <= .5), None)
        throttle = next(
            (row for row in after_onset if float(row["brake"]) <= .5 and float(row["throttle"]) >= .5),
            None,
        )
        section = int(statistics.mode(int(row["section"]) for row in intervention))
        timestep = float(measurement.get("control_timestep_seconds") or .05)
        speeds = {}
        for waypoint in downstream:
            samples = [
                float(row["speed_kmh"]) for row in lap_rows
                if abs(int(row["custom_waypoint_index"]) - waypoint) <= tolerance
            ]
            speeds[str(waypoint)] = _mean_or_none(samples)
        traversals.append({
            "attempt_id": measurement["attempt_id"], "lap": lap,
            "entry_speed_kmh": float(intervention[0]["speed_kmh"]),
            "exit_speed_kmh": float(intervention[-1]["speed_kmh"]),
            "minimum_speed_kmh": min(float(row["speed_kmh"]) for row in minimum_window),
            "braking_onset_waypoint": None if not braking else int(braking[0]["custom_waypoint_index"]),
            "braking_release_waypoint": None if release is None else int(release["custom_waypoint_index"]),
            "total_brake_ticks": len(braking),
            "throttle_reapplication_waypoint": None if throttle is None else int(throttle["custom_waypoint_index"]),
            "downstream_speed_kmh": speeds, "section": section,
            "section_time_seconds": sum(int(row["section"]) == section for row in lap_rows) * timestep,
        })
    return traversals


def _focused_summary(
    root: Path, records: List[Dict[str, Any]], specification: Dict[str, Any], penalty: float,
) -> Dict[str, Any]:
    measurements = [record.get("measurement") for record in records if record.get("measurement")]
    traversals = [
        traversal for measurement in measurements
        for traversal in _focused_traversals(root, measurement, specification)
    ]
    fields = (
        "entry_speed_kmh", "exit_speed_kmh", "minimum_speed_kmh",
        "braking_onset_waypoint", "braking_release_waypoint", "total_brake_ticks",
        "throttle_reapplication_waypoint", "section_time_seconds",
    )
    aggregate = {
        field: _mean_or_none([float(item[field]) for item in traversals if item[field] is not None])
        for field in fields
    }
    aggregate["downstream_speed_kmh"] = {
        str(waypoint): _mean_or_none([
            item["downstream_speed_kmh"][str(waypoint)] for item in traversals
            if item["downstream_speed_kmh"][str(waypoint)] is not None
        ]) for waypoint in specification["downstream_waypoints"]
    }
    aggregate["total_race_time_seconds"] = _mean_or_none([
        float(item["elapsed_time_seconds"]) for item in measurements if item.get("outcome") == "finished"
    ])
    aggregate["collision_rate"] = (
        sum(item.get("outcome") == "collision" for item in measurements) / len(measurements)
        if measurements else None
    )
    return {
        "attempts": len(measurements), "traversals": len(traversals),
        "aggregate": aggregate, "per_traversal": traversals,
        "outcomes": summarize_measurements(measurements, penalty, root),
    }


def _focused_event_analysis(
    root: Path, records: List[Dict[str, Any]], specification: Dict[str, Any], penalty: float,
    expected_measurements_per_cohort: int = 1,
) -> Dict[str, Any]:
    cohorts = {
        cohort: _focused_summary(
            root, [record for record in records if record["cohort"] == cohort],
            specification, penalty,
        ) for cohort in ("control", "restricted")
    }
    return _compare_focused_cohorts(
        cohorts, specification, expected_measurements_per_cohort,
    )


def _compare_focused_cohorts(
    cohorts: Dict[str, Dict[str, Any]], specification: Dict[str, Any],
    expected_measurements_per_cohort: int = 1,
) -> Dict[str, Any]:
    control_summary, treatment_summary = cohorts["control"], cohorts["restricted"]
    control, treatment = control_summary["aggregate"], treatment_summary["aggregate"]
    availability = {
        "control": int(control_summary.get("attempts", 0)) > 0,
        "restricted": int(treatment_summary.get("attempts", 0)) > 0,
    }
    completeness = {
        "control": int(control_summary.get("attempts", 0)) >= expected_measurements_per_cohort,
        "restricted": int(treatment_summary.get("attempts", 0)) >= expected_measurements_per_cohort,
    }
    cross_cohort_available = all(completeness.values())
    adjusted = {}
    for field in (
        "entry_speed_kmh", "exit_speed_kmh", "minimum_speed_kmh",
        "braking_onset_waypoint", "braking_release_waypoint", "total_brake_ticks",
        "throttle_reapplication_waypoint", "section_time_seconds", "total_race_time_seconds",
        "collision_rate",
    ):
        adjusted[field] = (
            None if not cross_cohort_available or control[field] is None or treatment[field] is None
            else treatment[field] - control[field]
        )
    adjusted["downstream_speed_kmh"] = {
        waypoint: (
            None if not cross_cohort_available or control["downstream_speed_kmh"][waypoint] is None or treatment["downstream_speed_kmh"][waypoint] is None
            else treatment["downstream_speed_kmh"][waypoint] - control["downstream_speed_kmh"][waypoint]
        ) for waypoint in control["downstream_speed_kmh"]
    }
    actual_gain = None
    if adjusted["total_race_time_seconds"] is not None:
        actual_gain = -adjusted["total_race_time_seconds"]
    modest_delta = adjusted["exit_speed_kmh"]
    offline_gain = None
    if modest_delta is not None:
        offline_gain = (
            float(specification["offline_full_gain_seconds"])
            * modest_delta / float(specification["offline_full_delta_kmh"])
        )
    return {
        "specification": specification, "cohorts": cohorts,
        "status": "complete" if all(completeness.values()) else "incomplete",
        "complete": all(completeness.values()),
        "cohort_measurements_available": availability,
        "cohort_measurements_complete": completeness,
        "missing_cohorts": [name for name, available in availability.items() if not available],
        "incomplete_cohorts": [name for name, complete in completeness.items() if not complete],
        "expected_measurements_per_cohort": expected_measurements_per_cohort,
        "restricted_minus_control": adjusted,
        "actual_total_race_gain_seconds": actual_gain,
        "offline_predicted_gain_for_observed_exit_delta_seconds": offline_gain,
        "actual_minus_offline_predicted_gain_seconds": (
            None if actual_gain is None or offline_gain is None else actual_gain - offline_gain
        ),
        "offline_comparison_note": "Local linearization of the 1.817 s full-envelope opportunity at the observed intervention-exit speed delta.",
    }


def run_causal_campaign(
    config: Dict[str, Any], registry: Dict[str, Any], root: Path, resume: bool = True,
    progress_callback: Optional[Callable[[str], None]] = None,
) -> Dict[str, Any]:
    commit = git_output(root, "rev-parse", "%s^{commit}" % config["baseline_commit"])
    causal = config["causal"]
    schedule = causal_schedule(
        int(causal.get("repetitions", 5)), bool(causal.get("include_controls", True))
    )
    key = _key(config, commit)
    results_root = root / config.get("results_dir", "experiment_results")
    campaign_dir = results_root / "causal" / key
    progress_path, result_path, report_path = (campaign_dir / x for x in ("progress.json", "result.json", "report.md"))
    parameters = {
        "control": dict(causal.get("control_parameters", {})),
        "restricted": dict(causal["candidate_parameters"]),
    }
    with evaluation_lock(results_root):
        if resume and result_path.exists() and read_json(result_path).get("campaign_complete"):
            result = read_json(result_path)
            if "arrival_analysis" not in result and progress_path.exists():
                saved_progress = read_json(progress_path)
                result["arrival_analysis"] = _arrival_analysis(
                    root, results_root,
                    [r.get("measurement") for r in saved_progress["records"] if r["cohort"] == "restricted"],
                )
                write_json(result_path, result)
                _write_report(report_path, result)
            result["resume"] = {"skipped_completed": True}; return result
        progress = read_json(progress_path) if resume and progress_path.exists() else {
            "schema_version": SCHEMA_VERSION, "campaign_key": key, "schedule": schedule,
            "next_schedule_index": 0, "records": [], "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        }
        write_json(progress_path, progress)
        completed_indices = {
            int(record["schedule_index"])
            for record in progress["records"]
            if record.get("measurement") is not None
        }
        for index in range(len(schedule)):
            if index in completed_indices:
                continue
            cohort = schedule[index]
            if progress_callback: progress_callback("[%d/%d] restarting CARLA for %s" % (index + 1, len(schedule), cohort))
            recovery = run_recovery(config["recovery"], config["carla"])
            recovery.update({"experiment_name": config["name"], "campaign_key": key, "reason": "causal_forced_clean", "schedule_index": index})
            recovery_path = campaign_dir / "recoveries" / ("%03d.json" % index)
            write_json(recovery_path, recovery); append_jsonl(results_root / "ledger.jsonl", recovery)
            record = {"schedule_index": index, "cohort": cohort, "recovery_path": str(recovery_path.relative_to(root))}
            if recovery.get("success"):
                attempts, stop_reason = execute_with_recovery(dict(config, baseline_commit=commit), parameters[cohort], cohort == "control", registry, root)
                record.update({"attempt_ids": [x["attempt_id"] for x in attempts], "measurement": _terminal(attempts), "stop_reason": stop_reason})
            else:
                record.update({"attempt_ids": [], "measurement": None, "infrastructure_error": recovery.get("error")})
            progress["records"] = [r for r in progress["records"] if int(r["schedule_index"]) != index]
            progress["records"].append(record)
            progress["records"].sort(key=lambda r: int(r["schedule_index"]))
            progress["next_schedule_index"] = next(
                (i for i in range(len(schedule)) if i not in {
                    int(r["schedule_index"]) for r in progress["records"] if r.get("measurement") is not None
                }),
                len(schedule),
            )
            write_json(progress_path, progress)
            if progress_callback:
                m = record.get("measurement")
                progress_callback("[%d/%d] outcome=%s elapsed=%s" % (index + 1, len(schedule), "infra_error" if m is None else m["outcome"], "n/a" if m is None else m.get("elapsed_time_seconds")))
        penalty = float(causal.get("failure_penalty_seconds", 600))
        cohorts = {name: summarize_measurements([r.get("measurement") for r in progress["records"] if r["cohort"] == name], penalty, root) for name in parameters}
        result = {
            "schema_version": SCHEMA_VERSION, "event_type": "upstream_speed_causal_campaign",
            "campaign_key": key, "experiment_name": config["name"], "baseline_commit": commit,
            "parameters": parameters, "schedule": schedule, "cohorts": cohorts,
            "campaign_complete": all(
                any(int(r["schedule_index"]) == i and r.get("measurement") is not None for r in progress["records"])
                for i in range(len(schedule))
            ),
            "pending_schedule_indices": [
                i for i in range(len(schedule))
                if not any(
                    int(record["schedule_index"]) == i and record.get("measurement") is not None
                    for record in progress["records"]
                )
            ],
            "attempt_ids": [a for r in progress["records"] for a in r.get("attempt_ids", [])],
            "result_path": str(result_path.relative_to(root)), "human_report_path": str(report_path.relative_to(root)),
            "completed_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        }
        result["arrival_analysis"] = _arrival_analysis(
            root, results_root,
            [r.get("measurement") for r in progress["records"] if r["cohort"] == "restricted"],
        )
        if causal.get("event_analysis"):
            result["focused_event_analysis"] = _focused_event_analysis(
                root, progress["records"], causal["event_analysis"], penalty,
                int(causal.get("repetitions", 5)),
            )
        write_json(result_path, result); _write_report(report_path, result); append_jsonl(results_root / "ledger.jsonl", result)
        return result
