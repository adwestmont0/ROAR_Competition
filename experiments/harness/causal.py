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
    lines = [
        "# WP 1240–1295 upstream-speed causal validation", "",
        "Every measurement used a forced-clean CARLA start.", "",
    ]
    for label, summary in (("Baseline control", control), ("Restricted throttle", restricted)):
        elapsed = summary.get("finished_elapsed_seconds")
        lines += [
            "## " + label, "",
            "- Completion rate: %s" % ("unavailable" if summary["completion_rate"] is None else "%.1f%%" % (100 * summary["completion_rate"])),
            "- Collision rate: %s" % ("unavailable" if summary["collision_rate"] is None else "%.1f%%" % (100 * summary["collision_rate"])),
            "- Finished mean: %s" % ("unavailable" if elapsed is None else "%.3f s" % elapsed["mean"]), "",
        ]
    lines += ["This campaign tests causal stability, not lap-time acceptance. See result.json for the authoritative record.", ""]
    arrival = result.get("arrival_analysis")
    if arrival:
        baseline = arrival["baseline"]
        restricted_arrival = arrival["restricted"]
        lines += [
            "## Causal intermediate variable", "",
            "- Arrival window: custom WP %d–%d" % tuple(arrival["custom_waypoint_window"]),
            "- Baseline traversals: %d; mean arrival speed: %.3f km/h" % (baseline["traversals"], baseline["mean_speed_kmh"]),
            "- Restricted traversals: %d; mean arrival speed: %.3f km/h" % (restricted_arrival["traversals"], restricted_arrival["mean_speed_kmh"]),
            "- Mean shift: %+.3f km/h" % arrival["restricted_minus_baseline_mean_kmh"],
            "- Restricted collisions in WP 1380–1420: %d" % arrival["restricted_zone_collisions"], "",
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
    return {
        "custom_waypoint_window": [1402, 1409], "baseline": baseline, "restricted": treatment,
        "restricted_minus_baseline_mean_kmh": treatment["mean_speed_kmh"] - baseline["mean_speed_kmh"],
        "restricted_zone_collisions": sum(
            item.get("outcome") == "collision" and 1380 <= int(item.get("custom_waypoint_index") or -1) <= 1420
            for item in usable
        ),
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
            "attempt_ids": [a for r in progress["records"] for a in r.get("attempt_ids", [])],
            "result_path": str(result_path.relative_to(root)), "human_report_path": str(report_path.relative_to(root)),
            "completed_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        }
        result["arrival_analysis"] = _arrival_analysis(
            root, results_root,
            [r.get("measurement") for r in progress["records"] if r["cohort"] == "restricted"],
        )
        write_json(result_path, result); _write_report(report_path, result); append_jsonl(results_root / "ledger.jsonl", result)
        return result
