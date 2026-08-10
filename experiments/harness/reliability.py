import datetime as dt
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from .core import (
    append_jsonl,
    dry_run,
    execute_with_recovery,
    git_output,
    read_json,
    run_recovery,
    write_json,
)
from .evaluation import (
    CONTROLLER_OUTCOMES,
    evaluation_lock,
    summarize_measurements,
)


RELIABILITY_SCHEMA_VERSION = 1


def campaign_schedule(repetitions_per_cohort: int) -> List[str]:
    if repetitions_per_cohort < 1:
        raise ValueError("repetitions_per_cohort must be positive")
    return [cohort for _ in range(repetitions_per_cohort) for cohort in ("warm", "forced_clean")]


def _campaign_key(config: Dict[str, Any], commit: str) -> str:
    reliability = config.get("reliability", {})
    identity = {
        "schema_version": RELIABILITY_SCHEMA_VERSION,
        "baseline_commit": commit,
        "parameters": dict(config.get("parameters", {})),
        "repetitions_per_cohort": int(reliability.get("repetitions_per_cohort", 10)),
        "material_improvement": float(reliability.get("material_completion_rate_improvement", 0.20)),
        "minimum_valid_attempts_per_cohort": int(reliability.get("minimum_valid_attempts_per_cohort", 8)),
        "schedule": "warm_then_forced_clean_interleaved",
    }
    return hashlib.sha256(
        json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()[:20]


def _terminal_measurement(attempts: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    for attempt in reversed(attempts):
        if attempt.get("outcome") in CONTROLLER_OUTCOMES:
            return attempt
    return None


def wilson_interval(successes: int, total: int, z: float = 1.96) -> Optional[List[float]]:
    if total == 0:
        return None
    probability = successes / total
    denominator = 1 + z * z / total
    center = (probability + z * z / (2 * total)) / denominator
    margin = z * math.sqrt(
        probability * (1 - probability) / total + z * z / (4 * total * total)
    ) / denominator
    return [max(0.0, center - margin), min(1.0, center + margin)]


def reliability_decision(
    warm: Dict[str, Any], forced: Dict[str, Any], minimum_valid: int,
    material_improvement: float,
) -> Dict[str, Any]:
    warm_count = warm["controller_measurements"]
    forced_count = forced["controller_measurements"]
    difference = None
    if warm["completion_rate"] is not None and forced["completion_rate"] is not None:
        difference = forced["completion_rate"] - warm["completion_rate"]
    if warm_count < minimum_valid or forced_count < minimum_valid:
        conclusion = "incomplete"
    elif difference is not None and difference >= material_improvement:
        conclusion = "simulator_state_factor_supported"
    else:
        conclusion = "no_material_clean_start_improvement"
    return {
        "conclusion": conclusion,
        "forced_clean_minus_warm_completion_rate": difference,
        "material_improvement_threshold": material_improvement,
        "minimum_valid_attempts_per_cohort": minimum_valid,
        "warm_completion_rate_95pct_wilson": wilson_interval(
            warm["outcome_counts"].get("finished", 0), warm_count
        ),
        "forced_clean_completion_rate_95pct_wilson": wilson_interval(
            forced["outcome_counts"].get("finished", 0), forced_count
        ),
    }


def render_reliability_report(result: Dict[str, Any]) -> str:
    warm = result["cohorts"]["warm"]
    forced = result["cohorts"]["forced_clean"]
    decision = result["decision"]
    labels = {
        "incomplete": "INCOMPLETE",
        "simulator_state_factor_supported": "SIMULATOR-STATE FACTOR SUPPORTED",
        "no_material_clean_start_improvement": "NO MATERIAL CLEAN-START IMPROVEMENT",
    }
    lines = [
        "# Baseline clean-start A/B reliability campaign", "",
        "## Conclusion", "", "**%s.**" % labels[decision["conclusion"]], "",
    ]
    if decision["conclusion"] == "simulator_state_factor_supported":
        lines.append("Forced CARLA restarts improved completion rate by at least the configured material threshold, supporting persistent simulator state as a factor.")
    elif decision["conclusion"] == "no_material_clean_start_improvement":
        lines.append("Forced CARLA restarts did not materially improve completion rate; the baseline controller appears stochastic or marginal in this setup, subject to the reported sample uncertainty.")
    else:
        lines.append("The campaign lacks enough controller-valid attempts in one or both cohorts to decide.")
    lines.extend(["", "## Cohort results", ""])
    for name, summary in (("Warm server", warm), ("Forced clean start", forced)):
        elapsed = summary["finished_elapsed_seconds"]
        interval = decision[("warm" if name == "Warm server" else "forced_clean") + "_completion_rate_95pct_wilson"]
        lines.extend([
            "### " + name, "",
            "- Controller-valid attempts: %d of %d scheduled" % (summary["controller_measurements"], summary["scheduled_repetitions"]),
            "- Infrastructure-only misses: %d" % summary["missing_due_to_infrastructure"],
            "- Completion rate: %s" % ("unavailable" if summary["completion_rate"] is None else "%.1f%%" % (100 * summary["completion_rate"])),
            "- Collision rate: %s" % ("unavailable" if summary["collision_rate"] is None else "%.1f%%" % (100 * summary["collision_rate"])),
            "- Completion-rate 95%% Wilson interval: %s" % ("unavailable" if interval is None else "%.1f%%–%.1f%%" % (100 * interval[0], 100 * interval[1])),
            "- Finished mean time: %s" % ("unavailable" if elapsed is None else "%.3f s" % elapsed["mean"]),
        ])
    difference = decision["forced_clean_minus_warm_completion_rate"]
    lines.extend([
        "", "## A/B effect", "",
        "- Forced-clean minus warm completion rate: %s" % ("unavailable" if difference is None else "%+.1f percentage points" % (100 * difference)),
        "- Material-improvement threshold: %.1f percentage points" % (100 * decision["material_improvement_threshold"]),
        "", "The JSON result is the authoritative machine-readable record.", "",
    ])
    return "\n".join(lines)


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def dry_run_reliability(
    config: Dict[str, Any], registry: Dict[str, Any], root: Path
) -> Dict[str, Any]:
    concrete = dict(config)
    concrete["parameters"] = dict(config.get("parameters", {}))
    source = dry_run(concrete, registry, root)[0]
    repetitions = int(config.get("reliability", {}).get("repetitions_per_cohort", 10))
    return {
        "baseline_commit": source["baseline_commit"],
        "parameters": source["parameters"],
        "source_diff": source["source_diff"],
        "schedule": campaign_schedule(repetitions),
        "total_scheduled_attempts": repetitions * 2,
    }


def run_reliability_campaign(
    config: Dict[str, Any], registry: Dict[str, Any], root: Path,
    resume: bool = True,
    progress_callback: Optional[Callable[[str], None]] = None,
) -> Dict[str, Any]:
    commit = git_output(root, "rev-parse", "%s^{commit}" % config["baseline_commit"])
    reliability = config.get("reliability", {})
    repetitions = int(reliability.get("repetitions_per_cohort", 10))
    schedule = campaign_schedule(repetitions)
    key = _campaign_key(config, commit)
    results_root = root / config.get("results_dir", "experiment_results")
    campaign_dir = results_root / "reliability" / key
    progress_path = campaign_dir / "progress.json"
    result_path = campaign_dir / "result.json"
    report_path = campaign_dir / "report.md"
    parameters = dict(config.get("parameters", {}))

    with evaluation_lock(results_root):
        if resume and result_path.exists() and read_json(result_path).get("campaign_complete"):
            result = read_json(result_path)
            result["resume"] = {"skipped_completed": True}
            if not report_path.exists():
                _write_text(report_path, render_reliability_report(result))
            return result
        if resume and progress_path.exists():
            progress = read_json(progress_path)
        else:
            progress = {
                "schema_version": RELIABILITY_SCHEMA_VERSION,
                "campaign_key": key,
                "baseline_commit": commit,
                "parameters": parameters,
                "schedule": schedule,
                "next_schedule_index": 0,
                "records": [],
                "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            }
            write_json(progress_path, progress)

        for index in range(int(progress["next_schedule_index"]), len(schedule)):
            cohort = schedule[index]
            if progress_callback:
                progress_callback("[%d/%d] starting %s baseline attempt" % (index + 1, len(schedule), cohort))
            record: Dict[str, Any] = {"schedule_index": index, "cohort": cohort}
            if cohort == "forced_clean":
                if progress_callback:
                    progress_callback("[%d/%d] restarting CARLA through SSM" % (index + 1, len(schedule)))
                recovery_config = config.get("recovery")
                recovery = (
                    run_recovery(recovery_config, config["carla"])
                    if recovery_config is not None
                    else {"event_type": "infrastructure_recovery", "success": False, "error": "recovery_not_configured"}
                )
                recovery.update({
                    "experiment_name": config["name"], "campaign_key": key,
                    "reason": "scheduled_forced_clean_start", "schedule_index": index,
                })
                recovery_path = campaign_dir / "recoveries" / ("%03d.json" % index)
                write_json(recovery_path, recovery)
                append_jsonl(results_root / "ledger.jsonl", recovery)
                record["scheduled_recovery_path"] = str(recovery_path.relative_to(root))
                if not recovery.get("success"):
                    if progress_callback:
                        progress_callback("[%d/%d] scheduled restart failed: %s" % (index + 1, len(schedule), recovery.get("error")))
                    record["measurement"] = None
                    record["attempt_ids"] = []
                    record["infrastructure_error"] = recovery.get("error")
                    progress["records"].append(record)
                    progress["next_schedule_index"] = index + 1
                    write_json(progress_path, progress)
                    continue
            attempts, stop_reason = execute_with_recovery(
                dict(config, baseline_commit=commit), parameters, True, registry, root
            )
            record["attempt_ids"] = [attempt["attempt_id"] for attempt in attempts]
            record["measurement"] = _terminal_measurement(attempts)
            record["stop_reason"] = stop_reason
            if progress_callback:
                measurement = record["measurement"]
                progress_callback("[%d/%d] outcome=%s elapsed=%s" % (
                    index + 1, len(schedule),
                    "infra_error" if measurement is None else measurement.get("outcome"),
                    "n/a" if measurement is None else measurement.get("elapsed_time_seconds"),
                ))
            progress["records"].append(record)
            progress["next_schedule_index"] = index + 1
            write_json(progress_path, progress)
            if stop_reason:
                break

        measurements = {
            cohort: [record.get("measurement") for record in progress["records"] if record["cohort"] == cohort]
            for cohort in ("warm", "forced_clean")
        }
        penalty = float(reliability.get("failure_penalty_seconds", 600.0))
        cohorts = {
            cohort: summarize_measurements(items, penalty, root)
            for cohort, items in measurements.items()
        }
        decision = reliability_decision(
            cohorts["warm"], cohorts["forced_clean"],
            int(reliability.get("minimum_valid_attempts_per_cohort", 8)),
            float(reliability.get("material_completion_rate_improvement", 0.20)),
        )
        result = {
            "schema_version": RELIABILITY_SCHEMA_VERSION,
            "event_type": "baseline_reliability_campaign",
            "campaign_key": key,
            "experiment_name": config["name"],
            "baseline_commit": commit,
            "parameters": parameters,
            "campaign_complete": int(progress["next_schedule_index"]) >= len(schedule),
            "schedule": schedule,
            "cohorts": cohorts,
            "decision": decision,
            "attempt_ids": [attempt_id for record in progress["records"] for attempt_id in record.get("attempt_ids", [])],
            "result_path": str(result_path.relative_to(root)),
            "human_report_path": str(report_path.relative_to(root)),
            "completed_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        }
        write_json(result_path, result)
        _write_text(report_path, render_reliability_report(result))
        append_jsonl(results_root / "ledger.jsonl", result)
        return result
