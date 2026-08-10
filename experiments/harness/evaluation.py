import contextlib
import csv
import datetime as dt
import fcntl
import hashlib
import json
import math
import os
import socket
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Tuple

from .core import (
    append_jsonl,
    dry_run,
    execute_with_recovery,
    experiment_id,
    git_output,
    read_json,
    write_json,
)


SCHEMA_VERSION = 2
CONTROLLER_OUTCOMES = {
    "finished", "collision", "timeout", "exception", "controller_hang"
}
DEFAULT_ACCEPTANCE = {
    "min_controller_measurements": None,
    "min_completion_rate": 1.0,
    "max_collision_rate": 0.0,
    "require_all_controls_finished": True,
    "max_control_drift_seconds": 1.0,
    "max_control_adjusted_delta_seconds": None,
}


def _stats(values: List[float]) -> Optional[Dict[str, float]]:
    if not values:
        return None
    return {
        "count": len(values),
        "mean": statistics.mean(values),
        "median": statistics.median(values),
        "minimum": min(values),
        "maximum": max(values),
        "stdev": statistics.stdev(values) if len(values) > 1 else 0.0,
    }


def _terminal_measurement(attempts: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    for attempt in reversed(attempts):
        if attempt.get("outcome") in CONTROLLER_OUTCOMES:
            return attempt
    return None


def _section_times(root: Path, result: Dict[str, Any]) -> Dict[str, float]:
    telemetry_path = result.get("telemetry_path")
    if not telemetry_path:
        return {}
    ticks_path = (root / telemetry_path).parent / "ticks.csv"
    if not ticks_path.exists():
        return {}
    counts: Dict[str, int] = defaultdict(int)
    with ticks_path.open(encoding="utf-8", newline="") as infile:
        for row in csv.DictReader(infile):
            counts[row["section"]] += 1
    timestep = result.get("control_timestep_seconds")
    if timestep is None:
        return {}
    return {section: count * float(timestep) for section, count in counts.items()}


def summarize_measurements(
    measurements: List[Optional[Dict[str, Any]]],
    penalty_seconds: float,
    root: Path,
) -> Dict[str, Any]:
    usable = [item for item in measurements if item is not None]
    counts = Counter(item["outcome"] for item in usable)
    finished_times = [
        float(item["elapsed_time_seconds"])
        for item in usable
        if item["outcome"] == "finished" and item.get("elapsed_time_seconds") is not None
    ]
    scores = [
        float(item["elapsed_time_seconds"])
        if item["outcome"] == "finished" and item.get("elapsed_time_seconds") is not None
        else penalty_seconds
        for item in usable
    ]
    section_values: Dict[str, List[float]] = defaultdict(list)
    for item in usable:
        if item["outcome"] == "finished":
            for section, seconds in _section_times(root, item).items():
                section_values[section].append(seconds)
    terminal_failures = [
        {
            "attempt_id": item["attempt_id"],
            "outcome": item["outcome"],
            "elapsed_time_seconds": item.get("elapsed_time_seconds"),
            "official_waypoint_index": item.get("official_waypoint_index"),
            "custom_waypoint_index": item.get("custom_waypoint_index"),
            "terminal_location": item.get("terminal_location"),
            "collision_impulse": item.get("collision_impulse"),
        }
        for item in usable
        if item["outcome"] != "finished"
    ]
    return {
        "scheduled_repetitions": len(measurements),
        "controller_measurements": len(usable),
        "missing_due_to_infrastructure": len(measurements) - len(usable),
        "outcome_counts": dict(sorted(counts.items())),
        "completion_rate": counts.get("finished", 0) / len(usable) if usable else None,
        "collision_rate": counts.get("collision", 0) / len(usable) if usable else None,
        "finished_elapsed_seconds": _stats(finished_times),
        "penalized_objective_seconds": _stats(scores),
        "section_elapsed_seconds": {
            section: _stats(values)
            for section, values in sorted(section_values.items(), key=lambda item: int(item[0]))
        },
        "measurement_attempt_ids": [item["attempt_id"] for item in usable],
        "terminal_failures": terminal_failures,
    }


def _format_seconds(value: Optional[float]) -> str:
    return "unavailable" if value is None else "%.3f s" % value


def _collision_interpretation(summary: Dict[str, Any]) -> Optional[str]:
    collisions = [
        item for item in summary["terminal_failures"]
        if item["outcome"] == "collision"
    ]
    if not collisions:
        return None
    waypoints = [
        item["official_waypoint_index"] for item in collisions
        if item["official_waypoint_index"] is not None
    ]
    elapsed = [
        float(item["elapsed_time_seconds"]) for item in collisions
        if item["elapsed_time_seconds"] is not None
    ]
    locations = [
        item["terminal_location"] for item in collisions
        if item["terminal_location"] is not None
    ]
    detail = "%d collision(s)" % len(collisions)
    if waypoints:
        detail += " at official waypoint%s %d–%d" % (
            "" if min(waypoints) == max(waypoints) else "s",
            min(waypoints), max(waypoints),
        )
    if elapsed:
        detail += " after %.2f–%.2f seconds" % (min(elapsed), max(elapsed))
    waypoint_clustered = (
        len(collisions) >= 2
        and waypoints and max(waypoints) - min(waypoints) <= 10
        and elapsed and max(elapsed) - min(elapsed) <= 1.0
    )
    maximum_separation = 0.0
    for index, first in enumerate(locations):
        for second in locations[index + 1:]:
            maximum_separation = max(
                maximum_separation,
                math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(first, second))),
            )
    spatially_clustered = len(locations) >= 2 and maximum_separation <= 15.0
    if waypoint_clustered:
        detail += "; the tight clustering indicates a repeatable stability failure"
    elif spatially_clustered:
        detail += "; despite different lap-indexed waypoints, all impacts are within %.1f m and indicate a repeatable physical failure zone" % maximum_separation
    elif len(collisions) >= 2:
        detail += "; the failures are dispersed and may be stochastic or multi-modal"
    return detail + "."


def render_candidate_report(result: Dict[str, Any]) -> str:
    candidate = result["candidate"]
    controls = result["controls"]
    objective = result["objective"]
    lines = [
        "# Candidate evaluation: %s" % (result.get("candidate_name") or result["experiment_id"]),
        "",
        "## Decision",
        "",
        "**%s.**" % result["status"].upper(),
        "",
    ]
    if result["status"] == "inconclusive":
        lines.append("The candidate cannot be compared reliably because its control runs were invalid.")
    elif result["status"] == "rejected":
        lines.append("The candidate completed evaluation but failed one or more candidate acceptance rules.")
    elif result["status"] == "accepted":
        lines.append("The candidate and its controls satisfied every configured acceptance rule.")
    else:
        lines.append("The requested evaluation did not produce enough valid measurements.")
    lines.extend(["", "## Configuration", "", "```json", json.dumps(result["parameters"], indent=2, sort_keys=True), "```", ""])
    finished = candidate["finished_elapsed_seconds"]
    lines.extend([
        "## Candidate performance",
        "",
        "- Controller measurements: %d of %d scheduled" % (
            candidate["controller_measurements"], candidate["scheduled_repetitions"]),
        "- Completion rate: %s" % (
            "unavailable" if candidate["completion_rate"] is None else "%.1f%%" % (100 * candidate["completion_rate"])),
        "- Collision rate: %s" % (
            "unavailable" if candidate["collision_rate"] is None else "%.1f%%" % (100 * candidate["collision_rate"])),
    ])
    if finished:
        lines.extend([
            "- Finished mean: %s" % _format_seconds(finished["mean"]),
            "- Finished median: %s" % _format_seconds(finished["median"]),
            "- Finished range: %s–%s" % (
                _format_seconds(finished["minimum"]), _format_seconds(finished["maximum"])),
            "- Finished standard deviation: %s" % _format_seconds(finished["stdev"]),
        ])
    collision_text = _collision_interpretation(candidate)
    if collision_text:
        lines.extend(["", collision_text])

    before = controls["before"]["finished_elapsed_seconds"]
    after = controls["after"]["finished_elapsed_seconds"]
    lines.extend([
        "", "## Controls", "",
        "- Controls valid: %s" % ("yes" if result["controls_valid"] else "no"),
        "- Before-control finished mean: %s" % _format_seconds(before["mean"] if before else None),
        "- After-control finished mean: %s" % _format_seconds(after["mean"] if after else None),
    ])
    if before and after:
        lines.append("- Control drift: %+.3f s" % (after["mean"] - before["mean"]))
    delta = objective["control_adjusted_delta_seconds"]
    lines.append("- Control-adjusted candidate delta: %s" % (
        "unavailable because controls were invalid" if delta is None else "%+.3f s" % delta
    ))

    section_deltas = objective["section_control_adjusted_delta_seconds"]
    if section_deltas:
        lines.extend(["", "## Section deltas", "", "Negative values are faster than the controls.", ""])
        for section, value in sorted(section_deltas.items(), key=lambda item: item[1]):
            lines.append("- Section %s: %+.3f s" % (section, value))

    lines.extend(["", "## Acceptance checks", ""])
    for check in result["acceptance_checks"]:
        lines.append("- %s %s — actual `%s`, required `%s`" % (
            "PASS" if check["passed"] else "FAIL", check["name"],
            check["actual"], check["expected"],
        ))

    if len(result.get("changed_parameters", result["parameters"])) > 1 and result["status"] == "rejected":
        lines.extend([
            "", "## Suggested next experiment", "",
            "Multiple parameters changed, so this result cannot identify which change caused the failure. Test each parameter independently against the pinned baseline before combining them.",
        ])
    lines.extend(["", "The JSON summary remains the authoritative machine-readable record.", ""])
    return "\n".join(lines)


def write_candidate_report(path: Path, result: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(render_candidate_report(result), encoding="utf-8")
    os.replace(str(temporary), str(path))


def _control_spec(controls: Any) -> Dict[str, Any]:
    if isinstance(controls, int):
        controls = {"before": controls, "after": controls}
    controls = dict(controls or {})
    return {
        "before": int(controls.get("before", controls.get("repetitions", 1))),
        "after": int(controls.get("after", controls.get("repetitions", 1))),
        "parameters": dict(controls.get("parameters", {})),
    }


def _evaluation_key(
    config: Dict[str, Any], parameters: Dict[str, Any], repetitions: int,
    controls: Dict[str, Any], acceptance: Dict[str, Any], penalty_seconds: float,
) -> str:
    identity = {
        "schema_version": SCHEMA_VERSION,
        "baseline_commit": config["baseline_commit"],
        "parameters": parameters,
        "repetitions": repetitions,
        "controls": controls,
        "acceptance": acceptance,
        "failure_penalty_seconds": penalty_seconds,
    }
    return hashlib.sha256(
        json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()[:20]


@contextlib.contextmanager
def evaluation_lock(results_root: Path) -> Iterator[None]:
    results_root.mkdir(parents=True, exist_ok=True)
    path = results_root / "evaluation.lock"
    with path.open("a+", encoding="utf-8") as lock_file:
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock_file.seek(0)
            owner = lock_file.read().strip() or "unknown owner"
            raise RuntimeError("Another CARLA evaluation holds the lock: " + owner)
        lock_file.seek(0)
        lock_file.truncate()
        json.dump(
            {
                "pid": os.getpid(), "host": socket.gethostname(),
                "acquired_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            },
            lock_file,
        )
        lock_file.flush()
        try:
            yield
        finally:
            lock_file.seek(0)
            lock_file.truncate()
            lock_file.flush()
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _run_measurement(
    config: Dict[str, Any], parameters: Dict[str, Any], is_control: bool,
    registry: Dict[str, Any], root: Path,
) -> Tuple[List[Dict[str, Any]], Optional[Dict[str, Any]], Optional[str]]:
    attempts, stop_reason = execute_with_recovery(
        config, parameters, is_control, registry, root
    )
    return attempts, _terminal_measurement(attempts), stop_reason


def _rule(name: str, actual: Any, expected: Any, passed: bool) -> Dict[str, Any]:
    return {"name": name, "actual": actual, "expected": expected, "passed": passed}


def _section_deltas(
    candidate: Dict[str, Any], control: Dict[str, Any]
) -> Dict[str, float]:
    output = {}
    for section, candidate_stats in candidate["section_elapsed_seconds"].items():
        control_stats = control["section_elapsed_seconds"].get(section)
        if candidate_stats and control_stats:
            output[section] = candidate_stats["mean"] - control_stats["mean"]
    return output


def _acceptance(
    candidate: Dict[str, Any], before: Dict[str, Any], after: Dict[str, Any],
    controls: Dict[str, Any], rules: Dict[str, Any], delta: Optional[float],
) -> Tuple[str, List[Dict[str, Any]]]:
    minimum = rules["min_controller_measurements"]
    checks = [
        _rule("minimum_controller_measurements", candidate["controller_measurements"], minimum,
              candidate["controller_measurements"] >= minimum),
        _rule("minimum_completion_rate", candidate["completion_rate"], rules["min_completion_rate"],
              candidate["completion_rate"] is not None and candidate["completion_rate"] >= rules["min_completion_rate"]),
        _rule("maximum_collision_rate", candidate["collision_rate"], rules["max_collision_rate"],
              candidate["collision_rate"] is not None and candidate["collision_rate"] <= rules["max_collision_rate"]),
    ]
    control_finished = (
        before["controller_measurements"] == controls["before"]
        and after["controller_measurements"] == controls["after"]
        and before["completion_rate"] in (1.0, None if controls["before"] == 0 else -1)
        and after["completion_rate"] in (1.0, None if controls["after"] == 0 else -1)
    )
    if rules["require_all_controls_finished"]:
        checks.append(_rule("all_controls_finished", control_finished, True, control_finished))
    before_time = before["finished_elapsed_seconds"]
    after_time = after["finished_elapsed_seconds"]
    drift = None
    if before_time and after_time:
        drift = after_time["mean"] - before_time["mean"]
    if rules["max_control_drift_seconds"] is not None and controls["before"] and controls["after"]:
        checks.append(_rule("maximum_absolute_control_drift_seconds", drift,
                            rules["max_control_drift_seconds"],
                            drift is not None and abs(drift) <= rules["max_control_drift_seconds"]))
    if rules["max_control_adjusted_delta_seconds"] is not None:
        checks.append(_rule("maximum_control_adjusted_delta_seconds", delta,
                            rules["max_control_adjusted_delta_seconds"],
                            delta is not None and delta <= rules["max_control_adjusted_delta_seconds"]))
    if candidate["controller_measurements"] < minimum:
        return "incomplete", checks
    control_rule_names = {
        "all_controls_finished", "maximum_absolute_control_drift_seconds"
    }
    if any(
        not item["passed"] and item["name"] in control_rule_names
        for item in checks
    ):
        return "inconclusive", checks
    return ("accepted" if all(item["passed"] for item in checks) else "rejected"), checks


class CandidateEvaluator:
    """Stable execution boundary used by search/optimization clients."""

    def __init__(self, config: Dict[str, Any], registry: Dict[str, Any], root: Path):
        self.config = dict(config)
        self.config["baseline_commit"] = git_output(
            root, "rev-parse", "%s^{commit}" % config["baseline_commit"]
        )
        self.registry = registry
        self.root = root
        self.results_root = root / config.get("results_dir", "experiment_results")

    def _changed_parameters(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        return {
            name: value
            for name, value in parameters.items()
            if self.registry.get(name, {}).get("baseline_value") != value
        }

    def evaluate(
        self, parameters: Dict[str, Any], repetitions: int, controls: Any,
        candidate_name: Optional[str] = None, resume: bool = True,
    ) -> Dict[str, Any]:
        with evaluation_lock(self.results_root):
            return self._evaluate_locked(
                parameters, repetitions, controls, candidate_name, resume
            )

    def _evaluate_locked(
        self, parameters: Dict[str, Any], repetitions: int, controls: Any,
        candidate_name: Optional[str], resume: bool,
    ) -> Dict[str, Any]:
        if repetitions < 1:
            raise ValueError("repetitions must be positive")
        parameters = dict(parameters)
        control = _control_spec(controls)
        if control["before"] < 0 or control["after"] < 0:
            raise ValueError("control counts must be non-negative")
        evaluation = self.config.get("evaluation", {})
        penalty = float(evaluation.get("failure_penalty_seconds", 600.0))
        rules = dict(DEFAULT_ACCEPTANCE)
        rules.update(evaluation.get("acceptance", {}))
        if rules["min_controller_measurements"] is None:
            rules["min_controller_measurements"] = repetitions
        key = _evaluation_key(self.config, parameters, repetitions, control, rules, penalty)
        summary_path = self.results_root / "candidate_summaries" / (key + ".json")
        report_path = summary_path.with_suffix(".md")
        if resume and summary_path.exists():
            prior = read_json(summary_path)
            if prior.get("status") in {"accepted", "rejected", "inconclusive"}:
                prior["changed_parameters"] = self._changed_parameters(prior["parameters"])
                prior["human_report_path"] = str(report_path.relative_to(self.root))
                write_json(summary_path, prior)
                if not report_path.exists():
                    write_candidate_report(report_path, prior)
                resumed = dict(prior)
                resumed["resume"] = {"skipped_completed": True, "source": str(summary_path.relative_to(self.root))}
                return resumed

        attempts: List[Dict[str, Any]] = []
        before_items: List[Optional[Dict[str, Any]]] = []
        candidate_items: List[Optional[Dict[str, Any]]] = []
        after_items: List[Optional[Dict[str, Any]]] = []
        stop_reason = None
        phases = (
            (control["before"], control["parameters"], True, before_items),
            (repetitions, parameters, False, candidate_items),
            (control["after"], control["parameters"], True, after_items),
        )
        for count, phase_parameters, is_control, destination in phases:
            for _ in range(count):
                run_attempts, measurement, stop_reason = _run_measurement(
                    self.config, phase_parameters, is_control, self.registry, self.root
                )
                attempts.extend(run_attempts)
                destination.append(measurement)
                if stop_reason:
                    break
            if stop_reason:
                break

        candidate_summary = summarize_measurements(candidate_items, penalty, self.root)
        before_summary = summarize_measurements(before_items, penalty, self.root)
        after_summary = summarize_measurements(after_items, penalty, self.root)
        control_items = before_items + after_items
        control_summary = summarize_measurements(control_items, penalty, self.root)
        candidate_objective = candidate_summary["penalized_objective_seconds"]
        control_objective = control_summary["penalized_objective_seconds"]
        raw_delta = None
        if candidate_objective and control_objective:
            raw_delta = candidate_objective["mean"] - control_objective["mean"]
        status, acceptance_checks = _acceptance(
            candidate_summary, before_summary, after_summary, control, rules, raw_delta
        )
        if stop_reason:
            status = "incomplete"
        controls_valid = not any(
            not item["passed"] and item["name"] in {
                "all_controls_finished", "maximum_absolute_control_drift_seconds"
            }
            for item in acceptance_checks
        )
        delta = raw_delta if controls_valid else None
        result = {
            "schema_version": SCHEMA_VERSION,
            "event_type": "candidate_summary",
            "evaluation_key": key,
            "candidate_name": candidate_name,
            "experiment_name": self.config["name"],
            "experiment_id": experiment_id(parameters),
            "baseline_commit": self.config["baseline_commit"],
            "parameters": parameters,
            "changed_parameters": self._changed_parameters(parameters),
            "requested": {"repetitions": repetitions, "controls": control},
            "status": status,
            "accepted": status == "accepted",
            "controls_valid": controls_valid,
            "stop_reason": stop_reason,
            "acceptance_rules": rules,
            "acceptance_checks": acceptance_checks,
            "candidate": candidate_summary,
            "controls": {"before": before_summary, "after": after_summary, "combined": control_summary},
            "objective": {
                "failure_penalty_seconds": penalty,
                "penalized_seconds": candidate_objective,
                "control_adjusted_delta_seconds": delta,
                "section_control_adjusted_delta_seconds": _section_deltas(
                    candidate_summary, control_summary
                ) if controls_valid else {},
            },
            "attempt_ids": [item["attempt_id"] for item in attempts],
            "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "result_path": str(summary_path.relative_to(self.root)),
            "human_report_path": str(report_path.relative_to(self.root)),
        }
        write_json(summary_path, result)
        write_candidate_report(report_path, result)
        append_jsonl(self.results_root / "ledger.jsonl", result)
        return result


def evaluate(
    parameters: Dict[str, Any], repetitions: int, controls: Any, *,
    config: Dict[str, Any], registry: Dict[str, Any], root: Path,
    candidate_name: Optional[str] = None, resume: bool = True,
) -> Dict[str, Any]:
    """Evaluate one parameter configuration and return a normalized result."""
    return CandidateEvaluator(config, registry, root).evaluate(
        parameters, repetitions, controls, candidate_name, resume
    )


def dry_run_candidates(
    config: Dict[str, Any], registry: Dict[str, Any], root: Path
) -> List[Dict[str, Any]]:
    candidates = config.get("candidates")
    if not isinstance(candidates, list) or not candidates:
        raise ValueError("Candidate evaluation requires a non-empty candidates list")
    output = []
    for candidate in candidates:
        concrete = dict(config)
        concrete.pop("candidates", None)
        concrete["parameters"] = dict(candidate.get("parameters", {}))
        item = dry_run(concrete, registry, root)[0]
        item["candidate_name"] = candidate.get("name")
        output.append(item)
    return output


def evaluate_candidates(
    config: Dict[str, Any], registry: Dict[str, Any], root: Path,
    resume: bool = True,
) -> Dict[str, Any]:
    candidates = config.get("candidates")
    if not isinstance(candidates, list) or not candidates:
        raise ValueError("Candidate evaluation requires a non-empty candidates list")
    evaluation = config.get("evaluation", {})
    repetitions = int(evaluation.get("repetitions", 1))
    controls = evaluation.get("controls", {
        "before": int(evaluation.get("control_repetitions", 1)),
        "after": int(evaluation.get("control_repetitions", 1)),
    })
    results = []
    evaluator = CandidateEvaluator(config, registry, root)
    for candidate in candidates:
        results.append(evaluator.evaluate(
            dict(candidate.get("parameters", {})), repetitions, controls,
            candidate.get("name"), resume,
        ))
    return {
        "schema_version": SCHEMA_VERSION,
        "event_type": "candidate_evaluation_batch",
        "experiment_name": config["name"],
        "candidate_results": results,
    }
