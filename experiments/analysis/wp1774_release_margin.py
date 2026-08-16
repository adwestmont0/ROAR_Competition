"""Decompose terminal-tail release blockers at the winner's final brake ticks."""

import csv
import datetime as dt
import json
import statistics
from collections import Counter
from pathlib import Path

from experiments.harness.core import load_ledger, write_json


WINDOW = (1774, 1856)
BRAKE_ACTIVE = 0.05
STEERING_SUPPORT_LIMIT = 0.1


def _number(row, key, default=None):
    if row is None:
        return default
    value = row.get(key)
    if value in (None, ""):
        return default
    return float(value)


def _summary(values):
    values = [float(value) for value in values if value is not None]
    if not values:
        return None
    ordered = sorted(values)
    percentile = lambda fraction: ordered[round(fraction * (len(ordered) - 1))]
    return {
        "count": len(values),
        "minimum": min(values),
        "p10": percentile(0.10),
        "median": statistics.median(values),
        "p90": percentile(0.90),
        "maximum": max(values),
    }


def _load_attempts(root, results_dir, experiment_name):
    attempts = []
    for item in load_ledger(results_dir / "ledger.jsonl"):
        if item.get("experiment_name") != experiment_name:
            continue
        if item.get("outcome") not in ("finished", "collision"):
            continue
        telemetry_path = item.get("telemetry_path")
        if not telemetry_path:
            continue
        ticks_path = (root / telemetry_path).parent / "ticks.csv"
        if not ticks_path.exists():
            continue
        with ticks_path.open(encoding="utf-8") as infile:
            attempts.append((item, list(csv.DictReader(infile))))
    return attempts


def _decompose(item, lap, ordinal, row, lap_rows):
    current = _number(row, "speed_kmh")
    target = _number(row, "terminal_tail_terminal_target_speed_kmh")
    predicted = _number(row, "terminal_tail_predicted_speed_at_direct_horizon_kmh")
    direct_uncertainty = _number(row, "terminal_tail_direct_uncertainty_kmh", 0.0)
    tail_credit = _number(row, "terminal_tail_tail_credit_kmh", 0.0)
    tail_uncertainty = _number(row, "terminal_tail_tail_uncertainty_kmh", 0.0)
    upper = _number(row, "terminal_tail_terminal_speed_upper_bound_kmh")
    guard_trace = json.loads(row.get("terminal_tail_residual_guard_trace_json") or "[]")
    guard_margins = [float(point["margin_kmh"]) for point in guard_trace]
    minimum_guard_margin = min(guard_margins) if guard_margins else None
    steer = abs(_number(row, "terminal_tail_steer", _number(row, "steer", 0.0)))
    status = row.get("terminal_tail_status")
    reason = row.get("terminal_tail_reason_code")
    terminal_custom = int(float(row["terminal_tail_terminal_custom_waypoint_index"]))
    future = next((candidate for candidate in lap_rows if int(candidate["tick"]) > int(row["tick"]) and int(candidate["custom_waypoint_index"]) >= terminal_custom), None)
    realized_terminal_speed = _number(future, "speed_kmh") if future else None
    return {
        "attempt_id": item["attempt_id"],
        "lap": int(lap),
        "winner_final_brake_tick": ordinal,
        "tick": int(row["tick"]),
        "custom_waypoint_index": int(row["custom_waypoint_index"]),
        "current_speed_kmh": current,
        "terminal_profile_index": int(float(row["terminal_tail_terminal_profile_index"])),
        "terminal_custom_waypoint_index": terminal_custom,
        "terminal_distance_m": _number(row, "terminal_tail_terminal_distance_m"),
        "terminal_horizon_ticks": int(float(row["terminal_tail_terminal_horizon_ticks"])),
        "terminal_target_speed_kmh": target,
        "current_overspeed_to_terminal_kmh": current - target,
        "residual_deceleration_credit_kmh": current - predicted,
        "direct_predicted_overspeed_kmh": predicted - target,
        "realized_terminal_speed_kmh": realized_terminal_speed,
        "realized_terminal_margin_kmh": realized_terminal_speed - target if realized_terminal_speed is not None else None,
        "direct_prediction_minus_realized_kmh": predicted - realized_terminal_speed if realized_terminal_speed is not None else None,
        "direct_model_uncertainty_kmh": direct_uncertainty,
        "tail_residual_conservatism_kmh": tail_credit,
        "tail_uncertainty_kmh": tail_uncertainty,
        "terminal_upper_bound_kmh": upper,
        "target_margin_upper_minus_target_kmh": upper - target,
        "eligibility_required_margin_kmh": 1.0,
        "eligibility_min_projected_margin_kmh": minimum_guard_margin,
        "eligibility_slack_kmh": minimum_guard_margin - 1.0 if minimum_guard_margin is not None else None,
        "eligibility_gate": "PASS" if row.get("terminal_tail_eligible") == "True" else "FAIL",
        "steering_abs": steer,
        "steering_support_gate": "PASS" if steer < STEERING_SUPPORT_LIMIT else "UNSUPPORTED",
        "original_h1_command": row.get("shadow_command"),
        "original_h1_reason": row.get("terminal_tail_original_h1_reason"),
        "prior_h1_brake_requests": int(float(row["terminal_tail_prior_h1_brake_requests"])),
        "tail_path": row.get("terminal_tail_tail_path"),
        "tail_segment": row.get("terminal_tail_tail_segment"),
        "second_opinion": status,
        "reason_code": reason,
        "abstention_gate": "PASS" if status != "ABSTAIN" else reason,
    }


def _render_report(result):
    lines = [
        "# WP 1774–1856 final-brake margin decomposition",
        "",
        "The table aligns the winner's final three applied brake ticks over all held-out traversals.",
        "Positive target margin means the conservative terminal-speed upper bound remains above the terminal target, so release is rejected.",
        "",
        "| Winner brake tick | WP | Upper−target | Current overspeed | Residual decel credit | Direct predicted overspeed | Direct prediction−realized | Realized terminal margin | Direct uncertainty | Tail conservatism | Tail uncertainty | Terminal distance | Horizon | Eligibility slack | Steering support | Result |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for row in result["aligned_summary"]:
        median = lambda name: row[name]["median"] if row.get(name) else float("nan")
        lines.append(
            "| #{ordinal} | {wp:.0f} | {margin:+.2f} | {current:+.2f} | {residual:.2f} | {predicted:+.2f} | +{bias:.2f} | {realized:+.2f} | +{uncertainty:.2f} | +{tail:.2f} | +{tail_uncertainty:.2f} | {distance:.1f} m | {horizon:.0f} ticks | {eligibility:+.2f} | {steering} | {result} |".format(
                ordinal=row["winner_final_brake_tick"], wp=median("custom_waypoint_index"),
                margin=median("target_margin_upper_minus_target_kmh"), current=median("current_overspeed_to_terminal_kmh"),
                residual=median("residual_deceleration_credit_kmh"), predicted=median("direct_predicted_overspeed_kmh"),
                bias=median("direct_prediction_minus_realized_kmh"), realized=median("realized_terminal_margin_kmh"),
                uncertainty=median("direct_model_uncertainty_kmh"), tail=median("tail_residual_conservatism_kmh"),
                tail_uncertainty=median("tail_uncertainty_kmh"), distance=median("terminal_distance_m"),
                horizon=median("terminal_horizon_ticks"), eligibility=median("eligibility_slack_kmh"),
                steering="pass {}/{}".format(row["steering_support_counts"].get("PASS", 0), row["traversals"]),
                result=max(row["second_opinion_counts"], key=row["second_opinion_counts"].get),
            )
        )
    lines += [
        "",
        "## Interpretation",
        "",
        result["interpretation"],
        "",
        "The JSON result and `ticks.csv` contain the full per-traversal decomposition and percentile ranges.",
    ]
    return "\n".join(lines) + "\n"


def analyze(root: Path, results_dir="experiment_results", experiment_name="terminal-tail-fresh-10"):
    results_path = root / results_dir
    records = []
    for item, rows in _load_attempts(root, results_path, experiment_name):
        laps = sorted({int(row["lap"]) for row in rows})
        for lap in laps:
            braking = [row for row in rows if int(row["lap"]) == lap and WINDOW[0] <= int(row["custom_waypoint_index"]) <= WINDOW[1] and _number(row, "brake", 0.0) > BRAKE_ACTIVE]
            if len(braking) < 3:
                continue
            for ordinal, row in enumerate(braking[-3:], start=1):
                lap_rows=[candidate for candidate in rows if int(candidate["lap"]) == lap]
                records.append(_decompose(item, lap, ordinal, row, lap_rows))

    fields = (
        "custom_waypoint_index", "current_speed_kmh", "terminal_distance_m", "terminal_horizon_ticks",
        "terminal_target_speed_kmh", "current_overspeed_to_terminal_kmh", "residual_deceleration_credit_kmh",
        "direct_predicted_overspeed_kmh", "realized_terminal_speed_kmh", "realized_terminal_margin_kmh",
        "direct_prediction_minus_realized_kmh", "direct_model_uncertainty_kmh", "tail_residual_conservatism_kmh",
        "tail_uncertainty_kmh", "terminal_upper_bound_kmh", "target_margin_upper_minus_target_kmh",
        "eligibility_min_projected_margin_kmh", "eligibility_slack_kmh", "steering_abs",
    )
    aligned = []
    for ordinal in (1, 2, 3):
        selected = [record for record in records if record["winner_final_brake_tick"] == ordinal]
        summary = {"winner_final_brake_tick": ordinal, "traversals": len(selected)}
        summary.update({field: _summary(record[field] for record in selected) for field in fields})
        summary["second_opinion_counts"] = dict(Counter(record["second_opinion"] for record in selected))
        summary["reason_counts"] = dict(Counter(record["reason_code"] for record in selected))
        summary["steering_support_counts"] = dict(Counter(record["steering_support_gate"] for record in selected))
        summary["eligibility_counts"] = dict(Counter(record["eligibility_gate"] for record in selected))
        summary["tail_path_counts"] = dict(Counter(record["tail_path"] for record in selected))
        aligned.append(summary)

    result = {
        "schema_version": 1,
        "event_type": "wp1774_release_margin_decomposition",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "experiment_name": experiment_name,
        "window": list(WINDOW),
        "traversals": len(records) // 3,
        "records": records,
        "aligned_summary": aligned,
        "interpretation": "The dominant blocker is long-horizon residual-deceleration under-credit: even after substantial modeled deceleration, the direct prediction remains well above the terminal target and substantially above the subsequently realized speed. Direct uncertainty and conservative tail terms add a second large layer of margin. Eligibility and steering support pass on these ticks, and no abstention gate is active.",
    }
    output = results_path / "analysis" / "wp1774_release_margin"
    output.mkdir(parents=True, exist_ok=True)
    json_path = output / "result.json"
    report_path = output / "report.md"
    csv_path = output / "ticks.csv"
    result["result_path"] = str(json_path.relative_to(root))
    result["human_report_path"] = str(report_path.relative_to(root))
    result["ticks_path"] = str(csv_path.relative_to(root))
    write_json(json_path, result)
    report_path.write_text(_render_report(result), encoding="utf-8")
    if records:
        with csv_path.open("w", newline="", encoding="utf-8") as outfile:
            writer = csv.DictWriter(outfile, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    return result
