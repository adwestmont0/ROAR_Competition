import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

from .core import (
    dry_run,
    execute_with_recovery,
    experiment_id,
    load_ledger,
    read_json,
    repo_root,
    run_plan,
)
from .evaluation import dry_run_candidates, evaluate_candidates
from .reliability import dry_run_reliability, run_reliability_campaign
from .causal import dry_run_causal, run_causal_campaign
from experiments.search.adapter import run_search
from experiments.analysis.velocity_profile import analyze as analyze_velocity_profile
from experiments.analysis.velocity_profile_v2 import analyze as analyze_velocity_profile_v2
from experiments.analysis.counterfactual_opportunity import analyze as analyze_counterfactual
from experiments.analysis.shadow_disagreement import analyze as analyze_shadow_disagreement
from experiments.analysis.shadow_calibration import analyze as analyze_shadow_calibration
from experiments.analysis.residual_deceleration import analyze as analyze_residual_deceleration
from experiments.analysis.terminal_release import analyze as analyze_terminal_release
from experiments.analysis.terminal_prediction_v2 import analyze as analyze_terminal_prediction_v2
from experiments.analysis.terminal_tail_bridge import analyze as analyze_terminal_tail_bridge
from experiments.analysis.terminal_live_validation import analyze as analyze_terminal_live_validation
from experiments.analysis.wp1774_release_margin import analyze as analyze_wp1774_release_margin
from experiments.analysis.long_horizon_residual import analyze as analyze_long_horizon_residual
from experiments.analysis.residual_uncertainty import analyze as analyze_residual_uncertainty
from experiments.analysis.residual_regimes import analyze as analyze_residual_regimes


def load_inputs(config_path: Path) -> tuple:
    root = repo_root()
    config = read_json(config_path.resolve())
    registry = read_json(root / "experiments" / "parameters.json")
    return root, config, registry


def print_results(results: Any) -> None:
    print(json.dumps(results, indent=2, sort_keys=True))


def print_human_reports(results: Dict[str, Any], root: Path) -> None:
    for candidate in results.get("candidate_results", []):
        report_path = candidate.get("human_report_path")
        if not report_path:
            continue
        path = root / report_path
        if not path.exists():
            continue
        print("\n" + "=" * 72, file=sys.stderr)
        print("Human report: " + report_path, file=sys.stderr)
        print("=" * 72, file=sys.stderr)
        print(path.read_text(encoding="utf-8"), file=sys.stderr)


def print_human_report(result: Dict[str, Any], root: Path) -> None:
    path_value = result.get("human_report_path")
    if not path_value:
        return
    path = root / path_value
    if path.exists():
        print("\n" + "=" * 72, file=sys.stderr)
        print("Human report: " + path_value, file=sys.stderr)
        print("=" * 72, file=sys.stderr)
        print(path.read_text(encoding="utf-8"), file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description="ROAR controlled experiment harness")
    subparsers = parser.add_subparsers(dest="command", required=True)

    for command in ("dry-run", "run"):
        child = subparsers.add_parser(command)
        child.add_argument("config", type=Path)

    evaluate = subparsers.add_parser("evaluate")
    evaluate.add_argument("config", type=Path)
    evaluate.add_argument("--dry-run", action="store_true")
    evaluate.add_argument("--no-resume", action="store_true")
    evaluate.add_argument("--json-only", action="store_true")

    search = subparsers.add_parser("search")
    search.add_argument("config", type=Path)
    search.add_argument("--dry-run", action="store_true")
    search.add_argument("--no-resume", action="store_true")
    search.add_argument("--json-only", action="store_true")

    reliability = subparsers.add_parser("reliability")
    reliability.add_argument("config", type=Path)
    reliability.add_argument("--dry-run", action="store_true")
    reliability.add_argument("--no-resume", action="store_true")
    reliability.add_argument("--json-only", action="store_true")

    causal = subparsers.add_parser("causal")
    causal.add_argument("config", type=Path)
    causal.add_argument("--dry-run", action="store_true")
    causal.add_argument("--no-resume", action="store_true")
    causal.add_argument("--json-only", action="store_true")

    profile = subparsers.add_parser("velocity-profile")
    profile.add_argument("--results-dir", default="experiment_results")
    profile.add_argument("--bin-m", type=float, default=20.0)
    profile.add_argument("--interval-m", type=float, default=100.0)
    profile.add_argument("--json-only", action="store_true")

    profile_v2 = subparsers.add_parser("velocity-profile-v2")
    profile_v2.add_argument("--results-dir", default="experiment_results")
    profile_v2.add_argument("--spacing-m", type=float, default=5.0)
    profile_v2.add_argument("--interval-m", type=float, default=100.0)
    profile_v2.add_argument("--json-only", action="store_true")

    counterfactual = subparsers.add_parser("counterfactual-opportunity")
    counterfactual.add_argument("--results-dir", default="experiment_results")
    counterfactual.add_argument("--extension-m", type=float, default=300.0)
    counterfactual.add_argument("--json-only", action="store_true")

    disagreement = subparsers.add_parser("shadow-disagreement")
    disagreement.add_argument("--results-dir", default="experiment_results")
    disagreement.add_argument("--json-only", action="store_true")

    calibration = subparsers.add_parser("shadow-calibration")
    calibration.add_argument("--results-dir", default="experiment_results")
    calibration.add_argument("--json-only", action="store_true")

    residual = subparsers.add_parser("residual-deceleration")
    residual.add_argument("--results-dir", default="experiment_results")
    residual.add_argument("--json-only", action="store_true")

    terminal = subparsers.add_parser("terminal-release")
    terminal.add_argument("--results-dir", default="experiment_results")
    terminal.add_argument("--json-only", action="store_true")

    terminal_v2 = subparsers.add_parser("terminal-prediction-v2")
    terminal_v2.add_argument("--results-dir", default="experiment_results")
    terminal_v2.add_argument("--json-only", action="store_true")

    tail_bridge = subparsers.add_parser("terminal-tail-bridge")
    tail_bridge.add_argument("--results-dir", default="experiment_results")
    tail_bridge.add_argument("--json-only", action="store_true")

    live_validation = subparsers.add_parser("terminal-live-validation")
    live_validation.add_argument("--results-dir", default="experiment_results")
    live_validation.add_argument("--tolerance", type=float, default=1e-9)
    live_validation.add_argument("--experiment-name")

    wp1774 = subparsers.add_parser("wp1774-release-margin")
    wp1774.add_argument("--results-dir", default="experiment_results")
    wp1774.add_argument("--experiment-name", default="terminal-tail-fresh-10")

    long_residual = subparsers.add_parser("long-horizon-residual")
    long_residual.add_argument("--results-dir", default="experiment_results")

    residual_uncertainty = subparsers.add_parser("residual-uncertainty")
    residual_uncertainty.add_argument("--results-dir", default="experiment_results")

    residual_regimes = subparsers.add_parser("residual-regimes")
    residual_regimes.add_argument("--results-dir", default="experiment_results")

    run_one = subparsers.add_parser("run-one")
    run_one.add_argument("config", type=Path)
    run_one.add_argument("--experiment-id", required=True)

    rerun = subparsers.add_parser("rerun")
    rerun.add_argument("--attempt-id", required=True)
    rerun.add_argument("--ledger", type=Path, default=Path("experiment_results/ledger.jsonl"))

    listing = subparsers.add_parser("list")
    listing.add_argument("--ledger", type=Path, default=Path("experiment_results/ledger.jsonl"))

    args = parser.parse_args()
    root = repo_root()

    if args.command == "list":
        print_results(load_ledger((root / args.ledger).resolve()))
        return 0

    if args.command == "rerun":
        ledger = load_ledger((root / args.ledger).resolve())
        matches = [item for item in ledger if item["attempt_id"] == args.attempt_id]
        if len(matches) != 1:
            raise ValueError(f"Expected one ledger entry for {args.attempt_id}, found {len(matches)}")
        previous = matches[0]
        result_path = root / previous["stdout_path"]
        resolved_path = result_path.parent / "resolved_config.json"
        resolved = read_json(resolved_path)
        config: Dict[str, Any] = {
            "name": resolved["experiment_name"],
            "baseline_commit": resolved["baseline_commit"],
            "results_dir": str((root / args.ledger).resolve().parent.relative_to(root)),
            "carla": resolved["carla"],
            "execution": resolved["execution"],
            "recovery": resolved.get("recovery"),
            "parameters": resolved["parameters"],
        }
        registry = read_json(root / "experiments" / "parameters.json")
        results, stop_reason = execute_with_recovery(
            config,
            resolved["parameters"],
            bool(resolved["is_control"]),
            registry,
            root,
        )
        print_results(results)
        if stop_reason:
            print(f"Rerun stopped: {stop_reason}", file=sys.stderr)
            return 1
        return 0

    if args.command == "velocity-profile":
        result = analyze_velocity_profile(root, args.results_dir, args.bin_m, args.interval_m)
        print_results(result)
        if not args.json_only:
            print_human_report(result, root)
        return 0

    if args.command == "velocity-profile-v2":
        result = analyze_velocity_profile_v2(root, args.results_dir, args.spacing_m, args.interval_m)
        print_results(result)
        if not args.json_only:
            print_human_report(result, root)
        return 0

    if args.command == "counterfactual-opportunity":
        result = analyze_counterfactual(root, args.results_dir, args.extension_m)
        print_results(result)
        if not args.json_only:
            print_human_report(result, root)
        return 0

    if args.command == "shadow-disagreement":
        result = analyze_shadow_disagreement(root, args.results_dir)
        print_results(result)
        if not args.json_only:
            print_human_report(result, root)
        return 0

    if args.command == "shadow-calibration":
        result = analyze_shadow_calibration(root, args.results_dir)
        print_results(result)
        if not args.json_only:
            print_human_report(result, root)
        return 0

    if args.command == "residual-deceleration":
        result = analyze_residual_deceleration(root, args.results_dir)
        print_results(result)
        if not args.json_only:
            print_human_report(result, root)
        return 0

    if args.command == "terminal-release":
        result = analyze_terminal_release(root, args.results_dir)
        print_results(result)
        if not args.json_only:
            print_human_report(result, root)
        return 0

    if args.command == "terminal-prediction-v2":
        result = analyze_terminal_prediction_v2(root, args.results_dir)
        print_results(result)
        if not args.json_only:
            print_human_report(result, root)
        return 0

    if args.command == "terminal-tail-bridge":
        result = analyze_terminal_tail_bridge(root, args.results_dir)
        print_results(result)
        if not args.json_only:
            print_human_report(result, root)
        return 0

    if args.command == "terminal-live-validation":
        print_results(analyze_terminal_live_validation(root,args.results_dir,args.tolerance,args.experiment_name))
        return 0

    if args.command == "wp1774-release-margin":
        result=analyze_wp1774_release_margin(root,args.results_dir,args.experiment_name)
        print_results(result)
        print_human_report(result,root)
        return 0

    if args.command == "long-horizon-residual":
        result=analyze_long_horizon_residual(root,args.results_dir)
        print_results(result)
        print_human_report(result,root)
        return 0

    if args.command == "residual-uncertainty":
        result=analyze_residual_uncertainty(root,args.results_dir)
        print_results(result)
        print_human_report(result,root)
        return 0

    if args.command == "residual-regimes":
        result=analyze_residual_regimes(root,args.results_dir)
        print_results(result)
        print_human_report(result,root)
        return 0

    root, config, registry = load_inputs(args.config)
    if args.command == "evaluate":
        if args.dry_run:
            print_results(dry_run_candidates(config, registry, root))
        else:
            results = evaluate_candidates(config, registry, root, resume=not args.no_resume)
            print_results(results)
            sys.stdout.flush()
            if not args.json_only:
                print_human_reports(results, root)
    elif args.command == "search":
        results = run_search(config, registry, root, args.dry_run, not args.no_resume)
        print_results(results)
        sys.stdout.flush()
        if not args.dry_run and not args.json_only:
            print_human_reports({"candidate_results": results.get("candidate_results", [])}, root)
    elif args.command == "reliability":
        if args.dry_run:
            print_results(dry_run_reliability(config, registry, root))
        else:
            result = run_reliability_campaign(
                config, registry, root, not args.no_resume,
                progress_callback=lambda message: print(message, file=sys.stderr, flush=True),
            )
            print_results(result)
            sys.stdout.flush()
            if not args.json_only:
                print_human_report(result, root)
    elif args.command == "causal":
        if args.dry_run:
            print_results(dry_run_causal(config, registry, root))
        else:
            result = run_causal_campaign(
                config, registry, root, not args.no_resume,
                progress_callback=lambda message: print(message, file=sys.stderr, flush=True),
            )
            print_results(result)
            sys.stdout.flush()
            if not args.json_only:
                print_human_report(result, root)
    elif args.command == "dry-run":
        print_results(dry_run(config, registry, root))
    elif args.command == "run":
        print_results(run_plan(config, registry, root))
    elif args.command == "run-one":
        print_results(
            run_plan(config, registry, root, only_experiment_id=args.experiment_id)
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"{type(error).__name__}: {error}", file=sys.stderr)
        raise
