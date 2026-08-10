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
from experiments.search.adapter import run_search


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
