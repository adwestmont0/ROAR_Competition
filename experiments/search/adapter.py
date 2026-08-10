import datetime as dt
import math
import random
import uuid
from pathlib import Path
from typing import Any, Dict, List

from experiments.harness.core import append_jsonl, write_json
from experiments.harness.evaluation import CandidateEvaluator


def _halton(index: int, base: int) -> float:
    value = 0.0
    fraction = 1.0
    while index > 0:
        fraction /= base
        value += fraction * (index % base)
        index //= base
    return value


def _primes(count: int) -> List[int]:
    output: List[int] = []
    candidate = 2
    while len(output) < count:
        if all(candidate % divisor for divisor in range(2, int(math.sqrt(candidate)) + 1)):
            output.append(candidate)
        candidate += 1
    return output


def propose(config: Dict[str, Any], registry: Dict[str, Any]) -> List[Dict[str, Any]]:
    search = config.get("search", {})
    samples = int(search.get("samples", 1))
    if samples < 1:
        raise ValueError("search.samples must be positive")
    mode = search.get("mode", "halton")
    if mode not in {"random", "halton"}:
        raise ValueError("search.mode must be random or halton")
    space = search.get("space", {})
    if not space:
        raise ValueError("search.space must not be empty")
    names = sorted(space)
    for name in names:
        if name not in registry:
            raise ValueError("Unregistered search parameter: " + name)
        bounds = space[name]
        if float(bounds["minimum"]) > float(bounds["maximum"]):
            raise ValueError("Invalid search bounds for " + name)
    fixed = dict(search.get("fixed_parameters", {}))
    unknown_fixed = sorted(set(fixed) - set(registry))
    if unknown_fixed:
        raise ValueError("Unregistered fixed parameters: " + ", ".join(unknown_fixed))
    generator = random.Random(search.get("seed", 0))
    bases = _primes(len(names))
    offset = int(search.get("halton_start_index", 1))
    candidates = []
    for sample_index in range(samples):
        parameters = dict(fixed)
        for dimension, name in enumerate(names):
            bounds = space[name]
            unit = (
                generator.random()
                if mode == "random"
                else _halton(offset + sample_index, bases[dimension])
            )
            minimum = float(bounds["minimum"])
            maximum = float(bounds["maximum"])
            value = minimum + unit * (maximum - minimum)
            precision = int(bounds.get("precision", 3))
            parameters[name] = round(value, precision)
        candidates.append(parameters)
    return candidates


def run_search(
    config: Dict[str, Any], registry: Dict[str, Any], root: Path,
    dry_run: bool = False, resume: bool = True,
) -> Dict[str, Any]:
    candidates = propose(config, registry)
    if dry_run:
        return {"mode": config.get("search", {}).get("mode", "halton"), "candidates": candidates}
    evaluation = config.get("evaluation", {})
    repetitions = int(evaluation.get("repetitions", 1))
    controls = evaluation.get("controls", {"before": 1, "after": 1})
    evaluator = CandidateEvaluator(config, registry, root)
    results = [
        evaluator.evaluate(
            parameters, repetitions, controls,
            candidate_name="search-%03d" % (index + 1), resume=resume,
        )
        for index, parameters in enumerate(candidates)
    ]
    ranked = sorted(
        [item for item in results if item["accepted"] and item["objective"]["control_adjusted_delta_seconds"] is not None],
        key=lambda item: item["objective"]["control_adjusted_delta_seconds"],
    )
    search_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + "-" + uuid.uuid4().hex[:8]
    output = {
        "schema_version": 1,
        "event_type": "candidate_search",
        "search_id": search_id,
        "experiment_name": config["name"],
        "search": config["search"],
        "candidate_results": results,
        "accepted_ranking": [
            {
                "rank": index + 1,
                "evaluation_key": item["evaluation_key"],
                "parameters": item["parameters"],
                "control_adjusted_delta_seconds": item["objective"]["control_adjusted_delta_seconds"],
            }
            for index, item in enumerate(ranked)
        ],
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    results_root = root / config.get("results_dir", "experiment_results")
    output_path = results_root / "searches" / (search_id + ".json")
    output["result_path"] = str(output_path.relative_to(root))
    write_json(output_path, output)
    append_jsonl(results_root / "ledger.jsonl", output)
    return output
