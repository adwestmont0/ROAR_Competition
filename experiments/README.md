# ROAR experiment harness

The harness runs controlled experiments from a disposable export of the pinned
baseline commit. It never edits or runs the active working tree.

## Safety and reproducibility

- The default baseline is commit `fa7ff6f`.
- Parameters must be registered in `experiments/parameters.json`.
- Substitutions require an exact baseline-text match and are applied only inside
  the disposable checkout.
- CARLA must report map `Carla/Maps/Monza` before a controller is started.
- Each attempt writes stdout, stderr, the exact source diff, resolved config,
  preflight data, telemetry, provenance, and a result document.
- Provenance includes Python, CARLA client/server, ROAR_PY module paths and Git
  commits when available, baseline/harness Git state, map, and control timestep.
- Every result is appended to `experiment_results/ledger.jsonl`.
- `experiment_id` identifies a parameter configuration. `attempt_id` identifies
  one particular repetition or retry.

## Commands

From the repository root:

```bash
python -m experiments.harness.cli dry-run experiments/configs/section3_mu_sweep.json
python -m experiments.harness.cli run experiments/configs/section3_mu_sweep.json
python -m experiments.harness.cli list
```

Evaluate named parameter configurations with repeated, bracketing controls:

```bash
python -m experiments.harness.cli evaluate \
  experiments/configs/section3_mu_candidates.json
```

Review all candidate substitutions without contacting CARLA:

```bash
python -m experiments.harness.cli evaluate \
  experiments/configs/section3_mu_candidates.json --dry-run
```

Candidate evaluation excludes infrastructure failures from controller metrics,
penalizes collision/timeout/exception/controller-hang outcomes, reports completion
and collision rates, aggregates finished total and section times, and emits a
normalized result under `experiment_results/candidate_summaries/`. The
`objective.control_adjusted_delta_seconds` value is negative when a candidate is
faster than its bracketing controls. An optimizer can treat each candidate result
as one configuration-level observation while retaining its underlying attempt IDs.
Each candidate also writes a same-named Markdown report containing a plain-English
decision, performance/control interpretation, collision clustering, section deltas,
acceptance failures, and a suggested isolation step when multiple changes fail.
The CLI prints these reports to stderr after emitting JSON on stdout. Use
`--json-only` to suppress terminal reports for machine-only workflows.

### Stable candidate API

Search code should depend on `CandidateEvaluator`, not on runner or CARLA details:

```python
from experiments.harness.evaluation import CandidateEvaluator

evaluator = CandidateEvaluator(config, parameter_registry, repository_root)
result = evaluator.evaluate(
    parameters={
        "throttle.section_mu.3": 3.50,
        "throttle.section_mu.4": 3.00,
    },
    repetitions=3,
    controls={"before": 1, "after": 1, "parameters": {}},
)
```

The normalized schema reports `accepted`, acceptance checks, controller outcome
rates, penalized and finished-time statistics, before/after/combined controls,
total and per-section control-adjusted deltas, attempt IDs, and provenance links.
Completed configuration identities are skipped by default; pass `resume=False`
or CLI `--no-resume` to force a fresh evaluation. A repository-local advisory
lock prevents concurrent processes from controlling the single CARLA server.

Arbitrary combinations of registered parameters are accepted. The registry now
contains every section-specific throttle `mu` override, while substitution remains
exact-match against the pinned winner source.

### Proposal adapter

The first API consumer is deliberately simple and lives outside the harness core.
It supports seeded random or deterministic Halton (quasi-random) proposals:

```bash
python -m experiments.harness.cli search \
  experiments/configs/multi_mu_search.json --dry-run

python -m experiments.harness.cli search \
  experiments/configs/multi_mu_search.json
```

The adapter proposes candidates, calls the stable evaluator, and ranks only
accepted results by control-adjusted time. It contains no Bayesian or adaptive
optimizer logic.

Run only one configuration by copying its ID from dry-run output:

```bash
python -m experiments.harness.cli run-one \
  experiments/configs/section3_mu_sweep.json \
  --experiment-id 'throttle.section_mu.3=3.3--...'
```

Rerun the exact resolved configuration of a prior attempt:

```bash
python -m experiments.harness.cli rerun --attempt-id '<attempt-id>'
```

## Classification

- `finished`, `collision`, and `timeout` come from runner telemetry.
- `infra_error` means preflight failed, CARLA became unavailable, or logs show a
  server/RPC failure.
- `controller_hang` means the client exceeded its watchdog while CARLA still
  passed a second preflight.
- `exception` is a non-infrastructure client or harness failure.

Infrastructure errors may be retried according to `max_infra_retries`. A valid
collision is never retried as an infrastructure failure.

When `recovery.provider` is `aws_ssm`, an infrastructure failure invokes the
registered Windows restart script through Systems Manager, records the command
ID/stdout/stderr as a separate ledger event, waits until CARLA responds on
`Carla/Maps/Monza`, and only then retries the same parameter configuration.
Controller hangs remain distinct and do not automatically trigger this recovery.

Known-good baseline runs can be inserted at the start, end, and after every N
primary experiments with the `controls` block. Sweep execution can stop after
configured consecutive collisions, infrastructure failures, controller hangs,
or any failed known-good control.
