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
