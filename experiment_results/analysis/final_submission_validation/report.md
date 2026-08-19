# Final submission validation gate

Date: 2026-08-19 UTC

Candidate freeze: `fca8332`
Resolved experiment ID:
`qualifying_brake.two_event_combination=1__qualifying_brake.wp2501_left_foot_percent=10__qualifying_brake.wp794_lap3_final_tick=1__runtime.low_bandwidth_sensor_mode=1--a9af10edf771`

## Reproducibility checks

- Five-run forced-clean schedule resolved successfully.
- The generated source diff was identical to the committed known-good export.
- All generated controller sources compiled.
- 34 relevant harness/config unit tests passed.
- The frozen candidate contains WP1232 release -1, WP2501 release -2,
  lap-3 WP794 final-tick suppression, and WP2501 0.10 throttle overlap on the
  final five applied brake ticks. No rejected Spring parameter was enabled.

## Online validation status

No controller-valid attempt ran. The first AWS SSM restart completed
successfully, stopped and restarted the Windows CARLA processes, and reported
port 2000 listening. CARLA then failed all 48 readiness probes over 249.2 s
with `rpc::rpc_error during call in function get_sensor_token`. The harness
recorded `carla_readiness_timeout` and no attempt ID.

The second forced-clean cycle showed the same readiness pattern and was
manually stopped before its timeout. This avoids misclassifying infrastructure
failure as controller unreliability.

Checkpoint/provenance:
`experiment_results/causal/7d72b126d3dc82b9cefd/`.

## Required follow-up

Restore CARLA sensor RPC readiness, then resume:

```bash
python3 -m experiments.harness.cli causal \
  experiments/configs/qualifying_brake_final_validation.json
```

The checkpoint correctly remains at schedule index 0. Submission reliability
is therefore not newly established by this gate; the prior validated component
campaigns remain the current evidence.
