# Qualifying brake campaign resume note

- Branch: `qualifying-brake-search`
- HEAD: this resume-note commit (`git rev-parse HEAD` after checkout)
- Stable source baseline: `ee30402`
- Known-good qualifying export: `experiments/configs/qualifying_brake_known_good.json`
  - WP1232 onset 0 / release early 1
  - WP2501 onset 0 / release early 2
  - lap-3 WP794 final raw brake tick suppressed
  - WP2501 throttle 0.10 overlaps exactly the final five applied brake ticks;
    brake remains unchanged at 1.0
  - WP2501 left-foot validation: mean adjusted delta -0.150 s, median
    -0.150 s, SD 0.025 s; 5/5 finishes and zero collisions
- Combination report: `experiment_results/analysis/qualifying_brake_two_event_combination/report.md`
- Forensic report: `experiment_results/analysis/qualifying_brake_two_event_forensics/report.md`

## WP794 status

The clean rerun completed on 2026-08-18. Four treatments finished at 321.250/321.200/321.200/321.200 s; one treatment collided near WP1410 before lap-3 WP794. Mean completed adjusted delta was -0.100 s, with -0.138/-0.088 s at +100/+200 m. Every completed treatment suppressed exactly the sixth raw WP794 request and produced no re-request. **Keep as a qualifying candidate**, with campaign reliability recorded as 4/5.

Report: `experiment_results/analysis/qualifying_brake_wp794_lap3_validation_rerun2/report.md`.

## WP2501 left-foot braking status

Throttle 0.10 was causally validated and is part of the known-good export.
Position-aligned offline forensics classified it as a persistent downstream
phase gain. Its dominant effect is a lap-2 WP794 WP794/WP820 to WP793/WP819
decision shift with the same six brake ticks, followed by approximately 0.120 s
gain to WP1232.

Reports:

- `experiment_results/analysis/wp2501_left_foot_validation/report.md`
- `experiment_results/analysis/wp2501_left_foot_forensics/report.md`

## Historical resume note

Natural-regime mining is complete. WP1781 15→14 is the strongest transition: it is winner-selected through an upstream waypoint/lookahead phase change and retains about 0.10–0.13 s through +100/+200 m. Direct WP1781 suppression did not reproduce that state.

Next recommended experiment: a one-tick-earlier throttle-reapplication A/B immediately after the existing WP1232 omitted final tick, measuring whether WP1781 naturally selects 14 ticks. Do not modify WP1781 directly.

Report: `experiment_results/analysis/natural_brake_regimes/report.md`.

Raw telemetry and the append-only ledger remain under ignored `experiment_results/` on this worktree. Compact completed campaign reports and analysis outputs should be force-tracked when committing this phase.

## Final freeze and validation gate (2026-08-19)

- Frozen qualifying-controller commit: `fca8332` (`Promote validated WP2501 left-foot braking`).
- Final validation config: `experiments/configs/qualifying_brake_final_validation.json`.
- The dry run reproduced experiment ID
  `qualifying_brake.two_event_combination=1__qualifying_brake.wp2501_left_foot_percent=10__qualifying_brake.wp794_lap3_final_tick=1__runtime.low_bandwidth_sensor_mode=1--a9af10edf771`,
  identical to the known-good export.
- Relevant harness/config tests: 34/34 passed; all controller sources compiled.
- The five-run forced-clean campaign did not obtain a controller-valid attempt.
  Its first restart succeeded at the Windows/port level, but 48 readiness probes
  failed with `rpc::rpc_error during call in function get_sensor_token`; the
  campaign was stopped during the second identical recovery cycle to avoid
  repeating infrastructure timeouts.
- Ignored provenance: `experiment_results/causal/7d72b126d3dc82b9cefd/`.
- Resume the exact campaign after CARLA sensor RPC readiness is restored:
  `python3 -m experiments.harness.cli causal experiments/configs/qualifying_brake_final_validation.json`.
  The checkpoint will retry schedule index 0; no vehicle attempt has been counted.

Submission contents and reproducibility instructions are in
`experiments/QUALIFYING_SUBMISSION_MANIFEST.md`.
