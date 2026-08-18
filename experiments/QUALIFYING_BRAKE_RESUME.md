# Qualifying brake campaign resume note

- Branch: `qualifying-brake-search`
- HEAD: this resume-note commit (`git rev-parse HEAD` after checkout)
- Stable source baseline: `ee30402`
- Known-good qualifying export: `experiments/configs/qualifying_brake_two_event_combination.json`
  - WP1232 onset 0 / release early 1
  - WP2501 onset 0 / release early 2
  - mean adjusted delta -0.410 s; raw treatment mean 321.410 s
  - 5/5 finishes; zero collisions
- Combination report: `experiment_results/analysis/qualifying_brake_two_event_combination/report.md`
- Forensic report: `experiment_results/analysis/qualifying_brake_two_event_forensics/report.md`

## WP794 status

The clean rerun completed on 2026-08-18. Four treatments finished at 321.250/321.200/321.200/321.200 s; one treatment collided near WP1410 before lap-3 WP794. Mean completed adjusted delta was -0.100 s, with -0.138/-0.088 s at +100/+200 m. Every completed treatment suppressed exactly the sixth raw WP794 request and produced no re-request. **Keep as a qualifying candidate**, with campaign reliability recorded as 4/5.

Report: `experiment_results/analysis/qualifying_brake_wp794_lap3_validation_rerun2/report.md`.

## Resume

Natural-regime mining is complete. WP1781 15→14 is the strongest transition: it is winner-selected through an upstream waypoint/lookahead phase change and retains about 0.10–0.13 s through +100/+200 m. Direct WP1781 suppression did not reproduce that state.

Next recommended experiment: a one-tick-earlier throttle-reapplication A/B immediately after the existing WP1232 omitted final tick, measuring whether WP1781 naturally selects 14 ticks. Do not modify WP1781 directly.

Report: `experiment_results/analysis/natural_brake_regimes/report.md`.

Raw telemetry and the append-only ledger remain under ignored `experiment_results/` on this worktree. Compact completed campaign reports and analysis outputs should be force-tracked when committing this phase.
