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

`experiments/configs/qualifying_brake_wp794_lap3_validation.json` is prepared and dry-run verified. Its control is the known-good two-event combination; treatment adds only the lap-3 WP794 sixth/raw-final-brake-tick omission.

The campaign was stopped at user request on 2026-08-17. Completed usable attempts:

- control `20260817T083122.325857Z-8c032674`: 321.400 s
- treatment `20260817T083310.938591Z-351a3e2a`: 321.300 s

The following treatment was interrupted and is not a result: `20260817T083457.515642Z-47860190`. No WP794 keep/drop conclusion is justified.

## Resume

1. Re-run the complete WP794 interleaved configuration from the start; do not combine the partial sample with the new five-treatment decision set.
2. If WP794 validates, retain only as a candidate; do not extend its boundary.
3. Then run offline natural brake-regime mining across the seven documented zones, including the WP1781 natural-versus-direct comparison.

Raw telemetry and the append-only ledger remain under ignored `experiment_results/` on this worktree. Compact completed campaign reports and analysis outputs are force-tracked. No natural brake-regime mining has been performed yet.
