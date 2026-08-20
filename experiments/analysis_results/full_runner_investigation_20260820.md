# Full-runner investigation archive — 2026-08-20

This is an analysis archive only. The frozen qualifying controller was not changed:

- controller commit: `ec73c4e19c94b8b731915150f89090f402890afb`
- frozen tag: `summer-2026-submission-final`
- qualifying branch: `summer-2026-submission`
- active modifications: WP1232 release -1; WP2501 release -2; lap-3 WP794 final-tick suppression; WP2501 LFB throttle 0.10 for five final applied-brake ticks.

No runner or infrastructure files were changed, and no experimental guard was promoted.

## Reference and full-runner results

The low-bandwidth reference (128x96, visualization off) produced approximately 321.05–321.15 s.

The exact frozen Summer-derived controller under the bundled full path (1024x768, visualization on) produced:

| Campaign | Attempts | Result |
|---|---:|---|
| Frozen baseline B1–B5 | 5 | 2 finishes: 321.600, 322.150 s; collisions at WP1404 (lap 2, 171.85 s), WP1403 (lap 3, 276.90 s), WP1403 (lap 1, 66.70 s) |
| WP794 lap-3 suppression rollback R1–R4 | 4 | 1 finish: 322.050 s; collisions WP662 (lap 1, 37.90 s), WP1405 (lap 1, 66.85 s), WP1405 (lap 2, 172.05 s) |

The baseline campaign has fewer than three finishes, so no full-runner timing/regime correlation was inferred. The repeated WP1403–1410 failure remains the dominant reliability problem. Removing the lap-3 WP794 suppression did not remove it; most rollback failures occurred before lap 3/WP794, and no WP847 conclusion was established.

## WP1232 guard work

The earlier 183 km/h candidate-speed guard was run under the full runner. Controls reached candidate speeds of about 184.6 and 186.3 km/h and collided near WP1405/WP1409; treatment candidate speeds stayed about 178.8–182.2 km/h, so the guard activated zero times. One treatment finished at 321.600 s and another reached WP847 before an unrelated collision. The guard therefore has no causal validation and was not retained.

The proposed 151.0 km/h event-minimum guard failed its offline feasibility gate. Event-wide minima separated known collision precursors (153.165–153.674 km/h) from completed traversals (at most 149.519 km/h), but those minima are only known after the release decision. Release-point speeds overlap (safe approximately 175–182 km/h; collision precursors approximately 181–185 km/h). Applying the guard at the candidate tick would alter safe traversals; waiting for the eventual minimum is too late. No CARLA treatment was run.

## Spring 2026 diagnostic

Advay Pure Pursuit commit `27d965d` was run in a separate checkout with the bundled 1024x768/full-visualization path. Only temporary, non-behavioral setup was used: the runner endpoint was pointed at the remote CARLA host, Windows-style asset aliases were supplied, and collision handling was instrumented to stop at the first major collision instead of entering the runner's automatic respawn loop. The checked-out `competition_runner.py` and `infrastructure.py` were not modified.

- Attempt 1: major collision at approximately WP522, 27.85 s; no WP1232/WP1781/WP2501 traversal.
- Attempt 2: completed in 320.450 s with no major collision.
- The third attempt was interrupted before launch; no result exists.

This is only 1/2 finished and is insufficient to classify Spring as full-runner robust or sensitive. The result is **INCONCLUSIVE**. The published ~320.4 s is reachable once, but full-runner reliability was not established. The Spring braking architecture remains a candidate for future consideration, not a submission change.

## Hypothesis disposition

Rejected or falsified:

- Ordinary simulator drift as the main explanation for low/full divergence; matched bracketing runs support runtime/control-phase sensitivity.
- Removing lap-3 WP794's final tick as a fix for WP1405–1410 reliability.
- The 183 km/h guard as a validated causal fix (it never activated in treatment).
- The 151 km/h minimum-speed guard as an implementable safe discriminator.

Unresolved:

- The precise first control-phase split that produces the full-runner WP1232/WP1403–1410 failure.
- Whether a legal event-local, phase-robust controller adaptation can recover low-runner performance without harming reliability.
- Spring controller reliability under a larger, clean full-runner sample.

## Artifact locations

Large telemetry remains intentionally ignored and local:

- `experiment_results/full_runner_baseline5/`
- `experiment_results/wp794_full_resolution_rollback/`
- `experiment_results/analysis/wp1232_rebrake_forensics/`
- `experiment_results/spring2026_full_runner_diagnostic/`

The archived logs and reports are retained for local forensic use. They are not added to Git because they contain generated telemetry/log volume. The frozen submission tag and controller files remain the source of truth for submission behavior.
