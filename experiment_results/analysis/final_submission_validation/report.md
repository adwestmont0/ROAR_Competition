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

## Online validation result

The campaign completed using the known-compatible environment:
`/home/ec2-user/venvs/roar/bin/python`, CARLA client 0.9.12, and CARLA server
0.9.12-dirty. Every attempt used an independent successful SSM CARLA restart.

| Run | Outcome | Time / terminal point |
|---:|---|---:|
| 1 | finished | 321.150 s |
| 2 | finished | 321.150 s |
| 3 | collision | 172.100 s, lap 2 WP1410 |
| 4 | finished | 321.200 s |
| 5 | finished | 321.300 s |

Completion was 4/5 (80%) with one collision. Completed mean was 321.200 s,
median 321.175 s, sample SD 0.071 s, and range 321.150-321.300 s. The collision
was in the repeatedly observed WP1409-1410 baseline failure region.

All completed runs applied exactly one WP1232 suppression, two WP2501
suppressions, and five WP2501 0.10 throttle/brake-overlap ticks per lap. Each
also applied exactly one lap-3 WP794 suppression. Brake remained 1.0 on every
WP2501 overlap tick. The collision run applied the expected lap-1 behavior and
lap-2 WP1232 suppression before terminating; it had not yet reached lap-2
WP2501 or lap-3 WP794.

Authoritative provenance and result:
`experiment_results/causal/7d72b126d3dc82b9cefd/`.

## Environment correction

The initial failed readiness attempt used system `/usr/bin/python3` with CARLA
client 0.9.15 against the custom 0.9.12-dirty server. That mismatch caused
`get_sensor_token` failures. It was not a server or controller failure. All
CARLA commands must use the ROAR virtual environment above.
