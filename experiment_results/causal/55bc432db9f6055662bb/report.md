# WP1799 Single Brake Tick Causal

Every measurement used a forced-clean CARLA start.

## Baseline control

- Completion rate: 100.0%
- Collision rate: 0.0%
- Finished mean: 321.860 s

## Treatment

- Completion rate: 100.0%
- Collision rate: 0.0%
- Finished mean: 322.050 s

This campaign tests causal stability, not lap-time acceptance. See result.json for the authoritative record.

## Focused braking-event measurements

Aggregation status: **COMPLETE**.
Missing cohorts: none.

Incomplete cohorts: none.

| Metric | Control | Treatment | Treatment − control |
|---|---:|---:|---:|
| Entry speed (km/h) | 256.331 | 256.345 | 0.014 |
| Exit speed (km/h) | 221.620 | 221.860 | 0.240 |
| Minimum speed (km/h) | 173.439 | 169.778 | -3.660 |
| Brake onset (WP) | 1780.667 | 1781.933 | 1.267 |
| Brake release (WP) | 1804.533 | 1805.267 | 0.733 |
| Brake ticks | 15.000 | 14.733 | -0.267 |
| Throttle reapplied (WP) | 1804.533 | 1805.267 | 0.733 |
| Section time (s) | 12.507 | 12.530 | 0.023 |
| Total race time (s) | 321.860 | 322.050 | 0.190 |
| Collision rate | 0.000 | 0.000 | 0.000 |
| Speed at WP 1849 (km/h) | 173.215 | 170.063 | -3.152 |
| Speed at WP 2000 (km/h) | 189.633 | 188.514 | -1.119 |
| Speed at WP 2200 (km/h) | 242.721 | 242.065 | -0.656 |

- Actual total-race gain: -0.190
- Offline prediction at observed modest exit-speed delta: 0.014
- Actual minus offline prediction: -0.204
