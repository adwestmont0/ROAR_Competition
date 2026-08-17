# Initial qualifying braking timing sweep

Opening control: 321.600 s. Closing control: 321.750 s. Control midpoint: 321.675 s.
All non-infrastructure attempts are recorded in `result.json`; transient CARLA
preflight failures are excluded from controller outcomes.

## Full-race outcomes

| Event | Onset | Release | Outcome | Time | Delta vs control midpoint |
|---|---:|---:|---|---:|---:|
| WP2501 | 0 | -1 | finished | 321.600 | -0.075 |
| WP2501 | 0 | -2 | finished | 321.400 | -0.275 |
| WP2501 | +1 | -1 | finished | 321.350 | -0.325 |
| WP2501 | +1 | -2 | finished | 321.450 | -0.225 |
| WP1232 | 0 | -1 | finished | 321.400 | -0.275 |
| WP1232 | 0 | -2 | collision at WP1409 | 67.000 | n/a |
| WP1232 | +1 | -1 | finished | 321.650 | -0.025 |
| WP1232 | +1 | -2 | collision at WP1408 | 172.050 | n/a |
| WP1781 | 0 | -1 | finished | 321.750 | +0.075 |
| WP1781 | 0 | -2 | finished | 321.700 | +0.025 |
| WP1781 | +1 | -1 | finished | 322.400 | +0.725 |
| WP1781 | +1 | -2 | finished | 322.200 | +0.525 |

## Persistent timing effects

Values are treatment-minus-control seconds from raw brake onset. Negative is faster.

| Event/config | Next brake | +25 m | +50 m | +100 m | +200 m |
|---|---:|---:|---:|---:|---:|
| WP2501 0/-1 | -0.025 | -0.067 | -0.083 | -0.050 | -0.038 |
| WP2501 0/-2 | -0.050 | -0.050 | -0.117 | -0.117 | -0.113 |
| WP2501 +1/-1 | -0.050 | +0.000 | -0.050 | -0.017 | -0.013 |
| WP2501 +1/-2 | +0.000 | +0.000 | -0.100 | -0.067 | -0.013 |
| WP1232 0/-1 | -0.067 | -0.117 | -0.100 | -0.083 | -0.067 |
| WP1232 0/-2 | unavailable | -0.300 | -0.250 | -0.200 | -0.150 |
| WP1232 +1/-1 | -0.033 | -0.100 | -0.050 | -0.050 | -0.033 |
| WP1232 +1/-2 | -0.100 | -0.050 | +0.000 | -0.050 | -0.050 |
| WP1781 0/-1 | -0.017 | -0.042 | -0.075 | -0.092 | -0.058 |
| WP1781 0/-2 | -0.033 | -0.108 | -0.142 | -0.142 | -0.125 |
| WP1781 +1/-1 | +0.133 | +0.058 | +0.058 | +0.075 | +0.158 |
| WP1781 +1/-2 | +0.133 | +0.092 | +0.092 | +0.125 | +0.192 |

## Initial decisions

- WP2501 `0/-2` and `+1/-1` are promising but need repeats. `0/-2` has the
  stronger persistent +200 m gain; `+1/-1` has the fastest exploratory race time.
- WP1232 `0/-1` is promising and sits immediately before a repeatable crash
  boundary at release -2. Onset delay is not useful here.
- WP1781 onset delay is decisively slower. Release-only timing gains through
  +200 m do not improve the completed race time, so this event is not retained.
- No independent changes should be combined until repeat runs validate WP2501
  and WP1232 candidates.
