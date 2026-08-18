# WP794 lap-3 final-tick validation

Control: validated WP1232 release-1 + WP2501 release-2 combination. Treatment: control plus one omitted sixth raw/final WP794 brake request on lap 3.

| Candidate | Mean adjusted | Median | Stddev | Finishes | Collisions | +100 m | +200 m | Recommendation |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| Lap-3 WP794 final-tick omission | **-0.100 s** | **-0.0875 s** | **0.035 s** | 4/5 | 1 | **-0.138 s** | **-0.088 s** | **KEEP as candidate** |

Completed treatment laps were 321.250, 321.200, 321.200, and 321.200 s. Their adjacent-control-adjusted deltas were -0.100, -0.150, -0.075, and -0.075 s. Controls were 321.350, 321.350, 321.200, and 321.350 s.

The collision occurred at 172.05 s near WP1410, before the lap-3 WP794 intervention could execute. It is counted against campaign reliability but is not causally attributable to WP794.

Every completed treatment traversal requested six raw winner brake ticks, applied five, and suppressed exactly one. There were zero raw winner re-requests after suppression. Brake release moved to 0.250 s from onset; throttle reapplication remained at 0.300 s. Mean treatment entry speed was 192.243 km/h. Mean minimum and release speeds were 155.410 and 186.392 km/h.

Mean local time deltas were -0.025/-0.100/-0.138/-0.088 s at +25/+50/+100/+200 m. Time to the next major brake improved by 0.056 s. The effect therefore persists and is not merely a one-tick displacement.

The validated interventions remained intact in all four completed treatments:

- WP1232: 24.33 raw, 23.33 applied, exactly one suppression per traversal; re-request in 4/12 traversals.
- WP2501: 41 raw, 39 applied, exactly two suppressions per traversal; re-request in 12/12 traversals.

No boundary extension or additional CARLA experiment was performed.
