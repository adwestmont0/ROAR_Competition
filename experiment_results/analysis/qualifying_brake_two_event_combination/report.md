# Two-event qualifying brake combination validation

Configuration: WP1232 onset 0 / release early 1; WP2501 onset 0 / release early 2. Stable winner baseline `ee30402`; no other controller parameters changed.

## Outcome

The combination is deterministically faster: 5/5 treatments finished, with zero collisions, a mean nearby-control-adjusted lap delta of **-0.410 s**, median **-0.400 s**, and sample standard deviation **0.095 s**. Both interventions retained downstream local gains through 200 m.

Relative to the independently measured sum (-0.710 s), the interaction is:

`-0.410 - (-0.430 + -0.280) = +0.300 s`

Thus 0.300 s of the theoretical additive gain was lost. This is a **destructive interaction relative to perfect additivity**, although the combined controller remains materially and repeatably faster than control.

## Attempt sequence

| Sequence | Type | Raw lap (s) | Nearby control reference (s) | Adjusted delta (s) |
|---:|---|---:|---:|---:|
| 1 | Control | 321.800 | — | — |
| 2 | Combined | 321.400 | 321.775 | -0.375 |
| 3 | Combined | 321.500 | 321.775 | -0.275 |
| 4 | Control | 321.750 | — | — |
| 5 | Combined | 321.350 | 321.850 | -0.500 |
| 6 | Combined | 321.350 | 321.850 | -0.500 |
| 7 | Control | 321.950 | — | — |
| 8 | Combined | 321.450 | 321.850 | -0.400 |
| 9 | Control | 321.750 | — | — |

Adjusted range: -0.500 to -0.275 s. Raw treatment mean: 321.410 s. Four stale-synchronous-mode preflight attempts were infrastructure retries and are excluded from controller attempts.

## Event-local telemetry

Values are treatment means across five attempts; each attempt contains three event traversals. Time deltas use the adjacent before/after controls. Event-entry delta is averaged across all three laps, so it includes gains accumulated on earlier laps.

| Event | Entry-time delta | Next-brake delta | +25 m | +50 m | +100 m | +200 m | Entry speed | Minimum speed | Release speed |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| WP1232 | -0.150 s | -0.088 s | -0.203 s | -0.095 s | -0.088 s | -0.073 s | 254.740 km/h | 148.988 km/h | 179.190 km/h |
| WP2501 | -0.238 s | -0.035 s | -0.143 s | -0.237 s | -0.197 s | -0.203 s | 257.544 km/h | 90.865 km/h | 130.325 km/h |

WP1232 averaged 24.467 raw winner brake ticks, 23.467 applied ticks, and exactly one suppressed tick per traversal. The winner re-requested after suppression in 7/15 traversals; all recorded re-requests were immediate. In 8/15 traversals it stayed released. Despite compensation, its mean gain persisted to the next brake and through 200 m.

WP2501 averaged 41.133 raw winner brake ticks, 39.133 applied ticks, and exactly two suppressed ticks per traversal. It produced 17 immediate raw re-requests across 15 traversals (some traversals re-requested after both omissions). Nevertheless, the intervention retained -0.197 s at +100 m and -0.203 s at +200 m. Compensation therefore did not erase its local advantage.

Across the campaign there were 45 suppressed brake ticks (15 at WP1232 and 30 at WP2501) and 24 recorded compensating raw winner requests.

## Interpretation

Both local effects survive together, but their independently measured full-lap gains do not add. WP1232 retains nearly its prior +100/+200 m persistence (-0.088/-0.073 s versus -0.087/-0.065 s independently). WP2501 is also locally strong (-0.197/-0.203 s versus -0.140/-0.143 s independently). Telemetry therefore does not show one intervention locally destroying the other. The missing 0.300 s appears downstream/outside these measured event horizons: the combined lap gain is statistically indistinguishable from the stronger WP1232 candidate alone (-0.410 versus -0.430 s), rather than the expected -0.710 s sum.

Recommendation: retain the combined configuration as a deterministic qualifying candidate, but label it non-additive. No further controller change or boundary search was performed.
