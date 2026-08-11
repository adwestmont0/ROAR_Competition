# H1 shadow-execution validation

## Conclusion

H1 is accepted as non-interfering observational instrumentation. In the
interleaved forced-clean A/B campaign, both shadow-disabled control and
shadow-enabled treatment completed 5/5 runs. The shadow-enabled mean was only
0.030 s slower, substantially smaller than the approximately 0.12–0.13 s
within-cohort standard deviations.

## Interleaved A/B

| Metric | Shadow disabled | Shadow enabled | Enabled − disabled |
|---|---:|---:|---:|
| Finished runs | 5/5 | 5/5 | 0 |
| Collisions | 0 | 0 | 0 |
| Mean race time | 321.710 s | 321.740 s | +0.030 s |
| Median race time | 321.650 s | 321.750 s | +0.100 s |
| Race-time standard deviation | 0.119 s | 0.129 s | — |
| WP 1402–1409 mean speed | 172.437 km/h | 172.712 km/h | +0.275 km/h |
| WP 1402–1409 median speed | 172.684 km/h | 172.759 km/h | +0.075 km/h |

The arrival-speed mean is sensitive to two unusually slow control traversals.
An unpaired approximate 95% interval for the mean difference is
−0.127 to +0.677 km/h, so this sample does not establish a systematic speed
shift. The much smaller median difference supports that interpretation.

## Instrumentation integrity

- Shadow-enabled ticks populated successfully: 32,174 of 32,174.
- Shadow-disabled ticks reported `disabled_by_experiment`: 32,171 of 32,171.
- Applied/raw throttle or brake mismatches: 0.
- Brake-counter invariant failures: 0.
- Shadow-profile mean spatial error: 1.511 m.
- Shadow-profile maximum spatial error: 3.895 m.
- The validated CARLA action is dispatched before shadow evaluation.

## Preliminary campaign

The earlier non-interleaved five-run campaign completed 4/5. Its one collision
was at the established marginal area around custom WP 1410, at 175.29 km/h and
with a low impulse of 160.4. No shadow command leaked into applied control.
The subsequent paired A/B was introduced specifically to distinguish ordinary
baseline variability from shadow-execution effects; it found no reliability or
material timing disadvantage attributable to H1.

Authoritative machine-readable results are stored under campaign keys
`ba3e036212b021a74660` and `d884c90cf4c3c182c8f9`.
