# WP2501 left-foot-braking initial sweep

## Scope and implementation

The current qualifying controller remained the control: WP1232 release-1, WP2501 release-2, and lap-3 WP794 release-1. The failed conditional WP1232 suppression was not enabled.

Treatment changed only applied throttle during WP2501 raw brake ordinals 35–39, the final five normally applied ticks before the existing ordinal-40/41 release suppressions. Brake and gear returned by the validated controller were preserved exactly. WP1232, WP1781, WP794, target speeds, lookahead, steering, and PID logic were untouched.

The runner independently clips and forwards throttle and brake to `carla.VehicleControl`. Offline translation preserved throttle 0.20 with brake 1.0, and runtime telemetry confirmed exactly five simultaneous positive-throttle/full-brake ticks per traversal.

## Initial results

Negative timing deltas are faster. Local values are treatment minus the mean of the immediately adjacent completed controls, matched by lap and physical horizon.

| Throttle overlap | Finish | Local segment delta | +100 m | +200 m | Release speed delta | Brake ticks | Collision |
|---:|---|---:|---:|---:|---:|---|---|
| 0.00 control | 5/6 | reference | reference | reference | reference | usually 41 raw / 39 applied; occasional 42/40 | 1 at WP1410 |
| 0.05 | No | ~0.000 s | ~0.000 s | **-0.025 s** | +0.388 km/h | 41 raw / 39 applied | WP1410, lap 2 |
| **0.10** | **Yes, 321.150 s** | **-0.025 s** | **-0.050 s** | **-0.025 s** | +3.310 km/h aggregate | 41 raw / 39 applied, all laps | None |
| 0.15 | No | ~0.000 s | ~0.000 s | **-0.025 s** | +0.583 km/h | 41 raw / 39 applied | WP1410, lap 2 |
| 0.20 | Yes, 321.400 s | **+0.075 s** | **+0.083 s** | **+0.088 s** | -4.986 km/h aggregate | 41.67 raw / 39.67 applied | None |

The 0.10 run was -0.150 s against both adjacent 321.300 s controls. The 0.20 run was neutral against its adjacent-control midpoint: 321.400 versus controls 321.500 and 321.300.

The opening control collision occurred before any treatment run. It and the 0.05/0.15 collisions all occurred at WP1410 around 172 s. Consequently, the two treatment collisions are preserved as failures but cannot be attributed to overlap throttle from one observation each. Six stale-synchronous-state infrastructure attempts were automatically recovered and excluded; they contain no driving result.

## Brake and mechanism checks

Applied overlap worked: telemetry recorded throttle at the requested magnitude concurrently with brake 1.0 for ordinals 35–39. The raw winner throttle remained -1.0. Gear was deliberately unchanged and remained the winner's braking command; no gear or RPM experiment was introduced. Engine RPM was not logged.

The intervention did not merely prime post-release acceleration:

- First full-throttle timing after release was unchanged at every level.
- At 0.05 and 0.15, brake counts remained 41 raw/39 applied and minimum/release speed increased modestly. This indicates overlap reduced net deceleration while preserving the brake command.
- At 0.10, the winner retained 41/39 on all laps and completed 0.150 s faster. The aggregate +3.310 km/h release delta is inflated by adjacent controls naturally choosing a 42/40 tick regime on lap 3. On laps 1–2, where both sides used 41/39, release improvements were approximately +0.20 and +0.79 km/h. The result is promising but phase-confounded and needs repeats.
- At 0.20, the winner naturally added an extra raw/applied brake tick on laps 2 and 3 (42/40), moved release to WP2561–2562, and lost 0.075–0.088 s locally. Although the override never altered brake directly, the changed physical trajectory crossed a controller decision boundary and provoked compensating braking.

Thus the observed effect is primarily a changed late-braking trajectory, not demonstrated earlier torque delivery after release. No claim about drivetrain priming is supported without RPM/engine-state telemetry.

## Decision

The only level deserving repeat validation is **0.10**:

- completed at 321.150 s;
- -0.150 s adjusted full-run result;
- retained the intended 41 raw/39 applied brake regime on all laps;
- local gains persisted through +200 m and the next brake;
- no collision.

This is exploratory evidence from one run, not validation. Repeat 0.10 three to five times with interleaved controls before testing WP1232. Do not retain 0.20; do not treat 0.05 or 0.15 as safe without repeats.

## Required answers

1. **Does simultaneous throttle+brake work in CARLA?** Yes. Both commands reached actuation and produced a measurable change while brake remained 1.0.
2. **Is there a useful WP2501 level?** 0.10 is promising; 0.20 is beyond the useful boundary. The collision status of 0.05/0.15 is unresolved because an opening control failed identically.
3. **Does it change braking or mainly post-release acceleration?** It changes net braking trajectory. Full-throttle timing was unchanged, and 0.20 provoked compensating brake behavior. Earlier effective torque delivery is not established.
4. **Which value deserves repeats?** 0.10 only.
5. **Proceed to WP1232?** No. Validate WP2501 0.10 first.
