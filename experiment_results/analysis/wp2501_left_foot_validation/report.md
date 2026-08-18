# WP2501 left-foot-braking 0.10 validation

Campaign: `qualifying-brake-wp2501-left-foot-validation`

Valid schedule: C T T C T T C T C. All nine valid attempts finished without
collision. Infrastructure-only stale-synchronous-mode retries are excluded.

| Attempt | Mode | Raw time (s) | Nearby control (s) | Adjusted delta (s) |
|---|---:|---:|---:|---:|
| `f2c524be` | Control | 321.250 | — | — |
| `89553b85` | Treatment | 321.050 | 321.225 | -0.175 |
| `eb0d8a30` | Treatment | 321.100 | 321.225 | -0.125 |
| `3e31c44a` | Control | 321.200 | — | — |
| `50987091` | Treatment | 321.100 | 321.225 | -0.125 |
| `4ce4acd1` | Treatment | 321.050 | 321.225 | -0.175 |
| `259af7c5` | Control | 321.250 | — | — |
| `4c725de0` | Treatment | 321.100 | 321.250 | -0.150 |
| `61aad7c3` | Control | 321.250 | — | — |

Treatment adjusted mean was -0.150 s, median -0.150 s, sample standard
deviation 0.025 s, and range -0.175 to -0.125 s. Treatments finished 5/5;
controls finished 4/4. There were no collisions.

All 15 treatment and all 12 control traversals naturally selected the 41 raw /
39 applied brake-tick regime. Every treatment traversal had exactly five overlap
ticks at throttle 0.10; controls had none. No 42/40 traversal occurred.

On the matched 41/39 population, treatment-control means were:

- release speed: +0.285 km/h
- minimum speed: +0.796 km/h
- acceleration at the final braking transition: approximately +0.38 to
  +0.40 m/s^2 (less net deceleration)
- first full-throttle timing: 0.000 s
- +25 m time: 0.000 s
- +50 m time: -0.043 s
- +100 m time: 0.000 s
- +200 m time: -0.005 s (ten comparable non-terminal traversals)
- time to the next major brake: -0.005 s

Brake counts, release waypoint, and first-full-throttle timing were unchanged.
RPM was unavailable. The supported mechanism is reduced net braking during the
overlap, not drivetrain priming or a changed winner brake regime.

Conclusion: the observed full-run improvement was deterministic in this sample,
but the local advantage did not persist materially through +100/+200 m. Under
the campaign's stated KEEP criteria, 0.10 should not yet be promoted into the
known-good controller, and WP1232 should not be started from this result.
