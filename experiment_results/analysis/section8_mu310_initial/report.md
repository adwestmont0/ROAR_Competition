# Section 8 mu=3.10 initial transfer gate

Campaign: `qualifying-brake-section8-mu310-initial`
Date: 2026-08-19 UTC

## Scope

Control retained WP1232 release -1, WP2501 release -2, lap-3 WP794 final-tick
suppression, and WP2501 0.10 left-foot braking. Treatment changed only the two
Section-8 mu references from 2.75 to 3.10. Section 8 is controller WP1944-2358;
WP2501 is downstream in Section 9.

## Attempts

| Attempt | Mode | Outcome | Time / terminal point |
|---|---|---|---:|
| `20260819T070312.411652Z-b5384c64` | control | finished | 321.450 s |
| `20260819T070452.889926Z-5bceddb2` | treatment | finished | 321.100 s |
| `20260819T070632.106213Z-fe18cead` | control | collision | WP1410, 172.050 s |
| `20260819T070737.218231Z-3aedfb81` | treatment | finished | 321.200 s |
| `20260819T070917.090338Z-36760b26` | control | finished | 321.100 s |

Infrastructure preflight/recovery attempts are retained in the ledger but are
not vehicle attempts.

Using the mean of the two completed bracket controls (321.275 s), treatment
deltas are -0.175 and -0.075 s: mean/median -0.125 s, sample SD 0.071 s, range
-0.175 to -0.075 s. The completed control spread is 0.350 s, so these lap-time
deltas are not independently persuasive.

## Causal gate

At physical anchors relative to S8 entry, treatment-control differences were:

| Anchor | Lap 1 | Lap 2 | Lap 3 mean |
|---|---:|---:|---:|
| +100 m | 0.000 | +0.025 | 0.000 s |
| +200 m | 0.000 | 0.000 | 0.000 s |
| midpoint (WP2152) | 0.000 | +0.025 | 0.000 s |
| S8 exit (WP2359) | 0.000 | +0.025 | 0.000 s |
| +100 m after S8 | 0.000 | 0.000 | 0.000 s |
| +200 m after S8 | 0.000 | 0.000 | 0.000 s |

The lap-2 +0.025 s value is half of the 0.05-s telemetry/control tick and is
not a gain. It vanishes by +100 m after S8.

The higher mu increased curvature-derived target/recommended speeds at only
8-11 sampled S8 waypoint decisions per lap (maximum recommended-speed increase
approximately 18.6-20.4 km/h). The applied command and winner decision branch
did not differ at any matched sampled S8 waypoint: both remained
`throttle_full`. Thus the changed constraint was non-binding.

WP2501 onset remained WP2500, lookahead WP2535, two release suppressions and
five 0.10 LFB ticks in every completed traversal. Treatment was 41 raw / 39
applied ticks on all six traversals. The later completed control also used
41/39 on all laps; the early control's lap-2/3 42/40 regime is run-phase
variation, not a repeatable treatment response.

## Decision

Classification: **D - NEUTRAL**.

The treatment finished 2/2 with no collision; controls finished 2/3 with one
known WP1410 control failure. There is no treatment collision signal, but also
no local gain, persistent downstream gain, or beneficial regime transition.
The validation extension was therefore not run and S8 mu=3.10 should not be
promoted.
