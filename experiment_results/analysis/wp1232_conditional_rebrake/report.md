# WP1232 conditional re-brake suppression

## Decision

**UNSTABLE — DROP.** The campaign stopped at the mandated safety boundary after both treatment attempts collided at WP1403 on lap 2. The failures were deterministic to the recorded tick and location and closely reproduce the prior unconditional WP1232 release-2 failure near WP1408–1410.

No third treatment, adaptive change, or follow-up experiment was run.

## Campaign accounting

The valid attempt sequence was one completed control followed by two treatment collisions. An initial launch with the system Python produced one pre-experiment infrastructure error because it loaded CARLA client 0.9.15; it was stopped and relaunched with the validated ROAR virtualenv/CARLA 0.9.12. The validated launch also recorded one stale-synchronous-state infrastructure error before automatic recovery. Neither infrastructure event produced usable vehicle telemetry and neither is counted as a control or treatment attempt.

| Mode | Triggered traversals | Local +100 m | Local +200 m | WP1781 14-tick rate | Mean adjusted full-run | Finishes | Collisions |
|---|---:|---:|---:|---:|---:|---:|---:|
| Control | 0 | reference | reference | 3/3 (100%) overall; clean 2/2, re-brake 1/1 | 0 by definition | 1/1 | 0 |
| Treatment | 2 | +0.000 s vs matched re-brake | **-0.050 s** vs matched re-brake | Triggered: unavailable; both crashed before WP1781. Non-triggered: 2/2 | unavailable | 0/2 | **2** |

The sole completed control time was 321.250 s. Treatments collided at 171.750 s, so no treatment three-lap time or adjusted full-run distribution exists. Conditional-suppression-count versus completed-lap correlation is therefore undefined.

Local comparisons use the same suppression position and fixed physical horizons against the 21 historical comparable RE-BRAKE controls (including six from the closest WP794 rerun-2 configuration). Historical re-brake elapsed times were 0.50/1.05/2.25/4.45 s at +25/+50/+100/+200 m. Both treatments measured 0.50/1.05/2.25/4.40 s. Thus there was no resolved gain through +100 m and a one-tick 0.05 s gain at +200 m.

## Triggered traversals

| Run/Lap | WP1232 phase | Raw re-brake | Suppressed | +200 m gain | WP1779 phase change | WP1781 lookahead | Target speed | Natural brake ticks | Outcome |
|---|---|---|---|---:|---|---|---|---|---|
| `79875a76` / L2 | onset WP1231; base suppression WP1273; decision WP1274 | `brake_initiate`, ordinal 25 | Yes, exactly one | -0.050 s | unavailable | unavailable | unavailable | unavailable | Collision WP1403, 171.750 s |
| `1e9426a4` / L2 | onset WP1231; base suppression WP1273; decision WP1274 | `brake_initiate`, ordinal 25 | Yes, exactly one | -0.050 s | unavailable | unavailable | unavailable | unavailable | Collision WP1403, 171.750 s |

Both interventions occurred at simulation time 165.850 s. Both collisions followed approximately 5.90 s and 270 m later. Collision impulses were 4286.34 and 4282.75. The intervention unquestionably fired earlier in the same traversal.

## Causal trace

The two treatment traces were nearly identical:

1. WP1232 unfavorable phase: applied brake ordinal 23 at WP1272.
2. Existing release-1 suppression: raw ordinal 24 at WP1273, applied brake zero.
3. Fresh winner re-request: `brake_initiate`, raw ordinal 25 at WP1274 and about 178.95 km/h.
4. Conditional treatment: applied brake zero, neutral throttle zero; no positive throttle fabricated.
5. Next winner decision: natural positive throttle on WP1275 (`throttle_maintain_over`).
6. +25/+50/+100 m: no tick-resolved timing gain versus historical re-brake.
7. +200 m: 0.05 s gain.
8. Collision: WP1403 on lap 2, before WP1779 or WP1781 could be observed.

Clean-release treatment traversals received no conditional suppression. Each treatment completed lap 1 without triggering and naturally selected the WP1781 14-tick regime, with onset WP1779, lookahead WP1814, and target about 224.146 km/h. These are non-triggered observations and cannot establish treatment efficacy.

The current control contained two clean-release traversals and one natural re-brake traversal; all three selected WP1781=14. This small contemporaneous sample also shows why historical matched controls were necessary, but it does not rescue the treatment's stability outcome.

## Implementation invariants

Before CARLA:

- Historical replay triggered 21/21 identified re-brakes and 0/36 clean releases.
- The gate required WP1232, the tick immediately following the existing release-1 suppression, raw brake, and the exact `brake_initiate` branch.
- A consumed latch limited it to one extra suppression per traversal.
- No positive throttle magnitude was fabricated.
- Control source contained only the existing WP1232/WP2501 combination and lap-3 WP794 override.
- Treatment source contained no WP1781 edit.
- Unit suite passed 34/34 tests.

Runtime telemetry confirmed exactly one conditional suppression in each treatment attempt, only on the intended lap-2 re-brake. WP2501 behaved normally on completed lap 1. Neither treatment reached lap-3 WP794; therefore WP794 runtime equivalence could not be re-observed in treatment.

## Required conclusions

1. **Did it fire only in intended re-brake cases?** Yes. It fired once in each treatment, at the immediate ordinal-25 `brake_initiate`; it did not fire on either clean lap-1 traversal.
2. **Did it recover the ~0.04 s local cost?** Partially. No resolved gain through +100 m; 0.05 s at +200 m.
3. **Did the gain persist to WP1779?** Unresolved. Both triggered traversals crashed before WP1779.
4. **Did it increase WP1781=14 selection?** Unresolved for triggered traversals. Non-triggered clean traversals remained 2/2, but they are not causal evidence.
5. **Did it improve WP1781 exit/downstream performance?** Unresolved; WP1781 was never reached after a trigger.
6. **Did it improve full-run time?** No evaluable treatment finish; 0/2 completed.
7. **Did WP1408–1410 instability return?** Yes, strongly. Both treatments collided at WP1403, about five waypoints earlier but physically the same downstream failure region.
8. **Should it be retained?** No. Drop the conditional modification and retain the pre-experiment qualifying controller.
