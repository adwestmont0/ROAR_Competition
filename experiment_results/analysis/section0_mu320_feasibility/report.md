# Section 0 mu=3.20 offline feasibility gate

Date: 2026-08-19 UTC

## Physical scope

The current controller's section boundaries are
`[2611, 254, 557, 739, 1158, 1317, 1516, 1881, 1944, 2359]`.
S0 therefore wraps across the lap seam from WP2611 through WP253. WP391 and
WP427 are downstream in S1; WP627 is in S2.

Spring 2026 uses
`[2611, 322, 557, 739, 1158, 1317, 1516, 1881, 1944, 2359]`.
Its S0 starts at the identical physical WP2611 seam but continues through
WP321. Thus current S0 is a physical subset of Spring S0, not an exactly equal
section interval. Changing current S0 alone would not reproduce Spring's
WP254-321 treatment.

## Historical binding analysis

Ten completed runs with the current known-good configuration contributed 9,507
S0 ticks. They came from the WP2501 LFB initial/validation and S8 initial-gate
campaigns; the S8 setting is outside S0 and cannot affect this binding test.

| Measurement | Result |
|---|---:|
| S0 ticks | 9,507 |
| raw throttle=1.0, raw brake=0.0 | 9,507 (100%) |
| `throttle_full` branch | 9,487 |
| `throttle_full_speed_drop` branch | 20 |
| ticks where mu affected selected target | 3,400 |
| minimum recommended-speed minus current-speed margin | +80.686 km/h |
| counterfactual throttle/brake predicate flips at mu=3.20 | 0 |

For the 3,400 curvature-constrained ticks, the selected target-speed range was
229.289-299.894 km/h. Recomputing the selected recommendation with
`sqrt(3.20 / 2.75)` scaling produced no crossing of either `speed/recommended
> 1` or the controller's braking threshold. The command remains full throttle.

The remaining S0 samples were at the 300 km/h cap, so changing mu does not
change their target at all.

## Downstream implication

Because applied commands cannot change anywhere in current S0, the treatment
cannot create a deterministic speed or arrival-phase perturbation at WP391.
Historical WP391 raw-brake counts varied naturally (mostly 16, with occasional
17 in this cohort), but S0 mu=3.20 supplies no causal lever capable of changing
their frequency. WP427, WP627, and the validated WP794/WP1232/WP2501 overrides
likewise receive identical upstream vehicle commands.

## Decision

The mandatory behavioral-binding gate failed. No CARLA runs, controller edits,
or parameter implementation were performed. Classification: **D - NEUTRAL
(predicted from an exact non-binding command-path test)**. Do not promote S0
mu=3.20 in the current controller.
