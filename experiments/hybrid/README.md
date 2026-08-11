# Hybrid controller research

This workstream seeks a controller faster than the validated 321.75-second
Summer 2025 baseline. Historical submissions are evidence and ablation sources,
not a target architecture and not code to splice together wholesale.

## Provenance-backed findings

The source revisions are pinned in `historical_sources.json`.

| Component | Historical evidence | Compatibility | Decision |
|---|---|---|---|
| Summer 2024 pure pursuit | 325.6 s winner; current lateral file has the identical SHA-256 | Already present | Retain as applied baseline |
| Custom racing line | Major improvement in Summer/Fall 2024 lineage | Already present and validated | Retain initially |
| Distance-weighted preview steering | Spring 2025 winner concept; used again in Spring 2026 third-place lineage | Can produce a shadow target without changing longitudinal control | Evaluate in shadow mode |
| Left-foot braking | Fall 2025 winner; throttle is retained while brake remains active near release | Orthogonal to path and lateral control, but changes CARLA actuation semantics | Evaluate event-by-event in shadow, then causal online tests |
| Precomputed turn geometry | Fall 2024 experimental controller reported 327.35 s | Useful as a stable geometric feature, not as a complete controller | Feed the predictive planner |
| Expanded section friction and recovery | Spring 2026 winner reported 320.4 s | Directly applicable but still coarse and potentially marginal | Use as benchmark, not final architecture |
| Stanley tracking | Spring 2026 experiment reported 348.9 s | Removes beneficial corner cutting | Reject as primary lateral controller |
| Textbook friction profile | Spring 2026 experiment reported 361 s | CARLA limits were badly calibrated | Retain empirical envelopes from velocity-profile v2 |
| Minimum-curvature racing line | Spring 2026 experiment crashed | Insufficient wall margin | Defer until boundaries and uncertainty are modeled |

## Proposed novel hybrid

The intended controller is not a historical submission. It combines their
supported mechanisms inside a distance-domain predictive architecture:

1. Keep the validated custom racing line and pure-pursuit controller as the
   initial applied lateral system.
2. Use the v2 empirical acceleration/deceleration envelopes and precomputed
   curvature to plan speed over complete braking events.
3. Track that profile with a stateful event controller, including hysteresis,
   rather than recomputing an independent brake-tick threshold each frame.
4. Test a clean-room left-foot-braking actuator only near planned brake release,
   where historical evidence says it improves response.
5. Compute distance-weighted preview steering in shadow and promote it only in
   intervals where it reduces steering rate without increasing line error or
   consuming the stability margin.
6. Apply empirical stability caps and uncertainty penalties learned from both
   successful and failed traversals.

## Candidate ladder

Each stage must pass offline support checks and forced-clean repeated trials
before the next stage is applied.

### H0: provenance benchmark

Reproduce the public Spring 2026 pure-pursuit winner as an independent benchmark.
Do not merge its parameters into the baseline. This determines whether its
reported 320.4-second improvement transfers to this CARLA build.

### H1: longitudinal shadow planner

For every tick, emit a shadow target speed, planned brake phase, planned release
distance, and shadow throttle/brake. Applied control remains the validated
baseline. Compare event timing against successful telemetry and collision
boundaries.

Implementation status: the runtime planner lives in
`competition_code/shadow/longitudinal.py`. Its compact profile is exported from
velocity-profile v2 by `experiments/hybrid/export_shadow_profile.py` and records
the source result's SHA-256. `submission.py` invokes it only after constructing
the final validated control dictionary. The validated action is dispatched
before shadow evaluation begins; the shadow result is stored separately for
telemetry and is never passed to `vehicle.apply_action()`.

### H2: release-phase actuator

Apply simultaneous throttle and brake only in one support-qualified event and
only during the final planned braking phase. The first test measures exit speed,
downstream persistence, segment time, and stability; it does not tune broad
sections.

### H3: weighted-preview lateral shadow

Compute the alternative target and steering command without applying it. Rank
intervals by reduced steering variation, maintained racing-line margin, and
compatibility with the longitudinal plan.

### H4: coupled predictive controller

Promote independently validated longitudinal and lateral mechanisms into a
short-horizon controller with a shared lateral/longitudinal friction budget.

## Acceptance principles

- Compare candidates with interleaved forced-clean controls.
- Treat infrastructure failures as pending, never as controller failures.
- Require full completion before considering time improvement.
- Prefer event-local interventions over section-wide constants.
- Preserve the baseline source before every attempt.
- Do not optimize speed in empirically stability-constrained regions such as
  custom WP 1240–1409 without an explicit margin experiment.

## Source-use rule

The audited public repositories did not expose a license file at their roots.
Their reports and behavior may inform independent designs, but source code must
not be copied into this repository without clarified licensing.
