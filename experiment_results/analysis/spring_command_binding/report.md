# Spring 2026 batch command-binding screen

Date: 2026-08-19 UTC

## Method

Fixed-trajectory replay of 64,228 ticks from ten completed runs carrying the
current known-good qualifying modifications. The two S8-treatment runs were
included because that experiment proved its changed recommendation never
changed a command. For mu/max-speed candidates, every curvature constraint was
reconstructed from recorded position and lookahead rather than assuming the
logged selected constraint remained selected. The replay maintained its own
brake counter for changes capable of altering longitudinal state.

Counts below are independently applied changes, not a combined Spring replay.
They establish whether a parameter is behaviorally alive; they do not predict
closed-loop lap time.

Baseline replay reproduced all 64,228 logged applied commands, decision
branches, and post-run brake-counter states exactly (zero mismatches).

## Scalar ranking

| Rank | Change | Applied command ticks | Runs | Concentration | Interpretation | Risk |
|---:|---|---:|---:|---|---|---|
| 1 | S2 mu 3.35 -> 3.40 | 46 | 10/10 | WP627 only | Removes/downgrades 32 brake initiations and changes 14 coast/maintain commands | Medium-low; isolated short event |
| 2 | S4 mu 2.95 -> 3.05 | 67 | 10/10 | WP1232 only | Removes 39 initiations and 28 continuing brake commands on fixed trajectories | High; directly overlaps release -1 and known release -2 instability |
| 3 | Brake threshold multiplier 1.00 -> 1.05 | 6 | 3/10 | WP627 1, WP794 3, WP2501 2 | Sparse/non-repeatable avoided initiations | High interaction risk across validated events |
| 4 | Throttle-up multiplier 1.25 -> 1.35 | 0 applied (2,182 raw) | 0/10 applied | Distributed | Both old and new values clip to throttle=1.0 | None, but no opportunity |
| 5 | Full-throttle threshold 0.90 -> 0.95 | 0 | 0/10 | Distributed branch labels only | 1,735 branch selections change, but both paths apply throttle=1.0 and reset the same state | No opportunity |
| 6 | Max speed 300 -> 305 | 0 | 0/10 | None | 305 never becomes an applied-command constraint | No opportunity |

### S2 details

All 46 command changes occur in Section 2 at WP625-649:

* 20 `brake_initiate` -> `throttle_maintain_over`
* 12 `brake_initiate` -> `throttle_down`
* 14 `throttle_down` -> `throttle_maintain`

Per-run changed ticks were 4-6, with changes present in every run. By lap the
counts were lap 1: 10, lap 2: 27, lap 3: 9. This is the cleanest repeatable
binding signal and aligns with the known natural WP627 discrete regimes.

### S4 details

All 67 changes occur at WP1232-1299:

* 36 `brake_initiate` -> `throttle_maintain_over`
* 22 `brake_counter` -> `throttle_maintain`
* 6 `brake_counter` -> `throttle_maintain_over`
* 3 `brake_initiate` -> `throttle_down`

Although strongly binding, this is effectively a broad brake reduction at the
same event where unconditional release -2 and conditional re-brake suppression
were unstable near WP1403-1410. It should not be the first transfer test.

## Structurally invasive Spring changes

| Change | Applied command ticks | Internal-state ticks | Scope | Assessment |
|---|---:|---:|---|---|
| `/3`, max-8 brake counter | 0 | 3,210 | Multiple brake events | No first-order applied difference on recorded trajectories, but very large counter-state rewrite; unsuitable as an isolated low-risk transfer |
| 0.6/0.1 light-brake branches | 230 | command itself changes | WP391/427/627/794/1232/1781/2501 | Strongly binding but global and invasive; 230 simultaneous throttle+brake changes across every major event |

Light braking changed 230 commands in all ten runs: WP427 29, WP627 65,
WP794 30, WP1232 27, WP1781 63, WP2501 15, and WP391 1. This is not suitable
until the localized scalar opportunities are exhausted.

## Recommendation

Run exactly one isolated A/B experiment: **S2 mu=3.40**, retaining every
current qualifying modification. Begin with C-T-C-T-C and focus on WP627 raw
and applied brake decisions, natural regime, +100/+200 m persistence, and the
next downstream event. Do not combine it with S4 or any Spring global logic.

S2 ranks above S4 despite fewer changed commands because it is repeatable in
10/10 runs, physically localized, and does not directly overlap an existing
override or a demonstrated instability boundary.
