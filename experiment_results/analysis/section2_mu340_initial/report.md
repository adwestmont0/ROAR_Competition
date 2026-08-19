# Section 2 mu=3.40 initial transfer gate

Date: 2026-08-19 UTC

## Attempts

| Run | Mode | Outcome | Time / terminal point |
|---|---|---|---|
| C1 | control | collision | WP849, lap 3, 255.450 s |
| T1 | S2 mu=3.40 | collision | WP1409, lap 2, 171.850 s |
| C2 | control | finished | 321.250 s |
| T2 | S2 mu=3.40 | finished | 321.250 s |
| C3 | control | finished | 321.100 s |

The only completed treatment is +0.075 s versus the mean of its adjacent
completed controls (321.175 s). Treatment finished 1/2 with one collision;
controls finished 2/3 with one collision. The treatment collision reproduces
the historically sensitive WP1409 region and is plausibly downstream of S2,
but one occurrence does not distinguish causation from the known baseline
failure mode.

## WP627 regimes

| Mode | Traversals | 5 ticks | 4 ticks | 3 ticks |
|---|---:|---:|---:|---:|
| Control | 9 | 7 (77.8%) | 2 (22.2%) | 0 |
| Treatment | 5 | 0 | 5 (100%) | 0 |

Treatment therefore produced the predicted command effect: every observed
WP627 traversal selected four raw/applied brake ticks. Mean matched deltas were
approximately +1.81 km/h at release, +0.56 km/h at minimum speed, and +0.21
km/h by WP723. The speed benefit decayed substantially through the corner exit.

## Common-position timing

Treatment minus its adjacent controls, averaged across five comparable
traversals:

| Anchor | Delta |
|---|---:|
| WP627 entry | 0.000 s |
| +25 m | 0.000 s |
| +50 m | 0.000 s |
| +100 m | -0.025 s |
| +200 m | -0.010 s |
| WP794 entry | -0.020 s |
| WP794 exit | -0.020 s |
| next major brake (WP1232 entry) | -0.060 s |

Resolution is one 0.05-s control tick. The local advantage is small and not
monotonic: the +100 m gain mostly disappears by +200 m. Individual WP794-entry
deltas were -0.050, 0.000, -0.050, 0.000, and 0.000 s.

## WP794 interaction

Controls selected the WP793/lookahead-819 favorable phase in 2/9 traversals;
treatment selected it in 1/5. Both are about 20-22%, so there is no frequency
shift. All traversals retained six raw ticks; the existing lap-3 suppression
retained five applied ticks where reached. Treatment did not create a
repeatable WP794 onset, lookahead, target-speed, or brake-regime change.

WP1232 release -1, WP2501 release -2, WP2501 five-tick 0.10 LFB, and the lap-3
WP794 suppression were verified unchanged in every traversal that reached
their activation point.

## Decision

Classification: **C - SPEED TRADEOFF ONLY**, with an unresolved stability
warning. S2 mu=3.40 reliably converts WP627 to four ticks and briefly raises
speed, but the timing benefit is marginal by WP794, no beneficial downstream
regime becomes more frequent, the sole completed run is +0.075 s slower, and
one of two treatment attempts collided at WP1409. Do not extend or promote.
