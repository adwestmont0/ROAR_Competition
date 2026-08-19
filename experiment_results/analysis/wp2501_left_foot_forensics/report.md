# WP2501 0.10 full-run forensics

No CARLA run or controller change was made. Five completed treatments were
compared with their bracketing controls using interpolated monotonic official
race progress. The official index is used because the custom route wraps near
WP2622 while the race lap seam is near WP2609.

## Time accounting

| Region | Lap | Delta entering | Delta leaving | Increment | Repeatability | Supported mechanism |
|---|---:|---:|---:|---:|---:|---|
| Before WP2501 | 1 | 0.000 s | 0.000 s | 0.000 s | 5/5 | Treatment not active; matched starting behavior |
| WP2501 release to race seam | 1 | 0.000 s | -0.050 s | -0.050 s | 5/5 | Reduced net braking; small higher-speed carry to seam |
| Seam to WP391 | 2 | -0.050 s | 0.000 s | +0.050 s | 5/5 | Initial seam advantage is surrendered before WP391 |
| WP391 through WP794 entry | 2 | 0.000 s | 0.000 s | approximately 0 | mixed | One-tick WP391/WP627 phase regimes create oscillatory, nonpersistent changes |
| WP794 entry to WP1232 entry | 2 | 0.000 s | -0.110 s | -0.110 s | 5/5 | WP794 decision moves one waypoint earlier; same six ticks, higher target/exit speed; segment -0.120 s |
| WP1232 through lap-2 seam | 2 | -0.110 s | -0.110 s | 0.000 s | 5/5 | Advantage is carried through the rest of lap 2 |
| Lap-3 seam to WP1232 | 3 | -0.110 s | -0.143 s | -0.033 s | 4/5 | Distributed WP627/WP794/arrival-phase effects |
| WP1232 through WP1781 entry | 3 | -0.143 s | -0.150 s | -0.007 s | 3/5 | Clean release occurs in 3/5 treatments versus 0/4 controls |
| WP1781 through finish | 3 | -0.150 s | -0.150 s | approximately 0 | 5/5 final | Favorable WP1781 frequency rises, but local gains/losses overlap and net out |

The non-overlapping coarse accounting is -0.050 + 0.050 - 0.110 - 0.033
- 0.007 = -0.150 s. Values are quantized by the 0.05 s control interval and
interpolated between physical-progress samples, so individual components have
roughly 0.02--0.05 s resolution.

## Principal lap-2 transition

At WP794 on lap 2, all controls and treatments used 6/6 raw/applied brake
ticks. Nevertheless, every treatment entered a different discrete phase:

| Metric | Control | Treatment | Delta |
|---|---:|---:|---:|
| Brake onset waypoint | 794 | 793 | -1 waypoint |
| Lookahead waypoint | 820 | 819 | -1 waypoint |
| Selected target speed | 161.447 km/h | 164.541 km/h | +3.094 km/h |
| Recommended speed | 185.290 km/h | 188.587 km/h | +3.298 km/h |
| Entry speed | 188.377 km/h | 188.749 km/h | +0.372 km/h |
| Minimum speed | 152.858 km/h | 154.281 km/h | +1.423 km/h |
| Release/exit speed | 183.694 km/h | 184.487 km/h | +0.793 km/h |
| Time to WP1232 onset | 17.100 s | 16.980 s | -0.120 s |

Thus the important effect is not a brake-count reduction. It is a repeatable
one-waypoint decision-phase transition followed by a persistent acceleration
gain.

## Regimes

| Zone | Control regimes | Treatment regimes | Meaningful shift |
|---|---|---|---|
| WP391 | 16 ticks 12/12 | 16 ticks 10/15; 17 ticks 5/15 | Yes, but the added tick is not itself favorable; phase precursor |
| WP427 | Lap 1: 11; laps 2-3: 12, all traversals | Same | No |
| WP627 | 4 ticks 7/12; 5 ticks 5/12 | 4 ticks 6/15; 5 ticks 9/15 | Yes; treatment more often takes the longer regime, but subsequent WP794 state is faster |
| WP794 | L1-2 6/6; L3 6/5, all controls | Same counts in all treatments | Count no; lap-2 onset/lookahead phase shifts one waypoint in 5/5 |
| WP1232 | Clean 7/12; re-brake 5/12 | Clean 13/15; re-brake 2/15 | Yes; clean release rises from 58.3% to 86.7% |
| WP1781 | 14 ticks 11/12; 15 ticks 1/12 | 14 ticks 15/15 | Small favorable shift, concentrated on lap 3 |
| WP2501 | 41 raw/39 applied 12/12 | 41/39 15/15 | No regime change; five overlap ticks verified |

On lap 3 specifically, WP1232 controls re-braked 4/4 while treatments cleanly
released 3/5. WP1781 controls selected 14 ticks in 3/4 and 15 in 1/4, while
treatments selected 14 ticks in 5/5. These shifts are consistent with a carried
arrival-phase change, although their marginal time cannot be added independently
to the WP794 gain.

## Artifact checks

The adjusted full-run mean remains -0.150 s under every control method:

| Control reference | Mean | Range / spread |
|---|---:|---:|
| Bracketing midpoint | -0.150 s | -0.175 to -0.125; SD 0.025 |
| Time-interpolated controls | -0.150 s | -0.183 to -0.116; SD 0.033 |
| Preceding control | -0.150 s | -0.200 to -0.100; SD 0.035 |
| Following control | -0.150 s | -0.200 to -0.100; SD 0.035 |
| Nearest control | -0.150 s | -0.200 to -0.100; SD 0.050 |

Controls were 321.200--321.250 s, treatments 321.050--321.100 s. Treatment
and control traces were effectively coincident at the first WP2501 entry, so a
starting-state advantage is unsupported. Fixed qualifying parameters were the
same; the only treatment parameter was the opt-in WP2501 overlap. Telemetry
verified exactly five overlap ticks and unchanged 41/39 braking. Infrastructure
startup retries had no driving telemetry and were excluded.

Official-progress interpolation reproduces the -0.150 s finish effect. The
independent custom-waypoint/release-anchor analysis agrees that the immediate
+100/+200 m WP2501 gain is tiny; the apparent contradiction is resolved by the
later lap-2 phase transition, not by changing the local estimate.

## Conclusion

Classification: **A — persistent downstream phase gain**.

The 0.10 perturbation should be promoted as a validated qualifying candidate.
No further discriminating experiment is required before that decision. This
report does not modify or enable the controller treatment.
