# Forensic time accounting: WP1232 + WP2501 combination

## Executive result

The combined controller does **not** surrender 0.300 s at one hidden corner. It retains both intended local gains. The apparent non-additivity has two different components:

1. A real, directly observed giveback of about **0.041 s on lap 3 from WP801 to approximately WP900**, repeatable in 4/5 runs. Four runs leave the six-tick WP794–801 brake event 0.49–1.20 km/h slower than their paired controls and carry that deficit down the following acceleration zone.
2. A larger **overlap/phase interaction**: separate WP1232-only and WP2501-only campaigns each credited downstream discrete-phase gains to their own treatment. Their arithmetic sum predicts gains that cannot both be reproduced when the interventions coexist. The largest single interaction increment is **+0.130 s on lap 2 from WP441 to WP627**. Importantly, the combined controller actually gains 0.076 s there; it simply gains about 0.130 s less than the independent traces predict.

The physically aligned endpoint interaction is +0.295 s with an estimated standard error of 0.075 s. This agrees with the lap-time estimate of +0.300 s.

## Method

Each treatment was paired with its immediate preceding and following completed controls. Time was interpolated at 2 m intervals along an unwrapped, 5,606.26 m waypoint-distance axis. The axis uses first physical progress through each corrected waypoint, not equal controller ticks. Every trace was zeroed at the common start position.

`delta_t(s) = treatment crossing time(s) - mean(adjacent-control crossing time(s))`

The independent WP1232 and WP2501 runs were processed identically. The interaction trace is:

`combined trace - (WP1232-only trace + WP2501-only trace)`

The position-aligned trace ends at -0.445 s rather than the official adjusted -0.410 s because the last common interpolable physical position precedes the timing terminal by a few waypoints. The unreconciled +0.035 s is terminal/alignment residual, within one control tick.

## Material full-run accounting

The following regions reconcile the mean position-aligned result. Small changes below roughly 0.02–0.025 s are grouped with adjacent regions because individual samples are quantized at 0.05 s.

| Region | Delta entering | Delta leaving | Gain/loss | Supported mechanism | Repeatability |
|---|---:|---:|---:|---|---|
| Lap 1 start → WP1232 | 0.000 | +0.001 | neutral | No command/state divergence before the first intervention | 5/5 |
| Lap 1 WP1232 → WP1409 | +0.001 | -0.040 | **-0.041 gain** | One applied brake tick omitted; release/throttle 0.060 s earlier; higher release speed | 4/5 strong, fifth weak |
| Lap 1 WP1409 → WP2501 | -0.040 | -0.055 | -0.015 gain/carry | Gain persists through WP1781; no new override | 4/5 |
| Lap 1 WP2501 → finish | -0.055 | -0.130 | **-0.075 gain** | Two applied ticks omitted; approximately 8 km/h higher release speed; downstream acceleration | 5/5 |
| Lap 2 start → WP443 | -0.110 | -0.080 | **+0.030 loss** | WP391/WP427 waypoint-phase cascade; one-waypoint lookahead/onset shifts and one-tick brake-count changes | 3/5 material; small same-sign loss in 2/5 |
| Lap 2 WP443 → WP627 | -0.080 | -0.156 | **-0.076 gain** | Faster traversal into WP627; discrete brake phase/release changes | 5/5 |
| Lap 2 WP627 → WP1232 | -0.156 | -0.180 | -0.024 gain/carry | Mostly retained speed; no material mode change | 4/5 |
| Lap 2 WP1232 → WP1804 | -0.180 | -0.220 | **-0.040 gain** | WP1232 omission plus naturally shortened WP1781 braking | 3/5 strong, 2/5 neutral |
| Lap 2 WP1804 → finish | -0.220 | -0.255 | -0.035 gain | WP2501 local gain partly retained to finish | 4/5 |
| Lap 3 start → WP443 | -0.285 | -0.260 | +0.025 loss | Same WP391/WP427 phase pattern, weaker than lap 2 | 4/5 same sign, below threshold individually |
| Lap 3 WP443 → WP801 | -0.260 | -0.312 | **-0.052 gain** | Faster intermediate sectors; no override outside validated events | 4/5 |
| Lap 3 WP801 → WP900 | -0.312 | -0.272 | **+0.041 loss** | Lower WP794–801 exit speed after the same six full-brake ticks; phase/entry-state effect | **4/5** |
| Lap 3 WP900 → WP1232 | -0.272 | -0.279 | -0.007 carry | Speed deficit decays; commands predominantly full throttle | 5/5 |
| Lap 3 WP1232 → WP1781 | -0.279 | -0.376 | **-0.097 gain** | WP1232 gain plus downstream acceleration and shortened WP1781 braking | 5/5 gain, magnitude variable |
| Lap 3 WP1781 → WP1804 | -0.376 | -0.359 | +0.017 loss | Altered natural WP1781 brake duration/minimum-speed tradeoff; below material threshold | 3/5 |
| Lap 3 WP1804 → finish | -0.359 | -0.445 | **-0.086 gain** | WP2501 release gain and final acceleration | 5/5 |

## Automatically identified loss regions

Using a 0.03 s deterioration threshold after 30 m smoothing produced one robust physical loss event:

| Rank | Lap | Start | End | Entry delta | Exit delta | Surrendered | Repeatability |
|---:|---:|---|---|---:|---:|---:|---|
| 1 | 3 | WP801 | approximately WP900 | -0.312 | -0.272 | **0.041 s** | 4/5 |

Because telemetry is quantized at 0.05 s, a secondary event was retained at a relaxed one-half-tick threshold:

| Rank | Lap | Start | End | Entry delta | Exit delta | Surrendered | Repeatability |
|---:|---:|---|---|---:|---:|---:|---|
| 2 | 2 | WP391 | WP443 | -0.098 | -0.080 | 0.018 s net (0.024–0.028 s internal steps) | same sign 5/5; material in 3/5 |

No other merged absolute deterioration was both at least 0.03 s and repeatable in most runs.

## Seven braking zones

Metrics below average 15 traversals (three laps across five treatments) against adjacent controls. Onset and release times are relative to zone entry. `Cumulative in/out` averages the three lap-specific trace values, so it describes the whole campaign rather than only the first passage.

| Zone | Entry speed | Onset waypoint / time | Raw / applied ticks | Release waypoint / time | Minimum speed | Exit speed | Segment time | Cumulative in → out |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| WP391–415 | +0.004 km/h | +0.37 WP / 0.000 s | -0.37 / -0.37 | -0.40 WP / -0.018 s | -0.975 km/h | -0.311 km/h | +0.003 s | -0.122 → -0.113 s |
| WP427–441 | -0.079 km/h | -0.47 WP / +0.003 s | +0.03 / +0.03 | -0.47 WP / +0.005 s | -0.234 km/h | +1.347 km/h | -0.008 s | -0.125 → -0.110 s |
| WP627–633 | +0.216 km/h | +0.20 WP / -0.007 s | -0.20 / -0.20 | 0.00 WP / -0.017 s | +0.126 km/h | +0.126 km/h | -0.017 s | -0.150 → -0.156 s |
| WP794–801 | -0.042 km/h | +0.07 WP / 0.000 s | -0.07 / -0.07 | 0.00 WP / -0.003 s | -0.219 km/h | -0.240 km/h | +0.003 s | -0.156 → -0.163 s |
| WP1232–1276 | -0.004 km/h | +0.27 WP / 0.000 s | +0.03 / **-0.97** | -1.03 WP / **-0.060 s** | +0.244 km/h | **+3.360 km/h** | **-0.092 s** | -0.153 → -0.155 s |
| WP1781–1804 | +0.003 km/h | 0.00 WP / 0.000 s | **-1.10 / -1.10** | -1.97 WP / **-0.055 s** | -2.168 km/h | **+2.514 km/h** | -0.025 s | -0.212 → -0.205 s |
| WP2501–2561 | +0.034 km/h | -0.80 WP / 0.000 s | -0.67 / **-2.67** | -1.87 WP / **-0.087 s** | +1.522 km/h | **+8.072 km/h** | -0.023 s | -0.221 → -0.232 s |

The altered state from WP1232 is visible at WP1781: the winner naturally requests 1.10 fewer brake ticks on average and releases 0.055 s earlier. That is beneficial overall, not the missing loss. WP2501 also begins about 0.8 waypoint earlier because the arriving vehicle phase differs, but its two intentional omissions still dominate and its downstream gain survives.

## Controller mechanism and discrete effects

### Largest repeatable absolute loss: WP794–900, lap 3

All five treatments request six binary full-brake ticks around WP794–801, exactly matching their paired controls except one treatment that shifts the sequence one waypoint earlier. Nevertheless, four treatments leave the zone slower:

- exit/minimum-speed deficits in those four runs: approximately 0.49–1.20 km/h;
- mean speed deficit over WP800–850: 0.493 km/h;
- mean target-speed deficit over WP800–850: 0.396 km/h;
- mean cumulative time surrendered by WP900: 0.041 s;
- commands after the event are essentially full throttle and zero brake, so the lost time is carried rather than created on the straight.

The telemetry supports an **altered entry/decision phase into the same six-tick binary braking sequence**. It does not support delayed throttle, extra brake ticks, section changes, or steering oscillation as the cause. The final treatment is the counterexample: it shifts the last brake request from WP801 to WP800, exits about 1.0 km/h faster, and does not show the loss. This makes the effect phase-sensitive rather than a deterministic additional-braking rule.

### WP391/WP427 cascade

At WP391, six of fifteen traversals move onset/lookahead one waypoint later and remove one brake tick. On lap 2 this occurs in three runs; those runs reach approximately 3.48 km/h lower minimum speed and surrender about 0.028 s across WP391–415. At WP427, eight of fifteen traversals move onset/lookahead one waypoint earlier. The lap-2 phase is extremely repeatable: the trace advances by almost exactly +0.024 s toward zero in all five runs across WP427–443.

No steering or section-mode divergence explains this. The logged branch remains `brake_initiate`; lookahead count is nearly unchanged. The supported mechanism is waypoint/control-tick quantization changing selected lookahead waypoint, target speed, and binary brake phase. Some of this time is recovered before WP627.

### Other controller state

- WP1232: one applied tick is removed as intended; the winner re-requests in 7/15 traversals. The downstream gain persists.
- WP2501: two applied ticks are removed as intended; 17 immediate re-requests occur across 15 traversals. The downstream gain persists.
- WP1781: no override is active, yet altered arrival phase changes brake duration by roughly one tick and release by 0.055 s.
- No material steering correction, section transition, or persistent lookahead-count change correlates with the major loss.
- PID integrator/derivative terms are not separately logged. Target speed, recommended speed, speed error, brake counters, constraint selection, and lookahead are logged; they do not establish a hidden PID-state cause. Any more specific PID claim is unresolved.

## Is the +0.300 s interaction real?

Using the reported independent standard deviations and five runs per candidate:

- expected additive mean: -0.710 s;
- expected-additive standard error: `sqrt(0.086^2/5 + 0.124^2/5) = 0.068 s`;
- combined standard error: `0.095/sqrt(5) = 0.042 s`;
- interaction standard error: approximately **0.080 s**;
- approximate 95% interval for the +0.300 s interaction: **+0.144 to +0.456 s**.

The independently reconstructed physical trace gives +0.295 ± 0.075 s (standard error) at the last common position. Therefore the sign and a substantial portion of the non-additivity are convincing. Under the simple independent-run model, roughly **at least 0.14 s** is real at 95% confidence; the point estimate is 0.30 s.

Control drift is not negligible: validation controls span 321.600–321.900 s and combination controls span 321.750–321.950 s. Adjacent-control interpolation removes linear drift but cannot remove sub-bracket changes. Interior control midpoint residuals reach 0.225 s. Consequently, the exact 0.300 s magnitude should not be treated as millisecond-precise. A defensible interpretation is:

- about **0.15–0.30 s is repeatable physical/phase interaction**;
- up to roughly **0.10–0.15 s could plausibly be campaign/control interpolation and lap-level estimation error**;
- the hypothesis of zero interaction is not supported by the five-run trace ensemble.

The largest interaction-trace increments are not necessarily absolute losses:

- lap 2 WP441→627: +0.130 s interaction, while combined delta improves by 0.076 s;
- lap 2 WP801→1232: +0.051 s interaction;
- lap 3 WP441→627: +0.069 s interaction;
- lap 3 WP801→1232: +0.062 s interaction.

These are overlapping downstream/phase benefits credited to both independent candidates, not four places where the combined vehicle visibly brakes away 0.31 s.

## Ranked next experiments — proposals only

| Rank | Location | Observed mechanism | Recoverable time | Confidence | Smallest experiment | Risk |
|---:|---|---|---:|---|---|---|
| 1 | WP794–801, lap 3 only | Same six full-brake ticks entered in a slower discrete phase; final WP801 tick leaves 4/5 runs slower | approximately 0.04 s/run | Medium-high | A/B one omitted expected final brake tick at this event, with all other combined settings fixed | Medium: may shift stability boundary or merely move braking one tick |
| 2 | Campaign-wide factorial | Independent campaigns double-count downstream phase gains | resolves 0.15–0.30 s uncertainty, not directly a speed gain | High | Interleave control, WP1232-only, WP2501-only, and combined in one contemporaneous randomized block | Low controller risk; simulator time cost |
| 3 | WP391–443 | One-waypoint lookahead/onset shifts create repeatable half-tick losses and later recovery | approximately 0.02–0.03 s | Medium | Diagnostic A/B that pins only the observed final WP391 brake decision for one tick | Medium-high: two close braking events are phase-coupled |

No proposal has been implemented.

## Required conclusions

1. **Where does the combined controller gain time?** At WP1232 and its downstream acceleration, at WP2501 and the final acceleration, and through beneficial naturally shortened braking at WP1781. The largest accounted gains are lap 3 WP1232→1781 (-0.097 s) and lap 3 WP1804→finish (-0.086 s).
2. **Where does it give time back?** Primarily lap 3 WP801→900 (+0.041 s, 4/5), with smaller quantized losses around WP391/WP427 on laps 2–3.
3. **What causes the largest repeatable loss?** An altered entry/control-tick phase into the unchanged six-tick WP794–801 binary braking event, producing lower exit speed in 4/5 runs. Telemetry rules out extra tick count, delayed throttle, section change, and steering oscillation.
4. **How much of the apparent 0.300 s non-additivity is convincingly real?** The point estimate is 0.295–0.300 s with about 0.075–0.080 s standard error. At least roughly 0.14 s is supported at 95% confidence; up to 0.10–0.15 s may be campaign/control-interpolation noise. Most of it is overlapping phase benefit rather than literal time surrender.
5. **What is the highest-value next experiment?** A single lap-3-only omission of the expected final WP794–801 brake tick, tested as a tightly interleaved A/B against the unchanged combined controller. It targets the only >0.03 s repeatable absolute giveback and changes one command on one event.
