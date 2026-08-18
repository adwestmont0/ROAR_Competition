# Natural brake-regime mining

## Scope and method

Raw winner requests—not applied commands—were grouped across original controls, the validated independent WP1232 and WP2501 campaigns, the combined campaign, and the completed WP794 campaign. A traversal was excluded from “natural” grouping when that same physical event was directly overridden. Comparisons were also matched within configuration and lap where both adjacent regimes occurred, reducing campaign and lap confounding.

## Regime summary

| Zone | Natural regimes (traversals) | Faster regime | Persistent gain | Apparent trigger | Evidence | Opportunity |
|---|---|---|---|---|---|---|
| WP391–415 | 16 (86), 15 (29) | **15** | matched +100/-0.077 s; +200/-0.027 s | onset about one waypoint later; target about 5.2 km/h lower; nearly identical matched entry speed | **FASTER AND STABLE**, moderate | Preserve the faster lap-seam/arrival phase from preceding WP2501 exit |
| WP427–441 | 13 (5), 12 (69), 11 (41) | 12 versus 13; 11 uncertain | 13→12: +100/-0.114, +200/-0.099 s; 12→11 matched evidence weak | waypoint/target phase changes, but few matched strata | **INSUFFICIENT EVIDENCE** | Diagnostic only; do not optimize yet |
| WP627–633 | 5 (80), 4 (34), 3 (1) | **4** | matched +100/-0.090 s; +200/-0.060 s | onset one waypoint later (WP628), target 1.4 km/h lower, prior WP427 exit about 3.8 km/h faster in combined runs | **FASTER AND STABLE**, strong | Small upstream WP427 exit-phase change; avoid direct WP627 override |
| WP794–801 | 6 (111) | none naturally observed | — | winner naturally selected only six ticks | No natural transition | Phase-1 forced five-applied-tick candidate is separate |
| WP1232–1276 | 25 (18), 24 (38) | neither clearly | +25 is 0.093 s worse; +100 neutral; +200 only -0.029 s | 24 starts about 0.7 waypoint later with target 6.9 km/h lower | **NEUTRAL** | No new natural-regime experiment |
| WP1781–1804 | 15 (60), 14 (54) | **14** | matched +25/-0.050, +50/-0.072, +100/-0.127, +200/-0.106 s | onset around WP1779 instead of WP1781; lookahead 1814 instead of 1816; target +4.7 km/h; same entry speed | **FASTER AND STABLE**, strongest | Upstream WP1232 exit/arrival-phase probe |
| WP2501–2561 | 42 (28), 41 (28) | 41 does not persist | +100/+0.050 and +200/+0.050 s slower in the only matched stratum | regimes are largely separated by upstream configuration, so trigger is confounded | **SLOWER / low confidence** | Do not pursue fewer natural ticks here |

The three-tick WP627 regime has one sample and is insufficient evidence.

## WP1781: natural versus direct shortening

The natural 14-tick regime is a different controller decision, not an omitted command:

- Natural 14 ticks: onset WP1779, lookahead WP1814, target about 224.0 km/h, release WP1802 after 0.700 s, exit about 222.4 km/h.
- Natural 15 ticks: onset near WP1781, lookahead near WP1816, target about 218.9 km/h, release near WP1805 after 0.750 s, exit about 218.2 km/h.
- Entry speeds are effectively identical. Entry acceleration is only about 0.04 m/s² higher in the 14-tick regime.

Within combined configurations, 14-tick traversals leave the preceding WP1232 event about 0.093 km/h faster and reach WP1781 about 0.086 s sooner after the prior brake release. That small phase difference moves the lookahead/target selection roughly two waypoints and the winner itself chooses 14 ticks.

Direct WP1781 release suppression did not recreate this state. With onset 0/release-1, two of three laps still requested 15 raw ticks and merely applied 14; those laps began later with a lower target and reached only about 221.6 km/h at release. The one naturally occurring 14-tick lap inside that run behaved like the natural regime. Direct onset-delay variants were worse: they generated 16–17 raw requests, longer 0.80–0.85 s events, release around WP1806–1808, and exit speeds around 213–216 km/h. This is the observed compensation that made direct intervention slow.

Direct campaign adjusted results reinforce the distinction:

- onset 0/release-1: approximately +0.075 s versus midpoint control;
- onset 0/release-2: approximately +0.025 s;
- onset +1 variants: approximately +0.53 to +0.73 s slower.

Natural shortening is beneficial because upstream arrival phase changes the winner’s constraint/target selection before braking. Direct shortening changes applied commands after the winner has already selected the longer regime.

## Ranked upstream opportunities

1. **WP1781 natural 15→14 via WP1232 exit phase.** Conditional opportunity approximately 0.10–0.13 s through +100/+200 m; high repeatability and strongest causal evidence. Smallest probe: one-tick-earlier throttle reapplication immediately after the existing WP1232 omitted final tick, without touching WP1781. Risk medium-high because WP1232 has a known stability boundary.
2. **WP627 natural 5→4 via WP427 exit phase.** Opportunity approximately 0.06–0.09 s; strong matched evidence. Smallest probe: a one-tick WP427 release/exit-phase A/B while observing whether WP627 naturally moves to onset WP628/four ticks. Risk medium because WP391/WP427 are tightly coupled.
3. **WP391 natural 16→15 via lap-seam/WP2501 exit phase.** Opportunity approximately 0.03–0.08 s; moderate evidence. Smallest probe: preserve the higher-speed one-waypoint-later WP392 onset phase through the lap seam. Risk medium and mechanism less local.

No Phase-2 recommendation was implemented.

## Required conclusions

1. WP794 recovered more than the predicted 0.04 s in completed runs: mean adjusted -0.100 s, with -0.138/-0.088 s at +100/+200 m. Keep as a candidate; reliability was 4/5 because of an upstream pre-intervention collision.
2. WP1781 15→14 is the most promising natural transition.
3. The transition is caused by arrival/waypoint phase changing lookahead from roughly WP1816 to WP1814 and raising the selected target by about 4.7 km/h; entry speed itself is not the discriminator.
4. Natural WP1781 shortening is winner-selected before braking. Direct suppression modifies commands after selection of the longer regime and, with onset delay, causes compensating 16–17-tick requests.
5. Highest-value next experiment: an upstream WP1232 exit-phase A/B that advances throttle reapplication by exactly one tick and measures whether WP1781 naturally selects 14 ticks. Do not modify WP1781 directly.
