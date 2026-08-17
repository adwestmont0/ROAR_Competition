# Qualifying brake timing repeat validation

Six interleaved controls and five attempts per candidate were run. All 21
controller measurements finished; infrastructure-only preflight retries are
excluded. Controls were 321.800, 321.750, 321.750, 321.750, 321.900, and
321.600 seconds. Each treatment was adjusted against the midpoint of its
immediately surrounding controls.

| Candidate | Raw mean | Adjusted mean | Adjusted median | Adjusted stddev | Adjusted range | Finishes | +100 m | +200 m | Post-final compensation |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| WP2501 0/-2 | 321.490 | -0.280 | -0.350 | 0.124 | -0.375 to -0.075 | 5/5 | -0.140 | -0.143 | immediate, 7/15 traversals |
| WP2501 +1/-1 | 321.590 | -0.180 | -0.200 | 0.154 | -0.400 to +0.000 | 5/5 | -0.013 | -0.013 | immediate, 14/15 traversals |
| WP1232 0/-1 | 321.340 | -0.430 | -0.450 | 0.086 | -0.525 to -0.300 | 5/5 | -0.087 | -0.065 | immediate, 7/15 traversals |

## Classification

- WP2501 0/-2: deterministic faster. It passes every acceptance criterion and
  has the stronger WP2501 causal persistence.
- WP2501 +1/-1: likely faster but inferior/noisier. One adjusted run was neutral,
  nearly every traversal compensated, and the gain was almost fully erased by
  +100/+200 m. Do not retain it over 0/-2.
- WP1232 0/-1: deterministic faster. It completed 5/5 with the lowest adjusted
  mean and standard deviation and retained a gain through the next brake.

The surviving independent candidates are WP2501 0/-2 and WP1232 0/-1. Their
evidence is strong enough to authorize a subsequent two-event combination test,
but no combination was run in this campaign.
