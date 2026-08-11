# Terminal-tail tick-t causal boundary

The second opinion executes only after the winner command has been dispatched
and after original H1 has produced its shadow result. It cannot modify either.

| Input | Category | Source available at tick t |
|---|---|---|
| Target-speed profile, spacing, custom-index map | A: immutable | Frozen profile/policy artifacts loaded at initialization |
| Direct coefficients, RMSE, tail envelope and bounds | A: immutable | Frozen policy artifact |
| Current profile index | B: current observation | Original H1 spatial alignment at tick t |
| Current speed | B: current observation | Current velocimeter observation |
| Applied steering and steering change | B/C | Winner command at t and telemetry-only previous steering |
| Current and three-sample mean acceleration | B/C | Current speed minus stored t-1 speed; prior acceleration samples |
| Original H1 command and reason | B | Original H1 output computed before the opinion |
| Prior H1 brake-request count | C | Separate telemetry-only history through t-1 |
| Projected +2/+4/+6/+8 positions and targets | A+B+C | Pure integration of frozen direct predictions; not realized future state |
| Next smoothed minimum terminal | A+B | Pure search of immutable profile from current index |

Prohibited category-D inputs include future speed, future actual profile index,
future winner/H1 commands, future steering, and future target samples selected
from realized motion. The live evaluator accepts none of these values.

Future telemetry is used only by the retrospective safety audit after a logged
decision; it is never an eligibility or decision input.
