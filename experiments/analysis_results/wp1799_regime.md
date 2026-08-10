# WP1799 longitudinal-controller regime analysis

This is offline replay analysis. No controller source is changed.

## Population and method

The five treatment attempts contain 15 complete lap traversals. Four traversals
applied 14 brake ticks in custom WP 1779–1849; eleven applied 15. Each telemetry
state was replayed through the unmodified winner `ThrottleController`. The replay
reconstructed the selected `SpeedData`, radius sample, permissible-speed formula,
pre/post `brake_ticks`, and exact conditional branch. The treatment's post-controller
command suppression was then overlaid from recorded telemetry.

The selected constraint throughout the separating event was `wide-mid`: interesting
waypoints `[1, 3, 5]`, with `distance_to_section = close_distance = 3 m`.
Its permissible speed is:

`recommended = sqrt(825 * (target_speed^2 / 675 + 3))`

## Regime outcomes

| Regime | Traversals | Mean minimum speed | Mean WP1849 speed | Behavior |
|---|---:|---:|---:|---|
| 14 applied brake ticks | 4 | ~174.9 km/h | ~175.1 km/h | early raw brake decision, one suppressed tick, no replacement |
| 15 applied brake ticks | 11 | ~167.9 km/h | ~168.2 km/h | later raw onset, longer queued brake burst, late compensation |

## Earliest separation

At WP1779 the 15-tick regime has approximately `speed=256.35`,
`recommended=254.72`, and ratio `1.0064`. The hard-brake boundary is
`1 + 2.4/current_speed = 1.0094`, so it takes `throttle_down` rather than
`brake_initiate`. It does not encounter the suppressed first brake command until
WP1781.

The 14-tick regime encounters a sufficiently lower wide-mid permissible speed at
WP1779–1780 that the ratio is already above the same hard-brake boundary. Its first
`brake_initiate` is suppressed there; its next applied brake arrives one controller
tick earlier than in the compensating regime.

Thus the first separating internal decision is the comparison:

```python
if percent_of_max > 1 + true_percent_change_per_tick:
```

The difference is caused by the selected wide-mid curvature/permissible-speed value,
not by a different lateral command.

## Representative 14-tick traversal

`raw` is the controller output; `applied` includes the reviewed one-tick suppression.
All rows select the wide-mid constraint.

| WP | Speed | Recommended | Ratio | Counter pre→post | Δspeed | Radius | Curve target | Branch | Raw brake | Applied brake |
|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|
| 1780 | 256.28 | 250.46 | 1.0233 | 0→0 | +0.04 | 117.5 | 222.0 | brake_initiate | 1 | 0 |
| 1781 | 256.28 | 245.36 | 1.0445 | 0→1 | -0.01 | 112.6 | 217.3 | brake_initiate | 1 | 1 |
| 1783 | 255.91 | 241.86 | 1.0581 | 1→0 | -0.37 | 109.3 | 214.1 | brake_counter | 1 | 1 |
| 1785 | 254.85 | 237.69 | 1.0722 | 0→1 | -1.06 | 105.3 | 210.2 | brake_initiate | 1 | 1 |
| 1786 | 253.17 | 236.25 | 1.0716 | 1→0 | -1.68 | 104.0 | 208.9 | brake_counter | 1 | 1 |
| 1788 | 250.99 | 234.43 | 1.0706 | 0→1 | -2.17 | 102.4 | 207.2 | brake_initiate | 1 | 1 |
| 1790 | 248.49 | 235.52 | 1.0551 | 1→0 | -2.50 | 103.3 | 208.2 | brake_counter | 1 | 1 |
| 1791 | 245.74 | 233.46 | 1.0526 | 0→1 | -2.75 | 101.5 | 206.3 | brake_initiate | 1 | 1 |
| 1793 | 242.81 | 232.00 | 1.0466 | 1→0 | -2.93 | 100.1 | 205.0 | brake_counter | 1 | 1 |
| 1795 | 239.73 | 230.17 | 1.0415 | 0→0 | -3.08 | 98.5 | 203.3 | brake_initiate | 1 | 1 |
| 1796 | 236.54 | 229.05 | 1.0327 | 0→0 | -3.19 | 97.5 | 202.2 | brake_initiate | 1 | 1 |
| 1798 | 233.27 | 226.63 | 1.0293 | 0→0 | -3.28 | 95.3 | 200.0 | brake_initiate | 1 | 1 |
| 1799 | 229.92 | 224.09 | 1.0260 | 0→0 | -3.35 | 93.1 | 197.6 | brake_initiate | 1 | 1 |
| 1801 | 226.52 | 222.21 | 1.0194 | 0→0 | -3.40 | 91.5 | 195.9 | brake_initiate | 1 | 1 |
| 1802 | 223.08 | 219.31 | 1.0172 | 0→0 | -3.44 | 89.0 | 193.2 | brake_initiate | 1 | 1 |
| 1804 | 219.63 | 217.68 | 1.0090 | 0→0 | -3.45 | 87.6 | 191.7 | throttle_maintain_over | 0 | 0 |
| 1805 | 216.24 | 214.66 | 1.0074 | 0→0 | -3.39 | 85.0 | 188.9 | throttle_maintain_over | 0 | 0 |

## Representative 15-tick compensating traversal

| WP | Speed | Recommended | Ratio | Counter pre→post | Δspeed | Radius | Curve target | Branch | Raw brake | Applied brake |
|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|
| 1779 | 256.34 | 254.72 | 1.0064 | 0→0 | +0.04 | 121.7 | 226.0 | throttle_down | 0 | 0 |
| 1781 | 256.36 | 246.84 | 1.0385 | 0→0 | +0.02 | 114.0 | 218.7 | brake_initiate | 1 | 0 |
| 1782 | 256.27 | 243.27 | 1.0534 | 0→1 | -0.09 | 110.6 | 215.4 | brake_initiate | 1 | 1 |
| 1784 | 255.74 | 239.78 | 1.0666 | 1→0 | -0.52 | 107.3 | 212.2 | brake_counter | 1 | 1 |
| 1786 | 254.54 | 236.80 | 1.0749 | 0→2 | -1.20 | 104.5 | 209.4 | brake_initiate | 1 | 1 |
| 1787 | 252.75 | 235.29 | 1.0742 | 2→1 | -1.79 | 103.1 | 208.0 | brake_counter | 1 | 1 |
| 1789 | 250.50 | 234.21 | 1.0696 | 1→0 | -2.25 | 102.1 | 207.0 | brake_counter | 1 | 1 |
| 1791 | 247.94 | 234.51 | 1.0573 | 0→1 | -2.57 | 102.4 | 207.3 | brake_initiate | 1 | 1 |
| 1792 | 245.14 | 232.31 | 1.0552 | 1→0 | -2.79 | 100.4 | 205.3 | brake_counter | 1 | 1 |
| 1794 | 242.17 | 230.84 | 1.0491 | 0→1 | -2.97 | 99.1 | 203.9 | brake_initiate | 1 | 1 |
| 1796 | 239.07 | 228.99 | 1.0440 | 1→0 | -3.10 | 97.4 | 202.2 | brake_counter | 1 | 1 |
| 1797 | 235.85 | 227.73 | 1.0357 | 0→0 | -3.21 | 96.3 | 201.0 | brake_initiate | 1 | 1 |
| 1799 | 232.56 | 225.45 | 1.0315 | 0→0 | -3.29 | 94.3 | 198.9 | brake_initiate | 1 | 1 |
| 1800 | 229.20 | 223.46 | 1.0257 | 0→0 | -3.36 | 92.5 | 197.1 | brake_initiate | 1 | 1 |
| 1802 | 225.78 | 220.49 | 1.0240 | 0→0 | -3.42 | 90.0 | 194.3 | brake_initiate | 1 | 1 |
| 1803 | 222.33 | 218.41 | 1.0180 | 0→0 | -3.45 | 88.2 | 192.4 | brake_initiate | 1 | 1 |
| 1805 | 218.87 | 216.02 | 1.0132 | 0→0 | -3.46 | 86.2 | 190.1 | brake_initiate | 1 | 1 |
| 1806 | 215.42 | 213.56 | 1.0087 | 0→0 | -3.45 | 84.1 | 187.9 | throttle_maintain_over | 0 | 0 |

## Conditional that creates the compensating tick

The decisive queued-state difference occurs at WP1786. The compensating trace has:

`round((254.54 - 236.80) / 7) = round(2.534) = 3`

The current brake is returned immediately, and `run()` decrements the new counter to
2. Those two queued ticks are then consumed by the `if self.brake_ticks > 0` branch at
WP1787 and WP1789. In the 14-tick regime, the analogous initiation stays below the
2.5 rounding boundary and leaves only one queued tick.

At WP1803 the accumulated phase difference causes the visible late compensation:
ratio `1.0180` exceeds `1 + 2.4/222.33 = 1.0108`. With `brake_ticks == 0` and
`speed_change < 2.5`, the controller enters `brake_initiate`. The successful regime
is already at ratio ~`1.009`, below its ~`1.011` hard-brake boundary, and takes
`throttle_maintain_over`.

## Proposed next intervention (not applied)

Add a small `+0.75 km/h` bias to the selected permissible speed before it enters
the braking state machine. This is enough to move the critical WP1786 speed gap
below the `17.5 km/h` rounding boundary while being far smaller than the requested
2–3 km/h vehicle-speed effect. The controller then computes the shorter brake burst
itself; no final command is suppressed and its counter remains consistent.

The proposed source diff is:

```diff
--- a/competition_code/submission.py
+++ b/competition_code/submission.py
@@
         throttle, brake, gear = self.throttle_controller.run(
             waypoints_for_throttle,
             vehicle_location,
             current_speed_kmh,
             self.current_section,
+            self.current_waypoint_idx,
         )
```

```diff
--- a/competition_code/ThrottleController.py
+++ b/competition_code/ThrottleController.py
@@
-    def run(self, waypoints, current_location, current_speed, current_section):
+    def run(
+        self, waypoints, current_location, current_speed, current_section,
+        current_waypoint_index=None,
+    ):
         self.tick_counter += 1
         throttle, brake = self.get_throttle_and_brake(
-            current_location, current_speed, current_section, waypoints
+            current_location, current_speed, current_section, waypoints,
+            current_waypoint_index,
         )
@@
-    def get_throttle_and_brake(
-        self, current_location, current_speed, current_section, waypoints
-    ):
+    def get_throttle_and_brake(
+        self, current_location, current_speed, current_section, waypoints,
+        current_waypoint_index=None,
+    ):
@@
         update = self.select_speed(speed_data)
+        if (
+            current_waypoint_index is not None
+            and 1779 <= current_waypoint_index <= 1803
+        ):
+            update.recommended_speed_now += 0.75
+            update.speed_diff = current_speed - update.recommended_speed_now
```

This proposal preserves every existing curvature sample, target-speed calculation,
threshold, counter update, and throttle/brake branch. It changes only the selected
longitudinal envelope presented to that unchanged state machine in the observed
event window. It must be dry-run and replay-checked before an online campaign;
specifically, replay should confirm that the late WP1805 branch is not merely shifted
outside the window.
