import csv
import json
import os
from datetime import datetime, timezone
from typing import Any, Dict, Optional

import numpy as np


class RaceTelemetry:
    """Collect read-only race data and write it after the timed run."""

    fieldnames = [
        "tick",
        "sim_time_seconds",
        "x",
        "y",
        "z",
        "speed_kmh",
        "throttle",
        "brake",
        "steer",
        "target_gear",
        "official_waypoint_index",
        "custom_waypoint_index",
        "section",
        "lap",
        "collision_impulse",
    ]

    def __init__(self, output_root: str) -> None:
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.output_dir = os.path.join(output_root, "runs", run_id)
        self.samples = []
        self.collision_count = 0
        self.max_collision_impulse = 0.0

    @staticmethod
    def _scalar(value: Any, default: float = 0.0) -> float:
        if value is None:
            return default
        array = np.asarray(value)
        if array.size == 0:
            return default
        return float(array.reshape(-1)[0])

    def record_collision(self, impulse: float) -> None:
        self.collision_count += 1
        self.max_collision_impulse = max(self.max_collision_impulse, float(impulse))

    def record_tick(
        self,
        tick: int,
        sim_time_seconds: float,
        location: Any,
        velocity: Any,
        control: Dict[str, Any],
        official_waypoint_index: int,
        solution: Any,
        collision_impulse: float,
    ) -> None:
        location_array = np.asarray(location).reshape(-1)
        speed_kmh = float(np.linalg.norm(velocity) * 3.6)
        self.samples.append(
            {
                "tick": tick,
                "sim_time_seconds": float(sim_time_seconds),
                "x": float(location_array[0]),
                "y": float(location_array[1]),
                "z": float(location_array[2]),
                "speed_kmh": speed_kmh,
                "throttle": self._scalar(control.get("throttle")),
                "brake": self._scalar(control.get("brake")),
                "steer": self._scalar(control.get("steer")),
                "target_gear": int(self._scalar(control.get("target_gear"))),
                "official_waypoint_index": int(official_waypoint_index),
                "custom_waypoint_index": int(
                    getattr(solution, "current_waypoint_idx", -1)
                ),
                "section": int(getattr(solution, "current_section", -1)),
                "lap": int(getattr(solution, "lapNum", -1)),
                "collision_impulse": float(collision_impulse),
            }
        )

    def write(
        self,
        status: str,
        elapsed_time: float,
        control_timestep: float,
        terminal_event: Optional[Dict[str, Any]] = None,
    ) -> str:
        os.makedirs(self.output_dir, exist_ok=False)
        samples_path = os.path.join(self.output_dir, "ticks.csv")
        with open(samples_path, "w", newline="") as outfile:
            writer = csv.DictWriter(outfile, fieldnames=self.fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)

        speeds = [sample["speed_kmh"] for sample in self.samples]
        summary = {
            "status": status,
            "elapsed_time_seconds": float(elapsed_time),
            "control_timestep_seconds": float(control_timestep),
            "recorded_ticks": len(self.samples),
            "average_speed_kmh": float(np.mean(speeds)) if speeds else 0.0,
            "maximum_speed_kmh": max(speeds) if speeds else 0.0,
            "collision_count": self.collision_count,
            "maximum_collision_impulse": self.max_collision_impulse,
            "samples_file": "ticks.csv",
        }
        if terminal_event is not None:
            summary.update(terminal_event)
        summary_path = os.path.join(self.output_dir, "summary.json")
        with open(summary_path, "w") as outfile:
            json.dump(summary, outfile, indent=2)
            outfile.write("\n")
        return summary_path
