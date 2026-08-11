import json
import os
import hashlib
from collections import deque
from typing import Any, Dict, Optional

import numpy as np
from .terminal_policy import evaluate_terminal_tail, smooth_profile


class ShadowLongitudinalPlanner:
    """Track an offline velocity profile without modifying applied control."""

    def __init__(self, profile: Dict[str, Any]) -> None:
        points = profile["points"]
        if len(points) < 3:
            raise ValueError("shadow velocity profile requires at least three points")
        self.profile_version = int(profile.get("schema_version", 1))
        self.source_result_sha256 = profile.get("source_result_sha256")
        self.xy = np.asarray([[point["x"], point["y"]] for point in points], dtype=float)
        self.s = np.asarray([point["s_m"] for point in points], dtype=float)
        self.ds = np.asarray([point["ds_m"] for point in points], dtype=float)
        self.speed = np.asarray([point["target_speed_kmh"] for point in points], dtype=float)
        self.acceleration = np.asarray([point["acceleration_mps2"] for point in points], dtype=float)
        self.deceleration = np.asarray([point["deceleration_mps2"] for point in points], dtype=float)
        self.constraints = [str(point["active_constraint"]) for point in points]
        self.confidence = [str(point["confidence"]) for point in points]
        self.support = np.asarray([point["support"] for point in points], dtype=int)
        self.last_index: Optional[int] = None
        self.terminal_acceleration_history = deque(maxlen=3)
        self.terminal_previous_speed_kmh: Optional[float] = None
        self.terminal_previous_steer: Optional[float] = None
        self.terminal_h1_brake_requests = 0
        self.terminal_h1_gap_ticks = 99
        self.profile_identity_sha256 = self._profile_checksum()
        terminal_targets=self.speed.tolist()
        self.terminal_profile = {"target_speed_kmh":terminal_targets,"smoothed_target_speed_kmh":smooth_profile(terminal_targets),"ds_m":self.ds.tolist()}

    def _profile_checksum(self) -> str:
        digest=hashlib.sha256()
        for value in (self.xy,self.s,self.ds,self.speed,self.acceleration,self.deceleration):digest.update(np.asarray(value).tobytes())
        return digest.hexdigest()

    @classmethod
    def from_file(cls, path: str) -> "ShadowLongitudinalPlanner":
        with open(path, encoding="utf-8") as infile:
            return cls(json.load(infile))

    @classmethod
    def load_default(cls) -> "ShadowLongitudinalPlanner":
        return cls.from_file(os.path.join(os.path.dirname(__file__), "velocity_profile.json"))

    def _nearest_index(self, location: Any) -> int:
        xy = np.asarray(location, dtype=float).reshape(-1)[:2]
        count = len(self.xy)
        if self.last_index is None:
            index = int(np.argmin(np.sum((self.xy - xy) ** 2, axis=1)))
        else:
            # The racing-line index is monotonic modulo the lap boundary. Looking
            # only forward prevents a nearby previous branch of the track from
            # pulling the shadow state backward.
            offsets = np.arange(0, 25)
            candidates = (self.last_index + offsets) % count
            local = int(np.argmin(np.sum((self.xy[candidates] - xy) ** 2, axis=1)))
            index = int(candidates[local])
            if float(np.linalg.norm(self.xy[index] - xy)) > 15.0:
                index = int(np.argmin(np.sum((self.xy - xy) ** 2, axis=1)))
        self.last_index = index
        return index

    def _phase(self, index: int) -> str:
        following = (index + 1) % len(self.speed)
        delta = float(self.speed[following] - self.speed[index])
        if delta < -0.25:
            return "braking"
        if delta > 0.25:
            return "accelerating"
        return "steady"

    def _distance_to_phase(self, index: int, desired: str, horizon_m: float = 500.0) -> Optional[float]:
        distance = 0.0
        for offset in range(1, len(self.speed)):
            previous = (index + offset - 1) % len(self.speed)
            candidate = (index + offset) % len(self.speed)
            distance += float(self.ds[previous])
            if distance > horizon_m:
                return None
            if self._phase(candidate) == desired:
                return distance
        return None

    def observe(
        self,
        location: Any,
        speed_kmh: float,
        applied_control: Dict[str, Any],
        custom_waypoint_index: int,
        terminal_tail_enabled: bool = False,
    ) -> Dict[str, Any]:
        index = self._nearest_index(location)
        following = (index + 1) % len(self.speed)
        target = float(self.speed[index])
        target_next = float(self.speed[following])
        ds = max(0.01, float(self.ds[index]))
        required_acceleration = (
            (target_next / 3.6) ** 2 - (target / 3.6) ** 2
        ) / (2.0 * ds)
        error = float(speed_kmh) - target
        acceleration_limit = max(0.1, float(self.acceleration[index]))
        deceleration_limit = max(0.1, float(self.deceleration[index]))

        if required_acceleration < -0.25 or error > 1.0:
            shadow_brake = min(1.0, max(0.0, -required_acceleration / deceleration_limit))
            if error > 1.0:
                shadow_brake = max(shadow_brake, min(1.0, error / 12.0))
            shadow_throttle = 0.0
            command = "brake"
        elif required_acceleration > 0.15 or error < -1.0:
            shadow_throttle = min(1.0, max(0.0, required_acceleration / acceleration_limit))
            if error < -1.0:
                shadow_throttle = max(shadow_throttle, min(1.0, -error / 8.0))
            shadow_brake = 0.0
            command = "throttle"
        else:
            shadow_throttle = 0.0
            shadow_brake = 0.0
            command = "coast"

        applied_throttle = float(np.asarray(applied_control.get("throttle", 0.0)).reshape(-1)[0])
        applied_brake = float(np.asarray(applied_control.get("brake", 0.0)).reshape(-1)[0])
        phase = self._phase(index)
        output = {
            "available": True,
            "profile_version": self.profile_version,
            "source_result_sha256": self.source_result_sha256,
            "profile_index": index,
            "profile_s_m": float(self.s[index]),
            "profile_distance_error_m": float(np.linalg.norm(self.xy[index] - np.asarray(location)[:2])),
            "custom_waypoint_index": int(custom_waypoint_index),
            "target_speed_kmh": target,
            "speed_error_kmh": error,
            "phase": phase,
            "command": command,
            "required_acceleration_mps2": required_acceleration,
            "acceleration_limit_mps2": acceleration_limit,
            "deceleration_limit_mps2": deceleration_limit,
            "distance_to_braking_m": 0.0 if phase == "braking" else self._distance_to_phase(index, "braking"),
            "distance_to_release_m": 0.0 if phase != "braking" else self._distance_to_phase(index, "accelerating"),
            "throttle": shadow_throttle,
            "brake": shadow_brake,
            "throttle_delta": shadow_throttle - applied_throttle,
            "brake_delta": shadow_brake - applied_brake,
            "active_constraint": self.constraints[index],
            "confidence": self.confidence[index],
            "support": int(self.support[index]),
        }
        if terminal_tail_enabled:
            control_snapshot={key:float(np.asarray(applied_control.get(key,0.0)).reshape(-1)[0]) for key in ("throttle","brake","steer")}
            original_snapshot=dict(output);index_snapshot=self.last_index;profile_before=self._profile_checksum()
            steer=control_snapshot["steer"]
            acceleration=0.0 if self.terminal_previous_speed_kmh is None else ((float(speed_kmh)-self.terminal_previous_speed_kmh)/3.6)/.05
            self.terminal_acceleration_history.append(acceleration)
            mean_acceleration=float(sum(self.terminal_acceleration_history)/len(self.terminal_acceleration_history))
            steer_change=0.0 if self.terminal_previous_steer is None else steer-self.terminal_previous_steer
            reason=("profile_gradient" if required_acceleration < -.25 else "speed_error" if error>1.0 else "none")
            prior=(0 if shadow_brake>0.01 and self.terminal_h1_gap_ticks>2 else self.terminal_h1_brake_requests)
            opinion=evaluate_terminal_tail({"profile_index":index,"speed_kmh":float(speed_kmh),"current_acceleration_mps2":acceleration,"mean_recent_acceleration_mps2":mean_acceleration,"steer":steer,"steer_change":steer_change,"original_h1_brake":shadow_brake>0.01,"original_h1_reason":reason,"prior_h1_brake_requests":prior},self.terminal_profile)
            if shadow_brake>0.01:
                if self.terminal_h1_gap_ticks>2:self.terminal_h1_brake_requests=0
                self.terminal_h1_brake_requests+=1;self.terminal_h1_gap_ticks=0
            else:self.terminal_h1_gap_ticks+=1
            self.terminal_previous_speed_kmh=float(speed_kmh);self.terminal_previous_steer=steer
            invariants={"applied_control_unchanged":control_snapshot=={key:float(np.asarray(applied_control.get(key,0.0)).reshape(-1)[0]) for key in ("throttle","brake","steer")},"original_h1_output_unchanged":original_snapshot==output,"original_h1_index_unchanged":index_snapshot==self.last_index,"profile_unchanged":profile_before==self._profile_checksum()==self.profile_identity_sha256}
            output["terminal_tail"]={**opinion,"current_acceleration_mps2":acceleration,"mean_recent_acceleration_mps2":mean_acceleration,"steer":steer,"steer_change":steer_change,"original_h1_reason":reason,"prior_h1_brake_requests":prior,"invariants":invariants}
        return output


def unavailable_shadow(error: Exception) -> Dict[str, Any]:
    return {"available": False, "error": "%s: %s" % (type(error).__name__, error)}
