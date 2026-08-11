"""Pure helpers shared by offline replay and live shadow opinions.

These functions have no mutable state and use only information available at the
current control tick.  In particular, they never inspect future telemetry.
"""

import math
import statistics
from typing import Dict, Iterable, List, Tuple

import numpy as np

from .terminal_policy_data import load_policy_data


RESIDUAL_GUARD_HORIZONS = (2, 4, 6, 8)
DIRECT_ERROR_HORIZONS = (2, 4, 6, 8, 10, 12, 16, 20, 24)
TAIL_HORIZONS = (28, 32, 36, 40, 48)


def advance_profile_index(ds_m: Iterable[float], start_index: int,
                          distance_m: float) -> int:
    """Advance around a closed profile by a non-negative arc distance."""
    spacing = list(ds_m)
    count = len(spacing)
    if not count:
        raise ValueError("profile spacing must not be empty")
    index = int(start_index) % count
    remaining = max(0.0, float(distance_m))
    while remaining > max(0.0, float(spacing[index])) and remaining > 0.0:
        remaining -= max(0.0, float(spacing[index]))
        index = (index + 1) % count
    return index


def causal_residual_guard(
    current_speed_kmh: float,
    predicted_delta_mps: Dict[int, float],
    profile_target_speed_kmh: List[float],
    profile_ds_m: List[float],
    current_profile_index: int,
    margin_kmh: float = 1.0,
    timestep_seconds: float = 0.05,
    horizons: Tuple[int, ...] = RESIDUAL_GUARD_HORIZONS,
):
    """Evaluate the residual safety precondition without future telemetry.

    Distance is integrated from the model's predicted speed sequence and target
    speed is sampled from the immutable spatial profile at that distance.
    """
    distance = 0.0
    previous_speed = max(0.0, float(current_speed_kmh)) / 3.6
    trace = []
    maximum = max(horizons)
    for tick in range(1, maximum + 1):
        predicted_speed = max(
            0.0,
            float(current_speed_kmh) / 3.6 + float(predicted_delta_mps[tick]),
        )
        distance += 0.5 * (previous_speed + predicted_speed) * timestep_seconds
        previous_speed = predicted_speed
        if tick not in horizons:
            continue
        index = advance_profile_index(profile_ds_m, current_profile_index, distance)
        target = float(profile_target_speed_kmh[index])
        predicted_kmh = predicted_speed * 3.6
        trace.append({
            "horizon_ticks": tick,
            "projected_distance_m": distance,
            "projected_profile_index": index,
            "projected_target_speed_kmh": target,
            "predicted_speed_kmh": predicted_kmh,
            "margin_kmh": target - predicted_kmh,
        })
    return all(item["margin_kmh"] >= float(margin_kmh) for item in trace), trace


def smooth_profile(targets):
    count=len(targets)
    return [float(statistics.median((targets[(i-1)%count],targets[i],targets[(i+1)%count]))) for i in range(count)]


def next_smoothed_minimum(targets, ds_m, custom_indices, index):
    """Frozen next-minimum terminal objective (400 m / 5 km/h recovery)."""
    count=len(targets);distance=0.0;best=index;minimum=float(targets[index]);recovery=0
    for offset in range(1,count):
        previous=(index+offset-1)%count;cursor=(index+offset)%count
        distance+=float(ds_m[previous])
        if distance>400.0:break
        value=float(targets[cursor])
        if value<minimum:minimum=value;best=cursor;recovery=0
        elif value>=minimum+5.0:recovery+=1
        else:recovery=0
        if recovery>=3:break
    terminal_distance=0.0
    for offset in range(0,(best-index)%count):terminal_distance+=float(ds_m[(index+offset)%count])
    return {"profile_index":best,"custom_waypoint_index":int(custom_indices[best]),"target_speed_kmh":minimum,"distance_m":terminal_distance}


def _interpolate_tail(segments,ticks):
    cumulative={};total=0.0
    for h in TAIL_HORIZONS:total+=float(segments[str(h)]);cumulative[h]=total
    ticks=max(24,min(48,int(ticks)));upper=min((h for h in TAIL_HORIZONS if h>=ticks),default=48);lower=max((h for h in (24,)+TAIL_HORIZONS if h<=ticks),default=24)
    lo=0.0 if lower==24 else cumulative[lower];hi=cumulative[upper]
    return lo if upper==lower else lo+(hi-lo)*(ticks-lower)/(upper-lower),"%d-%d"%(lower,upper)


def evaluate_terminal_tail(inputs, profile, policy=None):
    """Return one deterministic telemetry-only terminal-tail opinion."""
    policy=load_policy_data() if policy is None else policy
    targets=[float(x) for x in profile["target_speed_kmh"]];ds=[float(x) for x in profile["ds_m"]];index=int(inputs["profile_index"]);smooth=profile.get("smoothed_target_speed_kmh") or smooth_profile(targets)
    base={"status":"NOT_ELIGIBLE","reason_code":"ORIGINAL_H1_NOT_PROFILE_GRADIENT_BRAKE","eligible":False,"tail_path":"none","tail_credit_kmh":0.0,"tail_uncertainty_kmh":0.0}
    if not inputs["original_h1_brake"] or inputs["original_h1_reason"]!="profile_gradient":return base
    if int(inputs["prior_h1_brake_requests"])+1<3:return dict(base,reason_code="INSUFFICIENT_H1_BRAKE_HISTORY")
    feature=np.asarray([1.0,float(inputs["current_acceleration_mps2"]),float(inputs["mean_recent_acceleration_mps2"]),float(inputs["speed_kmh"])/100.0,abs(float(inputs["steer"])),abs(float(inputs["steer_change"])),float(inputs["prior_h1_brake_requests"])/20.0,0.0])
    prediction={h:float(feature@np.asarray(policy["direct_coefficients"][str(h)])) for h in range(1,25)}
    guard,guard_trace=causal_residual_guard(inputs["speed_kmh"],prediction,smooth,ds,index,policy["residual_guard_margin_kmh"],policy["timestep_seconds"])
    common=dict(base,residual_guard_trace=guard_trace,direct_predictions_kmh={str(h):float(inputs["speed_kmh"])+prediction[h]*3.6 for h in range(1,25)})
    if not guard:return dict(common,reason_code="CAUSAL_RESIDUAL_GUARD_FAILED")
    terminal=next_smoothed_minimum(smooth,ds,policy["custom_waypoint_indices"],index);speed_mps=max(float(inputs["speed_kmh"])/3.6,1.0);ticks=max(1,int(math.ceil(terminal["distance_m"]/speed_mps/policy["timestep_seconds"])))
    horizon=min(ticks,24);predicted=float(inputs["speed_kmh"])+prediction[horizon]*3.6;nearest=min(DIRECT_ERROR_HORIZONS,key=lambda x:abs(x-horizon));direct_uncertainty=2*float(policy["direct_rmse_kmh"][str(nearest)])
    result=dict(common,eligible=True,terminal_profile_index=terminal["profile_index"],terminal_custom_waypoint_index=terminal["custom_waypoint_index"],terminal_target_speed_kmh=terminal["target_speed_kmh"],terminal_distance_m=terminal["distance_m"],terminal_horizon_ticks=ticks,direct_prediction_horizon_ticks=horizon,predicted_speed_at_direct_horizon_kmh=predicted,direct_uncertainty_kmh=direct_uncertainty)
    if ticks>int(policy["maximum_horizon_ticks"]):return dict(result,status="ABSTAIN",reason_code="TERMINAL_BEYOND_48",tail_path="abstain")
    tail_credit=0.0;segment="none";tail_uncertainty=0.0
    if ticks>24:
        tail_credit,segment=_interpolate_tail(policy["tail_segment_credit_kmh"],ticks);tail_uncertainty=float(policy["tail_worst_optimistic_error_kmh"])
    upper=predicted+direct_uncertainty+tail_credit+tail_uncertainty;margin=terminal["target_speed_kmh"]-upper
    status="RELEASE" if upper<=terminal["target_speed_kmh"] else "BRAKE"
    return dict(result,status=status,reason_code=("TERMINAL_BOUND_SAFE" if status=="RELEASE" else "TERMINAL_BOUND_UNSAFE"),tail_path=("piecewise" if ticks>24 else "direct"),tail_segment=segment,tail_credit_kmh=tail_credit,tail_uncertainty_kmh=tail_uncertainty,terminal_speed_upper_bound_kmh=upper,terminal_safety_margin_kmh=margin)
