"""Terminal-constraint gate for the validated residual-release model."""

import csv
import datetime as dt
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

from experiments.harness.core import read_json, write_json
from .residual_deceleration import (
    HORIZONS, _accelerations, _cross_validate, _features, _fit_linear,
    _predict_linear, _release_events,
)
from .shadow_calibration import WINDOWS, _eligible_runs, decision
from .shadow_disagreement import BRAKE_ACTIVE, _number
from competition_code.shadow.terminal_policy import causal_residual_guard


SCHEMA_VERSION = 1
EXTENDED_HORIZONS = (2, 4, 6, 8, 10, 12)


def _median(values: Iterable[float]) -> Optional[float]:
    values = [float(x) for x in values if x is not None and math.isfinite(float(x))]
    return float(statistics.median(values)) if values else None


def _smooth_profile(profile: List[Dict[str, Any]]) -> np.ndarray:
    speed = np.asarray([x["stability_capped_planned_speed_kmh"] for x in profile], dtype=float)
    return np.asarray([statistics.median(speed.take([(i-1)%len(speed), i, (i+1)%len(speed)])) for i in range(len(speed))])


def terminal_objective(profile: List[Dict[str, Any]], smooth: np.ndarray, index: int,
                       maximum_bins: int = 80) -> Dict[str, Any]:
    """Next profile bottom before a sustained >=5 km/h recovery."""
    count = len(profile); running_min = float("inf"); minimum_offset = 1; recovery = 0
    descended = False
    for offset in range(1, maximum_bins + 1):
        candidate = (index + offset) % count; value = float(smooth[candidate])
        if value < running_min:
            running_min, minimum_offset, recovery = value, offset, 0
        descended = descended or value < float(smooth[index]) - 2.0
        if descended and offset > minimum_offset and value >= running_min + 5.0:
            recovery += 1
            if recovery >= 3: break
        else:
            recovery = 0
    distance = sum(float(profile[(index+x)%count]["ds_m"]) for x in range(minimum_offset))
    terminal = (index + minimum_offset) % count
    return {"profile_index": terminal, "custom_waypoint_index": int(profile[terminal]["custom_waypoint_index"]),
            "target_speed_kmh": running_min, "distance_m": distance, "offset_bins": minimum_offset,
            "recovery_detected": recovery >= 3}


def _extended_cv(examples: List[Dict[str, Any]]) -> Dict[str, Any]:
    attempts=sorted({x["attempt_id"] for x in examples}); errors={h:[] for h in EXTENDED_HORIZONS}
    for held in attempts:
        train=[x for x in examples if x["attempt_id"]!=held]; test=[x for x in examples if x["attempt_id"]==held]
        model=_fit_linear(train)
        for item in test:
            prediction=_predict_linear(model,item["features"])
            for h in EXTENDED_HORIZONS: errors[h].append((prediction[h]-item["targets"][h])*3.6)
    return {"speed_rmse_kmh":{str(h):math.sqrt(statistics.mean(x*x for x in values)) for h,values in errors.items()},
            "speed_mae_kmh":{str(h):statistics.mean(abs(x) for x in values) for h,values in errors.items()}}


def _shadow_episodes(flags: List[bool], gap: int = 2) -> List[Optional[int]]:
    result=[None]*len(flags); event=-1; last_active=-999
    for i,active in enumerate(flags):
        if active:
            if i-last_active>gap+1: event+=1
            result[i]=event; last_active=i
        elif i-last_active<=gap: result[i]=event
    return result


def _collect_candidates(runs, profile, model) -> Tuple[List[Dict[str, Any]], Dict[str, List[Dict[str, Any]]]]:
    smooth=_smooth_profile(profile); profile_ds=[float(x["ds_m"]) for x in profile]; candidates=[]; lap_data={}
    for item,all_rows in runs:
        by_lap=defaultdict(list)
        for row in all_rows: by_lap[int(row["lap"])].append(row)
        for lap,rows in by_lap.items():
            key="%s|%d"%(item["attempt_id"],lap); acceleration=_accelerations(rows)
            original=[_number(x,"shadow_brake")>BRAKE_ACTIVE for x in rows]; episode_ids=_shadow_episodes(original)
            episode_onset={}; accumulated=Counter(); objectives=[]
            for i,row in enumerate(rows):
                objectives.append(terminal_objective(profile,smooth,int(row["shadow_profile_index"])))
                eid=episode_ids[i]
                if eid is not None and eid not in episode_onset: episode_onset[eid]=(i,_number(row,"speed_kmh"))
                if original[i] and eid is not None: accumulated[eid]+=1
                command,_,cause=decision(_number(row,"speed_kmh"),_number(row,"shadow_target_speed_kmh"),_number(row,"shadow_required_acceleration_mps2"),_number(row,"shadow_deceleration_limit_mps2"))
                if not (original[i] and cause=="profile_gradient" and eid is not None and accumulated[eid]>=3 and i>=3): continue
                features=_features(rows,acceleration,i,accumulated[eid]-1,0); delta=_predict_linear(model,features)
                residual_safe,residual_guard_trace=causal_residual_guard(
                    _number(row,"speed_kmh"),delta,smooth,profile_ds,
                    int(row["shadow_profile_index"]),margin_kmh=1.0,
                )
                if not residual_safe: continue
                objective=objectives[i]; speed_mps=max(_number(row,"speed_kmh")/3.6,1); ticks_to_terminal=max(1,int(math.ceil(objective["distance_m"]/speed_mps/.05)))
                prediction_horizon=min(12,max(1,ticks_to_terminal)); predicted=_number(row,"speed_kmh")+delta[prediction_horizon]*3.6
                onset_i,onset_speed=episode_onset[eid]; required=max(0.0,onset_speed-objective["target_speed_kmh"]); achieved=max(0.0,onset_speed-_number(row,"speed_kmh"))
                candidates.append({"key":key,"row_index":i,"attempt_id":item["attempt_id"],"lap":lap,"tick":int(row["tick"]),"custom_waypoint_index":int(row["custom_waypoint_index"]),
                    "winner_brake":_number(row,"brake")>BRAKE_ACTIVE,"original_h1_brake":True,
                    "speed_kmh":_number(row,"speed_kmh"),"recent_acceleration_mps2":acceleration[i],"h1_brake_ticks_accumulated":accumulated[eid],
                    "onset_speed_kmh":onset_speed,"terminal":objective,"terminal_changes_from_previous_tick": i>0 and objectives[i-1]["profile_index"]!=objective["profile_index"],
                    "predicted_terminal_speed_kmh":predicted,"prediction_horizon_ticks":prediction_horizon,"terminal_beyond_model_horizon":ticks_to_terminal>12,
                    "predicted_time_to_terminal_s":objective["distance_m"]/speed_mps,"raw_terminal_margin_kmh":predicted-objective["target_speed_kmh"],
                    "required_delta_v_kmh":required,"achieved_delta_v_kmh":achieved,"remaining_delta_v_kmh":max(0,required-achieved),
                    "braking_progress_fraction":min(1.0,achieved/max(required,1e-6)),"remaining_required_decel_kmh_per_m":max(0,required-achieved)/max(objective["distance_m"],1),
                    "residual_guard_mode":"causal_distance_projection","residual_guard_trace":residual_guard_trace,
                    "predicted_speeds_kmh":{str(h):_number(row,"speed_kmh")+delta[h]*3.6 for h in EXTENDED_HORIZONS}})
            lap_data[key]=rows
    return candidates,lap_data


def _evaluate(candidates: List[Dict[str,Any]], rmse: Dict[str,float], margin:float,k:float,progress:float)->Dict[str,Any]:
    accepted=[]
    for x in candidates:
        uncertainty=rmse[str(x["prediction_horizon_ticks"])] if str(x["prediction_horizon_ticks"]) in rmse else rmse["12"]
        bound=x["predicted_terminal_speed_kmh"]+k*uncertainty
        if bound<=x["terminal"]["target_speed_kmh"]-margin and x["braking_progress_fraction"]>=progress:
            accepted.append(x)
    useful=sum(not x["winner_brake"] for x in accepted); false=sum(x["winner_brake"] for x in accepted)
    useful_universe=sum(not x["winner_brake"] for x in candidates)
    return {"margin_kmh":margin,"uncertainty_multiplier":k,"minimum_progress_fraction":progress,"proposed_releases":len(accepted),"useful_releases":useful,"false_releases":false,
            "beyond_validated_horizon_releases":sum(x["terminal_beyond_model_horizon"] for x in accepted),
            "precision":useful/max(len(accepted),1),"useful_release_recall":useful/max(useful_universe,1),"accepted_keys":{(x["key"],x["row_index"]) for x in accepted}}


def _event_metrics(runs, policies:Dict[str,set])->Dict[str,Any]:
    values={"%d-%d"%w:[] for w in WINDOWS}
    for item,all_rows in runs:
        by_lap=defaultdict(list)
        for row in all_rows:by_lap[int(row["lap"])].append(row)
        for lap,rows in by_lap.items():
            key="%s|%d"%(item["attempt_id"],lap); winner=[_number(x,"brake")>BRAKE_ACTIVE for x in rows];original=[_number(x,"shadow_brake")>BRAKE_ACTIVE for x in rows]
            flags={"winner":winner,"original":original}
            for name,accepted in policies.items():flags[name]=[value and (key,i) not in accepted for i,value in enumerate(original)]
            for window in WINDOWS:
                members=[i for i,x in enumerate(rows) if window[0]<=int(x["custom_waypoint_index"])<=window[1]]
                if not members:continue
                def metric(series):
                    active=[i for i in members if series[i]]
                    return {"onset_wp":int(rows[active[0]]["custom_waypoint_index"]) if active else None,"release_wp":int(rows[active[-1]]["custom_waypoint_index"]) if active else None,"brake_ticks":len(active)}
                values["%d-%d"%window].append({name:metric(series) for name,series in flags.items()})
    return {window:{name:{field:_median(x[name][field] for x in rows) for field in ("onset_wp","release_wp","brake_ticks")} for name in rows[0]} for window,rows in values.items() if rows}


def _clusters(candidates:List[Dict[str,Any]],residual_selected:set,terminal_selected:set)->List[Dict[str,Any]]:
    false=[x for x in candidates if (x["key"],x["row_index"]) in residual_selected and x["winner_brake"]]
    groups=defaultdict(list)
    for x in false:groups[int(x["custom_waypoint_index"]//10)*10].append(x)
    output=[]
    for bucket,items in groups.items():
        output.append({"waypoint_range":[min(x["custom_waypoint_index"] for x in items),max(x["custom_waypoint_index"] for x in items)],"false_release_ticks":len(items),
            "rejected_by_terminal_gate":sum((x["key"],x["row_index"]) not in terminal_selected for x in items),
            "terminal_waypoint_median":_median(x["terminal"]["custom_waypoint_index"] for x in items),"remaining_delta_v_kmh_median":_median(x["remaining_delta_v_kmh"] for x in items),
            "distance_remaining_m_median":_median(x["terminal"]["distance_m"] for x in items),"h1_brake_ticks_median":_median(x["h1_brake_ticks_accumulated"] for x in items),
            "progress_fraction_median":_median(x["braking_progress_fraction"] for x in items),"raw_terminal_margin_kmh_median":_median(x["raw_terminal_margin_kmh"] for x in items)})
    return sorted(output,key=lambda x:x["false_release_ticks"],reverse=True)


def _report(result):
    lines=["# Terminal-constrained residual-release analysis","","## Braking objective","",result["terminal_objective"]["definition"],"",
        "Terminal identity remains stable on %.1f%% of adjacent candidate ticks."%(100*result["terminal_objective_stability"]["stable_fraction"]),"",
        "## Braking progress separation","","| Population | Samples | Median progress | Remaining Δv | Remaining distance |","|---|---:|---:|---:|---:|"]
    for name,x in result["braking_progress_analysis"].items():
        lines.append("| %s | %d | %.0f%% | %.1f km/h | %.1f m |"%(name,x["samples"],100*x["progress_fraction_median"],x["remaining_delta_v_kmh_median"],x["distance_remaining_m_median"]))
    lines += ["",
        "## Extended residual prediction","","Direct linear-history prediction is used through +12 ticks. Beyond +12, speed is held at the conservative +12 prediction, giving no credit for further deceleration.","",
        "| Horizon | RMSE | MAE |","|---:|---:|---:|"]
    for h in EXTENDED_HORIZONS:lines.append("| +%d | %.2f km/h | %.2f km/h |"%(h,result["prediction_error"]["speed_rmse_kmh"][str(h)],result["prediction_error"]["speed_mae_kmh"][str(h)]))
    lines += ["","## Precision/recall operating points","","| Margin | k | Progress | Releases | Useful | False | Precision | Useful recall |","|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for x in result["operating_points"]:lines.append("| %.1f | %.1f | %.0f%% | %d | %d | %d | %.1f%% | %.1f%% |"%(x["margin_kmh"],x["uncertainty_multiplier"],100*x["minimum_progress_fraction"],x["proposed_releases"],x["useful_releases"],x["false_releases"],100*x["precision"],100*x["useful_release_recall"]))
    s=result["selected_operating_point"];lines += ["","Selected offline point: margin %.1f km/h, k=%.1f, progress %.0f%% — %d useful, %d false releases; %d extend beyond the validated horizon."%(s["margin_kmh"],s["uncertainty_multiplier"],100*s["minimum_progress_fraction"],s["useful_releases"],s["false_releases"],s["beyond_validated_horizon_releases"]),"",
        "## Original false-release archetypes","","| WP cluster | False ticks | Terminal WP | Remaining Δv | Distance | H1 ticks | Progress | Rejected by gate |","|---|---:|---:|---:|---:|---:|---:|---:|"]
    for x in result["false_release_clusters"][:12]:
        lines.append("| %d–%d | %d | %.0f | %.1f km/h | %.1f m | %.1f | %.0f%% | %d |"%(x["waypoint_range"][0],x["waypoint_range"][1],x["false_release_ticks"],x["terminal_waypoint_median"],x["remaining_delta_v_kmh_median"],x["distance_remaining_m_median"],x["h1_brake_ticks_median"],100*x["progress_fraction_median"],x["rejected_by_terminal_gate"]))
    lines += ["",
        "## Major events","","| WP | Winner | Original H1 | Residual-only | Terminal-constrained |","|---|---|---|---|---|"]
    for wp,x in result["major_events"].items():
        def fmt(v):return "%s / %s / %s"%(v["onset_wp"],v["release_wp"],v["brake_ticks"])
        lines.append("| %s | %s | %s | %s | %s |"%(wp,fmt(x["winner"]),fmt(x["original"]),fmt(x["residual_only"]),fmt(x["terminal_constrained"])))
    lines += ["","## Recommendation","",result["recommendation"],"","No live or applied controller behavior was changed.",""]
    return "\n".join(lines)


def analyze(root:Path,results_dir:str="experiment_results")->Dict[str,Any]:
    results_root=root/results_dir;runs=_eligible_runs(root,results_root);profile=read_json(results_root/"analysis/velocity_profile_v2/latest.json")["distance_profile"]
    _,examples=_release_events(runs);model=_fit_linear(examples);prediction_error=_extended_cv(examples);candidates,_=_collect_candidates(runs,profile,model)
    margins=(0,1,2,3,5);ks=(0,1,2);progresses=(0,.25,.5,.75)
    evaluated=[_evaluate(candidates,prediction_error["speed_rmse_kmh"],m,k,p) for m in margins for k in ks for p in progresses]
    serializable=lambda x:{key:value for key,value in x.items() if key!="accepted_keys"}
    # Gate requires >=90% precision and retains the most useful releases; ties
    # favor fewer false releases and a stronger uncertainty bound.
    viable=[x for x in evaluated if x["precision"]>=.9 and x["uncertainty_multiplier"]>=2]
    selected=max(viable,key=lambda x:(x["useful_releases"],-x["false_releases"],-x["margin_kmh"])) if viable else max(evaluated,key=lambda x:x["precision"])
    residual=_evaluate(candidates,prediction_error["speed_rmse_kmh"],-1e6,0,0)
    events=_event_metrics(runs,{"residual_only":residual["accepted_keys"],"terminal_constrained":selected["accepted_keys"]})
    clusters=_clusters(candidates,residual["accepted_keys"],selected["accepted_keys"])
    false_reduction=527-selected["false_releases"]
    progress_analysis={label:{"samples":len(group),"progress_fraction_median":_median(x["braking_progress_fraction"] for x in group),
        "remaining_delta_v_kmh_median":_median(x["remaining_delta_v_kmh"] for x in group),"distance_remaining_m_median":_median(x["terminal"]["distance_m"] for x in group)}
        for label,group in (("useful",[x for x in candidates if not x["winner_brake"]]),("false_release",[x for x in candidates if x["winner_brake"]]))}
    ready=(selected["false_releases"]<=52 and selected["useful_releases"]>=.5*1930 and all(x["terminal_beyond_model_horizon"] is False for x in candidates if (x["key"],x["row_index"]) in selected["accepted_keys"]))
    result={"schema_version":SCHEMA_VERSION,"event_type":"terminal_constrained_release_analysis","created_utc":dt.datetime.now(dt.timezone.utc).isoformat(),
        "population":{"attempts":len(runs),"residual_release_candidates":len(candidates)},
        "terminal_objective":{"definition":"Next minimum of the 3-bin median-smoothed stability-capped profile, ending when three bins establish a sustained >=5 km/h recovery; scan limited to 400 m.","maximum_scan_m":400,"recovery_threshold_kmh":5,"recovery_bins":3},
        "terminal_objective_stability":{"candidate_ticks":len(candidates),"changed_from_previous_tick":sum(x["terminal_changes_from_previous_tick"] for x in candidates),"stable_fraction":1-sum(x["terminal_changes_from_previous_tick"] for x in candidates)/max(len(candidates),1)},
        "braking_progress_analysis":progress_analysis,
        "prediction_method":{"direct_horizon_ticks":12,"beyond_horizon":"hold +12 predicted speed constant; no credit for additional deceleration","prediction_error":prediction_error},"prediction_error":prediction_error,
        "operating_points":[serializable(x) for x in evaluated],"selected_operating_point":serializable(selected),
        "false_release_reduction_from_residual_only":false_reduction,"false_release_clusters":clusters,"major_events":events,
        "shadow_ready":ready,"recommendation":("Terminal gating passes every shadow-readiness criterion; a shadow-only implementation may be prepared." if ready else "Not shadow-ready. Although terminal gating improves release precision, it does not satisfy all order-of-magnitude false-release, useful-retention, horizon-support, and repeated-event gates. Continue offline refinement without waypoint exceptions."),}
    output=results_root/"analysis/terminal_release";output.mkdir(parents=True,exist_ok=True);result_path=output/"latest.json";report_path=output/"latest.md";trace_path=output/"wp1774_1856_trace.csv"
    selected_keys=selected["accepted_keys"];trace=[]
    for x in candidates:
        if 1774<=x["custom_waypoint_index"]<=1856:trace.append({**{k:v for k,v in x.items() if k not in ("key","row_index","terminal","predicted_speeds_kmh")},"terminal_waypoint":x["terminal"]["custom_waypoint_index"],"terminal_distance_m":x["terminal"]["distance_m"],"terminal_target_speed_kmh":x["terminal"]["target_speed_kmh"],"selected_release":(x["key"],x["row_index"]) in selected_keys,**{"predicted_speed_plus_%d_kmh"%h:x["predicted_speeds_kmh"][str(h)] for h in EXTENDED_HORIZONS}})
    with trace_path.open("w",encoding="utf-8",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(trace[0]));writer.writeheader();writer.writerows(trace)
    result.update({"result_path":str(result_path.relative_to(root)),"human_report_path":str(report_path.relative_to(root)),"wp1774_trace_path":str(trace_path.relative_to(root))});write_json(result_path,result);report_path.write_text(_report(result),encoding="utf-8")
    return result
