"""Conservative empirical bridge from the validated +24 residual horizon.

Offline analysis only.  The required-deceleration objective, two-RMSE direct
uncertainty, release condition, and brake-onset policy are deliberately fixed.
"""

import csv
import datetime as dt
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from experiments.harness.core import read_json, write_json
from competition_code.shadow.terminal_policy import evaluate_terminal_tail
from .residual_deceleration import _accelerations, _features
from .shadow_calibration import _eligible_runs
from .shadow_disagreement import BRAKE_ACTIVE, _number
from .terminal_prediction_v2 import (
    _bound, _collect_candidates, _cv, _event_metrics, _extended_examples,
    _fit, _median, _objective_constraints, _percentile, _smooth_profile,
    _tail_statistics,
)

SCHEMA_VERSION = 1
HORIZONS = (24, 28, 32, 36, 40, 48)
TAIL_HORIZONS = (28, 32, 36, 40, 48)
MAJOR_WINDOWS = ((382, 454), (789, 820), (1774, 1856), (2496, 2574))


def _prior_braking(rows, index):
    ticks = 0; integrated = 0.0
    for cursor in range(index - 1, -1, -1):
        brake = _number(rows[cursor], "brake")
        if brake <= BRAKE_ACTIVE: break
        ticks += 1; integrated += brake
    return ticks, integrated


def _long_release_events(runs):
    events = []
    for item, all_rows in runs:
        by_lap = defaultdict(list)
        for row in all_rows: by_lap[int(row["lap"])].append(row)
        for lap, rows in by_lap.items():
            acceleration = _accelerations(rows)
            for index in range(8, len(rows) - 25):
                if not (_number(rows[index-1], "brake") > BRAKE_ACTIVE and _number(rows[index], "brake") <= BRAKE_ACTIVE): continue
                prior_ticks, integrated = _prior_braking(rows, index)
                if prior_ticks < 2: continue
                maximum = 0
                for horizon in range(1, 49):
                    if index+horizon >= len(rows) or _number(rows[index+horizon], "brake") > BRAKE_ACTIVE: break
                    maximum = horizon
                if maximum < 24: continue
                speed0 = _number(rows[index], "speed_kmh")
                values = {}
                for horizon in HORIZONS:
                    if horizon > maximum: continue
                    cursor = index+horizon
                    acc_window = acceleration[max(index+20, cursor-4):cursor+1]
                    values[horizon] = {
                        "speed_delta_kmh": _number(rows[cursor], "speed_kmh")-speed0,
                        "acceleration_mps2": acceleration[cursor],
                        "mean_acceleration_mps2": statistics.mean(acc_window),
                        "throttle": _number(rows[cursor], "throttle"),
                    }
                v24 = values[24]
                events.append({
                    "attempt_id": item["attempt_id"], "lap": lap, "release_tick": int(rows[index]["tick"]),
                    "prior_brake_ticks": prior_ticks, "integrated_prior_brake": integrated,
                    "release_speed_kmh": speed0, "speed_24_kmh": speed0+v24["speed_delta_kmh"],
                    "acceleration_24_mps2": v24["acceleration_mps2"],
                    "mean_acceleration_20_24_mps2": v24["mean_acceleration_mps2"],
                    "acceleration_slope_20_24_mps2_per_tick": (acceleration[index+24]-acceleration[index+20])/4.0,
                    "achieved_delta_v_24_kmh": -v24["speed_delta_kmh"], "maximum_horizon": maximum,
                    "values": values,
                })
    return events


def _summary(events):
    trajectory = {}
    for horizon in HORIZONS:
        supported = [x for x in events if horizon in x["values"]]
        deltas = [x["values"][horizon]["speed_delta_kmh"] for x in supported]
        accels = [x["values"][horizon]["acceleration_mps2"] for x in supported]
        trajectory[str(horizon)] = {
            "events": len(supported),
            "speed_delta_kmh": {"p10":_percentile(deltas,10),"p25":_percentile(deltas,25),"median":_median(deltas),"p75":_percentile(deltas,75),"p90":_percentile(deltas,90)},
            "acceleration_mps2": {"p10":_percentile(accels,10),"p25":_percentile(accels,25),"median":_median(accels),"p75":_percentile(accels,75),"p90":_percentile(accels,90)},
            "meaningfully_decelerating_fraction": sum(a < -1.0 for a in accels)/max(len(accels),1),
            "coast_or_steady_fraction": sum(abs(a) <= 1.0 for a in accels)/max(len(accels),1),
            "throttle_positive_fraction": sum(x["values"][horizon]["throttle"] > .01 for x in supported)/max(len(supported),1),
            "median_throttle": _median(x["values"][horizon]["throttle"] for x in supported),
        }
    predictors=("acceleration_24_mps2","mean_acceleration_20_24_mps2","acceleration_slope_20_24_mps2_per_tick","speed_24_kmh","prior_brake_ticks","integrated_prior_brake","achieved_delta_v_24_kmh")
    supported=[x for x in events if 48 in x["values"]]
    y=np.asarray([x["values"][48]["speed_delta_kmh"]-x["values"][24]["speed_delta_kmh"] for x in supported])
    dependence={}
    for field in predictors:
        values=np.asarray([x[field] for x in supported],dtype=float)
        dependence[field]={"events":len(values),"pearson_r_with_additional_24_to_48_speed_change":float(np.corrcoef(values,y)[0,1]) if len(values)>2 and np.std(values)>0 else None}
    return {"trajectory":trajectory,"tail_state_dependence":dependence}


def _state(event):
    return np.asarray([1.0,event["acceleration_24_mps2"],event["mean_acceleration_20_24_mps2"],event["speed_24_kmh"]/100.0],dtype=float)


def _fit_bridge(events, kind):
    if kind=="percentile_tail":
        return {h:_percentile([x["values"][h]["speed_delta_kmh"]-x["values"][24]["speed_delta_kmh"] for x in events if h in x["values"]],90) for h in TAIL_HORIZONS}
    if kind=="piecewise_envelope":
        output={};previous=24
        for h in TAIL_HORIZONS:
            output[h]=_percentile([x["values"][h]["speed_delta_kmh"]-x["values"][previous]["speed_delta_kmh"] for x in events if h in x["values"] and previous in x["values"]],90);previous=h
        return output
    if kind=="decay_to_zero":
        best=None
        for tau in np.linspace(2,40,77):
            errors=[]
            for x in events:
                for h in TAIL_HORIZONS:
                    if h not in x["values"]:continue
                    pred=.05*sum(x["acceleration_24_mps2"]*math.exp(-step/tau) for step in range(1,h-23))*3.6
                    actual=x["values"][h]["speed_delta_kmh"]-x["values"][24]["speed_delta_kmh"]
                    errors.append((pred-actual)**2)
            score=statistics.mean(errors)
            if best is None or score<best[0]:best=(score,float(tau))
        return best[1]
    if kind=="state_conditioned":
        output={}
        for h in TAIL_HORIZONS:
            rows=[x for x in events if h in x["values"]];matrix=np.vstack([_state(x) for x in rows]);target=np.asarray([x["values"][h]["speed_delta_kmh"]-x["values"][24]["speed_delta_kmh"] for x in rows]);ridge=np.eye(4)*1e-4;ridge[0,0]=0
            output[h]=np.linalg.solve(matrix.T@matrix+ridge,matrix.T@target)
        return output
    raise ValueError(kind)


def _interpolate(values, ticks):
    ticks=max(24,min(48,ticks));upper=min((h for h in TAIL_HORIZONS if h>=ticks),default=48);lower=max((h for h in (24,)+TAIL_HORIZONS if h<=ticks),default=24)
    if lower==24:lo=0.0
    else:lo=float(values[lower])
    hi=float(values[upper])
    return lo if upper==lower else lo+(hi-lo)*(ticks-lower)/(upper-lower)


def _predict_bridge(model,kind,state,ticks):
    if kind=="zero_credit":return 0.0
    if kind=="percentile_tail":return _interpolate(model,ticks)
    if kind=="piecewise_envelope":
        # Each segment receives only its independently observed p90 loss.
        cumulative={};total=0.0
        for h in TAIL_HORIZONS:total+=model[h];cumulative[h]=total
        return _interpolate(cumulative,ticks)
    if kind=="decay_to_zero":return .05*sum(state["acceleration_24_mps2"]*math.exp(-step/model) for step in range(1,min(ticks,48)-23))*3.6
    if kind=="state_conditioned":return _interpolate({h:float(_state(state)@model[h]) for h in TAIL_HORIZONS},ticks)
    raise ValueError(kind)


def _cross_validate(events):
    kinds=("percentile_tail","decay_to_zero","piecewise_envelope","state_conditioned");errors={k:[] for k in kinds};attempts=sorted({x["attempt_id"] for x in events})
    for held in attempts:
        train=[x for x in events if x["attempt_id"]!=held];test=[x for x in events if x["attempt_id"]==held]
        models={k:_fit_bridge(train,k) for k in kinds}
        for x in test:
            for h in TAIL_HORIZONS:
                if h not in x["values"]:continue
                actual=x["values"][h]["speed_delta_kmh"]-x["values"][24]["speed_delta_kmh"]
                for kind in kinds:errors[kind].append(_predict_bridge(models[kind],kind,x,h)-actual)
    output={}
    for kind,values in errors.items():
        optimistic=[max(0.0,-x) for x in values]
        output[kind]={"samples":len(values),"mean_error_kmh":statistics.mean(values),"mae_kmh":statistics.mean(abs(x) for x in values),"rmse_kmh":math.sqrt(statistics.mean(x*x for x in values)),"worst_optimistic_error_kmh":max(optimistic),"p95_optimistic_error_kmh":_percentile(optimistic,95),"optimistic_error_counts":{">1":sum(x>1 for x in optimistic),">2":sum(x>2 for x in optimistic),">3":sum(x>3 for x in optimistic),">5":sum(x>5 for x in optimistic)}}
    return output


def _candidate_state(prediction,candidate):
    a24=(prediction[24]-prediction[23])/.05
    changes=[(prediction[h]-prediction[h-1])/.05 for h in range(20,25)]
    return {"acceleration_24_mps2":a24,"mean_acceleration_20_24_mps2":statistics.mean(changes),"speed_24_kmh":candidate["speed_kmh"]+prediction[24]*3.6}


def _evaluate(candidates,lap_data,profile,direct_model,direct_cv,bridge_model,bridge_cv,kind,objective="required_deceleration",keep=False):
    smooth=_smooth_profile(profile);accepted=set();audits=[];unsupported=0;within_before=within_after=beyond_after=false=useful=0
    acc_cache={key:_accelerations(rows) for key,rows in lap_data.items()}
    for x in candidates:
        rows=lap_data[x["key"]];row=rows[x["row_index"]];features=_features(rows,acc_cache[x["key"]],x["row_index"],x["h1_brake_ticks_accumulated"]-1,0);prediction={h:float(features@direct_model[h]) for h in range(1,25)}
        constraint=_objective_constraints(profile,smooth,int(row["shadow_profile_index"]),x["speed_kmh"],objective)[0]
        base=_bound(x,constraint,prediction,direct_cv,{},"direct_24_2sigma");ticks=base["ticks"];current_safe=base["upper_speed_kmh"]<=constraint["target_speed_kmh"]
        if ticks<=24:
            within_before+=current_safe;safe=current_safe;within_after+=safe;classification="within_24"
        elif kind=="zero_credit":
            # This reproduces v2: the +24 upper bound is frozen all the way to
            # the terminal.  It is reported as unsupported rather than eligible
            # for selection, but remains the exact comparison baseline.
            safe=current_safe;unsupported+=safe;classification="current_unvalidated_freeze_release" if safe else "insufficient_margin"
            beyond_after+=safe
        elif ticks>48:
            safe=False;unsupported+=1;classification="unsupported_terminal_too_far"
        else:
            state=_candidate_state(prediction,x);credit=_predict_bridge(bridge_model,kind,state,ticks)
            # Bound every optimistic error observed in attempt-held-out replay,
            # not merely its 95th percentile.  Direct and tail uncertainty stay
            # separate and are then added in terminal-speed space.
            tail_unc=bridge_cv[kind]["worst_optimistic_error_kmh"]
            upper=base["upper_speed_kmh"]+credit+tail_unc;safe=upper<=constraint["target_speed_kmh"]
            base=dict(base,tail_credit_kmh=credit,tail_uncertainty_kmh=tail_unc,upper_speed_kmh=upper);classification="safely_recoverable_release" if safe else ("model_uncertainty_too_large" if upper-tail_unc<=constraint["target_speed_kmh"] else "insufficient_margin")
            beyond_after+=safe
        if safe:
            accepted.add((x["key"],x["row_index"]));useful+=not x["winner_brake"];false+=x["winner_brake"]
        if keep and (ticks>24 or any(lo<=x["custom_waypoint_index"]<=hi for lo,hi in MAJOR_WINDOWS)):
            audits.append({"candidate":x,"constraint":constraint,"bound":base,"ticks":ticks,"current_safe":current_safe,"bridge_safe":safe,"classification":classification})
    universe=sum(not x["winner_brake"] for x in candidates)
    return {"objective":objective,"bridge":kind,"accepted":accepted,"audits":audits,"useful":useful,"false":false,"precision":useful/max(useful+false,1),"recall":useful/max(universe,1),"more_conservative_ticks":universe-useful,"less_conservative_ticks":false,"unsupported":unsupported,"within_24_before":within_before,"within_24_after":within_after,"beyond_24_after":beyond_after,"brake_onset_changes":0}


def _fmt_event(value):return "%s / %s / %s"%(value["onset_wp"],value["release_wp"],value["brake_ticks"])


def _report(result):
    lines=["# Conservative empirical tail bridge","","The required-deceleration objective, release gate, direct-model 2×RMSE bound, and brake onset are fixed.","","## Long-horizon release transient","","| Tick | Events | Speed Δ p10/p25/p50/p75/p90 | Median accel | Decelerating | Coast/steady | Throttle on |","|---:|---:|---|---:|---:|---:|---:|"]
    for h in HORIZONS:
        x=result["long_horizon_characterization"]["trajectory"][str(h)];d=x["speed_delta_kmh"]
        lines.append("| +%d | %d | %.1f / %.1f / %.1f / %.1f / %.1f | %.2f | %.1f%% | %.1f%% | %.1f%% |"%(h,x["events"],d["p10"],d["p25"],d["median"],d["p75"],d["p90"],x["acceleration_mps2"]["median"],100*x["meaningfully_decelerating_fraction"],100*x["coast_or_steady_fraction"],100*x["throttle_positive_fraction"]))
    lines += ["","## Attempt-held-out bridge errors","","| Bridge | MAE | RMSE | Worst optimistic | p95 optimistic | >1 / >2 / >3 / >5 km/h |","|---|---:|---:|---:|---:|---|"]
    for name,x in result["cross_validation"].items():
        c=x["optimistic_error_counts"];lines.append("| %s | %.2f | %.2f | %.2f | %.2f | %d / %d / %d / %d |"%(name,x["mae_kmh"],x["rmse_kmh"],x["worst_optimistic_error_kmh"],x["p95_optimistic_error_kmh"],c[">1"],c[">2"],c[">3"],c[">5"]))
    lines += ["","Selected causal policy: **%s + %s**. Tail uncertainty added to the terminal-speed upper bound: **%.2f km/h**."%(result["selected_objective"],result["selected_bridge"],result["tail_uncertainty_bound_kmh"]),result["selection_reason"]]
    lines += ["","## Fixed-gate replay","","| Objective | Bridge | Useful | Recall | False | Precision | Within-24 before/after | Beyond-24 accepted | Unsupported |","|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for x in result["replay"]:lines.append("| %s | %s | %d | %.1f%% | %d | %.1f%% | %d/%d | %d | %d |"%(x["objective"],x["bridge"],x["useful"],100*x["recall"],x["false"],100*x["precision"],x["within_24_before"],x["within_24_after"],x["beyond_24_after"],x["unsupported"]))
    c=result["beyond_24_classification"];lines += ["","## Current %d beyond-horizon decisions"%result["population"]["current_beyond_24_decisions"],"",*('- %s: %d'%(k,v) for k,v in c.items()),"","## Major events","","| WP | Winner | Original H1 | +24 freeze | Selected bridge |","|---|---|---|---|---|"]
    for wp,x in result["major_events"].items():lines.append("| %s | %s | %s | %s | %s |"%(wp,_fmt_event(x["winner"]),_fmt_event(x["original"]),_fmt_event(x["freeze_24"]),_fmt_event(x["selected_bridge"])))
    lines += ["","## Recommendation","",result["recommendation"],"","No live controller or shadow command was changed.",""]
    return "\n".join(lines)


def analyze(root:Path,results_dir="experiment_results"):
    rr=root/results_dir;runs=_eligible_runs(root,rr);profile=read_json(rr/"analysis/velocity_profile_v2/latest.json")["distance_profile"]
    long_events=_long_release_events(runs);characterization=_summary(long_events);bridge_cv=_cross_validate(long_events)
    _,examples=_extended_examples(runs);direct_model=_fit(examples);direct_cv=_cv(examples);candidates,lap_data=_collect_candidates(runs,profile,direct_model)
    kinds=("zero_credit","percentile_tail","decay_to_zero","piecewise_envelope","state_conditioned");models={k:(None if k=="zero_credit" else _fit_bridge(long_events,k)) for k in kinds}
    objectives=("next_minimum","required_deceleration")
    replay=[_evaluate(candidates,lap_data,profile,direct_model,direct_cv,models[k],bridge_cv,k,objective=o,keep=True) for o in objectives for k in kinds]
    # Tail safety outranks recall.  Only bridges whose held-out worst optimism
    # is <=1 km/h may compete; zero-credit is the unsupported v2 reference.
    viable=[x for x in replay if x["bridge"]!="zero_credit" and x["false"]==0 and bridge_cv[x["bridge"]]["worst_optimistic_error_kmh"]<=1.0]
    selected=(max(viable,key=lambda x:(x["useful"],-x["unsupported"])) if viable
              else min((x for x in replay if x["bridge"]!="zero_credit"),
                       key=lambda x:(x["false"],-x["useful"])))
    # Authoritative selected-policy replay goes through the exact pure function
    # used live.  The older vectorized implementation remains only for model
    # comparisons and must agree exactly on categorical release decisions.
    runtime_profile={"target_speed_kmh":[x["stability_capped_planned_speed_kmh"] for x in profile],"ds_m":[x["ds_m"] for x in profile]};shared_accepted=set();shared_counts=Counter();acc_cache={key:_accelerations(rows) for key,rows in lap_data.items()}
    for x in candidates:
        rows=lap_data[x["key"]];i=x["row_index"];acc=acc_cache[x["key"]];recent=acc[max(0,i-2):i+1]
        opinion=evaluate_terminal_tail({"profile_index":int(rows[i]["shadow_profile_index"]),"speed_kmh":x["speed_kmh"],"current_acceleration_mps2":acc[i],"mean_recent_acceleration_mps2":statistics.mean(recent),"steer":_number(rows[i],"steer"),"steer_change":_number(rows[i],"steer_change"),"original_h1_brake":True,"original_h1_reason":"profile_gradient","prior_h1_brake_requests":x["h1_brake_ticks_accumulated"]-1},runtime_profile)
        shared_counts[opinion["status"]]+=1
        if opinion["status"]=="RELEASE":shared_accepted.add((x["key"],x["row_index"]))
    shared_mismatches=selected["accepted"]^shared_accepted
    if shared_mismatches:raise AssertionError("shared terminal policy disagrees on %d decisions"%len(shared_mismatches))
    freeze=next(x for x in replay if x["bridge"]=="zero_credit" and x["objective"]==selected["objective"])
    major=_event_metrics(runs,{"freeze_24":freeze["accepted"],"selected_bridge":selected["accepted"]})
    # Classify exactly the v2 population that was accepted despite needing >24 ticks.
    current=freeze
    current_140=[a for a in current["audits"] if a["ticks"]>24 and a["current_safe"]]
    selected_by={(a["candidate"]["key"],a["candidate"]["row_index"]):a for a in selected["audits"]}
    classifications=Counter();rows=[]
    for a in current_140:
        x=a["candidate"];chosen=selected_by.get((x["key"],x["row_index"]),a);classification=chosen["classification"]
        classifications[classification]+=1;b=chosen["bound"];c=chosen["constraint"]
        rows.append({"attempt_id":x["attempt_id"],"lap":x["lap"],"tick":x["tick"],"current_waypoint":x["custom_waypoint_index"],"terminal_waypoint":c["custom_waypoint_index"],"distance_m":c["distance_m"],"terminal_ticks":chosen["ticks"],"ticks_beyond_24":max(0,chosen["ticks"]-24),"predicted_speed_plus_24_kmh":b["mean_speed_kmh"],"freeze_upper_speed_kmh":b["mean_speed_kmh"]+2*direct_cv["rmse_kmh"]["24"],"bridged_mean_speed_kmh":b.get("mean_speed_kmh",0)+b.get("tail_credit_kmh",0),"bridged_upper_speed_kmh":b["upper_speed_kmh"],"terminal_target_speed_kmh":c["target_speed_kmh"],"current_release":True,"bridge_release":chosen["bridge_safe"],"winner_brake":x["winner_brake"],"classification":classification})
    serial=lambda x:{k:v for k,v in x.items() if k not in ("accepted","audits")}
    # Unsupported >48 cases explicitly abstain and therefore do not prevent a
    # second-opinion-only shadow implementation.
    unsupported_accepts=sum(a["bridge_safe"] for a in selected["audits"] if a["classification"]=="unsupported_terminal_too_far")
    ready=selected["false"]==0 and selected["recall"]>=.5 and unsupported_accepts==0 and selected["within_24_before"]==selected["within_24_after"]
    result={"schema_version":SCHEMA_VERSION,"event_type":"terminal_tail_bridge_analysis","created_utc":dt.datetime.now(dt.timezone.utc).isoformat(),"population":{"attempts":len(runs),"clean_release_events_plus_24":len(long_events),"release_candidates":len(candidates),"current_beyond_24_decisions":len(current_140)},"eligibility_mode":"causal_distance_projection","shared_policy_equivalence":{"categorical_mismatches":len(shared_mismatches),"status_counts":dict(shared_counts)},"long_horizon_characterization":characterization,"cross_validation":bridge_cv,"selected_objective":selected["objective"],"selected_bridge":selected["bridge"],"selection_reason":"Only zero-false objective/bridge pairs with <=1 km/h worst attempt-held-out optimistic error were eligible; maximize useful releases within that safety set.","tail_uncertainty_bound_kmh":bridge_cv[selected["bridge"]]["worst_optimistic_error_kmh"],"replay":[serial(x) for x in replay],"beyond_24_classification":dict(classifications),"major_events":major,"shadow_ready":ready,"recommendation":("Shadow-only second-opinion implementation is supported; applied commands must remain unchanged." if ready else "Not shadow-ready. The bridge must continue to abstain wherever empirical tail support or its conservative optimism bound is insufficient.")}
    out=rr/"analysis/terminal_tail_bridge";out.mkdir(parents=True,exist_ok=True);jp=out/"latest.json";mp=out/"latest.md";cp=out/"beyond_24_decisions.csv"
    if rows:
        with cp.open("w",encoding="utf-8",newline="") as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    result.update({"result_path":str(jp.relative_to(root)),"human_report_path":str(mp.relative_to(root)),"beyond_24_decisions_path":str(cp.relative_to(root))});write_json(jp,result);mp.write_text(_report(result),encoding="utf-8");return result
