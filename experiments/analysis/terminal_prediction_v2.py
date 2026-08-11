"""Extended residual envelopes and alternative terminal objectives, offline only."""

import csv
import datetime as dt
import gc
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np

from experiments.harness.core import read_json, write_json
from .residual_deceleration import _accelerations, _features, _fit_linear, _predict_linear
from .shadow_calibration import WINDOWS, _eligible_runs
from .shadow_disagreement import BRAKE_ACTIVE, _number
from .terminal_release import (
    _collect_candidates, _event_metrics, _smooth_profile, terminal_objective,
)


SCHEMA_VERSION = 1
HORIZONS = (2, 4, 6, 8, 10, 12, 16, 20, 24)


def _median(values: Iterable[float]):
    values=[float(x) for x in values if x is not None and math.isfinite(float(x))]
    return float(statistics.median(values)) if values else None


def _percentile(values: Iterable[float], q: float):
    values=list(values);return float(np.percentile(values,q)) if values else None


def _extended_examples(runs):
    examples=[]; releases=[]
    for item,all_rows in runs:
        by_lap=defaultdict(list)
        for row in all_rows:by_lap[int(row["lap"])].append(row)
        for lap,rows in by_lap.items():
            acceleration=_accelerations(rows)
            for index in range(8,len(rows)-25):
                if not (_number(rows[index-1],"brake")>BRAKE_ACTIVE and _number(rows[index],"brake")<=BRAKE_ACTIVE):continue
                prior=0
                for cursor in range(index-1,-1,-1):
                    if _number(rows[cursor],"brake")<=BRAKE_ACTIVE:break
                    prior+=1
                if prior<2 or any(_number(rows[index+x],"brake")>BRAKE_ACTIVE for x in range(25)):continue
                release={"attempt_id":item["attempt_id"],"lap":lap,"index":index,"speed_kmh":_number(rows[index],"speed_kmh"),"prior_brake_ticks":prior,"abs_steer":abs(_number(rows[index],"steer")),"current_acceleration_mps2":acceleration[index],"deltas":{h:_number(rows[index+h],"speed_kmh")-_number(rows[index],"speed_kmh") for h in HORIZONS}}
                releases.append(release)
                # Multiple post-release anchors improve support while grouping CV
                # by attempt prevents trajectory leakage between train/test.
                for since in range(5):
                    anchor=index+since
                    if anchor+24>=len(rows) or any(_number(rows[anchor+x],"brake")>BRAKE_ACTIVE for x in range(25)):break
                    examples.append({"attempt_id":item["attempt_id"],"features":_features(rows,acceleration,anchor,prior,since),"targets":{h:(_number(rows[anchor+h],"speed_kmh")-_number(rows[anchor],"speed_kmh"))/3.6 for h in range(1,25)}})
    return releases,examples


def _fit(examples):
    x=np.vstack([e["features"] for e in examples]);ridge=np.eye(x.shape[1])*1e-5;ridge[0,0]=0
    return {h:np.linalg.solve(x.T@x+ridge,x.T@np.asarray([e["targets"][h] for e in examples])) for h in range(1,25)}


def _cv(examples):
    errors={h:[] for h in HORIZONS};attempts=sorted({x["attempt_id"] for x in examples})
    for held in attempts:
        train=[x for x in examples if x["attempt_id"]!=held];test=[x for x in examples if x["attempt_id"]==held];model=_fit(train)
        for e in test:
            prediction={h:float(e["features"]@model[h]) for h in HORIZONS}
            for h in HORIZONS:errors[h].append((prediction[h]-e["targets"][h])*3.6)
    return {"rmse_kmh":{str(h):math.sqrt(statistics.mean(x*x for x in v)) for h,v in errors.items()},"mae_kmh":{str(h):statistics.mean(abs(x) for x in v) for h,v in errors.items()},"samples":{str(h):len(v) for h,v in errors.items()}}


def _characterization(releases):
    trajectory={}
    for h in HORIZONS:
        values=[x["deltas"][h] for x in releases]
        trajectory[str(h)]={"samples":len(values),"speed_delta_kmh":{"p10":_percentile(values,10),"median":_median(values),"p90":_percentile(values,90)}}
    def groups(field,split):
        output=[]
        for label,predicate in split:
            members=[x for x in releases if predicate(x[field])]
            output.append({"group":label,"events":len(members),"plus_24_speed_delta_median_kmh":_median(x["deltas"][24] for x in members)})
        return output
    return {"events":len(releases),"trajectory":trajectory,"dependencies":{
        "prior_brake_duration":groups("prior_brake_ticks",[("below_12",lambda x:x<12),("12_or_more",lambda x:x>=12)]),
        "speed":groups("speed_kmh",[("below_200",lambda x:x<200),("200_or_more",lambda x:x>=200)]),
        "current_acceleration":groups("current_acceleration_mps2",[("stronger_than_minus_15",lambda x:x<-15),("minus_15_or_weaker",lambda x:x>=-15)]),
        "steering":groups("abs_steer",[("below_0.1",lambda x:x<.1),("0.1_or_more",lambda x:x>=.1)])}}


def _objective_constraints(profile,smooth,index,speed,kind):
    current=terminal_objective(profile,smooth,index);count=len(profile)
    if kind=="next_minimum":return [{"profile_index":current["profile_index"],"custom_waypoint_index":current["custom_waypoint_index"],"target_speed_kmh":current["target_speed_kmh"],"distance_m":current["distance_m"]}]
    points=[];distance=0.0
    for offset in range(1,81):
        distance+=float(profile[(index+offset-1)%count]["ds_m"]);i=(index+offset)%count
        points.append({"profile_index":i,"custom_waypoint_index":int(profile[i]["custom_waypoint_index"]),"target_speed_kmh":float(smooth[i]),"distance_m":distance})
        if i==current["profile_index"]:break
    if kind=="all_constraints":return points
    if kind=="required_deceleration":
        v=speed/3.6
        return [min(points,key=lambda p:((p["target_speed_kmh"]/3.6)**2-v*v)/(2*max(p["distance_m"],.1)))]
    if kind=="region_envelope":
        local=[]
        for j,p in enumerate(points):
            prev=points[max(0,j-1)]["target_speed_kmh"];nxt=points[min(len(points)-1,j+1)]["target_speed_kmh"]
            if p["target_speed_kmh"]<=prev and p["target_speed_kmh"]<=nxt:local.append(p)
        return local or [points[-1]]
    raise ValueError(kind)


def _tail_statistics(releases):
    additional={h:[x["deltas"][h]-x["deltas"][12] for x in releases] for h in (16,20,24)}
    return {str(h):{"p10":_percentile(v,10),"median":_median(v),"p90":_percentile(v,90)} for h,v in additional.items()}


def _bound(candidate,constraint,model_prediction,cv,tail,envelope):
    speed=candidate["speed_kmh"];ticks=max(1,int(math.ceil(constraint["distance_m"]/max(speed/3.6,1)/.05)))
    if envelope=="freeze_12":h=min(ticks,12);pred=speed+model_prediction[h]*3.6;unc=cv["rmse_kmh"][str(min((x for x in HORIZONS if x>=h),default=12))]
    elif envelope=="direct_24_2sigma":h=min(ticks,24);pred=speed+model_prediction[h]*3.6;nearest=min(HORIZONS,key=lambda x:abs(x-h));unc=2*cv["rmse_kmh"][str(nearest)]
    elif envelope=="p90_tail_2sigma":
        if ticks<=12:
            h=ticks;pred=speed+model_prediction[h]*3.6;nearest=min(HORIZONS,key=lambda x:abs(x-h));unc=2*cv["rmse_kmh"][str(nearest)]
        else:
            tail_h=min((16,20,24),key=lambda x:abs(x-min(ticks,24)));h=tail_h
            pred=speed+model_prediction[12]*3.6+tail[str(tail_h)]["p90"];unc=2*cv["rmse_kmh"]["12"]
    else:raise ValueError(envelope)
    return {"upper_speed_kmh":pred+unc,"mean_speed_kmh":pred,"uncertainty_kmh":unc,"ticks":ticks,"direct_horizon":ticks<=24}


def _evaluate(candidates,lap_data,profile,model,cv,tail,objective,envelope,keep_audits=False):
    smooth=_smooth_profile(profile);accepted=set();useful=false=0;beyond=0;audits=[];acceleration_cache={key:_accelerations(rows) for key,rows in lap_data.items()}
    for x in candidates:
        rows=lap_data[x["key"]];row=rows[x["row_index"]];acc=acceleration_cache[x["key"]];features=_features(rows,acc,x["row_index"],x["h1_brake_ticks_accumulated"]-1,0);prediction={h:float(features@model[h]) for h in range(1,25)}
        constraints=_objective_constraints(profile,smooth,int(row["shadow_profile_index"]),x["speed_kmh"],objective)
        bounds=[(constraint,_bound(x,constraint,prediction,cv,tail,envelope)) for constraint in constraints]
        safe=all(bound["upper_speed_kmh"]<=constraint["target_speed_kmh"] for constraint,bound in bounds)
        if safe:
            accepted.add((x["key"],x["row_index"]));useful+=not x["winner_brake"];false+=x["winner_brake"];beyond+=any(not b["direct_horizon"] for _,b in bounds)
        if keep_audits and any(lo<=x["custom_waypoint_index"]<=hi for lo,hi in ((382,454),(789,820),(1774,1856),(2496,2574))):
            audits.append({"candidate":x,"constraints":constraints,"bounds":bounds,"safe":safe})
    universe=sum(not x["winner_brake"] for x in candidates)
    return {"objective":objective,"envelope":envelope,"releases":len(accepted),"useful":useful,"false":false,"precision":useful/max(len(accepted),1),"recall":useful/max(universe,1),"beyond_24":beyond,"accepted":accepted,"audits":audits}


def _missed_classification(candidates,results,selected):
    selected_keys=selected["accepted"]
    useful=[x for x in candidates if not x["winner_brake"] and (x["key"],x["row_index"]) not in selected_keys];counts=Counter()
    by={(r["objective"],r["envelope"]):r for r in results}
    for x in useful:
        key=(x["key"],x["row_index"])
        if any(key in r["accepted"] for r in results if r["objective"]==selected["objective"] and r["envelope"]!="freeze_12"):counts["prediction horizon too short / tail too conservative"]+=1
        elif any(key in r["accepted"] for r in results if r["objective"]!=selected["objective"] and r["envelope"]==selected["envelope"]):counts["wrong terminal point"]+=1
        elif key in by.get(("next_minimum",selected["envelope"]),{}).get("accepted",set()) and key not in by.get(("all_constraints",selected["envelope"]),{}).get("accepted",set()):counts["multiple constraints not represented"]+=1
        else:counts["genuinely unsafe under tested conservative bounds"]+=1
    return dict(counts)


def _report(result):
    lines=["# Extended terminal prediction and objective comparison","","## Release transient extension","","| Horizon | Samples | RMSE | MAE | Speed Δ p10 / median / p90 |","|---:|---:|---:|---:|---:|"]
    for h in HORIZONS:
        t=result["transients"]["trajectory"][str(h)];e=result["prediction_error"]
        lines.append("| +%d | %d | %.2f | %.2f | %.1f / %.1f / %.1f km/h |"%(h,t["samples"],e["rmse_kmh"][str(h)],e["mae_kmh"][str(h)],t["speed_delta_kmh"]["p10"],t["speed_delta_kmh"]["median"],t["speed_delta_kmh"]["p90"]))
    lines += ["","## Fixed-gate comparison","","| Objective | Residual envelope | Useful | False | Precision | Recall | Beyond +24 |","|---|---|---:|---:|---:|---:|---:|"]
    for x in result["comparisons"]:lines.append("| %s | %s | %d | %d | %.1f%% | %.1f%% | %d |"%(x["objective"],x["envelope"],x["useful"],x["false"],100*x["precision"],100*x["recall"],x["beyond_24"]))
    s=result["selected"]
    lines += ["","Selected: **%s + %s** — %d useful, %d false, %.1f%% recall."%(s["objective"],s["envelope"],s["useful"],s["false"],100*s["recall"]),"","## Missed useful releases","",*("- %s: %d"%(k,v) for k,v in result["missed_release_classification"].items()),"","## Major events","","| WP | Winner | Original | Residual-only | Current terminal | Improved terminal |","|---|---|---|---|---|---|"]
    for wp,x in result["major_events"].items():
        def f(v):return "%s / %s / %s"%(v["onset_wp"],v["release_wp"],v["brake_ticks"])
        lines.append("| %s | %s | %s | %s | %s | %s |"%(wp,f(x["winner"]),f(x["original"]),f(x["residual_only"]),f(x["current_terminal"]),f(x["improved_terminal"])))
    lines += ["","### Improved-objective diagnostics",""]
    for wp,x in result["major_events"].items():
        d=x["improved_terminal_objective"]
        lines.append("- WP %s: %d/%d candidate ticks released; median terminal WP %s at %s m, median conservative margin %s km/h; %s."%(wp,d["released_ticks"],d["candidate_ticks"],d["terminal_waypoint_median"],d["terminal_distance_median_m"],d["terminal_safety_margin_median_kmh"],d["improvement_source"]))
    lines += ["","## Recommendation","",result["recommendation"],"","No live planner, onset logic, release gate, or target profile was changed.",""]
    return "\n".join(lines)


def analyze(root:Path,results_dir="experiment_results"):
    rr=root/results_dir;runs=_eligible_runs(root,rr);profile=read_json(rr/"analysis/velocity_profile_v2/latest.json")["distance_profile"]
    releases,examples=_extended_examples(runs);model=_fit(examples);cv=_cv(examples);transients=_characterization(releases);tail=_tail_statistics(releases)
    candidates,lap_data=_collect_candidates(runs,profile,model)
    objectives=("next_minimum","required_deceleration","all_constraints","region_envelope");envelopes=("freeze_12","direct_24_2sigma","p90_tail_2sigma")
    results=[_evaluate(candidates,lap_data,profile,model,cv,tail,o,e) for o in objectives for e in envelopes]
    viable=[x for x in results if x["false"]==0];selected=max(viable,key=lambda x:(x["useful"],-x["beyond_24"])) if viable else min(results,key=lambda x:x["false"])
    selected=_evaluate(candidates,lap_data,profile,model,cv,tail,selected["objective"],selected["envelope"],keep_audits=True)
    residual_keys={(x["key"],x["row_index"]) for x in candidates};current=next(x for x in results if x["objective"]=="next_minimum" and x["envelope"]=="freeze_12")
    major=_event_metrics(runs,{"residual_only":residual_keys,"current_terminal":current["accepted"],"improved_terminal":selected["accepted"]})
    for label,(lo,hi) in {"382-454":(382,454),"789-820":(789,820),"1774-1856":(1774,1856),"2496-2574":(2496,2574)}.items():
        event_audits=[a for a in selected["audits"] if lo<=a["candidate"]["custom_waypoint_index"]<=hi]
        safe=[a for a in event_audits if a["safe"]]
        margins=[c["target_speed_kmh"]-b["upper_speed_kmh"] for a in safe for c,b in a["bounds"]]
        distances=[c["distance_m"] for a in safe for c in a["constraints"]]
        terminal_wps=[c["custom_waypoint_index"] for a in safe for c in a["constraints"]]
        major[label]["improved_terminal_objective"]={"definition":selected["objective"],"candidate_ticks":len(event_audits),"released_ticks":len(safe),"terminal_waypoint_median":_median(terminal_wps),"terminal_distance_median_m":_median(distances),"terminal_safety_margin_median_kmh":_median(margins),"improvement_source":("both longer prediction and terminal selection" if selected["envelope"]!="freeze_12" and selected["objective"]!="next_minimum" else "longer prediction" if selected["envelope"]!="freeze_12" else "terminal selection")}
    missed=_missed_classification(candidates,results,selected);ready=selected["false"]==0 and selected["recall"]>=.5 and selected["beyond_24"]==0
    serial=lambda x:{k:v for k,v in x.items() if k not in ("accepted","audits")}
    population={"attempts":len(runs),"clean_24_tick_release_events":len(releases),"training_examples":len(examples),"release_candidates":len(candidates)}
    comparisons=[serial(x) for x in results];selected_summary=serial(selected);selected_audits=selected["audits"]
    # The accepted-key sets and per-lap rows are large and are no longer needed
    # once aggregation is complete. Release them before assembling output so the
    # full analysis remains usable on the experiment host's constrained memory.
    del candidates,lap_data,results,selected,releases,examples,runs
    gc.collect()
    result={"schema_version":SCHEMA_VERSION,"event_type":"extended_terminal_prediction_analysis","created_utc":dt.datetime.now(dt.timezone.utc).isoformat(),"population":population,"transients":transients,"prediction_error":cv,"tail_envelopes":{"empirical_additional_speed_change_after_plus_12_kmh":tail},"comparisons":comparisons,"selected":selected_summary,"missed_release_classification":missed,"major_events":major,"shadow_ready":ready,"recommendation":("The fixed gate passes all readiness criteria; prepare a shadow-only implementation." if ready else "Not shadow-ready. Keep the fixed safety gate and continue offline: the best zero-false configuration does not yet satisfy useful-retention and supported-horizon requirements across major events.")}
    out=rr/"analysis/terminal_prediction_v2";out.mkdir(parents=True,exist_ok=True);jp=out/"latest.json";mp=out/"latest.md";tp=out/"wp1774_1856_trace.csv"
    audit=[]
    for item in selected_audits:
        x=item["candidate"]
        if 1774<=x["custom_waypoint_index"]<=1856:audit.append({"attempt_id":x["attempt_id"],"lap":x["lap"],"tick":x["tick"],"waypoint":x["custom_waypoint_index"],"speed_kmh":x["speed_kmh"],"h1_ticks":x["h1_brake_ticks_accumulated"],"progress":x["braking_progress_fraction"],"objective":selected_summary["objective"],"constraint_waypoints":";".join(str(c["custom_waypoint_index"]) for c in item["constraints"]),"constraint_targets_kmh":";".join("%.2f"%c["target_speed_kmh"] for c in item["constraints"]),"constraint_distances_m":";".join("%.1f"%c["distance_m"] for c in item["constraints"]),"upper_predicted_speeds_kmh":";".join("%.2f"%b["upper_speed_kmh"] for _,b in item["bounds"]),"release":item["safe"]})
    if audit:
        with tp.open("w",encoding="utf-8",newline="") as f:w=csv.DictWriter(f,fieldnames=list(audit[0]));w.writeheader();w.writerows(audit)
    result.update({"result_path":str(jp.relative_to(root)),"human_report_path":str(mp.relative_to(root)),"wp1774_trace_path":str(tp.relative_to(root))});write_json(jp,result);mp.write_text(_report(result),encoding="utf-8");return result
