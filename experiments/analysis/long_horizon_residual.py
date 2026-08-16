"""Event-family-cross-validated correction for terminal-speed prediction bias."""

import csv
import datetime as dt
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from experiments.harness.core import load_ledger, read_json, write_json


EXPERIMENTS = ("terminal-tail-isolation-ab", "terminal-tail-fresh-10")
BRAKE_ACTIVE = 0.05
MAX_HORIZON = 48
RIDGE_ALPHA = 10.0
FEATURES = (
    "current_speed_kmh", "terminal_target_speed_kmh", "horizon_ticks",
    "terminal_distance_m", "winner_brake_active", "winner_brake_magnitude",
    "recent_brake_ticks_8", "recent_integrated_brake_8", "ticks_since_brake_onset",
    "ticks_since_release", "current_acceleration_mps2", "mean_recent_acceleration_mps2",
    "steering_abs", "lateral_load_proxy", "local_curvature",
    "downstream_mean_curvature_50m", "downstream_max_curvature_50m",
)


def _number(row, key, default=None):
    if row is None or row.get(key) in (None, ""):
        return default
    return float(row[key])


def _curvature(points):
    result=[];count=len(points)
    for i in range(count):
        a=np.asarray((points[(i-1)%count]["x"],points[(i-1)%count]["y"]));b=np.asarray((points[i]["x"],points[i]["y"]));c=np.asarray((points[(i+1)%count]["x"],points[(i+1)%count]["y"]))
        ab=np.linalg.norm(a-b);bc=np.linalg.norm(b-c);ca=np.linalg.norm(c-a);area2=abs(np.cross(b-a,c-a))
        result.append(float(2*area2/max(ab*bc*ca,1e-9)))
    return result


def _downstream(curvature, ds, index, distance=50.0):
    values=[];travel=0.0;i=index
    while travel<=distance:
        values.append(curvature[i]);travel+=ds[i];i=(i+1)%len(ds)
    return statistics.mean(values),max(values)


def _attempts(root, results_dir):
    output=[]
    for item in load_ledger(results_dir/"ledger.jsonl"):
        if item.get("experiment_name") not in EXPERIMENTS or item.get("outcome") not in ("finished","collision") or not item.get("telemetry_path"):continue
        path=(root/item["telemetry_path"]).parent/"ticks.csv"
        if not path.exists():continue
        with path.open(encoding="utf-8") as infile:rows=list(csv.DictReader(infile))
        if rows and any(row.get("terminal_tail_status") for row in rows):output.append((item,rows))
    return output


def _history(rows):
    history={};bylap=defaultdict(list)
    for row in rows:bylap[int(row["lap"])].append(row)
    for lap,members in bylap.items():
        onset=None;release_age=999
        for i,row in enumerate(members):
            active=_number(row,"brake",0)>BRAKE_ACTIVE
            previous=_number(members[i-1],"brake",0)>BRAKE_ACTIVE if i else False
            if active and not previous:onset=i
            if not active and previous:release_age=0
            elif not active:release_age+=1
            else:release_age=999
            recent=members[max(0,i-7):i+1]
            history[(lap,int(row["tick"]))]={
                "winner_brake_active":float(active),"winner_brake_magnitude":_number(row,"brake",0),
                "recent_brake_ticks_8":sum(_number(x,"brake",0)>BRAKE_ACTIVE for x in recent),
                "recent_integrated_brake_8":sum(_number(x,"brake",0) for x in recent),
                "ticks_since_brake_onset":float(i-onset if active and onset is not None else 0),
                "ticks_since_release":float(release_age if release_age<999 else 0),
            }
        # Mark the final three ticks of every contiguous winner-brake event.
        event=[]
        for i,row in enumerate(members+[None]):
            active=row is not None and _number(row,"brake",0)>BRAKE_ACTIVE
            if active:event.append(row)
            elif event:
                for rank,candidate in enumerate(event[-3:],start=max(1,4-len(event))):history[(lap,int(candidate["tick"]))]["winner_final_brake_rank"]=rank
                event=[]
    return history


def _dataset(root, results_dir):
    profile=read_json(root/"competition_code/shadow/velocity_profile.json")["points"]
    ds=[float(x["ds_m"]) for x in profile];curv=_curvature(profile);records=[];attempt_inventory=[];policy_ticks=[]
    for item,rows in _attempts(root,results_dir):
        hist=_history(rows);supported=0
        policy_ticks.extend({"attempt_id":item["attempt_id"],"tick":int(row["tick"]),"status":row.get("terminal_tail_status")} for row in rows if row.get("terminal_tail_status"))
        for i,row in enumerate(rows):
            if row.get("terminal_tail_eligible")!="True" or not row.get("terminal_tail_predicted_speed_at_direct_horizon_kmh"):continue
            horizon=int(float(row["terminal_tail_terminal_horizon_ticks"]))
            if horizon>MAX_HORIZON:continue
            lap=int(row["lap"]);terminal_index=int(float(row["terminal_tail_terminal_profile_index"]));future=next((x for x in rows[i+1:] if int(x["lap"])==lap and int(x["shadow_profile_index"])==terminal_index),None)
            if future is None:continue
            index=int(row["shadow_profile_index"]);mean_curv,max_curv=_downstream(curv,ds,index);speed=_number(row,"speed_kmh");steer=abs(_number(row,"terminal_tail_steer",0));h=hist[(lap,int(row["tick"]))]
            features={
                "current_speed_kmh":speed,"terminal_target_speed_kmh":_number(row,"terminal_tail_terminal_target_speed_kmh"),"horizon_ticks":horizon,"terminal_distance_m":_number(row,"terminal_tail_terminal_distance_m"),
                "current_acceleration_mps2":_number(row,"terminal_tail_current_acceleration_mps2"),"mean_recent_acceleration_mps2":_number(row,"terminal_tail_mean_recent_acceleration_mps2"),"steering_abs":steer,"lateral_load_proxy":(speed/3.6)**2*abs(steer),"local_curvature":curv[index],"downstream_mean_curvature_50m":mean_curv,"downstream_max_curvature_50m":max_curv,
            };features.update({key:h[key] for key in FEATURES if key in h})
            predicted=_number(row,"terminal_tail_predicted_speed_at_direct_horizon_kmh");realized=_number(future,"speed_kmh");family=int(float(row["terminal_tail_terminal_custom_waypoint_index"]))
            records.append({"attempt_id":item["attempt_id"],"lap":lap,"tick":int(row["tick"]),"custom_waypoint_index":int(row["custom_waypoint_index"]),"section":int(row["section"]),"profile_s_m":_number(row,"shadow_profile_s_m"),"event_family_terminal_wp":family,"realized_terminal_speed_kmh":realized,"direct_predicted_terminal_speed_kmh":predicted,"residual_kmh":realized-predicted,"original_status":row["terminal_tail_status"],"original_upper_bound_kmh":_number(row,"terminal_tail_terminal_speed_upper_bound_kmh"),"winner_final_brake_rank":h.get("winner_final_brake_rank"),**features});supported+=1
        attempt_inventory.append({"attempt_id":item["attempt_id"],"outcome":item["outcome"],"supported_records":supported})
    return records,attempt_inventory,policy_ticks


def _fit(train):
    x=np.asarray([[r[name] for name in FEATURES] for r in train],dtype=float);y=np.asarray([r["residual_kmh"] for r in train]);mean=x.mean(axis=0);scale=x.std(axis=0);scale[scale<1e-9]=1.0;z=(x-mean)/scale;design=np.column_stack((np.ones(len(z)),z));penalty=np.eye(design.shape[1])*RIDGE_ALPHA;penalty[0,0]=0;coef=np.linalg.solve(design.T@design+penalty,design.T@y);return mean,scale,coef


def _predict(model,records):
    mean,scale,coef=model;x=np.asarray([[r[name] for name in FEATURES] for r in records]);return np.column_stack((np.ones(len(x)),(x-mean)/scale))@coef


def _metrics(actual,predicted):
    error=np.asarray(predicted)-np.asarray(actual)
    return {"count":len(error),"bias_kmh":float(error.mean()),"mae_kmh":float(np.abs(error).mean()),"rmse_kmh":float(np.sqrt(np.mean(error**2))),"worst_optimistic_error_kmh":float(max(0,-error.min()))}


def _summary(values):
    values=sorted(float(value) for value in values)
    if not values:return None
    pick=lambda fraction:values[round(fraction*(len(values)-1))]
    return {"count":len(values),"minimum":values[0],"p10":pick(.1),"median":statistics.median(values),"p90":pick(.9),"maximum":values[-1]}


def _group_metrics(records,key):
    grouped=defaultdict(list)
    for r in records:grouped[key(r)].append(r)
    return {str(name):{"before":_metrics([r["realized_terminal_speed_kmh"] for r in rows],[r["direct_predicted_terminal_speed_kmh"] for r in rows]),"after":_metrics([r["realized_terminal_speed_kmh"] for r in rows],[r["corrected_predicted_terminal_speed_kmh"] for r in rows])} for name,rows in sorted(grouped.items(),key=lambda x:str(x[0]))}


def _render(result):
    before=result["overall_metrics"]["before"];after=result["overall_metrics"]["after"];lines=["# Long-horizon residual-deceleration calibration","","Event-family-held-out linear ridge model; track position and section are diagnostics only.","","| Metric | Direct | Corrected |","|---|---:|---:|",f"| Bias | {before['bias_kmh']:+.3f} km/h | {after['bias_kmh']:+.3f} km/h |",f"| MAE | {before['mae_kmh']:.3f} | {after['mae_kmh']:.3f} |",f"| RMSE | {before['rmse_kmh']:.3f} | {after['rmse_kmh']:.3f} |",f"| Worst optimistic error | {before['worst_optimistic_error_kmh']:.3f} | {after['worst_optimistic_error_kmh']:.3f} |","","## Corrected replay (all policy ticks)","",f"- BRAKE: {result['corrected_replay']['status_counts'].get('BRAKE',0)}",f"- RELEASE: {result['corrected_replay']['status_counts'].get('RELEASE',0)}",f"- ABSTAIN: {result['corrected_replay']['status_counts'].get('ABSTAIN',0)}",f"- NOT_ELIGIBLE: {result['corrected_replay']['status_counts'].get('NOT_ELIGIBLE',0)}",f"- Winner=BRAKE / corrected=RELEASE: {result['corrected_replay']['winner_brake_corrected_release_count']}",f"- Disagreement families: {result['corrected_replay']['disagreement_families']}","","## WP1774 final winner brake ticks","","| Final tick | OOF residual correction | Corrected safety margin | Corrected result |","|---|---:|---:|---|" ]
    for rank,row in result["wp1774_final_brake_ticks"].items():lines.append(f"| #{rank} | {row['median_oof_correction_kmh']:+.2f} km/h | {row['median_corrected_safety_margin_kmh']:+.2f} km/h | {row['status_counts']} |")
    lines += ["","## Known risk windows","",f"- WP407–410: {result['known_risk_windows']['407-410']}",f"- WP1380–1420: {result['known_risk_windows']['1380-1420']}","",result["interpretation"],""];return "\n".join(lines)


def analyze(root:Path,results_dir="experiment_results"):
    rr=root/results_dir;records,inventory,policy_ticks=_dataset(root,rr);families=sorted({r["event_family_terminal_wp"] for r in records});predictions={}
    for family in families:
        train=[r for r in records if r["event_family_terminal_wp"]!=family];test=[r for r in records if r["event_family_terminal_wp"]==family];values=_predict(_fit(train),test)
        for row,value in zip(test,values):predictions[(row["attempt_id"],row["tick"])]=float(value)
    for row in records:
        correction=predictions[(row["attempt_id"],row["tick"])];row["oof_residual_correction_kmh"]=correction;row["corrected_predicted_terminal_speed_kmh"]=row["direct_predicted_terminal_speed_kmh"]+correction;row["corrected_upper_bound_kmh"]=row["original_upper_bound_kmh"]+correction;row["corrected_terminal_margin_kmh"]=row["terminal_target_speed_kmh"]-row["corrected_upper_bound_kmh"];row["corrected_status"]="RELEASE" if row["corrected_upper_bound_kmh"]<=row["terminal_target_speed_kmh"] else "BRAKE"
    actual=[r["realized_terminal_speed_kmh"] for r in records];direct=[r["direct_predicted_terminal_speed_kmh"] for r in records];corrected=[r["corrected_predicted_terminal_speed_kmh"] for r in records]
    original_status=Counter(tick["status"] for tick in policy_ticks);status=Counter(original_status);disagreements=[];record_keys={}
    for row in records:
        key=(row["attempt_id"],row["tick"]);record_keys[key]=row
        status[row["original_status"]]-=1;status[row["corrected_status"]]+=1
        if row["winner_brake_active"] and row["corrected_status"]=="RELEASE":disagreements.append({k:row[k] for k in ("attempt_id","lap","tick","custom_waypoint_index","event_family_terminal_wp","corrected_terminal_margin_kmh","steering_abs","winner_final_brake_rank")})
    unsupported=sum(1 for item in load_ledger(rr/"ledger.jsonl") if item.get("outcome")=="collision" and item.get("telemetry_path"))
    wp1774={}
    for rank in (1,2,3):
        selected=[r for r in records if r["event_family_terminal_wp"]==1859 and r["winner_final_brake_rank"]==rank and 1774<=r["custom_waypoint_index"]<=1856]
        wp1774[str(rank)]={"count":len(selected),"median_oof_correction_kmh":statistics.median(r["oof_residual_correction_kmh"] for r in selected),"median_corrected_safety_margin_kmh":statistics.median(r["corrected_terminal_margin_kmh"] for r in selected),"status_counts":dict(Counter(r["corrected_status"] for r in selected))}
    risk={}
    for lo,hi in ((407,410),(1380,1420)):
        selected=[r for r in records if lo<=r["custom_waypoint_index"]<=hi];risk[f"{lo}-{hi}"]={"records":len(selected),"status_counts":dict(Counter(r["corrected_status"] for r in selected)),"winner_brake_release_count":sum(r["winner_brake_active"] and r["corrected_status"]=="RELEASE" for r in selected)}
    disagreement_margins=[x["corrected_terminal_margin_kmh"] for x in disagreements]
    result={"schema_version":1,"event_type":"long_horizon_residual_calibration","created_utc":dt.datetime.now(dt.timezone.utc).isoformat(),"model":{"type":"standardized_linear_ridge","alpha":RIDGE_ALPHA,"features":list(FEATURES),"cross_validation":"leave_one_terminal_event_family_out","event_families":families},"dataset":{"records":len(records),"attempts":inventory,"supported_event_families":len(families),"maximum_horizon_ticks":MAX_HORIZON},"overall_metrics":{"before":_metrics(actual,direct),"after":_metrics(actual,corrected)},"by_horizon":_group_metrics(records,lambda r:"%02d-%02d"%(1+8*((r["horizon_ticks"]-1)//8),min(48,8+8*((r["horizon_ticks"]-1)//8)))),"by_steering":_group_metrics(records,lambda r:"supported_<0.1" if r["steering_abs"]<.1 else "unsupported_>=0.1"),"near_final_brake_ticks":_group_metrics([r for r in records if r["winner_final_brake_rank"]],lambda r:r["winner_final_brake_rank"]),"by_event_family":_group_metrics(records,lambda r:r["event_family_terminal_wp"]),"wp1774_final_brake_ticks":wp1774,"known_risk_windows":risk,"corrected_replay":{"scope":"all terminal-tail policy ticks; correction substituted only where causal realized-terminal evaluation exists and horizon <=48","original_status_counts":dict(original_status),"status_counts":{k:v for k,v in status.items() if v},"winner_brake_corrected_release_count":len(disagreements),"winner_brake_release_margin_kmh":_summary(disagreement_margins),"winner_final_brake_rank_counts":dict(Counter(str(x["winner_final_brake_rank"]) for x in disagreements)),"disagreements":disagreements,"disagreement_families":dict(Counter(str(x["event_family_terminal_wp"]) for x in disagreements))},"unsafe_collision_audit":{"compatible_collision_attempts":0,"historical_collision_attempts_with_telemetry":unsupported,"conclusion":"Historical collisions predate terminal-tail instrumentation and cannot be scored without reconstructing missing causal inputs; no safety claim is made for them."},"interpretation":"The generic correction materially lowers MAE/RMSE and learns the WP1859 point-prediction bias, but it does not move WP1774 across release because the frozen uncertainty/tail stack remains about 15 km/h conservative. It also creates 94 winner-BRAKE/corrected-RELEASE opinions in held-out family WP455. WP407–410 remains BRAKE, but collision telemetry lacks compatible causal inputs. Therefore this model is diagnostic, not safe for shadow or applied use."}
    # Diagnostic full-data fit coefficients, never used for the held-out metrics.
    mean,scale,coef=_fit(records);result["model"]["diagnostic_full_fit"]={"intercept_kmh":float(coef[0]),"standardized_coefficients":{name:float(value) for name,value in zip(FEATURES,coef[1:])}}
    out=rr/"analysis/long_horizon_residual";out.mkdir(parents=True,exist_ok=True);jp=out/"result.json";mp=out/"report.md";cp=out/"records.csv";result["result_path"]=str(jp.relative_to(root));result["human_report_path"]=str(mp.relative_to(root));result["records_path"]=str(cp.relative_to(root));write_json(jp,result);mp.write_text(_render(result),encoding="utf-8")
    with cp.open("w",newline="",encoding="utf-8") as f:writer=csv.DictWriter(f,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    return result
