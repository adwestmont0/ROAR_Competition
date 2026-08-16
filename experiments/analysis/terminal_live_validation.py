"""Validate live terminal-tail telemetry against the shared pure policy."""

import csv
import datetime as dt
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from competition_code.shadow.terminal_policy import evaluate_terminal_tail
from experiments.harness.core import load_ledger, read_json, write_json
from .shadow_disagreement import BRAKE_ACTIVE, _number

WINDOWS=((382,454),(789,820),(1774,1856),(2496,2574))
CATEGORICAL=("status","reason_code","eligible","terminal_profile_index","direct_prediction_horizon_ticks","tail_path","tail_segment")
NUMERIC=("terminal_distance_m","terminal_target_speed_kmh","predicted_speed_at_direct_horizon_kmh","direct_uncertainty_kmh","tail_credit_kmh","tail_uncertainty_kmh","terminal_speed_upper_bound_kmh","terminal_safety_margin_kmh")


def _eligible_attempts(root,results_dir,experiment_name=None):
    output=[]
    for item in load_ledger(results_dir/"ledger.jsonl"):
        if experiment_name and item.get("experiment_name")!=experiment_name:continue
        if item.get("outcome") not in ("finished","collision") or not item.get("telemetry_path"):continue
        path=(root/item["telemetry_path"]).parent/"ticks.csv"
        if not path.exists():continue
        with path.open(encoding="utf-8") as f:rows=list(csv.DictReader(f))
        if rows and any(row.get("terminal_tail_status") for row in rows):output.append((item,rows))
    return output


def _input(row):
    return {"profile_index":int(row["shadow_profile_index"]),"speed_kmh":_number(row,"speed_kmh"),"current_acceleration_mps2":_number(row,"terminal_tail_current_acceleration_mps2"),"mean_recent_acceleration_mps2":_number(row,"terminal_tail_mean_recent_acceleration_mps2"),"steer":_number(row,"terminal_tail_steer"),"steer_change":_number(row,"terminal_tail_steer_change"),"original_h1_brake":row["shadow_command"]=="brake","original_h1_reason":row["terminal_tail_original_h1_reason"],"prior_h1_brake_requests":int(float(row["terminal_tail_prior_h1_brake_requests"]))}


def _logged(row):
    result={"status":row["terminal_tail_status"],"reason_code":row["terminal_tail_reason_code"],"eligible":row["terminal_tail_eligible"]=="True","terminal_profile_index":int(float(row["terminal_tail_terminal_profile_index"])) if row["terminal_tail_terminal_profile_index"] else None,"direct_prediction_horizon_ticks":int(float(row["terminal_tail_direct_prediction_horizon_ticks"])) if row["terminal_tail_direct_prediction_horizon_ticks"] else None,"tail_path":row["terminal_tail_tail_path"],"tail_segment":row["terminal_tail_tail_segment"] or None}
    for name in NUMERIC:result[name]=_number(row,"terminal_tail_"+name,default=float("nan"))
    return result


def _event_metrics(rows,prefix):
    bylap=defaultdict(list)
    for row in rows:bylap[(row.get("_attempt_id"),int(row["lap"]))].append(row)
    output={}
    for window in WINDOWS:
        traversals=[]
        for _traversal,members in bylap.items():
            selected=[x for x in members if window[0]<=int(x["custom_waypoint_index"])<=window[1]]
            if not selected:continue
            if prefix=="winner":flags=[_number(x,"brake")>BRAKE_ACTIVE for x in selected]
            elif prefix=="h1":flags=[_number(x,"shadow_brake")>BRAKE_ACTIVE for x in selected]
            else:flags=[_number(x,"shadow_brake")>BRAKE_ACTIVE and x["terminal_tail_status"]!="RELEASE" for x in selected]
            active=[i for i,x in enumerate(flags) if x]
            traversals.append({"onset":int(selected[active[0]]["custom_waypoint_index"]) if active else None,"release":int(selected[active[-1]]["custom_waypoint_index"]) if active else None,"ticks":len(active),"abstentions":sum(x["terminal_tail_status"]=="ABSTAIN" for x in selected)})
        output["%d-%d"%window]=traversals
    return output


def analyze(root:Path,results_dir="experiment_results",tolerance=1e-9,experiment_name=None):
    rr=root/results_dir;attempts=_eligible_attempts(root,rr,experiment_name);profile_json=read_json(root/"competition_code/shadow/velocity_profile.json");profile={"target_speed_kmh":[x["target_speed_kmh"] for x in profile_json["points"]],"ds_m":[x["ds_m"] for x in profile_json["points"]]}
    categorical=Counter();errors=defaultdict(list);mismatches=[];statuses=Counter();invariant_failures=0;releases=[];all_rows=[]
    for item,rows in attempts:
        for row in rows:
            row["_attempt_id"]=item["attempt_id"]
        all_rows.extend(rows)
        for index,row in enumerate(rows):
            if not row.get("terminal_tail_status"):continue
            live=_logged(row);replay=evaluate_terminal_tail(_input(row),profile);statuses[live["status"]]+=1
            bad=[]
            for field in CATEGORICAL:
                if live.get(field)!=replay.get(field):categorical[field]+=1;bad.append(field)
            for field in NUMERIC:
                left=live.get(field);right=replay.get(field)
                if left is None or right is None or (isinstance(left,float) and math.isnan(left)):continue
                errors[field].append(abs(float(left)-float(right)))
            invariant_failures+=row.get("terminal_tail_all_invariants_pass")!="True"
            if bad:mismatches.append({"attempt_id":item["attempt_id"],"tick":row["tick"],"fields":bad})
            if live["status"]=="RELEASE":
                terminal=replay.get("terminal_profile_index");future=None
                if terminal is None:
                    releases.append({"classification":"D_indeterminate_replay_mismatch","waypoint":int(row["custom_waypoint_index"]),"abs_steer":abs(_number(row,"steer"))})
                    continue
                for candidate in rows[index+1:]:
                    if int(candidate["shadow_profile_index"])==terminal:future=candidate;break
                if _number(row,"brake")<=BRAKE_ACTIVE:classification="A_winner_released"
                elif future is None:classification="D_indeterminate"
                elif _number(future,"speed_kmh")<=replay["terminal_target_speed_kmh"]:classification="B_winner_braking_terminal_satisfied"
                else:classification="C_winner_braking_terminal_exceeded"
                releases.append({"classification":classification,"waypoint":int(row["custom_waypoint_index"]),"abs_steer":abs(_number(row,"steer"))})
    numerical={field:{"count":len(v),"maximum_absolute_error":max(v) if v else None,"p99_absolute_error":float(np.percentile(v,99)) if v else None,"exceeding_tolerance":sum(x>tolerance for x in v)} for field,v in errors.items()}
    finished=sum(item["outcome"]=="finished" for item,_ in attempts);collisions=sum(item["outcome"]=="collision" for item,_ in attempts)
    result={"schema_version":1,"event_type":"terminal_tail_live_validation","experiment_name":experiment_name,"created_utc":dt.datetime.now(dt.timezone.utc).isoformat(),"attempts":len(attempts),"finished":finished,"collisions":collisions,"compared_ticks":sum(statuses.values()),"decision_agreement_fraction":1-len(mismatches)/max(sum(statuses.values()),1),"categorical_mismatches":dict(categorical),"mismatch_examples":mismatches[:50],"numerical_equivalence":numerical,"invariant_failures":invariant_failures,"status_counts":dict(statuses),"retrospective_release_audit":dict(Counter(x["classification"] for x in releases)),"candidate_abs_steer_ge_0_1":sum(x["abs_steer"]>=.1 for x in releases),"major_events":{"winner":_event_metrics(all_rows,"winner"),"original_h1":_event_metrics(all_rows,"h1"),"terminal_tail":_event_metrics(all_rows,"tail")}}
    audit=result["retrospective_release_audit"]
    has_applied_candidate=audit.get("B_winner_braking_terminal_satisfied",0)>0
    result["recommendation"]=("A_READY_FOR_APPLIED_MICRO_AB_DESIGN" if len(attempts)>=10 and not mismatches and not invariant_failures and not audit.get("C_winner_braking_terminal_exceeded",0) and has_applied_candidate else "C_RETURN_TO_OFFLINE_MODELING" if len(attempts)>=10 and not mismatches and not invariant_failures and not has_applied_candidate else "B_COLLECT_MORE_SHADOW_DATA" if not mismatches and not invariant_failures else "C_RETURN_TO_OFFLINE_MODELING")
    result["applied_candidate_release_count"]=audit.get("B_winner_braking_terminal_satisfied",0)
    out=rr/"analysis/terminal_tail_live_validation";out.mkdir(parents=True,exist_ok=True);jp=out/"latest.json";result["result_path"]=str(jp.relative_to(root));write_json(jp,result);return result
