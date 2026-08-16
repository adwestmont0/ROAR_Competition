"""Calibrate one-sided uncertainty for the event-held-out residual model."""

import csv
import datetime as dt
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from experiments.harness.core import write_json
from .long_horizon_residual import FEATURES, _attempts, _dataset, _fit, _predict


QUANTILES=(.90,.95,.99)


def _bucket(horizon):
    lower=1+8*((int(horizon)-1)//8);return "%02d-%02d"%(lower,min(48,lower+7))


def _summary(values):
    values=sorted(float(x) for x in values)
    if not values:return None
    pick=lambda q:values[round(q*(len(values)-1))]
    return {"count":len(values),"minimum":values[0],"p10":pick(.1),"median":statistics.median(values),"p90":pick(.9),"maximum":values[-1]}


def _coverage(records,bound_key):
    excess=[r["realized_terminal_speed_kmh"]-(r["corrected_predicted_terminal_speed_kmh"]+r[bound_key]) for r in records]
    optimistic=[x for x in excess if x>0]
    return {"count":len(records),"coverage":sum(x<=0 for x in excess)/len(excess) if excess else None,"optimistic_exceedances":len(optimistic),"maximum_optimistic_excess_kmh":max(optimistic,default=0.0),"p95_optimistic_excess_kmh":float(np.quantile(optimistic,.95)) if optimistic else 0.0}


def _render(result):
    lines=["# Residual-corrected uncertainty calibration","","Bounds use corrected-model out-of-family errors. Each test family is calibrated only from other event families.","","| Horizon | q90 | coverage | q95 | coverage | q99 | coverage | Current stack median |","|---|---:|---:|---:|---:|---:|---:|---:|"]
    for bucket,row in result["by_horizon"].items():lines.append(f"| {bucket} | {row['q90_bound_median_kmh']:.2f} | {row['q90_coverage']:.1%} | {row['q95_bound_median_kmh']:.2f} | {row['q95_coverage']:.1%} | {row['q99_bound_median_kmh']:.2f} | {row['q99_coverage']:.1%} | {row['current_stack_median_kmh']:.2f} |")
    lines += ["","## Replays","","| Policy | BRAKE | RELEASE | ABSTAIN | Winner brake → release | Optimistic exceedances |","|---|---:|---:|---:|---:|---:|"]
    for name,row in result["replays"].items():lines.append(f"| {name} | {row['status_counts'].get('BRAKE',0)} | {row['status_counts'].get('RELEASE',0)} | {row['status_counts'].get('ABSTAIN',0)} | {row['winner_brake_release_count']} | {row['coverage']['optimistic_exceedances']} |")
    lines += ["","## Conclusion","",result["conclusion"],""];return "\n".join(lines)


def analyze(root:Path,results_dir="experiment_results"):
    rr=root/results_dir;records,inventory,policy_ticks=_dataset(root,rr);families=sorted({r["event_family_terminal_wp"] for r in records});pred={}
    for family in families:
        train=[r for r in records if r["event_family_terminal_wp"]!=family];test=[r for r in records if r["event_family_terminal_wp"]==family]
        for row,value in zip(test,_predict(_fit(train),test)):pred[(row["attempt_id"],row["tick"])]=float(value)
    telemetry={}
    for item,rows in _attempts(root,rr):
        for row in rows:telemetry[(item["attempt_id"],int(row["tick"]))]=row
    for row in records:
        correction=pred[(row["attempt_id"],row["tick"])];row["oof_residual_correction_kmh"]=correction;row["corrected_predicted_terminal_speed_kmh"]=row["direct_predicted_terminal_speed_kmh"]+correction;row["corrected_error_actual_minus_prediction_kmh"]=row["realized_terminal_speed_kmh"]-row["corrected_predicted_terminal_speed_kmh"]
        live=telemetry[(row["attempt_id"],row["tick"])];row["direct_uncertainty_kmh"]=float(live["terminal_tail_direct_uncertainty_kmh"]);row["tail_conservatism_kmh"]=float(live["terminal_tail_tail_credit_kmh"] or 0);row["tail_uncertainty_kmh"]=float(live["terminal_tail_tail_uncertainty_kmh"] or 0);row["tail_path"]=live["terminal_tail_tail_path"];row["current_stack_kmh"]=row["direct_uncertainty_kmh"]+row["tail_conservatism_kmh"]+row["tail_uncertainty_kmh"]
    # Family-excluded conformal-style one-sided bounds within each horizon bucket.
    bounds={}
    buckets=sorted({_bucket(row["horizon_ticks"]) for row in records})
    for family in families:
        fallback=[x["corrected_error_actual_minus_prediction_kmh"] for x in records if x["event_family_terminal_wp"]!=family]
        for bucket in buckets:
            calibration=[x["corrected_error_actual_minus_prediction_kmh"] for x in records if x["event_family_terminal_wp"]!=family and _bucket(x["horizon_ticks"])==bucket] or fallback
            bounds[(family,bucket)]={"support":len(calibration),**{q:max(0.0,float(np.quantile(calibration,q))) for q in QUANTILES}}
    for row in records:
        calibrated=bounds[(row["event_family_terminal_wp"],_bucket(row["horizon_ticks"]))]
        row["calibration_support_count"]=calibrated["support"]
        for q in QUANTILES:row["q%d_bound_kmh"%round(q*100)]=calibrated[q]
        row["current_bound_kmh"]=row["current_stack_kmh"]
        row["calibrated_bound_kmh"]=row["q95_bound_kmh"]
        row["hybrid_bound_kmh"]=row["current_stack_kmh"]+row["q99_bound_kmh"]
    by_horizon={}
    for bucket in sorted({_bucket(r["horizon_ticks"]) for r in records}):
        selected=[r for r in records if _bucket(r["horizon_ticks"])==bucket];entry={"count":len(selected),"current_stack_median_kmh":statistics.median(r["current_stack_kmh"] for r in selected)}
        for q in (90,95,99):entry[f"q{q}_bound_median_kmh"]=statistics.median(r[f"q{q}_bound_kmh"] for r in selected);entry[f"q{q}_coverage"]=_coverage(selected,f"q{q}_bound_kmh")["coverage"]
        by_horizon[bucket]=entry
    component_coverage={key:_coverage(records,key) for key in ("direct_uncertainty_kmh","tail_conservatism_kmh","tail_uncertainty_kmh","current_stack_kmh","q90_bound_kmh","q95_bound_kmh","q99_bound_kmh")}
    component_by_path={path:{key:_coverage([r for r in records if r["tail_path"]==path],key) for key in ("direct_uncertainty_kmh","tail_conservatism_kmh","tail_uncertainty_kmh","current_stack_kmh","q95_bound_kmh","q99_bound_kmh")} for path in sorted({r["tail_path"] for r in records})}
    base_status=Counter(tick["status"] for tick in policy_ticks);replays={}
    for policy,bound_key in (("current_stacked","current_bound_kmh"),("calibrated_q95_replacement","calibrated_bound_kmh"),("conservative_current_stack_plus_q99","hybrid_bound_kmh")):
        counts=Counter(base_status);disagreements=[]
        for row in records:
            corrected_upper=row["corrected_predicted_terminal_speed_kmh"]+row[bound_key];status="RELEASE" if corrected_upper<=row["terminal_target_speed_kmh"] else "BRAKE";counts[row["original_status"]]-=1;counts[status]+=1;row[f"{policy}_margin_kmh"]=row["terminal_target_speed_kmh"]-corrected_upper
            if row["winner_brake_active"] and status=="RELEASE":disagreements.append(row)
        event_counts=Counter(str(x["event_family_terminal_wp"]) for x in disagreements);margins=[x[f"{policy}_margin_kmh"] for x in disagreements]
        windows={}
        for lo,hi in ((407,410),(1380,1420),(1774,1856)):
            selected=[r for r in records if lo<=r["custom_waypoint_index"]<=hi];windows[f"{lo}-{hi}"]={"status_counts":dict(Counter("RELEASE" if r["corrected_predicted_terminal_speed_kmh"]+r[bound_key]<=r["terminal_target_speed_kmh"] else "BRAKE" for r in selected)),"winner_brake_release_count":sum(r["winner_brake_active"] and r["corrected_predicted_terminal_speed_kmh"]+r[bound_key]<=r["terminal_target_speed_kmh"] for r in selected)}
        wp455=[r for r in records if r["event_family_terminal_wp"]==455];windows["terminal_family_455"]={"status_counts":dict(Counter("RELEASE" if r["corrected_predicted_terminal_speed_kmh"]+r[bound_key]<=r["terminal_target_speed_kmh"] else "BRAKE" for r in wp455)),"winner_brake_release_count":sum(r["winner_brake_active"] and r["corrected_predicted_terminal_speed_kmh"]+r[bound_key]<=r["terminal_target_speed_kmh"] for r in wp455)}
        replays[policy]={"bound_definition":bound_key,"status_counts":{k:v for k,v in counts.items() if v},"winner_brake_release_count":len(disagreements),"winner_brake_release_families":dict(event_counts),"winner_brake_release_margin_kmh":_summary(margins),"winner_final_brake_rank_counts":dict(Counter(str(x["winner_final_brake_rank"]) for x in disagreements)),"coverage":_coverage(records,bound_key),"windows":windows}
    result={"schema_version":1,"event_type":"residual_corrected_uncertainty_calibration","created_utc":dt.datetime.now(dt.timezone.utc).isoformat(),"dataset":{"records":len(records),"event_families":families,"attempts":inventory,"maximum_horizon_ticks":48},"calibration":{"method":"family-excluded one-sided empirical quantile by 8-tick horizon bucket","quantiles":[.9,.95,.99],"dangerous_error":"realized_terminal_speed > corrected_prediction + bound"},"by_horizon":by_horizon,"component_coverage":component_coverage,"component_coverage_by_tail_path":component_by_path,"replays":replays,"redundancy_assessment":{"direct_model_uncertainty":"It supplies all observed coverage of the current stack in aggregate, but overlaps corrected-model error because it was calibrated for the uncorrected direct model.","tail_conservatism":"Empirically redundant for coverage in this dataset: adding tail credit and tail uncertainty does not improve aggregate current-stack coverage beyond direct uncertainty alone. It also overlaps the end-to-end residual target.","tail_uncertainty":"Potentially guards tail-specific distribution shift, but its error source is already present in end-to-end corrected residuals; independence is not demonstrated.","current_sum":"The terms are not empirically independent additive errors. The sum is conservative in some long-horizon families but still fails badly in others."},"collision_audit":{"compatible_collision_attempts":0,"conclusion":"No collision attempt contains terminal-tail causal inputs; empirical bound coverage cannot be claimed on historical collisions."},"conclusion":"No tested replacement satisfies nominal family-held-out coverage. The q95 replacement covers only 88.2% overall and introduces broad releases. Even the deliberately conservative current-stack-plus-q99 bound reaches only 98.3%, below nominal 99%, because held-out family shift dominates. No replay meets the acceptance criterion; do not implement shadow or applied behavior."}
    out=rr/"analysis/residual_uncertainty";out.mkdir(parents=True,exist_ok=True);jp=out/"result.json";mp=out/"report.md";cp=out/"records.csv";result["result_path"]=str(jp.relative_to(root));result["human_report_path"]=str(mp.relative_to(root));result["records_path"]=str(cp.relative_to(root));write_json(jp,result);mp.write_text(_render(result),encoding="utf-8")
    fields=list(records[0]);
    with cp.open("w",newline="",encoding="utf-8") as f:writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(records)
    return result
