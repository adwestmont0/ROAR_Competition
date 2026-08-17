"""Whole-lap empirical opportunity mining from successful baseline passes only."""

import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

from experiments.analysis.velocity_profile import _ledger
from experiments.harness.core import write_json


WINDOWS_M=(100,200,300,500)


def _f(row,key,default=0.0):
    value=row.get(key);return default if value in (None,"") else float(value)


def _interp(rows,s,key):
    xs=np.fromiter((r["_s"] for r in rows),float);i=int(np.searchsorted(xs,s))
    if i<=0:return _f(rows[0],key)
    if i>=len(rows):return _f(rows[-1],key)
    a,b=rows[i-1],rows[i];fraction=(s-a["_s"])/max(1e-9,b["_s"]-a["_s"])
    return _f(a,key)+fraction*(_f(b,key)-_f(a,key))


def _cluster_advantage(fast,other,key,seed=17,n=1000):
    rng=np.random.default_rng(seed)
    def groups(items):
        d=defaultdict(list)
        for x in items:d[x["attempt_id"]].append(float(x[key]))
        return [float(np.mean(v)) for v in d.values()]
    a,b=groups(fast),groups(other)
    draws=[np.mean(rng.choice(b,len(b),replace=True))-np.mean(rng.choice(a,len(a),replace=True)) for _ in range(n)]
    return {"other_minus_fast_mean":float(np.mean(b)-np.mean(a)),"cluster_bootstrap_95pct":[float(np.quantile(draws,.025)),float(np.quantile(draws,.975))],"fast_attempts":len(a),"other_attempts":len(b)}


def _cyclic_distance_to_interval(s,start,end,lap):
    values=[]
    for shift in (-lap,0,lap):
        x=s+shift
        values.append(0 if start<=x<=end else min(abs(x-start),abs(x-end)))
    return min(values)


def analyze(root:Path,results_dir="experiment_results",grid_m=25.0,downstream_buffer_m=100.0,min_support=40,practical_zero_s=.01):
    results=root/results_dir;ledger=_ledger(results/"ledger.jsonl")
    source=json.loads((results/"analysis/wp430_650_opportunity/result.json").read_text());accepted=set(t["attempt_id"] for t in source["traversals"])
    line=np.load(str(root/"competition_code/waypoints/waypointsPrimary.npz"))["locations"][35:,:2];segment=np.linalg.norm(np.diff(line,axis=0),axis=1);arc=np.r_[0,np.cumsum(segment)];lap=float(arc[-1]+np.linalg.norm(line[0]-line[-1]))
    # Curvature is geometry only, used to describe lateral load, never ranking.
    ext=np.vstack([line[-5:],line,line[:5]]);curvature=np.zeros(len(line))
    for i in range(len(line)):
        a,b,c=ext[i+3],ext[i+5],ext[i+7];ab=b-a;bc=c-b;ac=c-a;cross=abs(np.cross(ab,bc));curvature[i]=2*cross/max(1e-9,np.linalg.norm(ab)*np.linalg.norm(bc)*np.linalg.norm(ac))
    runs=[]
    for item in ledger:
        if item.get("attempt_id") not in accepted:continue
        p=(root/item["telemetry_path"]).parent/"ticks.csv";rows=list(csv.DictReader(p.open(newline="",encoding="utf-8")));wrap=0;prior=None
        for row_index,r in enumerate(rows):
            wp=int(_f(r,"custom_waypoint_index"))%len(line)
            if prior is not None and wp-prior < -len(line)/2:wrap+=1
            r["_wp"]=wp;r["_s"]=float(arc[wp]+wrap*lap);prior=wp
            nearby=np.asarray([(wp+offset)%len(line) for offset in range(-100,101)],dtype=int)
            position=np.asarray([_f(r,"x"),_f(r,"y")])
            r["_line_error"]=float(np.min(np.linalg.norm(line[nearby]-position,axis=1)))
            r["_lateral"]=(_f(r,"speed_kmh")/3.6)**2*curvature[wp]
        runs.append((item,rows,{"s":np.asarray([r["_s"] for r in rows]),"time":np.asarray([_f(r,"sim_time_seconds") for r in rows]),"speed":np.asarray([_f(r,"speed_kmh") for r in rows])}))
    collision_s=[]
    for item in ledger:
        if item.get("outcome")!="collision" or item.get("custom_waypoint_index") is None:continue
        collision_s.append(float(arc[int(item["custom_waypoint_index"])%len(line)]))
    starts=np.arange(0,lap,grid_m);candidates=[]
    for length in WINDOWS_M:
        for start in starts:
            samples=[]
            for item,rows,cache in runs:
                k0=math.ceil((rows[0]["_s"]-start)/lap);k1=math.floor((rows[-1]["_s"]-(start+length+downstream_buffer_m))/lap)
                for k in range(k0,k1+1):
                    a=start+k*lap;b=a+length;buffer_end=b+downstream_buffer_m
                    interior=[r for r in rows if a<=r["_s"]<=b]
                    if len(interior)<2:continue
                    entry=float(np.interp(a,cache["s"],cache["speed"]));exit_speed=float(np.interp(b,cache["s"],cache["speed"]))
                    duration=float(np.interp(b,cache["s"],cache["time"])-np.interp(a,cache["s"],cache["time"]))
                    buffered=float(np.interp(buffer_end,cache["s"],cache["time"])-np.interp(a,cache["s"],cache["time"]))
                    samples.append({"attempt_id":item["attempt_id"],"pass":k,"time_s":duration,"buffered_time_s":buffered,"entry_speed_kmh":entry,"exit_speed_kmh":exit_speed,"mean_abs_steer":float(np.mean([abs(_f(r,"steer")) for r in interior])),"mean_lateral_mps2":float(np.mean([r["_lateral"] for r in interior])),"mean_line_error_m":float(np.mean([r["_line_error"] for r in interior]))})
            if len(samples)<min_support:continue
            ordered=sorted(samples,key=lambda x:x["time_s"]);q=len(ordered)//4;fast=ordered[:q];slow=ordered[-q:];other=ordered[q:]
            advantage=_cluster_advantage(fast,other,"time_s");buffer_adv=_cluster_advantage(fast,other,"buffered_time_s")
            def delta(key):return float(np.mean([x[key] for x in fast])-np.mean([x[key] for x in other]))
            median=float(np.median([x["time_s"] for x in samples]));fast_mean=float(np.mean([x["time_s"] for x in fast]));gain=median-fast_mean
            near=sum(_cyclic_distance_to_interval(x,start,start+length,lap)<=50 for x in collision_s)
            entry_delta=delta("entry_speed_kmh");exit_delta=delta("exit_speed_kmh")
            reasons=[]
            if entry_delta < -1 and exit_delta>0:reasons.append("slower_upstream_entry")
            if buffer_adv["other_minus_fast_mean"] < max(practical_zero_s,.5*advantage["other_minus_fast_mean"]):reasons.append("downstream_gain_given_back")
            if advantage["cluster_bootstrap_95pct"][0]<=practical_zero_s:reasons.append("confidence_includes_practical_zero")
            if near:reasons.append("collision_evidence_within_50m")
            candidate={"start_m":float(start),"end_m":float((start+length)%lap),"window_m":length,"start_custom_wp":int(np.searchsorted(arc,start)%len(line)),"end_custom_wp":int(np.searchsorted(arc,(start+length)%lap)%len(line)),"support":len(samples),"attempt_support":len(set(x["attempt_id"] for x in samples)),"median_time_s":median,"fast_quartile_mean_s":fast_mean,"fast_quartile_median_s":float(np.median([x["time_s"] for x in fast])),"slow_quartile_mean_s":float(np.mean([x["time_s"] for x in slow])),"median_minus_fast_mean_gain_s":gain,"clustered_advantage":advantage,"buffered_clustered_advantage":buffer_adv,"fast_minus_other_entry_speed_kmh":entry_delta,"fast_minus_other_exit_speed_kmh":exit_delta,"fast_minus_other_mean_abs_steer":delta("mean_abs_steer"),"fast_minus_other_mean_lateral_mps2":delta("mean_lateral_mps2"),"fast_minus_other_mean_line_error_m":delta("mean_line_error_m"),"nearby_collision_count":near,"rejection_reasons":reasons,"survives":not reasons}
            candidates.append(candidate)
    survivors=sorted([x for x in candidates if x["survives"]],key=lambda x:x["median_minus_fast_mean_gain_s"],reverse=True)
    # Merge overlapping winners, retaining the strongest representative while
    # recording all corroborating window scales.
    regions=[]
    for c in survivors:
        interval=(c["start_m"],c["start_m"]+c["window_m"])
        found=None
        for region in regions:
            overlap=max(0,min(interval[1],region["unwrapped_end_m"])-max(interval[0],region["unwrapped_start_m"]))
            if overlap>=.6*min(c["window_m"],region["representative"]["window_m"]):found=region;break
        if found:found["corroborating_windows"].append(c);found["window_lengths_m"]=sorted(set(found["window_lengths_m"]+[c["window_m"]]))
        else:regions.append({"unwrapped_start_m":interval[0],"unwrapped_end_m":interval[1],"representative":c,"corroborating_windows":[c],"window_lengths_m":[c["window_m"]]})
    # Inspect top regions tick-wise for earliest control separation.
    for region in regions[:5]:
        c=region["representative"];start=c["start_m"];lookback=150.;profiles=defaultdict(list)
        # Recreate quartile membership at the representative window.
        observations=[]
        for item,rows,cache in runs:
            k0=math.ceil((rows[0]["_s"]-start)/lap);k1=math.floor((rows[-1]["_s"]-(start+c["window_m"]))/lap)
            for k in range(k0,k1+1):observations.append((item,rows,k,float(np.interp(start+k*lap+c["window_m"],cache["s"],cache["time"])-np.interp(start+k*lap,cache["s"],cache["time"]))))
        observations.sort(key=lambda x:x[3]);q=max(1,len(observations)//4);labels={ (o[0]["attempt_id"],o[2]):("fast" if i<q else "other") for i,o in enumerate(observations)}
        for item,rows,k,_ in observations:
            label=labels[(item["attempt_id"],k)];a=start+k*lap-lookback;b=start+k*lap+c["window_m"]
            for r in rows:
                if a<=r["_s"]<=b:
                    relbin=int(math.floor((r["_s"]-(start+k*lap))/10))*10
                    profiles[(relbin,label)].append(r)
        earliest=None
        for rel in sorted(set(x[0] for x in profiles)):
            f=profiles.get((rel,"fast"),[]);o=profiles.get((rel,"other"),[])
            if min(len(f),len(o))<10:continue
            metrics={"brake_fraction":np.mean([_f(x,"brake")>.01 for x in f])-np.mean([_f(x,"brake")>.01 for x in o]),"throttle_fraction":np.mean([_f(x,"throttle")>.9 for x in f])-np.mean([_f(x,"throttle")>.9 for x in o]),"recommended_speed_kmh":np.mean([_f(x,"controller_recommended_speed_kmh") for x in f])-np.mean([_f(x,"controller_recommended_speed_kmh") for x in o]),"abs_steer":np.mean([abs(_f(x,"steer")) for x in f])-np.mean([abs(_f(x,"steer")) for x in o]),"lookahead_count":np.mean([_f(x,"controller_lookahead_count") for x in f])-np.mean([_f(x,"controller_lookahead_count") for x in o])}
            if abs(metrics["brake_fraction"])>=.15 or abs(metrics["throttle_fraction"])>=.15 or abs(metrics["recommended_speed_kmh"])>=3 or abs(metrics["abs_steer"])>=.01 or abs(metrics["lookahead_count"])>=1:
                earliest={"relative_to_window_start_m":rel,"approx_custom_wp":int(np.searchsorted(arc,(start+rel)%lap)%len(line)),"fast_minus_other":{k:float(v) for k,v in metrics.items()}};break
        c["earliest_control_difference"]=earliest
        if earliest is None:c["causal_classification"]="D. stochastic dynamics/no clear control cause"
        elif abs(earliest["fast_minus_other"]["brake_fraction"])>=.15 or abs(earliest["fast_minus_other"]["throttle_fraction"])>=.15 or abs(earliest["fast_minus_other"]["recommended_speed_kmh"])>=3:c["causal_classification"]="A. controllable longitudinal difference"
        elif abs(earliest["fast_minus_other"]["abs_steer"])>=.01 or abs(earliest["fast_minus_other"]["lookahead_count"])>=1:c["causal_classification"]="B. controllable lateral/trajectory difference"
        else:c["causal_classification"]="D. stochastic dynamics/no clear control cause"
    result={"schema_version":1,"event_type":"empirical_whole_lap_opportunity_miner","configuration":{"windows_m":list(WINDOWS_M),"grid_m":grid_m,"downstream_buffer_m":downstream_buffer_m,"minimum_support":min_support,"minimum_gain_thresholds_s":[.05,.10],"practical_zero_s":practical_zero_s,"physics_planner_used_for_ranking":False},"population":{"attempts":len(runs)},"candidate_windows":len(candidates),"surviving_windows":len(survivors),"regions_at_least_0_05s":[r for r in regions if r["representative"]["median_minus_fast_mean_gain_s"]>=.05],"regions_at_least_0_10s":[r for r in regions if r["representative"]["median_minus_fast_mean_gain_s"]>=.10]}
    out=results/"analysis/empirical_opportunity_miner";rp,mp=out/"result.json",out/"report.md";result["result_path"]=str(rp.relative_to(root));result["human_report_path"]=str(mp.relative_to(root));write_json(rp,result);mp.parent.mkdir(parents=True,exist_ok=True);mp.write_text(render_report(result),encoding="utf-8");return result


def render_report(r):
    lines=["# Empirical whole-lap opportunity miner","","Successful baseline traversals only; physics planning was not used for ranking. No controller code was changed.","","- Candidate windows: %d"%r["candidate_windows"],"- Surviving windows: %d"%r["surviving_windows"],"- Merged regions >=0.05 s: %d"%len(r["regions_at_least_0_05s"]),"- Merged regions >=0.10 s: %d"%len(r["regions_at_least_0_10s"]),"","## Top empirical regions",""]
    for i,region in enumerate(r["regions_at_least_0_05s"][:5],1):
        x=region["representative"];lines += ["### %d. WP %d–%d (%dm)"%(i,x["start_custom_wp"],x["end_custom_wp"],x["window_m"]),"","- Median-minus-fast gain: %.3f s"%x["median_minus_fast_mean_gain_s"],"- Clustered advantage: %.3f s; 95%% CI %.3f–%.3f"%(x["clustered_advantage"]["other_minus_fast_mean"],*x["clustered_advantage"]["cluster_bootstrap_95pct"]),"- Buffered advantage: %.3f s"%x["buffered_clustered_advantage"]["other_minus_fast_mean"],"- Entry / exit speed difference: %+.2f / %+.2f km/h"%(x["fast_minus_other_entry_speed_kmh"],x["fast_minus_other_exit_speed_kmh"]),"- Steering / lateral / line-error differences: %+.4f / %+.3f / %+.3f"%(x["fast_minus_other_mean_abs_steer"],x["fast_minus_other_mean_lateral_mps2"],x["fast_minus_other_mean_line_error_m"]),"- Earliest control difference: `%s`"%json.dumps(x.get("earliest_control_difference"),sort_keys=True),"- Classification: **%s**"%x.get("causal_classification"),""]
    if not r["regions_at_least_0_05s"]:lines.append("No empirical region cleared the 0.05 s threshold and all rejection gates.")
    lines += ["","The JSON result contains both 0.05 s and 0.10 s ranked region sets plus corroborating overlapping windows."]
    return "\n".join(lines)+"\n"


if __name__=="__main__":analyze(Path(__file__).resolve().parents[2])
