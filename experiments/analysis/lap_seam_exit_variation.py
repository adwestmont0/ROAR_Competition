"""Offline successful-run audit of final-corner exit through the lap seam."""

import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from experiments.analysis.velocity_profile import _ledger
from experiments.harness.core import write_json


LANDMARKS = (0, 50, 100, 200, 300, 379)


def _f(row, key, default=0.0):
    value = row.get(key)
    return default if value in (None, "") else float(value)


def _bootstrap(fast, slow, getter, seed=11, draws=5000):
    rng=np.random.default_rng(seed)
    def clusters(group):
        d=defaultdict(list)
        for x in group:d[x["attempt_id"]].append(float(getter(x)))
        return [float(np.mean(v)) for v in d.values()]
    a,b=clusters(fast),clusters(slow)
    samples=[np.mean(rng.choice(a,len(a),replace=True))-np.mean(rng.choice(b,len(b),replace=True)) for _ in range(draws)]
    return {"difference":float(np.mean(a)-np.mean(b)),"cluster_bootstrap_95pct":[float(np.quantile(samples,.025)),float(np.quantile(samples,.975))],"fast_attempts":len(a),"slow_attempts":len(b)}


def _nearest(rows, distance): return min(rows,key=lambda r:abs(r["_relative_s"]-distance))


def analyze(root:Path,results_dir="experiment_results",upstream_m=500.0):
    results=root/results_dir
    source=json.loads((results/"analysis/wp430_650_opportunity/result.json").read_text())
    accepted=set(t["attempt_id"] for t in source["traversals"])
    line=np.load(str(root/"competition_code/waypoints/waypointsPrimary.npz"))["locations"][35:,:2]
    seg=np.linalg.norm(np.diff(line,axis=0),axis=1); arc=np.r_[0,np.cumsum(seg)]; lap_length=float(arc[-1]+np.linalg.norm(line[0]-line[-1]))
    v2=json.loads((results/"analysis/velocity_profile_v2/latest.json").read_text()); cur=defaultdict(list)
    for p in v2["distance_profile"]:cur[int(p["custom_waypoint_index"])].append(float(p["curvature_1pm"]))
    cur={k:float(np.median(v)) for k,v in cur.items()}
    traversals=[]
    for item in _ledger(results/"ledger.jsonl"):
        if item.get("attempt_id") not in accepted:continue
        path=(root/item["telemetry_path"]).parent/"ticks.csv"; rows=list(csv.DictReader(path.open(newline="",encoding="utf-8")))
        wraps=0; prior=None
        for r in rows:
            wp=int(_f(r,"custom_waypoint_index"))%len(line)
            if prior is not None and wp-prior < -len(line)/2:wraps+=1
            r["_continuous_s"]=float(arc[wp]+wraps*lap_length);r["_wp"]=wp;prior=wp
        seams=[]
        for i in range(1,len(rows)):
            if rows[i]["_wp"]-rows[i-1]["_wp"] < -len(line)/2: seams.append((i,wraps))
        for number,(idx,_) in enumerate(seams):
            seam_s=round(rows[idx]["_continuous_s"]/lap_length)*lap_length
            end_s=seam_s+arc[379]
            zone=[r for r in rows if seam_s-upstream_m<=r["_continuous_s"]<=end_s+5]
            if not zone or zone[0]["_continuous_s"]>seam_s-upstream_m+25 or zone[-1]["_continuous_s"]<end_s-25:continue
            for r in zone:r["_relative_s"]=r["_continuous_s"]-seam_s
            # Last contiguous brake episode before the seam is the final-corner event.
            pre=[r for r in zone if r["_relative_s"]<0]; episodes=[];active=[]
            for r in pre:
                if _f(r,"brake")>.01:active.append(r)
                elif active:episodes.append(active);active=[]
            if active:episodes.append(active)
            episodes=[e for e in episodes if len(e)>=2]
            if not episodes:continue
            brake=episodes[-1]; release_index=pre.index(brake[-1])+1; release=pre[min(release_index,len(pre)-1)]
            throttle_after=[r for r in zone if r["_continuous_s"]>=brake[-1]["_continuous_s"] and _f(r,"throttle")>.9 and _f(r,"brake")<=.01]
            throttle_row=throttle_after[0] if throttle_after else release
            speeds={str(w):_f(_nearest(zone,float(arc[w])),"speed_kmh") for w in LANDMARKS}
            all_event=[r for r in zone if brake[0]["_continuous_s"]<=r["_continuous_s"]<=end_s]
            errors=[];laterals=[]
            for r in all_event:
                position=np.asarray([_f(r,"x"),_f(r,"y")])
                # The controller index is a forward target, not the orthogonal
                # projection of the vehicle. Use true nearest-line distance.
                errors.append(float(np.min(np.linalg.norm(line-position,axis=1))))
                k=cur[min(cur,key=lambda x:abs(x-r["_wp"]))];laterals.append((_f(r,"speed_kmh")/3.6)**2*abs(k))
            anchor=_nearest(zone,-upstream_m); endpoint=_nearest(zone,float(arc[379]))
            traversals.append({
                "attempt_id":item["attempt_id"],"seam_number":number+1,"speeds_kmh":speeds,
                "upstream_anchor_speed_kmh":_f(anchor,"speed_kmh"),"corner_entry_speed_kmh":_f(brake[0],"speed_kmh"),
                "brake_onset_relative_m":brake[0]["_relative_s"],"brake_onset_custom_wp":brake[0]["_wp"],
                "brake_release_relative_m":release["_relative_s"],"brake_release_custom_wp":release["_wp"],
                "brake_ticks":len(brake),"integrated_brake":float(sum(_f(r,"brake") for r in brake)),
                "minimum_corner_speed_kmh":min(_f(r,"speed_kmh") for r in pre if r["_continuous_s"]>=brake[0]["_continuous_s"]),
                "throttle_reapplication_relative_m":throttle_row["_relative_s"],"throttle_reapplication_custom_wp":throttle_row["_wp"],
                "mean_abs_steer":float(np.mean([abs(_f(r,"steer")) for r in all_event])),"max_abs_steer":float(np.max([abs(_f(r,"steer")) for r in all_event])),
                "mean_lateral_acceleration_mps2":float(np.mean(laterals)),"max_lateral_acceleration_mps2":float(np.max(laterals)),
                "mean_racing_line_error_m":float(np.mean(errors)),"max_racing_line_error_m":float(np.max(errors)),
                "elapsed_anchor_to_wp379_s":_f(endpoint,"sim_time_seconds")-_f(anchor,"sim_time_seconds"),
                "elapsed_brake_onset_to_wp379_s":_f(endpoint,"sim_time_seconds")-_f(brake[0],"sim_time_seconds"),
            })
    x=np.asarray([[t["speeds_kmh"]["0"],t["speeds_kmh"]["50"]] for t in traversals]);score=((x-x.mean(0))/x.std(0)).mean(1)
    for t,s in zip(traversals,score):t["exit_score"]=float(s)
    ordered=sorted(traversals,key=lambda t:t["exit_score"]);q=len(ordered)//4;slow,fast=ordered[:q],ordered[-q:]
    fields={"anchor_speed":lambda t:t["upstream_anchor_speed_kmh"],"corner_entry_speed":lambda t:t["corner_entry_speed_kmh"],"brake_onset_m":lambda t:t["brake_onset_relative_m"],"brake_release_m":lambda t:t["brake_release_relative_m"],"brake_ticks":lambda t:t["brake_ticks"],"integrated_brake":lambda t:t["integrated_brake"],"minimum_speed":lambda t:t["minimum_corner_speed_kmh"],"throttle_reapplication_m":lambda t:t["throttle_reapplication_relative_m"],"mean_abs_steer":lambda t:t["mean_abs_steer"],"max_abs_steer":lambda t:t["max_abs_steer"],"mean_lateral":lambda t:t["mean_lateral_acceleration_mps2"],"max_lateral":lambda t:t["max_lateral_acceleration_mps2"],"mean_line_error":lambda t:t["mean_racing_line_error_m"],"max_line_error":lambda t:t["max_racing_line_error_m"],"anchor_to_379_time":lambda t:t["elapsed_anchor_to_wp379_s"],"brake_to_379_time":lambda t:t["elapsed_brake_onset_to_wp379_s"]}
    fields.update({f"speed_wp{w}":lambda t,w=w:t["speeds_kmh"][str(w)] for w in LANDMARKS})
    comparison={k:_bootstrap(fast,slow,v) for k,v in fields.items()}
    def summary(group):
        out={"traversals":len(group),"attempts":len(set(t["attempt_id"] for t in group))}
        for k,get in fields.items():
            vals=[get(t) for t in group];out[k]={"mean":float(np.mean(vals)),"median":float(np.median(vals)),"p10":float(np.quantile(vals,.1)),"p90":float(np.quantile(vals,.9))}
        return out
    time=comparison["anchor_to_379_time"];exit0=comparison["speed_wp0"];entry=comparison["anchor_speed"]
    if time["cluster_bootstrap_95pct"][1]<0:classification="EMPIRICALLY_DEMONSTRATED_NET_GAIN"
    elif exit0["cluster_bootstrap_95pct"][0]>0 and time["cluster_bootstrap_95pct"][0]<=0<=time["cluster_bootstrap_95pct"][1]:classification="SPEED_TRADEOFF_ONLY"
    elif abs(exit0["difference"])<1:classification="NO_EMPIRICAL_HEADROOM"
    else:classification="LOW_SUPPORT / SEAM_ARTIFACT"
    result={"schema_version":1,"event_type":"lap_seam_exit_variation","cyclic_line":{"lap_length_m":lap_length,"upstream_anchor_m":-upstream_m,"wp379_distance_after_seam_m":float(arc[379])},"population":{"attempts":len(set(t["attempt_id"] for t in traversals)),"traversals":len(traversals),"quartile_size":q},"ranking":"mean standardized speed at WP0 and WP50","classification":classification,"fastest_quartile":summary(fast),"slowest_quartile":summary(slow),"fast_minus_slow_clustered":comparison,"traversals":traversals}
    out=results/"analysis/lap_seam_exit_variation";rp,mp=out/"result.json",out/"report.md";result["result_path"]=str(rp.relative_to(root));result["human_report_path"]=str(mp.relative_to(root));write_json(rp,result);mp.parent.mkdir(parents=True,exist_ok=True);mp.write_text(render_report(result),encoding="utf-8");return result


def render_report(r):
    f,s,c=r["fastest_quartile"],r["slowest_quartile"],r["fast_minus_slow_clustered"]
    lines=["# Cyclic final-corner / lap-seam exit analysis","","Only successful baseline traversals are included. The seam was unwrapped in distance; no controller code was changed.","","## Classification","","**%s**"%r["classification"],"","| Metric | Fast exit Q | Slow exit Q | Fast−slow | Attempt-clustered 95% CI |","|---|---:|---:|---:|---:|"]
    labels=[("Upstream-anchor speed","anchor_speed"),("Corner-entry speed","corner_entry_speed"),("Brake onset, m before seam","brake_onset_m"),("Brake release, m before seam","brake_release_m"),("Brake ticks","brake_ticks"),("Integrated brake","integrated_brake"),("Minimum corner speed","minimum_speed"),("Throttle reapplication, relative m","throttle_reapplication_m")]+[("Speed WP%d"%w,"speed_wp%d"%w) for w in LANDMARKS]+[("Mean |steer|","mean_abs_steer"),("Max |steer|","max_abs_steer"),("Mean lateral acceleration","mean_lateral"),("Max lateral acceleration","max_lateral"),("Mean line error","mean_line_error"),("Max line error","max_line_error"),("Anchor→WP379 elapsed","anchor_to_379_time"),("Brake onset→WP379 elapsed","brake_to_379_time")]
    for label,key in labels:
        x=c[key];lines.append("| %s | %.3f | %.3f | %+.3f | [%+.3f, %+.3f] |"%(label,f[key]["mean"],s[key]["mean"],x["difference"],*x["cluster_bootstrap_95pct"]))
    lines += ["","## Interpretation",""]
    if r["classification"]=="EMPIRICALLY_DEMONSTRATED_NET_GAIN":lines.append("The faster seam exit survives the complete 500 m upstream-corner plus downstream interval and produces an attempt-clustered elapsed-time reduction.")
    elif r["classification"]=="SPEED_TRADEOFF_ONLY":lines.append("The faster seam exit is real, but its complete cyclic interval time is not meaningfully better; upstream cost pays for the exit advantage.")
    elif r["classification"]=="NO_EMPIRICAL_HEADROOM":lines.append("Successful traversals execute effectively the same seam exit.")
    else:lines.append("The apparent difference is not supported robustly after cyclic alignment and attempt clustering.")
    lines += ["","Complete per-traversal cyclic records are in `result.json`."]
    return "\n".join(lines)+"\n"


if __name__=="__main__":analyze(Path(__file__).resolve().parents[2])
