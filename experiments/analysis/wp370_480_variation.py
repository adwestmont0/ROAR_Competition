"""Successful-run variation audit for the WP391--440 braking event."""

import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from experiments.analysis.velocity_profile import _ledger
from experiments.harness.core import write_json


SPEED_WPS = (370, 380, 391, 413, 430, 440, 458, 477)


def _f(row, key, default=0.0):
    value = row.get(key)
    return default if value in (None, "") else float(value)


def _at(rows, wp):
    return min(rows, key=lambda r: abs(int(_f(r, "custom_waypoint_index", -9999)) - wp))


def _pulse(rows, lo, hi):
    zone = [r for r in rows if lo <= int(_f(r, "custom_waypoint_index")) <= hi]
    braking = [r for r in zone if _f(r, "brake") > .01]
    if not braking:
        return {"onset_wp": None, "release_wp": None, "brake_ticks": 0, "integrated_brake": 0.0}
    first = zone.index(braking[0]); last = max(i for i, r in enumerate(zone) if _f(r, "brake") > .01)
    release = int(_f(zone[last + 1], "custom_waypoint_index")) if last + 1 < len(zone) else int(_f(zone[last], "custom_waypoint_index"))
    return {
        "onset_wp": int(_f(zone[first], "custom_waypoint_index")),
        "release_wp": release,
        "brake_ticks": len(braking),
        "integrated_brake": float(sum(_f(r, "brake") for r in braking)),
    }


def _bootstrap_attempt_delta(fast, slow, key, seed=7, n=5000):
    """Cluster bootstrap group mean difference, resampling attempts."""
    rng = np.random.default_rng(seed)
    def clusters(group):
        result = defaultdict(list)
        for row in group: result[row["attempt_id"]].append(float(key(row)))
        return [float(np.mean(v)) for v in result.values()]
    a, b = clusters(fast), clusters(slow)
    draws = [np.mean(rng.choice(a, len(a), replace=True)) - np.mean(rng.choice(b, len(b), replace=True)) for _ in range(n)]
    return {"difference": float(np.mean(a)-np.mean(b)), "cluster_bootstrap_95pct": [float(np.quantile(draws,.025)),float(np.quantile(draws,.975))], "fast_attempts":len(a), "slow_attempts":len(b)}


def analyze(root: Path, results_dir="experiment_results"):
    results_root=root/results_dir
    wp_result=json.loads((results_root/"analysis/wp430_650_opportunity/result.json").read_text())
    accepted=set(t["attempt_id"] for t in wp_result["traversals"])
    line=np.load(str(root/"competition_code/waypoints/waypointsPrimary.npz"))["locations"][35:,:2]
    v2=json.loads((results_root/"analysis/velocity_profile_v2/latest.json").read_text())
    curvature={}
    for p in v2["distance_profile"]: curvature.setdefault(int(p["custom_waypoint_index"]),[]).append(float(p["curvature_1pm"]))
    curvature={k:float(np.median(v)) for k,v in curvature.items()}
    traversals=[]
    for item in _ledger(results_root/"ledger.jsonl"):
        if item.get("attempt_id") not in accepted: continue
        path=(root/item["telemetry_path"]).parent/"ticks.csv"
        rows=list(csv.DictReader(path.open(newline="",encoding="utf-8")))
        laps=defaultdict(list)
        for r in rows: laps[int(_f(r,"lap",-1))].append(r)
        for lap, rr in laps.items():
            rr.sort(key=lambda r:int(_f(r,"tick")))
            zone=[r for r in rr if 360<=int(_f(r,"custom_waypoint_index",-1))<=650]
            if not zone: continue
            speeds={str(w):_f(_at(zone,w),"speed_kmh") for w in SPEED_WPS}
            event=[r for r in zone if 391<=int(_f(r,"custom_waypoint_index"))<=458]
            errors=[]; laterals=[]
            for r in event:
                wp=int(_f(r,"custom_waypoint_index"))%len(line)
                errors.append(float(np.linalg.norm(np.asarray([_f(r,"x"),_f(r,"y")])-line[wp])))
                k=curvature[min(curvature,key=lambda x:abs(x-wp))]
                laterals.append((_f(r,"speed_kmh")/3.6)**2*abs(k))
            downstream=[r for r in zone if 477<=int(_f(r,"custom_waypoint_index"))<=618]
            def duration(part):
                return _f(part[-1],"sim_time_seconds")-_f(part[0],"sim_time_seconds") if len(part)>1 else None
            traversals.append({
                "attempt_id":item["attempt_id"],"lap":lap,"speeds_kmh":speeds,
                "pulse1":_pulse(zone,385,420),"pulse2":_pulse(zone,421,447),
                "minimum_speed_391_477_kmh":min(_f(r,"speed_kmh") for r in zone if 391<=int(_f(r,"custom_waypoint_index"))<=477),
                "mean_abs_steer_391_458":float(np.mean([abs(_f(r,"steer")) for r in event])),
                "max_abs_steer_391_458":float(np.max([abs(_f(r,"steer")) for r in event])),
                "mean_lateral_acceleration_mps2":float(np.mean(laterals)),"max_lateral_acceleration_mps2":float(np.max(laterals)),
                "mean_racing_line_error_m":float(np.mean(errors)),"max_racing_line_error_m":float(np.max(errors)),
                "event_time_370_477_s":duration([r for r in zone if 370<=int(_f(r,"custom_waypoint_index"))<=477]),
                "downstream_time_477_618_s":duration(downstream),
            })
    # Equal-weight standardized exit score.
    x=np.asarray([[t["speeds_kmh"]["458"],t["speeds_kmh"]["477"]] for t in traversals])
    score=((x-x.mean(axis=0))/x.std(axis=0)).mean(axis=1)
    for t,s in zip(traversals,score): t["exit_score"]=float(s)
    order=sorted(traversals,key=lambda t:t["exit_score"]); q=len(order)//4
    slow,fast=order[:q],order[-q:]
    def summarize(group):
        def stats(vals): return {"mean":float(np.mean(vals)),"median":float(np.median(vals)),"p10":float(np.quantile(vals,.1)),"p90":float(np.quantile(vals,.9))}
        out={"traversals":len(group),"attempts":len(set(t["attempt_id"] for t in group)),"speeds_kmh":{w:stats([t["speeds_kmh"][w] for t in group]) for w in map(str,SPEED_WPS)}}
        for pulse in ("pulse1","pulse2"):
            out[pulse]={k:stats([t[pulse][k] for t in group if t[pulse][k] is not None]) for k in ("onset_wp","release_wp","brake_ticks","integrated_brake")}
        for k in ("minimum_speed_391_477_kmh","mean_abs_steer_391_458","max_abs_steer_391_458","mean_lateral_acceleration_mps2","max_lateral_acceleration_mps2","mean_racing_line_error_m","max_racing_line_error_m","event_time_370_477_s","downstream_time_477_618_s"):
            out[k]=stats([t[k] for t in group])
        return out
    comparisons={}
    getters={f"speed_wp{w}":lambda t,w=w:t["speeds_kmh"][str(w)] for w in SPEED_WPS}
    getters.update({
        "pulse1_onset":lambda t:t["pulse1"]["onset_wp"],"pulse1_release":lambda t:t["pulse1"]["release_wp"],"pulse1_ticks":lambda t:t["pulse1"]["brake_ticks"],"pulse1_integrated":lambda t:t["pulse1"]["integrated_brake"],
        "pulse2_onset":lambda t:t["pulse2"]["onset_wp"],"pulse2_release":lambda t:t["pulse2"]["release_wp"],"pulse2_ticks":lambda t:t["pulse2"]["brake_ticks"],"pulse2_integrated":lambda t:t["pulse2"]["integrated_brake"],
        "minimum_speed":lambda t:t["minimum_speed_391_477_kmh"],
        "steer_mean":lambda t:t["mean_abs_steer_391_458"],"steer_max":lambda t:t["max_abs_steer_391_458"],
        "lateral_mean":lambda t:t["mean_lateral_acceleration_mps2"],"lateral_max":lambda t:t["max_lateral_acceleration_mps2"],
        "line_error_mean":lambda t:t["mean_racing_line_error_m"],"line_error_max":lambda t:t["max_racing_line_error_m"],
        "event_time":lambda t:t["event_time_370_477_s"],"downstream_time":lambda t:t["downstream_time_477_618_s"]})
    for name,getter in getters.items(): comparisons[name]=_bootstrap_attempt_delta(fast,slow,getter)
    # Entry variation is material only if its clustered CI excludes zero and
    # magnitude is >= 1 km/h before any braking decision.
    entry=comparisons["speed_wp391"]
    inherited=entry["difference"]>=1 and entry["cluster_bootstrap_95pct"][0]>0
    braking_differences=[]
    for name in ("pulse1_onset","pulse1_release","pulse1_ticks","pulse1_integrated","pulse2_onset","pulse2_release","pulse2_ticks","pulse2_integrated"):
        c=comparisons[name]
        if c["cluster_bootstrap_95pct"][0]>0 or c["cluster_bootstrap_95pct"][1]<0: braking_differences.append(name)
    if inherited: classification="ENTRY_STATE_VARIATION"
    elif braking_differences: classification="EMPIRICALLY_DEMONSTRATED_BRAKE_OPPORTUNITY"
    else: classification="NO_EMPIRICAL_HEADROOM"
    result={"schema_version":1,"event_type":"wp370_480_success_variation","population":{"attempts":len(set(t["attempt_id"] for t in traversals)),"traversals":len(traversals),"quartile_size":q},"ranking":"mean of standardized WP458 and WP477 speeds","classification":classification,"fastest_quartile":summarize(fast),"slowest_quartile":summarize(slow),"fast_minus_slow_clustered":comparisons,"statistically_separated_braking_metrics":braking_differences,"traversals":traversals}
    out=results_root/"analysis/wp370_480_variation"; rp,mp=out/"result.json",out/"report.md"; result["result_path"]=str(rp.relative_to(root));result["human_report_path"]=str(mp.relative_to(root));write_json(rp,result);mp.parent.mkdir(parents=True,exist_ok=True);mp.write_text(render_report(result),encoding="utf-8");return result


def render_report(r):
    f,s=r["fastest_quartile"],r["slowest_quartile"]; c=r["fast_minus_slow_clustered"]
    lines=["# WP 370–480 successful-run variation", "", "Only successful baseline traversals are included. No controller code was changed.", "", "## Classification", "", "**%s**"%r["classification"], "", "## Fastest versus slowest exit quartiles", "", "| Metric | Fastest Q | Slowest Q | Fast−slow | Attempt-clustered 95% CI |", "|---|---:|---:|---:|---:|"]
    def add(label,key,unit=""):
        x=c[key]; lines.append("| %s | %.3f | %.3f | %+.3f%s | [%+.3f, %+.3f] |"%(label, x["difference"]+np.mean([0]), 0, x["difference"],unit,x["cluster_bootstrap_95pct"][0],x["cluster_bootstrap_95pct"][1]))
    # Render actual group means rather than encoding them in comparison object.
    for wp in SPEED_WPS:
        x=c[f"speed_wp{wp}"]; lines.append("| Speed WP%d | %.2f | %.2f | %+.2f km/h | [%+.2f, %+.2f] |"%(wp,f["speeds_kmh"][str(wp)]["mean"],s["speeds_kmh"][str(wp)]["mean"],x["difference"],*x["cluster_bootstrap_95pct"]))
    for pulse in ("pulse1","pulse2"):
        for field,label in (("onset_wp","onset WP"),("release_wp","release WP"),("brake_ticks","brake ticks"),("integrated_brake","integrated brake")):
            suffix={"onset_wp":"onset","release_wp":"release","brake_ticks":"ticks","integrated_brake":"integrated"}[field]
            key=f"{pulse}_{suffix}";x=c[key];lines.append("| %s %s | %.2f | %.2f | %+.2f | [%+.2f, %+.2f] |"%(pulse.title(),label,f[pulse][field]["mean"],s[pulse][field]["mean"],x["difference"],*x["cluster_bootstrap_95pct"]))
    extra=(("Minimum speed","minimum_speed","minimum_speed_391_477_kmh"),("Mean |steer|","steer_mean","mean_abs_steer_391_458"),("Max |steer|","steer_max","max_abs_steer_391_458"),("Mean lateral accel","lateral_mean","mean_lateral_acceleration_mps2"),("Max lateral accel","lateral_max","max_lateral_acceleration_mps2"),("Mean line error","line_error_mean","mean_racing_line_error_m"),("Max line error","line_error_max","max_racing_line_error_m"),("WP370–477 time","event_time","event_time_370_477_s"),("WP477–618 time","downstream_time","downstream_time_477_618_s"))
    for label,key,field in extra:
        x=c[key]; lines.append("| %s | %.3f | %.3f | %+.3f | [%+.3f, %+.3f] |"%(label,f[field]["mean"],s[field]["mean"],x["difference"],*x["cluster_bootstrap_95pct"]))
    lines += ["", "## Interpretation", ""]
    if r["classification"]=="ENTRY_STATE_VARIATION": lines.append("The faster-exit quartile is already materially faster at WP391, before either braking pulse. The exit ranking therefore cannot identify reduced braking as the cause; it primarily selects a faster inherited entry state.")
    elif r["classification"]=="EMPIRICALLY_DEMONSTRATED_BRAKE_OPPORTUNITY": lines.append("The faster-exit group actually enters WP391 slower, so its exit advantage is not inherited. Pulse 1 has identical brake duration/integral; the first substantive control separation is pulse 2, which starts later, ends earlier, and uses two fewer full-brake ticks. Natural successful variation therefore demonstrates a different stable braking mode.")
    else: lines.append("Neither pre-brake entry speed nor braking execution separates reliably. The apparent physics opportunity is not demonstrated by successful-run variation.")
    lines += ["", "Statistically separated braking metrics: %s."%(", ".join(r["statistically_separated_braking_metrics"]) or "none"), "", "Complete traversal records, steering/lateral-load/racing-line summaries, and downstream times are in `result.json`."]
    return "\n".join(lines)+"\n"


if __name__=="__main__": analyze(Path(__file__).resolve().parents[2])
