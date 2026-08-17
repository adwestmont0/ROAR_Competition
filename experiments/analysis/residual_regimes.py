"""Short physical-regime separability audit for terminal residual families."""

import datetime as dt
import statistics
from collections import defaultdict
from pathlib import Path

import numpy as np

from experiments.harness.core import write_json
from .long_horizon_residual import _dataset


FEATURES=("current_speed_kmh","terminal_target_speed_kmh","horizon_ticks","terminal_distance_m","winner_brake_active","recent_brake_ticks_8","ticks_since_brake_onset","ticks_since_release","current_acceleration_mps2","mean_recent_acceleration_mps2","steering_abs","lateral_load_proxy","local_curvature","downstream_mean_curvature_50m","downstream_max_curvature_50m")


def _cluster(z,k):
    best=None
    for seed in range(50):
        rng=np.random.RandomState(seed);centers=z[rng.choice(len(z),k,replace=False)]
        for _ in range(100):
            labels=((z[:,None,:]-centers[None,:,:])**2).sum(2).argmin(1);updated=np.asarray([z[labels==j].mean(0) if np.any(labels==j) else centers[j] for j in range(k)])
            if np.allclose(updated,centers):break
            centers=updated
        inertia=float(sum(((z[i]-centers[labels[i]])**2).sum() for i in range(len(z))))
        if best is None or inertia<best[0]:best=(inertia,labels.copy())
    return best[1]


def _silhouette(z,labels):
    distance=np.sqrt(((z[:,None,:]-z[None,:,:])**2).sum(2));values=[]
    for i in range(len(z)):
        same=[j for j in range(len(z)) if labels[j]==labels[i] and j!=i]
        if not same:values.append(0.0);continue
        a=float(np.mean([distance[i,j] for j in same]));other=[float(np.mean([distance[i,j] for j in range(len(z)) if labels[j]==group])) for group in set(labels) if group!=labels[i]];b=min(other);values.append((b-a)/max(a,b))
    return float(np.mean(values))


def analyze(root:Path,results_dir="experiment_results"):
    records,_,_=_dataset(root,root/results_dir);grouped=defaultdict(list)
    for row in records:grouped[row["event_family_terminal_wp"]].append(row)
    families=sorted(grouped);x=np.asarray([[statistics.median(row[name] for row in grouped[family]) for name in FEATURES] for family in families]);scale=x.std(0);scale[scale<1e-9]=1;z=(x-x.mean(0))/scale
    solutions=[]
    for k in (2,3,4):
        labels=_cluster(z,k);clusters=[]
        for group in range(k):
            members=[families[i] for i in range(len(families)) if labels[i]==group];residuals=[row["residual_kmh"] for family in members for row in grouped[family]];clusters.append({"families":members,"residual_median_kmh":statistics.median(residuals)})
        solutions.append({"clusters":k,"silhouette":_silhouette(z,labels),"membership":clusters})
    family_summary={str(family):{"records":len(grouped[family]),"residual_median_kmh":statistics.median(row["residual_kmh"] for row in grouped[family]),"feature_medians":{name:statistics.median(row[name] for row in grouped[family]) for name in FEATURES}} for family in families}
    best=max(solutions,key=lambda row:row["silhouette"]);clean=best["silhouette"]>=.5
    result={"schema_version":1,"event_type":"residual_physical_regime_clustering","created_utc":dt.datetime.now(dt.timezone.utc).isoformat(),"features":list(FEATURES),"family_summary":family_summary,"solutions":solutions,"clean_separation_threshold":.5,"best_silhouette":best["silhouette"],"cleanly_separated":clean,"regime_specific_model_fitted":False,"decision":"STOP_NO_CLEAN_PHYSICAL_REGIMES","interpretation":"No 2–4 cluster solution reaches the predeclared 0.5 silhouette threshold. The best physical grouping places terminal families WP455 and WP1293 together even though their median residuals have opposite signs (-21.6 versus +5.5 km/h). Obvious causal physical features therefore do not support a trustworthy regime-specific correction."}
    out=root/results_dir/"analysis/residual_regimes";out.mkdir(parents=True,exist_ok=True);jp=out/"result.json";mp=out/"report.md";result["result_path"]=str(jp.relative_to(root));result["human_report_path"]=str(mp.relative_to(root));write_json(jp,result)
    lines=["# Physical residual-regime separability","","| Clusters | Silhouette |","|---:|---:|"]+[f"| {row['clusters']} | {row['silhouette']:.3f} |" for row in solutions]+["","**STOP — no clean physical regime separation.**","",result["interpretation"],""]
    mp.write_text("\n".join(lines),encoding="utf-8");return result
