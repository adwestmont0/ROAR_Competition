import numpy as np

from experiments.analysis.residual_regimes import _cluster, _silhouette


def test_separated_synthetic_clusters_have_high_silhouette():
    points=np.asarray([[0.,0.],[.1,0.],[10.,10.],[10.1,10.]])
    labels=_cluster(points,2)
    assert _silhouette(points,labels)>.9
