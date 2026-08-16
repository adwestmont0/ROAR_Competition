import numpy as np

from experiments.analysis.long_horizon_residual import FEATURES, _fit, _metrics, _predict


def _record(index):
    record={name:float(index+j) for j,name in enumerate(FEATURES)}
    record["residual_kmh"]=2.0+0.5*record[FEATURES[0]]
    return record


def test_interpretable_ridge_model_returns_finite_predictions():
    records=[_record(index) for index in range(1,30)]
    prediction=_predict(_fit(records),records)
    assert prediction.shape==(len(records),)
    assert np.isfinite(prediction).all()


def test_error_metrics_preserve_optimistic_direction():
    metrics=_metrics([10.0,10.0],[8.0,12.0])
    assert metrics["bias_kmh"]==0.0
    assert metrics["mae_kmh"]==2.0
    assert metrics["rmse_kmh"]==2.0
    assert metrics["worst_optimistic_error_kmh"]==2.0
