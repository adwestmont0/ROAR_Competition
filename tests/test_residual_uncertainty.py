from experiments.analysis.residual_uncertainty import _bucket, _coverage


def test_horizon_buckets_are_stable():
    assert _bucket(1)=="01-08"
    assert _bucket(8)=="01-08"
    assert _bucket(9)=="09-16"
    assert _bucket(48)=="41-48"


def test_one_sided_coverage_counts_only_optimistic_misses():
    rows=[
        {"realized_terminal_speed_kmh":10.0,"corrected_predicted_terminal_speed_kmh":9.0,"bound":1.0},
        {"realized_terminal_speed_kmh":12.0,"corrected_predicted_terminal_speed_kmh":9.0,"bound":1.0},
    ]
    result=_coverage(rows,"bound")
    assert result["coverage"]==0.5
    assert result["optimistic_exceedances"]==1
    assert result["maximum_optimistic_excess_kmh"]==2.0
