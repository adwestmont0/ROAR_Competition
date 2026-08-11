import unittest

from experiments.analysis.shadow_disagreement import classify_tick, group_flags, cluster_events


def row(**updates):
    base = {
        "speed_kmh": "180", "brake": "0", "shadow_brake": "0",
        "controller_recommended_speed_kmh": "180", "controller_target_speed_kmh": "180",
        "controller_selected_target_distance_m": "30", "shadow_target_speed_kmh": "180",
        "shadow_required_acceleration_mps2": "0", "shadow_phase": "accelerating",
    }
    base.update({key: str(value) for key, value in updates.items()})
    return base


class ShadowDisagreementTest(unittest.TestCase):
    def test_ignores_float_noise(self):
        self.assertFalse(classify_tick(row(shadow_brake=.01, shadow_target_speed_kmh=182))[0])

    def test_detects_brake_state_disagreement(self):
        flag, kinds, _ = classify_tick(row(brake=1, shadow_brake=0))
        self.assertTrue(flag)
        self.assertIn("winner_brakes_shadow_does_not", kinds)

    def test_groups_one_tick_gap(self):
        self.assertEqual(group_flags([False, True, False, True, False, False, True]), [(1, 4), (6, 7)])

    def test_ranking_penalizes_risk(self):
        def event(wp, gain, steer):
            return {"attempt_id": str(wp), "lap": 1, "start_s_m": float(wp), "end_s_m": float(wp+5), "start_waypoint": wp, "end_waypoint": wp+2, "section": 1, "primary_disagreement": "earlier release", "predicted_time_opportunity_200m_s": gain, "profile_alignment_error_max_m": 1, "max_abs_steering": steer, "max_lateral_acceleration_mps2": 5, "duration_ticks": 4, "magnitude": 2, "winner_minus_shadow_brake_integral": 1, "winner": {"mean_brake": 1, "brake_duration_ticks": 5}, "shadow": {"mean_brake": 0, "brake_duration_ticks": 4}, "downstream": {}, "active_constraints": {}}
        clusters = cluster_events([event(100, .1, .1), event(1300, .1, .8)], 2, 5800)
        safe = min(clusters, key=lambda x: x["risk_score"])
        risky = max(clusters, key=lambda x: x["risk_score"])
        self.assertGreater(safe["promising_score"], risky["promising_score"])


if __name__ == "__main__":
    unittest.main()
