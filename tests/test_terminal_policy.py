import unittest
import copy

from competition_code.shadow.terminal_policy import (
    advance_profile_index, causal_residual_guard,
    evaluate_terminal_tail,
)
from competition_code.shadow.terminal_policy_data import load_policy_data
from competition_code.shadow.longitudinal import ShadowLongitudinalPlanner


class TerminalPolicyTest(unittest.TestCase):
    def test_advance_wraps_closed_profile(self):
        self.assertEqual(advance_profile_index([5.0,5.0,5.0],2,6.0),0)

    def test_guard_projects_from_current_state_only(self):
        delta={tick:-0.1*tick for tick in range(1,9)}
        safe,trace=causal_residual_guard(180.0,delta,[200.0]*20,[5.0]*20,0)
        self.assertTrue(safe)
        self.assertEqual([x["horizon_ticks"] for x in trace],[2,4,6,8])
        self.assertGreater(trace[-1]["projected_distance_m"],trace[0]["projected_distance_m"])

    def test_guard_rejects_predicted_overspeed(self):
        delta={tick:0.0 for tick in range(1,9)}
        safe,_=causal_residual_guard(180.0,delta,[175.0]*20,[5.0]*20,0)
        self.assertFalse(safe)

    def test_pure_evaluator_does_not_mutate_inputs_or_profile(self):
        policy=load_policy_data();count=len(policy["custom_waypoint_indices"])
        profile={"target_speed_kmh":[200.0]*count,"ds_m":[5.0]*count}
        inputs={"profile_index":10,"speed_kmh":180.0,"current_acceleration_mps2":-5.0,"mean_recent_acceleration_mps2":-4.0,"steer":0.05,"steer_change":0.01,"original_h1_brake":True,"original_h1_reason":"profile_gradient","prior_h1_brake_requests":3}
        before_inputs=copy.deepcopy(inputs);before_profile=copy.deepcopy(profile)
        first=evaluate_terminal_tail(inputs,profile,policy);second=evaluate_terminal_tail(inputs,profile,policy)
        self.assertEqual(first,second)
        self.assertEqual(inputs,before_inputs);self.assertEqual(profile,before_profile)
        self.assertIn(first["status"],("BRAKE","RELEASE","ABSTAIN","NOT_ELIGIBLE"))

    def test_second_opinion_preserves_original_shadow_output_and_profile(self):
        policy=load_policy_data();count=len(policy["custom_waypoint_indices"])
        profile={"schema_version":1,"points":[{"x":float(i),"y":0.0,"s_m":float(i*5),"ds_m":5.0,"target_speed_kmh":200.0,"acceleration_mps2":3.0,"deceleration_mps2":10.0,"active_constraint":"test","confidence":"high","support":1} for i in range(count)]}
        planner=ShadowLongitudinalPlanner(profile);before=planner.profile_identity_sha256
        output=planner.observe([0.0,0.0,0.0],180.0,{"throttle":1.0,"brake":0.0,"steer":0.0},0,terminal_tail_enabled=True)
        self.assertIn("terminal_tail",output)
        self.assertTrue(all(output["terminal_tail"]["invariants"].values()))
        self.assertEqual(before,planner._profile_checksum())
        self.assertEqual(output["command"],"throttle")


if __name__=="__main__":unittest.main()
