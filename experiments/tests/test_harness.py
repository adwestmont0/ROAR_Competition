import io
import json
import tempfile
import unittest
import numpy as np
from pathlib import Path
from subprocess import CompletedProcess
from unittest.mock import patch

from experiments.harness.core import (
    apply_parameters,
    expand_config,
    experiment_id,
    execute_with_recovery,
    preflight,
    recover_with_ssm,
    should_stop,
    update_counters,
)
from experiments.harness.evaluation import summarize_measurements
from experiments.harness.evaluation import CandidateEvaluator, _acceptance, evaluation_lock, render_candidate_report
from experiments.search.adapter import propose
from experiments.harness.cli import print_human_reports
from experiments.harness.reliability import (
    campaign_schedule,
    reliability_decision,
    render_reliability_report,
    wilson_interval,
)
from experiments.harness.causal import causal_schedule
from experiments.analysis.velocity_profile_v2 import _solve_closed
from experiments.analysis.counterfactual_opportunity import CAUSES, _gain_attribution


class HarnessTests(unittest.TestCase):
    def test_counterfactual_cause_vocabulary_is_stable(self):
        self.assertEqual(
            CAUSES,
            {
                "target-speed limitation", "braking onset", "braking magnitude",
                "braking release", "throttle onset", "acceleration capability",
                "preceding-corner exit speed", "racing-line/curvature limitation",
                "unsupported model extrapolation",
            },
        )

    def test_counterfactual_gain_separates_carry_in_from_acceleration(self):
        profile = [
            {"baseline_median_speed_kmh": 100.0, "stability_capped_planned_speed_kmh": 110.0},
            {"baseline_median_speed_kmh": 105.0, "stability_capped_planned_speed_kmh": 120.0},
        ]
        attribution = _gain_attribution([0, 1], {0, 1}, profile, 5.0)
        self.assertGreater(attribution["initial_velocity_advantage_seconds"], 0.0)
        self.assertGreater(attribution["continued_acceleration_difference_seconds"], 0.0)
        self.assertAlmostEqual(
            attribution["total_gain_seconds"],
            attribution["initial_velocity_advantage_seconds"]
            + attribution["continued_acceleration_difference_seconds"],
        )

    def test_closed_velocity_solver_enforces_forward_and_backward_limits(self):
        limit = np.array([100.0, 300.0, 300.0, 80.0])
        curvature = np.zeros(4)
        envelope = {
            "speed_centers_kmh": np.array([0.0, 300.0]),
            "acceleration_low_lateral_mps2": np.array([2.0, 2.0]),
            "acceleration_high_lateral_mps2": np.array([2.0, 2.0]),
            "deceleration_mps2": np.array([4.0, 4.0]),
        }
        solved, iterations, error = _solve_closed(limit, curvature, 10.0, envelope)
        self.assertGreater(iterations, 0)
        self.assertLess(error, 0.0011)
        self.assertTrue(np.all(solved <= limit + 1e-8))
        velocity = solved / 3.6
        for index in range(4):
            following = (index + 1) % 4
            self.assertLessEqual(velocity[following] ** 2, velocity[index] ** 2 + 2 * 2.0 * 10.0 + 1e-6)
            self.assertLessEqual(velocity[index] ** 2, velocity[following] ** 2 + 2 * 4.0 * 10.0 + 1e-6)

    def test_causal_schedule_interleaves_forced_clean_cohorts(self):
        self.assertEqual(
            causal_schedule(3),
            ["control", "restricted", "control", "restricted", "control", "restricted"],
        )
        self.assertEqual(causal_schedule(3, include_controls=False), ["restricted"] * 3)

    def base_config(self):
        return {
            "name": "test",
            "baseline_commit": "fa7ff6f",
            "carla": {"expected_map": "Carla/Maps/Monza"},
            "sweep": {
                "parameter": "throttle.section_mu.3",
                "values": [3.4, 3.3],
                "repetitions": 1,
            },
            "controls": {
                "enabled": True,
                "every": 1,
                "at_start": True,
                "at_end": True,
                "parameters": {},
            },
        }

    def test_experiment_id_identifies_parameters(self):
        self.assertEqual(experiment_id({"a": 1}), experiment_id({"a": 1}))
        self.assertNotEqual(experiment_id({"a": 1}), experiment_id({"a": 2}))

    def test_controls_are_interleaved(self):
        expanded = expand_config(self.base_config())
        self.assertEqual(
            [item["is_control"] for item in expanded],
            [True, False, True, False, True],
        )

    def test_exact_parameter_substitution(self):
        registry = {
            "mu": {
                "file": "controller.py",
                "baseline_text": "mu = 3.4\n",
                "replacement_template": "mu = {value}\n",
                "type": "float",
                "minimum": 0.1,
                "maximum": 10.0,
                "required_matches": 1,
            }
        }
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "controller.py").write_text("mu = 3.4\n", encoding="utf-8")
            diff = apply_parameters(root, {"mu": 3.3}, registry)
            self.assertEqual(
                (root / "controller.py").read_text(encoding="utf-8"), "mu = 3.3\n"
            )
            self.assertIn("-mu = 3.4", diff)
            self.assertIn("+mu = 3.3", diff)

    def test_stop_conditions(self):
        counters = {"collision": 2, "infra_error": 0, "controller_hang": 0}
        self.assertIsNotNone(
            should_stop(counters, {"consecutive_collisions": 2})
        )

    def test_baseline_and_explicit_baseline_parameter_have_distinct_ids(self):
        self.assertNotEqual(
            experiment_id({}),
            experiment_id({"throttle.section_mu.3": 3.4}),
        )

    def test_successful_control_does_not_reset_primary_collision_streak(self):
        counters = {name: 0 for name in (
            "finished", "collision", "timeout", "exception",
            "infra_error", "controller_hang",
        )}
        update_counters(counters, "collision", is_control=False)
        update_counters(counters, "finished", is_control=True)
        self.assertEqual(counters["collision"], 1)

    def test_preflight_rejects_stale_synchronous_world(self):
        settings = type(
            "Settings", (), {"fixed_delta_seconds": 0.05, "synchronous_mode": True}
        )()
        world = unittest.mock.Mock()
        world.get_map.return_value.name = "Carla/Maps/Monza"
        world.get_settings.return_value = settings
        client = unittest.mock.Mock()
        client.get_world.return_value = world
        client.get_server_version.return_value = "server"
        client.get_client_version.return_value = "client"
        carla_module = type("Carla", (), {"Client": unittest.mock.Mock(return_value=client)})
        with patch.dict("sys.modules", {"carla": carla_module}):
            result = preflight(
                {
                    "host": "localhost",
                    "port": 2000,
                    "expected_map": "Carla/Maps/Monza",
                }
            )
        self.assertFalse(result["ok"])
        self.assertIn("synchronous mode", result["error"])

    @patch("experiments.harness.core.record_recovery")
    @patch("experiments.harness.core.execute_attempt")
    def test_execute_with_recovery_retries_infra_error(
        self, execute_attempt_mock, record_recovery_mock
    ):
        execute_attempt_mock.side_effect = [
            {"outcome": "infra_error", "attempt_id": "a", "experiment_id": "e"},
            {"outcome": "finished", "attempt_id": "b", "experiment_id": "e"},
        ]
        record_recovery_mock.return_value = {"success": True}
        attempts, stop_reason = execute_with_recovery(
            {
                "name": "test",
                "carla": {},
                "execution": {"max_infra_retries": 1},
            },
            {},
            True,
            {},
            Path("."),
        )
        self.assertEqual([item["outcome"] for item in attempts], ["infra_error", "finished"])
        self.assertIsNone(stop_reason)

    def test_candidate_summary_excludes_infrastructure_and_penalizes_collision(self):
        measurements = [
            {
                "attempt_id": "finished",
                "outcome": "finished",
                "elapsed_time_seconds": 320.0,
                "telemetry_path": None,
            },
            {
                "attempt_id": "collision",
                "outcome": "collision",
                "elapsed_time_seconds": 40.0,
                "telemetry_path": None,
            },
            None,
        ]
        summary = summarize_measurements(measurements, 600.0, Path("."))
        self.assertEqual(summary["controller_measurements"], 2)
        self.assertEqual(summary["missing_due_to_infrastructure"], 1)
        self.assertEqual(summary["completion_rate"], 0.5)
        self.assertEqual(summary["penalized_objective_seconds"]["mean"], 460.0)

    def test_acceptance_rejects_collision_rate(self):
        candidate = {
            "controller_measurements": 2,
            "completion_rate": 0.5,
            "collision_rate": 0.5,
        }
        empty_control = {
            "controller_measurements": 0,
            "completion_rate": None,
            "finished_elapsed_seconds": None,
        }
        status, checks = _acceptance(
            candidate,
            empty_control,
            empty_control,
            {"before": 0, "after": 0},
            {
                "min_controller_measurements": 2,
                "min_completion_rate": 0.5,
                "max_collision_rate": 0.0,
                "require_all_controls_finished": False,
                "max_control_drift_seconds": None,
                "max_control_adjusted_delta_seconds": None,
            },
            None,
        )
        self.assertEqual(status, "rejected")
        collision_check = next(item for item in checks if item["name"] == "maximum_collision_rate")
        self.assertFalse(collision_check["passed"])

    def test_failed_control_makes_evaluation_inconclusive(self):
        candidate = {
            "controller_measurements": 3,
            "completion_rate": 1.0,
            "collision_rate": 0.0,
        }
        failed_control = {
            "controller_measurements": 1,
            "completion_rate": 0.0,
            "finished_elapsed_seconds": None,
        }
        good_control = {
            "controller_measurements": 1,
            "completion_rate": 1.0,
            "finished_elapsed_seconds": {"mean": 321.8},
        }
        status, _ = _acceptance(
            candidate,
            failed_control,
            good_control,
            {"before": 1, "after": 1},
            {
                "min_controller_measurements": 3,
                "min_completion_rate": 1.0,
                "max_collision_rate": 0.0,
                "require_all_controls_finished": True,
                "max_control_drift_seconds": 1.0,
                "max_control_adjusted_delta_seconds": None,
            },
            -100.0,
        )
        self.assertEqual(status, "inconclusive")

    def test_evaluation_lock_excludes_second_process_owner(self):
        with tempfile.TemporaryDirectory() as temporary:
            results = Path(temporary)
            with evaluation_lock(results):
                with self.assertRaisesRegex(RuntimeError, "holds the lock"):
                    with evaluation_lock(results):
                        pass

    def test_halton_search_is_deterministic_and_multi_parameter(self):
        config = {
            "search": {
                "mode": "halton",
                "samples": 2,
                "space": {
                    "a": {"minimum": 0, "maximum": 1, "precision": 3},
                    "b": {"minimum": 10, "maximum": 20, "precision": 3},
                },
            }
        }
        registry = {"a": {}, "b": {}}
        first = propose(config, registry)
        second = propose(config, registry)
        self.assertEqual(first, second)
        self.assertEqual(set(first[0]), {"a", "b"})

    @patch("experiments.harness.evaluation.git_output", return_value="full-commit")
    @patch("experiments.harness.evaluation._evaluation_key", return_value="known-key")
    def test_completed_candidate_is_resumed_without_execution(self, _key, _git):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            summary_path = root / "results" / "candidate_summaries" / "known-key.json"
            summary_path.parent.mkdir(parents=True)
            summary_path.write_text(
                json.dumps({
                    "status": "accepted",
                    "evaluation_key": "known-key",
                    "parameters": {},
                }),
                encoding="utf-8",
            )
            summary_path.with_suffix(".md").write_text("existing report", encoding="utf-8")
            evaluator = CandidateEvaluator(
                {
                    "name": "test",
                    "baseline_commit": "short",
                    "results_dir": "results",
                },
                {},
                root,
            )
            result = evaluator.evaluate({}, 1, {"before": 0, "after": 0})
            self.assertTrue(result["resume"]["skipped_completed"])

    def test_human_report_suppresses_invalid_control_delta(self):
        result = {
            "candidate_name": "baseline",
            "experiment_id": "baseline-id",
            "status": "inconclusive",
            "parameters": {},
            "controls_valid": False,
            "candidate": {
                "controller_measurements": 1, "scheduled_repetitions": 1,
                "completion_rate": 1.0, "collision_rate": 0.0,
                "finished_elapsed_seconds": {
                    "mean": 321.8, "median": 321.8, "minimum": 321.8,
                    "maximum": 321.8, "stdev": 0.0,
                },
                "terminal_failures": [],
            },
            "controls": {
                "before": {"finished_elapsed_seconds": None},
                "after": {"finished_elapsed_seconds": {"mean": 321.8}},
            },
            "objective": {
                "control_adjusted_delta_seconds": None,
                "section_control_adjusted_delta_seconds": {},
            },
            "acceptance_checks": [],
        }
        report = render_candidate_report(result)
        self.assertIn("INCONCLUSIVE", report)
        self.assertIn("unavailable because controls were invalid", report)

    def test_cli_prints_saved_human_report_to_stderr(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            report = root / "results" / "candidate.md"
            report.parent.mkdir(parents=True)
            report.write_text("Plain English result", encoding="utf-8")
            with patch("sys.stderr", new_callable=io.StringIO) as stderr:
                print_human_reports(
                    {"candidate_results": [{"human_report_path": "results/candidate.md"}]},
                    root,
                )
            self.assertIn("Plain English result", stderr.getvalue())

    def test_reliability_schedule_is_interleaved(self):
        self.assertEqual(
            campaign_schedule(3),
            ["warm", "forced_clean", "warm", "forced_clean", "warm", "forced_clean"],
        )

    def test_reliability_decision_detects_material_clean_start_effect(self):
        warm = {
            "controller_measurements": 10,
            "completion_rate": 0.6,
            "outcome_counts": {"finished": 6, "collision": 4},
        }
        forced = {
            "controller_measurements": 10,
            "completion_rate": 0.9,
            "outcome_counts": {"finished": 9, "collision": 1},
        }
        decision = reliability_decision(warm, forced, 8, 0.2)
        self.assertEqual(decision["conclusion"], "simulator_state_factor_supported")
        self.assertAlmostEqual(decision["forced_clean_minus_warm_completion_rate"], 0.3)

    def test_wilson_interval_contains_observed_rate(self):
        interval = wilson_interval(7, 10)
        self.assertLessEqual(interval[0], 0.7)
        self.assertGreaterEqual(interval[1], 0.7)

    @patch("experiments.harness.core.preflight")
    @patch("experiments.harness.core.run_aws_command")
    def test_ssm_recovery_waits_for_monza(self, aws_command, carla_preflight):
        aws_command.side_effect = [
            CompletedProcess([], 0, "command-123\n", ""),
            CompletedProcess(
                [],
                0,
                json.dumps(
                    {
                        "Status": "Success",
                        "ResponseCode": 0,
                        "StandardOutputContent": "restarted",
                        "StandardErrorContent": "",
                    }
                ),
                "",
            ),
        ]
        carla_preflight.return_value = {
            "ok": True,
            "map": "Carla/Maps/Monza",
        }
        recovery = recover_with_ssm(
            {
                "region": "us-east-1",
                "instance_id": "i-test",
                "restart_command": "restart",
                "command_timeout_seconds": 1,
                "readiness_timeout_seconds": 1,
            },
            {"expected_map": "Carla/Maps/Monza"},
        )
        self.assertTrue(recovery["success"])
        self.assertEqual(recovery["command_id"], "command-123")
        self.assertEqual(recovery["readiness"]["map"], "Carla/Maps/Monza")


if __name__ == "__main__":
    unittest.main()
