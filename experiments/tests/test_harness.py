import json
import tempfile
import unittest
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


class HarnessTests(unittest.TestCase):
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
