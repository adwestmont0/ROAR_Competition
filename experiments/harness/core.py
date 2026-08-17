import contextlib
import datetime as dt
import difflib
import hashlib
import importlib
import importlib.metadata
import io
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


OUTCOMES = {
    "finished",
    "collision",
    "timeout",
    "exception",
    "infra_error",
    "controller_hang",
}

INFRA_PATTERNS = (
    "time-out of",
    "waiting for the simulator",
    "connection refused",
    "connection reset",
    "failed to connect",
    "rpc error",
    "carla server",
)


def read_json(path: Path) -> Dict[str, Any]:
    with path.open(encoding="utf-8") as infile:
        return json.load(infile)


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as outfile:
        json.dump(value, outfile, indent=2, sort_keys=True)
        outfile.write("\n")
        outfile.flush()
        os.fsync(outfile.fileno())
    os.replace(str(temporary), str(path))


def append_jsonl(path: Path, value: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    line = json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n"
    with path.open("a", encoding="utf-8") as outfile:
        outfile.write(line)
        outfile.flush()
        os.fsync(outfile.fileno())


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def git_output(root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=str(root), check=True, capture_output=True, text=True
    )
    return result.stdout.strip()


def canonical_parameters(parameters: Dict[str, Any]) -> str:
    return json.dumps(parameters, sort_keys=True, separators=(",", ":"))


def experiment_id(parameters: Dict[str, Any]) -> str:
    canonical = canonical_parameters(parameters)
    digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:12]
    if not parameters:
        return f"baseline-{digest}"
    readable = "__".join(
        f"{re.sub(r'[^A-Za-z0-9_.-]+', '-', name)}={value}"
        for name, value in sorted(parameters.items())
    )
    return f"{readable}--{digest}"


def new_attempt_id() -> str:
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return f"{timestamp}-{uuid.uuid4().hex[:8]}"


def validate_config(config: Dict[str, Any]) -> None:
    required = ("name", "baseline_commit", "carla")
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError(f"Missing config keys: {', '.join(missing)}")
    carla_config = config["carla"]
    if carla_config.get("expected_map") != "Carla/Maps/Monza":
        raise ValueError("carla.expected_map must be Carla/Maps/Monza")
    if not any(name in config for name in ("sweep", "parameters", "matrix")):
        raise ValueError("Config must contain sweep, parameters, or matrix")


def expand_config(config: Dict[str, Any]) -> List[Dict[str, Any]]:
    validate_config(config)
    primary: List[Dict[str, Any]] = []
    if "matrix" in config:
        repetitions = int(config.get("repetitions", 1))
        for parameters in config["matrix"]:
            for _ in range(repetitions):
                primary.append(dict(parameters))
    elif "sweep" in config:
        sweep = config["sweep"]
        repetitions = int(sweep.get("repetitions", 1))
        for value in sweep["values"]:
            for _ in range(repetitions):
                primary.append({sweep["parameter"]: value})
    else:
        repetitions = int(config.get("repetitions", 1))
        for _ in range(repetitions):
            primary.append(dict(config.get("parameters", {})))

    fixed_parameters = dict(config.get("fixed_parameters", {}))
    primary = [dict(fixed_parameters, **item) for item in primary]
    controls = config.get("controls", {})
    if not controls.get("enabled", False):
        return [{"parameters": item, "is_control": False} for item in primary]

    control_parameters = dict(
        fixed_parameters, **controls.get("parameters", {})
    )
    every = max(1, int(controls.get("every", 1)))
    expanded: List[Dict[str, Any]] = []
    if controls.get("at_start", True):
        expanded.append({"parameters": control_parameters, "is_control": True})
    for index, parameters in enumerate(primary, start=1):
        expanded.append({"parameters": parameters, "is_control": False})
        if index % every == 0 and index < len(primary):
            expanded.append({"parameters": control_parameters, "is_control": True})
    if controls.get("at_end", True):
        expanded.append({"parameters": control_parameters, "is_control": True})
    return expanded


def safe_extract_archive(archive_bytes: bytes, destination: Path) -> None:
    with tarfile.open(fileobj=io.BytesIO(archive_bytes), mode="r:") as archive:
        destination_resolved = destination.resolve()
        for member in archive.getmembers():
            target = (destination / member.name).resolve()
            if destination_resolved not in target.parents and target != destination_resolved:
                raise ValueError(f"Unsafe archive member: {member.name}")
        archive.extractall(str(destination))


def export_baseline(root: Path, commit: str, destination: Path) -> str:
    resolved_commit = git_output(root, "rev-parse", f"{commit}^{{commit}}")
    result = subprocess.run(
        ["git", "archive", resolved_commit],
        cwd=str(root),
        check=True,
        capture_output=True,
    )
    safe_extract_archive(result.stdout, destination)
    return resolved_commit


def format_parameter_value(specification: Dict[str, Any], value: Any) -> str:
    value_type = specification["type"]
    if value_type == "float":
        converted = float(value)
        if not specification["minimum"] <= converted <= specification["maximum"]:
            raise ValueError(f"Float value {converted} is outside registered range")
        return str(value)
    if value_type == "int":
        converted = int(value)
        if converted != value:
            raise ValueError(f"Expected integer, got {value}")
        return str(converted)
    raise ValueError(f"Unsupported parameter type: {value_type}")


def apply_parameters(
    checkout: Path,
    parameters: Dict[str, Any],
    registry: Dict[str, Any],
) -> str:
    diffs: List[str] = []
    for name, value in sorted(parameters.items()):
        if name not in registry:
            raise ValueError(f"Unregistered experiment parameter: {name}")
        specification = registry[name]
        formatted_value = format_parameter_value(specification, value)
        edits = specification.get("edits", [specification])
        for edit in edits:
            source_path = checkout / edit["file"]
            if "copy_from" in edit:
                if source_path.exists():
                    raise ValueError(f"{name} ({edit['file']}): destination already exists")
                source = Path(__file__).resolve().parents[2] / edit["copy_from"]
                source_path.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
                diffs.extend(
                    difflib.unified_diff(
                        [], source_path.read_text(encoding="utf-8").splitlines(keepends=True),
                        fromfile=f"a/{edit['file']}", tofile=f"b/{edit['file']}",
                    )
                )
                continue
            before = source_path.read_text(encoding="utf-8")
            baseline_text = edit["baseline_text"]
            required_matches = int(edit.get("required_matches", 1))
            matches = before.count(baseline_text)
            if matches != required_matches:
                raise ValueError(
                    f"{name} ({edit['file']}): expected {required_matches} exact "
                    f"baseline matches, found {matches}"
                )
            rendered = edit["replacement_template"].replace(
                "{value}", formatted_value
            )
            after = before.replace(baseline_text, rendered, required_matches)
            source_path.write_text(after, encoding="utf-8")
            diffs.extend(
                difflib.unified_diff(
                    before.splitlines(keepends=True),
                    after.splitlines(keepends=True),
                    fromfile=f"a/{edit['file']}",
                    tofile=f"b/{edit['file']}",
                )
            )
    return "".join(diffs)


def distribution_version(names: Iterable[str]) -> Optional[str]:
    for name in names:
        with contextlib.suppress(importlib.metadata.PackageNotFoundError):
            return importlib.metadata.version(name)
    return None


def module_provenance(module_name: str, distributions: Iterable[str]) -> Dict[str, Any]:
    result: Dict[str, Any] = {"module": module_name}
    try:
        module = importlib.import_module(module_name)
        module_path = getattr(module, "__file__", None)
        result["path"] = module_path
        result["version"] = distribution_version(distributions) or getattr(
            module, "__version__", None
        )
        if module_path is not None:
            module_directory = Path(module_path).resolve().parent
            git_result = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=str(module_directory),
                capture_output=True,
                text=True,
            )
            if git_result.returncode == 0:
                result["repository_commit"] = git_result.stdout.strip()
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
    return result


def local_provenance(root: Path, baseline_commit: str) -> Dict[str, Any]:
    repository_status = git_output(root, "status", "--porcelain")
    return {
        "python": {
            "version": sys.version,
            "executable": sys.executable,
            "platform": platform.platform(),
        },
        "carla_python": module_provenance("carla", ("carla",)),
        "roar_py": module_provenance(
            "roar_py_interface", ("roar-py-interface", "roar_py_interface")
        ),
        "roar_py_carla": module_provenance(
            "roar_py_carla", ("roar-py-carla", "roar_py_carla")
        ),
        "repo": {
            "baseline_commit": baseline_commit,
            "harness_commit": git_output(root, "rev-parse", "HEAD"),
            "harness_dirty": bool(repository_status),
            "harness_status": repository_status.splitlines(),
        },
    }


def aws_provenance(region: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {"region": region}
    version = subprocess.run(
        ["aws", "--version"], capture_output=True, text=True
    )
    result["cli_version"] = (version.stdout or version.stderr).strip()
    identity = subprocess.run(
        ["aws", "sts", "get-caller-identity", "--output", "json"],
        capture_output=True,
        text=True,
    )
    if identity.returncode == 0:
        result["caller_identity"] = json.loads(identity.stdout)
    else:
        result["identity_error"] = identity.stderr.strip()
    return result


def preflight(carla_config: Dict[str, Any]) -> Dict[str, Any]:
    started = time.monotonic()
    result: Dict[str, Any] = {
        "ok": False,
        "host": carla_config["host"],
        "port": int(carla_config["port"]),
        "expected_map": carla_config["expected_map"],
    }
    try:
        import carla

        client = carla.Client(carla_config["host"], int(carla_config["port"]))
        client.set_timeout(float(carla_config.get("timeout_seconds", 10)))
        world = client.get_world()
        map_name = world.get_map().name
        settings = world.get_settings()
        result.update(
            {
                "map": map_name,
                "server_version": client.get_server_version(),
                "client_version": client.get_client_version(),
                "world_fixed_delta_seconds": settings.fixed_delta_seconds,
                "synchronous_mode": settings.synchronous_mode,
            }
        )
        if map_name != carla_config["expected_map"]:
            result["error"] = (
                f"Expected map {carla_config['expected_map']}, found {map_name}"
            )
        elif settings.synchronous_mode:
            result["error"] = (
                "CARLA world is unexpectedly in synchronous mode; "
                "a previous client may have left stale simulation state"
            )
        else:
            result["ok"] = True
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
    result["latency_seconds"] = time.monotonic() - started
    return result


def run_aws_command(arguments: List[str]) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["aws", *arguments], capture_output=True, text=True
    )


def recover_with_ssm(
    recovery_config: Dict[str, Any], carla_config: Dict[str, Any]
) -> Dict[str, Any]:
    started = time.monotonic()
    event: Dict[str, Any] = {
        "event_type": "infrastructure_recovery",
        "provider": "aws_ssm",
        "region": recovery_config["region"],
        "instance_id": recovery_config["instance_id"],
        "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "success": False,
    }
    parameters = json.dumps(
        {"commands": [recovery_config["restart_command"]]}, separators=(",", ":")
    )
    send = run_aws_command(
        [
            "ssm",
            "send-command",
            "--region",
            recovery_config["region"],
            "--instance-ids",
            recovery_config["instance_id"],
            "--document-name",
            recovery_config.get("document_name", "AWS-RunPowerShellScript"),
            "--comment",
            "ROAR experiment harness CARLA recovery",
            "--parameters",
            parameters,
            "--query",
            "Command.CommandId",
            "--output",
            "text",
        ]
    )
    if send.returncode != 0:
        event["error"] = "send_command_failed"
        event["stderr"] = send.stderr.strip()
        event["wall_time_seconds"] = time.monotonic() - started
        return event

    command_id = send.stdout.strip()
    event["command_id"] = command_id
    command_deadline = time.monotonic() + float(
        recovery_config.get("command_timeout_seconds", 240)
    )
    terminal_statuses = {
        "Success",
        "Cancelled",
        "Failed",
        "TimedOut",
        "Cancelling",
    }
    invocation: Optional[Dict[str, Any]] = None
    while time.monotonic() < command_deadline:
        query = run_aws_command(
            [
                "ssm",
                "get-command-invocation",
                "--region",
                recovery_config["region"],
                "--command-id",
                command_id,
                "--instance-id",
                recovery_config["instance_id"],
                "--output",
                "json",
            ]
        )
        if query.returncode == 0:
            invocation = json.loads(query.stdout)
            if invocation.get("Status") in terminal_statuses:
                break
        elif "InvocationDoesNotExist" not in query.stderr:
            event["invocation_query_error"] = query.stderr.strip()
        time.sleep(2)

    if invocation is None:
        event["error"] = "command_invocation_unavailable"
        event["wall_time_seconds"] = time.monotonic() - started
        return event
    event["command_status"] = invocation.get("Status")
    event["command_response_code"] = invocation.get("ResponseCode")
    event["command_stdout"] = invocation.get("StandardOutputContent", "")
    event["command_stderr"] = invocation.get("StandardErrorContent", "")
    if invocation.get("Status") != "Success":
        event["error"] = "restart_command_failed"
        event["wall_time_seconds"] = time.monotonic() - started
        return event

    readiness_deadline = time.monotonic() + float(
        recovery_config.get("readiness_timeout_seconds", 240)
    )
    readiness_checks: List[Dict[str, Any]] = []
    while time.monotonic() < readiness_deadline:
        check = preflight(carla_config)
        readiness_checks.append(check)
        if check.get("ok"):
            event["success"] = True
            event["readiness"] = check
            break
        time.sleep(float(recovery_config.get("readiness_poll_seconds", 5)))
    event["readiness_attempts"] = len(readiness_checks)
    if not event["success"]:
        event["error"] = "carla_readiness_timeout"
        if readiness_checks:
            event["last_readiness"] = readiness_checks[-1]
    event["wall_time_seconds"] = time.monotonic() - started
    return event


def run_recovery(
    recovery_config: Dict[str, Any], carla_config: Dict[str, Any]
) -> Dict[str, Any]:
    provider = recovery_config.get("provider")
    if provider == "aws_ssm":
        return recover_with_ssm(recovery_config, carla_config)
    return {
        "event_type": "infrastructure_recovery",
        "provider": provider,
        "success": False,
        "error": f"unsupported_recovery_provider: {provider}",
    }


def compile_checkout(checkout: Path) -> None:
    files = [
        "competition_code/competition_runner.py",
        "competition_code/submission.py",
        "competition_code/LateralController.py",
        "competition_code/ThrottleController.py",
        "competition_code/telemetry.py",
    ]
    subprocess.run(
        [sys.executable, "-m", "py_compile", *files],
        cwd=str(checkout),
        check=True,
        capture_output=True,
        text=True,
    )


def find_telemetry(checkout: Path) -> Optional[Tuple[Path, Dict[str, Any]]]:
    summaries = sorted(
        (checkout / "competition_code" / "telemetry" / "runs").glob(
            "*/summary.json"
        )
    )
    if not summaries:
        return None
    summary_path = summaries[-1]
    return summary_path.parent, read_json(summary_path)


def log_indicates_infra_error(stdout_path: Path, stderr_path: Path) -> bool:
    text = ""
    for path in (stdout_path, stderr_path):
        if path.exists():
            text += path.read_text(encoding="utf-8", errors="replace").lower()
    return any(pattern in text for pattern in INFRA_PATTERNS)


def copy_telemetry(source: Path, destination: Path) -> None:
    if destination.exists():
        shutil.rmtree(str(destination))
    shutil.copytree(str(source), str(destination))


def execute_attempt(
    config: Dict[str, Any],
    parameters: Dict[str, Any],
    is_control: bool,
    registry: Dict[str, Any],
    root: Path,
) -> Dict[str, Any]:
    exp_id = experiment_id(parameters)
    att_id = new_attempt_id()
    results_root = root / config.get("results_dir", "experiment_results")
    attempt_dir = results_root / "runs" / att_id
    attempt_dir.mkdir(parents=True, exist_ok=False)
    stdout_path = attempt_dir / "stdout.log"
    stderr_path = attempt_dir / "stderr.log"
    resolved_path = attempt_dir / "resolved_config.json"
    preflight_path = attempt_dir / "preflight.json"
    diff_path = attempt_dir / "source.diff"
    started_utc = dt.datetime.now(dt.timezone.utc).isoformat()
    monotonic_start = time.monotonic()

    with tempfile.TemporaryDirectory(prefix="roar-experiment-") as temporary:
        checkout = Path(temporary)
        resolved_commit = export_baseline(root, config["baseline_commit"], checkout)
        provenance = local_provenance(root, resolved_commit)
        if config.get("recovery", {}).get("provider") == "aws_ssm":
            provenance["aws"] = aws_provenance(config["recovery"]["region"])
        source_diff = apply_parameters(checkout, parameters, registry)
        diff_path.write_text(source_diff, encoding="utf-8")
        resolved = {
            "experiment_name": config["name"],
            "experiment_id": exp_id,
            "attempt_id": att_id,
            "parameters": parameters,
            "is_control": is_control,
            "baseline_commit": resolved_commit,
            "carla": config["carla"],
            "execution": config.get("execution", {}),
            "recovery": config.get("recovery"),
        }
        write_json(resolved_path, resolved)

        try:
            compile_checkout(checkout)
        except subprocess.CalledProcessError as error:
            stderr_path.write_text(error.stderr or str(error), encoding="utf-8")
            outcome = "exception"
            preflight_result = {"ok": False, "skipped": "source compilation failed"}
            write_json(preflight_path, preflight_result)
            telemetry_summary = None
        else:
            preflight_result = preflight(config["carla"])
            write_json(preflight_path, preflight_result)
            provenance["carla_server"] = preflight_result
            if not preflight_result["ok"]:
                stdout_path.write_text("", encoding="utf-8")
                stderr_path.write_text(
                    preflight_result.get("error", "CARLA preflight failed") + "\n",
                    encoding="utf-8",
                )
                outcome = "infra_error"
                telemetry_summary = None
            else:
                timeout_seconds = float(
                    config.get("execution", {}).get("client_timeout_seconds", 600)
                )
                with stdout_path.open("w", encoding="utf-8") as stdout_file, stderr_path.open(
                    "w", encoding="utf-8"
                ) as stderr_file:
                    process = subprocess.Popen(
                        [sys.executable, "competition_runner.py"],
                        cwd=str(checkout / "competition_code"),
                        stdout=stdout_file,
                        stderr=stderr_file,
                        text=True,
                    )
                    try:
                        return_code = process.wait(timeout=timeout_seconds)
                    except subprocess.TimeoutExpired:
                        process.terminate()
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                        post_hang_preflight = preflight(config["carla"])
                        write_json(attempt_dir / "post_hang_preflight.json", post_hang_preflight)
                        outcome = (
                            "controller_hang"
                            if post_hang_preflight.get("ok")
                            else "infra_error"
                        )
                        return_code = process.returncode
                    else:
                        outcome = "exception"

                telemetry = find_telemetry(checkout)
                telemetry_summary = None
                if telemetry is not None:
                    telemetry_dir, telemetry_summary = telemetry
                    copy_telemetry(telemetry_dir, attempt_dir / "telemetry")
                    telemetry_status = telemetry_summary.get("status")
                    if telemetry_status in {"finished", "collision", "timeout"}:
                        outcome = telemetry_status
                elif outcome not in {"controller_hang", "infra_error"}:
                    outcome = (
                        "infra_error"
                        if log_indicates_infra_error(stdout_path, stderr_path)
                        else "exception"
                    )
                if return_code != 0 and outcome == "exception" and log_indicates_infra_error(
                    stdout_path, stderr_path
                ):
                    outcome = "infra_error"

    if outcome not in OUTCOMES:
        raise RuntimeError(f"Unknown outcome: {outcome}")
    elapsed_wall = time.monotonic() - monotonic_start
    summary = telemetry_summary or {}
    result = {
        "experiment_name": config["name"],
        "experiment_id": exp_id,
        "attempt_id": att_id,
        "is_control": is_control,
        "parameters": parameters,
        "baseline_commit": provenance["repo"]["baseline_commit"],
        "outcome": outcome,
        "started_utc": started_utc,
        "wall_time_seconds": elapsed_wall,
        "elapsed_time_seconds": summary.get("elapsed_time_seconds"),
        "control_timestep_seconds": summary.get("control_timestep_seconds"),
        "terminal_tick": summary.get("terminal_tick"),
        "official_waypoint_index": summary.get("official_waypoint_index"),
        "custom_waypoint_index": summary.get("custom_waypoint_index"),
        "terminal_location": summary.get("location"),
        "collision_impulse": summary.get("collision_impulse"),
        "telemetry_path": (
            str((attempt_dir / "telemetry" / "summary.json").relative_to(root))
            if (attempt_dir / "telemetry" / "summary.json").exists()
            else None
        ),
        "stdout_path": str(stdout_path.relative_to(root)),
        "stderr_path": str(stderr_path.relative_to(root)),
        "source_diff_path": str(diff_path.relative_to(root)),
        "provenance": provenance,
    }
    write_json(attempt_dir / "result.json", result)
    append_jsonl(results_root / "ledger.jsonl", result)
    return result


def dry_run(
    config: Dict[str, Any], registry: Dict[str, Any], root: Path
) -> List[Dict[str, Any]]:
    output: List[Dict[str, Any]] = []
    seen = set()
    for concrete in expand_config(config):
        parameters = concrete["parameters"]
        key = canonical_parameters(parameters)
        if key in seen:
            continue
        seen.add(key)
        with tempfile.TemporaryDirectory(prefix="roar-experiment-dry-run-") as temporary:
            checkout = Path(temporary)
            resolved_commit = export_baseline(root, config["baseline_commit"], checkout)
            source_diff = apply_parameters(checkout, parameters, registry)
            compile_checkout(checkout)
        output.append(
            {
                "experiment_id": experiment_id(parameters),
                "parameters": parameters,
                "baseline_commit": resolved_commit,
                "source_diff": source_diff,
            }
        )
    return output


def load_ledger(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as infile:
        return [json.loads(line) for line in infile if line.strip()]


def should_stop(
    counters: Dict[str, int], stop_conditions: Dict[str, Any]
) -> Optional[str]:
    checks = (
        ("collision", "consecutive_collisions"),
        ("infra_error", "consecutive_infra_failures"),
        ("controller_hang", "consecutive_controller_hangs"),
    )
    for outcome, setting in checks:
        limit = stop_conditions.get(setting)
        if limit is not None and counters[outcome] >= int(limit):
            return f"{setting} reached ({limit})"
    return None


def update_counters(
    counters: Dict[str, int], outcome: str, is_control: bool
) -> None:
    if is_control:
        counters["infra_error"] = (
            counters["infra_error"] + 1 if outcome == "infra_error" else 0
        )
        counters["controller_hang"] = (
            counters["controller_hang"] + 1
            if outcome == "controller_hang"
            else 0
        )
        return
    for key in counters:
        counters[key] = counters[key] + 1 if key == outcome else 0


def record_recovery(
    config: Dict[str, Any],
    root: Path,
    result: Dict[str, Any],
    reason: str,
) -> Dict[str, Any]:
    recovery_config = config.get("recovery")
    if recovery_config is None:
        recovery = {
            "event_type": "infrastructure_recovery",
            "success": False,
            "error": "recovery_not_configured",
        }
    else:
        recovery = run_recovery(recovery_config, config["carla"])
    recovery.update(
        {
            "experiment_name": config["name"],
            "experiment_id": result["experiment_id"],
            "source_attempt_id": result["attempt_id"],
            "reason": reason,
        }
    )
    results_root = root / config.get("results_dir", "experiment_results")
    recovery_path = results_root / "runs" / result["attempt_id"] / "recovery.json"
    write_json(recovery_path, recovery)
    append_jsonl(results_root / "ledger.jsonl", recovery)
    return recovery


def execute_with_recovery(
    config: Dict[str, Any],
    parameters: Dict[str, Any],
    is_control: bool,
    registry: Dict[str, Any],
    root: Path,
) -> Tuple[List[Dict[str, Any]], Optional[str]]:
    """Run one configuration, recovering/retrying infrastructure failures.

    Controller outcomes are never retried here. After a collision, however, the
    world is checked because CARLA can remain synchronously locked after the
    early-exit instrumentation closes the client.
    """
    attempts: List[Dict[str, Any]] = []
    max_infra_retries = int(
        config.get("execution", {}).get("max_infra_retries", 0)
    )
    infra_retries = 0
    while True:
        result = execute_attempt(config, parameters, is_control, registry, root)
        attempts.append(result)
        outcome = result["outcome"]
        if outcome == "infra_error" and infra_retries < max_infra_retries:
            infra_retries += 1
            recovery = record_recovery(config, root, result, "infra_error")
            if not recovery["success"]:
                return attempts, "infrastructure recovery failed"
            continue
        if outcome == "collision":
            cleanup_check = preflight(config["carla"])
            attempt_dir = (
                root
                / config.get("results_dir", "experiment_results")
                / "runs"
                / result["attempt_id"]
            )
            write_json(attempt_dir / "post_collision_preflight.json", cleanup_check)
            if not cleanup_check.get("ok"):
                recovery = record_recovery(
                    config, root, result, "post_collision_cleanup"
                )
                if not recovery["success"]:
                    return attempts, "post-collision infrastructure recovery failed"
        return attempts, None


def run_plan(
    config: Dict[str, Any],
    registry: Dict[str, Any],
    root: Path,
    only_experiment_id: Optional[str] = None,
) -> List[Dict[str, Any]]:
    results: List[Dict[str, Any]] = []
    counters = {outcome: 0 for outcome in OUTCOMES}
    stop_conditions = config.get("stop_conditions", {})
    for concrete in expand_config(config):
        parameters = concrete["parameters"]
        exp_id = experiment_id(parameters)
        if only_experiment_id is not None and exp_id != only_experiment_id:
            continue
        attempt_results, stop_reason = execute_with_recovery(
            config, parameters, concrete["is_control"], registry, root
        )
        results.extend(attempt_results)
        for result in attempt_results:
            outcome = result["outcome"]
            update_counters(counters, outcome, concrete["is_control"])
            counter_stop_reason = should_stop(counters, stop_conditions)
            if counter_stop_reason:
                stop_reason = counter_stop_reason
                break

        if concrete["is_control"] and stop_conditions.get(
            "stop_on_control_failure", True
        ) and result["outcome"] != "finished":
            break
        if stop_reason:
            print(f"Stopping sweep: {stop_reason}")
            break
        if only_experiment_id is not None:
            break
    return results
