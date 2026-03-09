#!/usr/bin/env python3

"""Run repeatable SE(3) circle-tracking sweeps against the Example 10 SITL path."""

from __future__ import annotations

import argparse
import copy
import json
import os
import pathlib
import select
import signal
import subprocess
import sys
import time
from typing import Any

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--spec",
        required=True,
        help="YAML file describing sweep cases.",
    )
    parser.add_argument(
        "--base-manager-yaml",
        default="/ros2_ws/src/peregrine_core/examples/hardware_abstraction_example/config/example10_se3_managers.yaml",
        help="Base manager override YAML.",
    )
    parser.add_argument(
        "--base-mission-yaml",
        default="/ros2_ws/src/peregrine_core/examples/hardware_abstraction_example/config/circle_figure8_se3_mission.yaml",
        help="Base mission YAML.",
    )
    parser.add_argument(
        "--launch-file",
        default="example10_se3_circle_figure8_demo.launch.py",
        help="Launch file in hardware_abstraction_example.",
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp/se3_circle_sweep",
        help="Directory for generated configs and metrics.",
    )
    parser.add_argument(
        "--timeout-s",
        type=int,
        default=420,
        help="Per-run launch timeout.",
    )
    parser.add_argument(
        "--ros-domain-id",
        default=os.environ.get("ROS_DOMAIN_ID", "42"),
        help="ROS_DOMAIN_ID to use for each run.",
    )
    parser.add_argument(
        "--ros-localhost-only",
        default=os.environ.get("ROS_LOCALHOST_ONLY", "1"),
        help="ROS_LOCALHOST_ONLY to use for each run.",
    )
    parser.add_argument(
        "--post-metrics-grace-s",
        type=float,
        default=20.0,
        help="Extra time to allow for landing and teardown after metrics.json first appears.",
    )
    return parser.parse_args()


def load_yaml(path: pathlib.Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def dump_yaml(path: pathlib.Path, value: Any) -> None:
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(value, stream, sort_keys=False)


def apply_override(document: Any, dotted_key: str, value: Any) -> None:
    cursor = document
    parts = dotted_key.split(".")
    for key in parts[:-1]:
        if key not in cursor or cursor[key] is None:
            cursor[key] = {}
        cursor = cursor[key]
    cursor[parts[-1]] = value


def apply_overrides(document: Any, overrides: dict[str, Any]) -> Any:
    updated = copy.deepcopy(document)
    for dotted_key, value in overrides.items():
        apply_override(updated, dotted_key, value)
    return updated


def launch_case(
    args: argparse.Namespace,
    case_name: str,
    manager_yaml: pathlib.Path,
    mission_yaml: pathlib.Path,
    metrics_json: pathlib.Path,
) -> tuple[int, str]:
    command = [
        "bash",
        "-lc",
        (
            "mkdir -p /tmp/ros_logs && "
            "export ROS_LOG_DIR=/tmp/ros_logs && "
            "source /opt/ros/humble/setup.bash && "
            "source /ros2_ws/install/setup.bash && "
            f"export ROS_DOMAIN_ID={args.ros_domain_id} ROS_LOCALHOST_ONLY={args.ros_localhost_only} && "
            f"timeout {args.timeout_s}s "
            "ros2 launch hardware_abstraction_example "
            f"{args.launch_file} "
            "mission_type:=circle "
            "start_circle_evaluator:=true "
            f"manager_overrides_file:={manager_yaml} "
            f"mission_config_file:={mission_yaml} "
            f"evaluator_output_path:={metrics_json}"
        ),
    ]

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )

    captured_lines: list[str] = []
    saw_completion = False
    deadline = time.time() + args.timeout_s
    metrics_detected_at: float | None = None

    try:
        while True:
            if process.stdout is None:
                break

            ready, _, _ = select.select([process.stdout], [], [], 0.5)
            if ready:
                line = process.stdout.readline()
                if line:
                    captured_lines.append(line)
                    if "Demo mission completed successfully." in line:
                        saw_completion = True
                        os.killpg(process.pid, signal.SIGINT)

            if metrics_detected_at is None and metrics_json.exists():
                metrics_detected_at = time.time()

            if process.poll() is not None:
                break

            now = time.time()
            if metrics_detected_at is not None and (now - metrics_detected_at) >= args.post_metrics_grace_s:
                os.killpg(process.pid, signal.SIGINT)
                time.sleep(2.0)
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                break

            if now >= deadline:
                os.killpg(process.pid, signal.SIGINT)
                time.sleep(2.0)
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                break
    finally:
        try:
            returncode = process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            returncode = process.wait(timeout=5.0)

    if saw_completion and returncode not in (0, 130, -2):
        captured_lines.append(f"[sweep] launch terminated with returncode {returncode} after success marker\n")

    return returncode, "".join(captured_lines)


def main() -> int:
    args = parse_args()
    spec_path = pathlib.Path(args.spec)
    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    spec = load_yaml(spec_path)
    cases = spec.get("cases", [])
    if not cases:
        raise SystemExit("spec has no cases")

    repeats = int(spec.get("repeats", 1))
    wait_between_runs_s = float(spec.get("wait_between_runs_s", 2.0))

    summary: list[dict[str, Any]] = []
    for case_index, case in enumerate(cases, start=1):
        case_name = case["name"]
        case_base_manager_yaml = pathlib.Path(case.get("base_manager_yaml", args.base_manager_yaml))
        case_base_mission_yaml = pathlib.Path(case.get("base_mission_yaml", args.base_mission_yaml))
        manager_overrides = case.get("manager_overrides", {})
        mission_overrides = case.get("mission_overrides", {})

        base_manager = load_yaml(case_base_manager_yaml)
        base_mission = load_yaml(case_base_mission_yaml)

        for repeat_index in range(1, repeats + 1):
            run_id = f"{case_index:02d}_{case_name}_r{repeat_index}"
            run_dir = output_dir / run_id
            run_dir.mkdir(parents=True, exist_ok=True)

            manager_yaml = run_dir / "manager.yaml"
            mission_yaml = run_dir / "mission.yaml"
            metrics_json = run_dir / "metrics.json"
            launch_stdout = run_dir / "launch.stdout.log"
            launch_stderr = run_dir / "launch.stderr.log"

            dump_yaml(manager_yaml, apply_overrides(base_manager, manager_overrides))
            dump_yaml(mission_yaml, apply_overrides(base_mission, mission_overrides))

            started_at = time.time()
            returncode, launch_output = launch_case(args, case_name, manager_yaml, mission_yaml, metrics_json)
            ended_at = time.time()

            launch_stdout.write_text(launch_output, encoding="utf-8")
            launch_stderr.write_text("", encoding="utf-8")

            metrics: dict[str, Any] = {}
            if metrics_json.exists():
                metrics = json.loads(metrics_json.read_text(encoding="utf-8"))

            summary.append(
                {
                    "run_id": run_id,
                    "case_name": case_name,
                    "repeat": repeat_index,
                    "manager_overrides": manager_overrides,
                    "mission_overrides": mission_overrides,
                    "returncode": returncode,
                    "duration_s": ended_at - started_at,
                    "metrics": metrics,
                }
            )

            rms_xy = metrics.get("rms_xy_error_m")
            status = metrics.get("status", "missing")
            print(
                f"{run_id}: returncode={returncode} status={status} "
                f"rms_xy={rms_xy if rms_xy is not None else 'n/a'}"
            )
            sys.stdout.flush()
            time.sleep(wait_between_runs_s)

    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    ranked = sorted(
        summary,
        key=lambda entry: (
            entry["metrics"].get("status") != "ok",
            entry["metrics"].get("rms_xy_error_m", float("inf")),
        ),
    )
    print("\nTop results:")
    for entry in ranked[: min(5, len(ranked))]:
        metrics = entry["metrics"]
        print(
            f"{entry['run_id']}: status={metrics.get('status', 'missing')} "
            f"rms_xy={metrics.get('rms_xy_error_m', 'n/a')} "
            f"max_xy={metrics.get('max_xy_error_m', 'n/a')}"
        )

    print(f"\nSummary written to {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
