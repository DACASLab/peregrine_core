#!/usr/bin/env python3
"""Evaluate SE(3) step responses using rise time, overshoot, settling, and coupling metrics."""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from peregrine_interfaces.msg import TrajectorySetpoint, UAVState
from rclpy.node import Node


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(msg) -> float:
    siny_cosp = 2.0 * ((msg.w * msg.z) + (msg.x * msg.y))
    cosy_cosp = 1.0 - (2.0 * ((msg.y * msg.y) + (msg.z * msg.z)))
    return math.atan2(siny_cosp, cosy_cosp)


class StepResponseEvaluator(Node):
    """Detects step changes in trajectory_setpoint and computes response metrics."""

    def __init__(self) -> None:
        super().__init__("step_response_evaluator")

        self.output_path = Path(self.declare_parameter("output_path", "/tmp/step_response_eval.json").value)
        self.step_sequence = list(
            self.declare_parameter(
                "step_sequence", ["x+", "x-", "y+", "y-", "z+", "z-", "yaw+", "yaw-"]
            ).value
        )
        self.lateral_step_m = float(self.declare_parameter("lateral_step_m", 1.0).value)
        self.vertical_step_m = float(self.declare_parameter("vertical_step_m", 0.75).value)
        self.yaw_step_deg = float(self.declare_parameter("yaw_step_deg", 20.0).value)
        self.position_step_threshold_m = float(self.declare_parameter("position_step_threshold_m", 0.4).value)
        self.yaw_step_threshold_deg = float(self.declare_parameter("yaw_step_threshold_deg", 8.0).value)
        self.rise_low_ratio = float(self.declare_parameter("rise_low_ratio", 0.1).value)
        self.rise_high_ratio = float(self.declare_parameter("rise_high_ratio", 0.9).value)
        self.settle_band_ratio = float(self.declare_parameter("settle_band_ratio", 0.05).value)
        self.final_window_s = float(self.declare_parameter("final_window_s", 1.0).value)
        self.shutdown_on_finish = bool(self.declare_parameter("shutdown_on_finish", True).value)

        self.latest_odom = None
        self.latest_uav_state: UAVState | None = None
        self.last_reference = None
        self.active_step = None
        self.completed_steps: list[dict] = []
        self.detected_step_count = 0
        self.finished = False
        self.mission_started = False

        self.create_subscription(UAVState, "uav_state", self._on_uav_state, 10)
        self.create_subscription(TrajectorySetpoint, "trajectory_setpoint", self._on_setpoint, 50)
        self.create_subscription(Odometry, "odometry", self._on_odometry, 100)

    def _on_uav_state(self, msg: UAVState) -> None:
        was_flying = self.latest_uav_state is not None and self.latest_uav_state.state == UAVState.STATE_FLYING
        self.latest_uav_state = msg
        is_flying = msg.state == UAVState.STATE_FLYING
        if is_flying and not was_flying:
            # Start each execute_trajectory action with a fresh reference so takeoff/hover
            # setpoint changes do not get mistaken for a tuning step.
            self.last_reference = None

        if msg.armed or msg.offboard or msg.state in (
            UAVState.STATE_TAKING_OFF,
            UAVState.STATE_HOVERING,
            UAVState.STATE_FLYING,
            UAVState.STATE_LANDING,
        ):
            self.mission_started = True

        if was_flying and not is_flying and not self.finished:
            self._finalize_active_step("uav_left_flying")

        if self.finished:
            return

        if self.mission_started and msg.state in (UAVState.STATE_LANDED, UAVState.STATE_IDLE):
            self._finalize_active_step("mission_complete")
            self._write_output("mission_complete")
        elif self.mission_started and msg.state == UAVState.STATE_EMERGENCY:
            self._finalize_active_step("emergency")
            self._write_output("emergency")

    def _on_setpoint(self, msg: TrajectorySetpoint) -> None:
        if self.latest_uav_state is None or self.latest_uav_state.state != UAVState.STATE_FLYING:
            return

        if not msg.use_position or not msg.use_yaw:
            return

        reference = {
            "t": msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9),
            "x": msg.position.x,
            "y": msg.position.y,
            "z": msg.position.z,
            "yaw": msg.yaw,
        }

        if self.last_reference is None:
            self.last_reference = reference
            return

        dx = reference["x"] - self.last_reference["x"]
        dy = reference["y"] - self.last_reference["y"]
        dz = reference["z"] - self.last_reference["z"]
        dyaw = wrap_angle(reference["yaw"] - self.last_reference["yaw"])
        lateral_mag = math.hypot(dx, dy)
        vertical_mag = abs(dz)
        yaw_mag = abs(dyaw)
        yaw_threshold = math.radians(self.yaw_step_threshold_deg)

        lateral_detected = lateral_mag >= max(self.position_step_threshold_m, 0.5 * self.lateral_step_m)
        vertical_detected = vertical_mag >= max(self.position_step_threshold_m, 0.5 * self.vertical_step_m)
        yaw_detected = yaw_mag >= max(yaw_threshold, 0.5 * math.radians(self.yaw_step_deg))

        if lateral_detected or vertical_detected or yaw_detected:
            self._finalize_active_step("next_step_detected")
            self._start_step(reference, self.last_reference, dx, dy, dz, dyaw)

        self.last_reference = reference

    def _start_step(self, reference, previous_reference, dx: float, dy: float, dz: float, dyaw: float) -> None:
        if self.latest_odom is None:
            return

        kind = "unknown"
        lateral_mag = math.hypot(dx, dy)
        lateral_threshold = max(self.position_step_threshold_m, 0.5 * self.lateral_step_m)
        vertical_threshold = max(self.position_step_threshold_m, 0.5 * self.vertical_step_m)
        yaw_threshold = max(math.radians(self.yaw_step_threshold_deg), 0.5 * math.radians(self.yaw_step_deg))
        if lateral_mag >= lateral_threshold and abs(dz) < vertical_threshold:
            kind = "lateral"
        elif abs(dz) >= vertical_threshold and lateral_mag < lateral_threshold:
            kind = "vertical"
        elif abs(dyaw) >= yaw_threshold:
            kind = "yaw"

        self.detected_step_count += 1
        label = (
            self.step_sequence[self.detected_step_count - 1]
            if (self.detected_step_count - 1) < len(self.step_sequence)
            else f"step_{self.detected_step_count}"
        )
        self.active_step = {
            "label": label,
            "kind": kind,
            "reference_start": previous_reference,
            "reference_end": reference,
            "step_dx": dx,
            "step_dy": dy,
            "step_dz": dz,
            "step_dyaw": dyaw,
            "step_time_s": reference["t"],
            "start_actual": {
                "x": self.latest_odom.pose.pose.position.x,
                "y": self.latest_odom.pose.pose.position.y,
                "z": self.latest_odom.pose.pose.position.z,
                "yaw": yaw_from_quaternion(self.latest_odom.pose.pose.orientation),
            },
            "samples": [],
        }

    def _on_odometry(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if self.active_step is None or self.finished:
            return

        timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        if timestamp < self.active_step["step_time_s"]:
            return

        self.active_step["samples"].append(
            {
                "t": timestamp,
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
            }
        )

    def _finalize_active_step(self, reason: str) -> None:
        if self.active_step is None:
            return
        metrics = self._compute_step_metrics(self.active_step, reason)
        self.completed_steps.append(metrics)
        self.active_step = None

    def _compute_step_metrics(self, step: dict, reason: str) -> dict:
        samples = step["samples"]
        if len(samples) < 2:
            return {
                "label": step["label"],
                "kind": step["kind"],
                "status": "insufficient_samples",
                "reason": reason,
            }

        start = step["start_actual"]
        kind = step["kind"]
        target = 0.0
        along = []
        cross = []
        vertical_cross = []
        yaw_cross = []

        if kind == "lateral":
            step_mag = math.hypot(step["step_dx"], step["step_dy"])
            ux = step["step_dx"] / max(step_mag, 1e-6)
            uy = step["step_dy"] / max(step_mag, 1e-6)
            vx = -uy
            vy = ux
            target = step_mag
            for sample in samples:
                dx = sample["x"] - start["x"]
                dy = sample["y"] - start["y"]
                along.append((sample["t"], (dx * ux) + (dy * uy)))
                cross.append(abs((dx * vx) + (dy * vy)))
                vertical_cross.append(abs(sample["z"] - start["z"]))
                yaw_cross.append(abs(wrap_angle(sample["yaw"] - start["yaw"])))
        elif kind == "vertical":
            sign = 1.0 if step["step_dz"] >= 0.0 else -1.0
            target = abs(step["step_dz"])
            for sample in samples:
                along.append((sample["t"], sign * (sample["z"] - start["z"])))
                cross.append(math.hypot(sample["x"] - start["x"], sample["y"] - start["y"]))
                yaw_cross.append(abs(wrap_angle(sample["yaw"] - start["yaw"])))
        elif kind == "yaw":
            sign = 1.0 if step["step_dyaw"] >= 0.0 else -1.0
            target = abs(step["step_dyaw"])
            for sample in samples:
                along.append((sample["t"], sign * wrap_angle(sample["yaw"] - start["yaw"])))
                cross.append(math.hypot(sample["x"] - start["x"], sample["y"] - start["y"]))
                vertical_cross.append(abs(sample["z"] - start["z"]))
        else:
            return {
                "label": step["label"],
                "kind": kind,
                "status": "ignored_unknown_step",
                "reason": reason,
            }

        values = [value for _, value in along]
        times = [timestamp - step["step_time_s"] for timestamp, _ in along]
        settle_band = max(self.settle_band_ratio * max(target, 1e-6), 0.02)
        low_threshold = self.rise_low_ratio * target
        high_threshold = self.rise_high_ratio * target

        rise_low_time = next((t for t, value in zip(times, values) if value >= low_threshold), None)
        rise_high_time = next((t for t, value in zip(times, values) if value >= high_threshold), None)
        rise_time = None
        if rise_low_time is not None and rise_high_time is not None:
            rise_time = rise_high_time - rise_low_time

        max_value = max(values)
        overshoot_pct = max(0.0, ((max_value - target) / max(target, 1e-6)) * 100.0)

        settling_time = None
        for idx, (t, value) in enumerate(zip(times, values)):
            if all(abs(target - later_value) <= settle_band for later_value in values[idx:]):
                settling_time = t
                break

        final_window_start = times[-1] - self.final_window_s
        final_values = [value for t, value in zip(times, values) if t >= final_window_start]
        final_mean = sum(final_values) / len(final_values)
        steady_state_error = target - final_mean

        result = {
            "label": step["label"],
            "kind": kind,
            "status": "ok",
            "reason": reason,
            "step_time_s": step["step_time_s"],
            "step_target": target,
            "rise_time_s": rise_time,
            "overshoot_pct": overshoot_pct,
            "settling_time_s": settling_time,
            "settled": settling_time is not None,
            "steady_state_error": steady_state_error,
            "final_mean_response": final_mean,
            "peak_along_response": max_value,
            "peak_cross_axis_error": max(cross) if cross else 0.0,
            "duration_s": times[-1],
        }

        if vertical_cross:
            result["peak_vertical_coupling_m"] = max(vertical_cross)
        if yaw_cross:
            result["peak_yaw_coupling_rad"] = max(yaw_cross)

        return result

    def _write_output(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True

        summary = {
            "status": "ok",
            "reason": reason,
            "num_steps_detected": len(self.completed_steps),
            "steps": self.completed_steps,
            "summary": self._summaries_by_kind(),
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="ascii")
        self.get_logger().info(
            "Step-response metrics written to %s: %d steps"
            % (str(self.output_path), len(self.completed_steps))
        )
        if self.shutdown_on_finish:
            self.create_timer(0.1, lambda: rclpy.shutdown())

    def _summaries_by_kind(self) -> dict:
        result = {}
        for kind in ("lateral", "vertical", "yaw"):
            metrics = [step for step in self.completed_steps if step.get("kind") == kind and step.get("status") == "ok"]
            if not metrics:
                continue
            def average(key: str):
                values = [step[key] for step in metrics if step.get(key) is not None]
                return (sum(values) / len(values)) if values else None

            result[kind] = {
                "num_steps": len(metrics),
                "avg_rise_time_s": average("rise_time_s"),
                "avg_settling_time_s": average("settling_time_s"),
                "avg_overshoot_pct": average("overshoot_pct"),
                "avg_steady_state_error": average("steady_state_error"),
                "avg_peak_cross_axis_error": average("peak_cross_axis_error"),
            }
        return result


def main() -> int:
    rclpy.init()
    node = StepResponseEvaluator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
