#!/usr/bin/env python3
"""Compute circle-tracking metrics from odometry and trajectory setpoints."""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from peregrine_interfaces.msg import TrajectorySetpoint, UAVState
from rclpy.node import Node


class CircleTrackingEvaluator(Node):
    """Collects tracking samples during the active circle segment and writes metrics."""

    def __init__(self) -> None:
        super().__init__("circle_tracking_evaluator")

        self.output_path = Path(self.declare_parameter("output_path", "/tmp/circle_eval.json").value)
        self.min_ref_speed_mps = float(self.declare_parameter("min_ref_speed_mps", 0.2).value)
        self.max_ref_age_s = float(self.declare_parameter("max_ref_age_s", 0.2).value)
        self.shutdown_on_finish = bool(self.declare_parameter("shutdown_on_finish", True).value)

        self.latest_uav_state: UAVState | None = None
        self.latest_setpoint: TrajectorySetpoint | None = None
        self.active = False
        self.finished = False
        self.samples: list[dict[str, float]] = []

        self.create_subscription(UAVState, "uav_state", self._on_uav_state, 10)
        self.create_subscription(TrajectorySetpoint, "trajectory_setpoint", self._on_setpoint, 50)
        self.create_subscription(Odometry, "odometry", self._on_odometry, 50)

    def _on_uav_state(self, msg: UAVState) -> None:
        was_active = self.active
        self.latest_uav_state = msg
        self.active = msg.state == UAVState.STATE_FLYING
        if was_active and not self.active and self.samples and not self.finished:
            self._finalize("uav_left_flying")

    def _on_setpoint(self, msg: TrajectorySetpoint) -> None:
        self.latest_setpoint = msg

    def _on_odometry(self, msg: Odometry) -> None:
        if self.finished or not self.active or self.latest_setpoint is None:
            return

        if not self.latest_setpoint.use_position or not self.latest_setpoint.use_velocity:
            return

        ref_speed = math.hypot(self.latest_setpoint.velocity.x, self.latest_setpoint.velocity.y)
        if ref_speed < self.min_ref_speed_mps:
            return

        ref_stamp = self.latest_setpoint.header.stamp.sec + (self.latest_setpoint.header.stamp.nanosec * 1e-9)
        odom_stamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        if abs(odom_stamp - ref_stamp) > self.max_ref_age_s:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        xr = self.latest_setpoint.position.x
        yr = self.latest_setpoint.position.y

        self.samples.append(
            {
                "t": odom_stamp,
                "x": x,
                "y": y,
                "xr": xr,
                "yr": yr,
                "vxr": self.latest_setpoint.velocity.x,
                "vyr": self.latest_setpoint.velocity.y,
                "xy_error": math.hypot(x - xr, y - yr),
                "ref_speed": ref_speed,
            }
        )

    def _finalize(self, reason: str) -> None:
        self.finished = True
        metrics = self._compute_metrics(reason)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="ascii")
        self.get_logger().info(
            "Circle tracking metrics written to %s: rms_xy=%.3f m max_xy=%.3f m samples=%d"
            % (
                str(self.output_path),
                metrics["rms_xy_error_m"],
                metrics["max_xy_error_m"],
                metrics["num_samples"],
            )
        )
        if self.shutdown_on_finish:
            self.create_timer(0.1, lambda: rclpy.shutdown())

    def _compute_metrics(self, reason: str) -> dict[str, float | int | str]:
        if not self.samples:
            return {
                "status": "no_samples",
                "reason": reason,
                "num_samples": 0,
            }

        ref_cx = sum(sample["xr"] for sample in self.samples) / len(self.samples)
        ref_cy = sum(sample["yr"] for sample in self.samples) / len(self.samples)

        radial_errors = []
        signed_radial_errors = []
        tangential_errors = []
        xy_errors = []
        for sample in self.samples:
            ref_dx = sample["xr"] - ref_cx
            ref_dy = sample["yr"] - ref_cy
            ref_r = math.hypot(ref_dx, ref_dy)
            if ref_r < 1e-6:
                continue

            radial_hat_x = ref_dx / ref_r
            radial_hat_y = ref_dy / ref_r

            ref_speed = math.hypot(sample["vxr"], sample["vyr"])
            if ref_speed >= 1e-6:
                tangent_hat_x = sample["vxr"] / ref_speed
                tangent_hat_y = sample["vyr"] / ref_speed
            else:
                tangent_hat_x = -radial_hat_y
                tangent_hat_y = radial_hat_x

            err_x = sample["x"] - sample["xr"]
            err_y = sample["y"] - sample["yr"]

            signed_radial = err_x * radial_hat_x + err_y * radial_hat_y
            tangential = err_x * tangent_hat_x + err_y * tangent_hat_y

            radial_errors.append(abs(signed_radial))
            signed_radial_errors.append(signed_radial)
            tangential_errors.append(tangential)
            xy_errors.append(sample["xy_error"])

        rms_xy = math.sqrt(sum(err * err for err in xy_errors) / len(xy_errors))
        rms_radial = math.sqrt(sum(err * err for err in radial_errors) / len(radial_errors))
        rms_tangential = math.sqrt(sum(err * err for err in tangential_errors) / len(tangential_errors))
        mean_xy = sum(xy_errors) / len(xy_errors)
        mean_radial = sum(radial_errors) / len(radial_errors)
        mean_signed_radial = sum(signed_radial_errors) / len(signed_radial_errors)
        mean_tangential = sum(tangential_errors) / len(tangential_errors)
        mean_ref_speed = sum(sample["ref_speed"] for sample in self.samples) / len(self.samples)
        mean_lag_s = (-mean_tangential / mean_ref_speed) if mean_ref_speed > 1e-6 else 0.0

        return {
            "status": "ok",
            "reason": reason,
            "num_samples": len(self.samples),
            "duration_s": self.samples[-1]["t"] - self.samples[0]["t"],
            "rms_xy_error_m": rms_xy,
            "mean_xy_error_m": mean_xy,
            "max_xy_error_m": max(xy_errors),
            "rms_radial_error_m": rms_radial,
            "mean_radial_error_m": mean_radial,
            "max_radial_error_m": max(radial_errors),
            "mean_signed_radial_error_m": mean_signed_radial,
            "rms_tangential_error_m": rms_tangential,
            "mean_tangential_error_m": mean_tangential,
            "max_tangential_error_m": max(abs(err) for err in tangential_errors),
            "mean_equivalent_lag_s": mean_lag_s,
            "mean_ref_speed_mps": mean_ref_speed,
            "ref_center_x_m": ref_cx,
            "ref_center_y_m": ref_cy,
        }


def main() -> int:
    rclpy.init()
    node = CircleTrackingEvaluator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
