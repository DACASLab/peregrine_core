#!/usr/bin/env python3
"""@file
@brief Minimal takeoff/hold/land runner for safety test pipelines.

This script keeps the vehicle airborne for a deterministic hold window so safety
faults can be injected and observed without manual action timing.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from peregrine_interfaces.action import Land, Takeoff
from peregrine_interfaces.msg import SafetyStatus, UAVState


class SafetyTakeoffHoldDemo(Node):
    """Runs takeoff -> hold -> optional land for safety scenario testing."""

    def __init__(self) -> None:
        super().__init__("safety_takeoff_hold_demo")

        self.takeoff_altitude_m = float(self.declare_parameter("takeoff_altitude_m", 4.0).value)
        self.climb_velocity_mps = float(self.declare_parameter("climb_velocity_mps", 1.0).value)
        self.hold_time_s = float(self.declare_parameter("hold_time_s", 30.0).value)
        self.landing_descent_velocity_mps = float(
            self.declare_parameter("landing_descent_velocity_mps", 0.8).value
        )
        self.preflight_wait_s = float(self.declare_parameter("preflight_wait_s", 40.0).value)
        self.server_wait_s = float(self.declare_parameter("server_wait_s", 20.0).value)
        self.action_timeout_s = float(self.declare_parameter("action_timeout_s", 180.0).value)
        self.auto_takeoff = bool(self.declare_parameter("auto_takeoff", True).value)
        self.auto_land_after_hold = bool(self.declare_parameter("auto_land_after_hold", True).value)

        self.latest_uav_state: UAVState | None = None
        self.last_safety_level = SafetyStatus.LEVEL_NOMINAL

        self.create_subscription(UAVState, "uav_state", self._on_uav_state, 10)
        self.create_subscription(SafetyStatus, "safety_status", self._on_safety_status, 10)

        self.takeoff_client = ActionClient(self, Takeoff, "uav_manager/takeoff")
        self.land_client = ActionClient(self, Land, "uav_manager/land")

    def run(self) -> int:
        if not self._wait_for_preflight_ready():
            return 1

        if not self._wait_for_servers():
            return 1

        if self.auto_takeoff:
            takeoff_goal = Takeoff.Goal()
            takeoff_goal.target_altitude_m = self.takeoff_altitude_m
            takeoff_goal.climb_velocity_mps = self.climb_velocity_mps
            if not self._send_goal(self.takeoff_client, takeoff_goal, "takeoff"):
                return 1

        self.get_logger().info("Holding for %.1fs to allow fault injection" % self.hold_time_s)
        hold_deadline = time.monotonic() + self.hold_time_s
        next_log = time.monotonic() + 5.0
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.latest_uav_state is not None and self.latest_uav_state.state in (
                UAVState.STATE_LANDED,
                UAVState.STATE_EMERGENCY,
            ):
                self.get_logger().info(
                    "Hold ended early: state=%s detail=%s"
                    % (self.latest_uav_state.mode, self.latest_uav_state.detail)
                )
                break

            now = time.monotonic()
            if now >= next_log and self.latest_uav_state is not None:
                self.get_logger().info(
                    "Hold status: mode=%s armed=%s deps_ready=%s safety_ready=%s"
                    % (
                        self.latest_uav_state.mode,
                        str(self.latest_uav_state.armed).lower(),
                        str(self.latest_uav_state.dependencies_ready).lower(),
                        str(self.latest_uav_state.safety_ready).lower(),
                    )
                )
                next_log = now + 5.0

        if self.auto_land_after_hold and self.latest_uav_state is not None and self.latest_uav_state.armed:
            land_goal = Land.Goal()
            land_goal.descent_velocity_mps = self.landing_descent_velocity_mps
            if not self._send_goal(self.land_client, land_goal, "land"):
                return 1

        self.get_logger().info("Safety takeoff/hold demo completed")
        return 0

    def _wait_for_preflight_ready(self) -> bool:
        self.get_logger().info("Waiting up to %.1fs for preflight readiness" % self.preflight_wait_s)
        deadline = time.monotonic() + self.preflight_wait_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_uav_state is not None and self.latest_uav_state.dependencies_ready:
                self.get_logger().info("Preflight ready: %s" % self.latest_uav_state.readiness_detail)
                return True

        if self.latest_uav_state is None:
            self.get_logger().error("Preflight readiness timeout: no uav_state")
        else:
            self.get_logger().error(
                "Preflight readiness timeout: %s" % self.latest_uav_state.readiness_detail
            )
        return False

    def _wait_for_servers(self) -> bool:
        clients = [
            (self.takeoff_client, "uav_manager/takeoff"),
            (self.land_client, "uav_manager/land"),
        ]
        for client, name in clients:
            self.get_logger().info("Waiting for action server %s" % name)
            if not client.wait_for_server(timeout_sec=self.server_wait_s):
                self.get_logger().error("Action server unavailable: %s" % name)
                return False
        return True

    def _send_goal(self, client: ActionClient, goal_msg, label: str) -> bool:
        self.get_logger().info("Sending %s goal" % label)
        goal_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=self.action_timeout_s)
        if not goal_future.done():
            self.get_logger().error("%s goal send timeout" % label)
            return False

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("%s goal rejected" % label)
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.action_timeout_s)
        if not result_future.done():
            self.get_logger().error("%s result timeout" % label)
            return False

        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("%s result is empty" % label)
            return False

        status = wrapped_result.status
        result = wrapped_result.result
        if status != 4 or not bool(result.success):
            self.get_logger().error(
                "%s failed (status=%d success=%s msg=%s)"
                % (label, status, str(bool(result.success)).lower(), result.message)
            )
            return False

        self.get_logger().info("%s succeeded" % label)
        return True

    def _on_uav_state(self, msg: UAVState) -> None:
        self.latest_uav_state = msg

    def _on_safety_status(self, msg: SafetyStatus) -> None:
        if msg.level != self.last_safety_level:
            self.last_safety_level = msg.level
            self.get_logger().warn("safety level changed -> %d reason=%s" % (msg.level, msg.reason))


def main() -> None:
    rclpy.init()
    node = SafetyTakeoffHoldDemo()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
