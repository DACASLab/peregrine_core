#!/usr/bin/env python3
"""Run a takeoff -> step-response sequence -> land mission through uav_manager."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from peregrine_interfaces.action import ExecuteTrajectory, Land, Takeoff
from peregrine_interfaces.msg import UAVState


class StepResponseDemo(Node):
    """Runs a configurable sequence of position/yaw steps for controller tuning."""

    _VALID_STEPS = {"x+", "x-", "y+", "y-", "z+", "z-", "yaw+", "yaw-"}

    def __init__(self) -> None:
        super().__init__("step_response_demo")

        self.takeoff_altitude_m = float(self.declare_parameter("takeoff_altitude_m", 5.0).value)
        self.climb_velocity_mps = float(self.declare_parameter("climb_velocity_mps", 1.0).value)
        self.landing_descent_velocity_mps = float(self.declare_parameter("landing_descent_velocity_mps", 0.8).value)
        self.step_sequence = list(
            self.declare_parameter(
                "step_sequence", ["x+", "x-", "y+", "y-", "z+", "z-", "yaw+", "yaw-"]
            ).value
        )
        self.lateral_step_m = float(self.declare_parameter("lateral_step_m", 1.0).value)
        self.vertical_step_m = float(self.declare_parameter("vertical_step_m", 0.75).value)
        self.yaw_step_deg = float(self.declare_parameter("yaw_step_deg", 20.0).value)
        self.pre_step_hold_s = float(self.declare_parameter("pre_step_hold_s", 2.0).value)
        self.post_step_hold_s = float(self.declare_parameter("post_step_hold_s", 8.0).value)
        self.inter_step_pause_s = float(self.declare_parameter("inter_step_pause_s", 1.0).value)
        self.preflight_wait_s = float(self.declare_parameter("preflight_wait_s", 30.0).value)
        self.post_ready_wait_s = float(self.declare_parameter("post_ready_wait_s", 5.0).value)
        self.server_wait_s = float(self.declare_parameter("server_wait_s", 20.0).value)
        self.action_timeout_s = float(self.declare_parameter("action_timeout_s", 240.0).value)

        self.latest_uav_state: UAVState | None = None
        self.create_subscription(UAVState, "uav_state", self._on_uav_state, 10)

        self.takeoff_client = ActionClient(self, Takeoff, "uav_manager/takeoff")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "uav_manager/execute_trajectory")
        self.land_client = ActionClient(self, Land, "uav_manager/land")

    def run(self) -> int:
        invalid_steps = [token for token in self.step_sequence if token not in self._VALID_STEPS]
        if invalid_steps:
            self.get_logger().error(f"Invalid step_sequence entries: {invalid_steps}")
            return 1

        if not self._wait_for_preflight_ready():
            return 1
        if not self._wait_for_servers():
            return 1

        takeoff_goal = Takeoff.Goal()
        takeoff_goal.target_altitude_m = self.takeoff_altitude_m
        takeoff_goal.climb_velocity_mps = self.climb_velocity_mps
        if not self._send_goal(self.takeoff_client, takeoff_goal, "takeoff"):
            return 1

        time.sleep(1.0)
        for index, token in enumerate(self.step_sequence, start=1):
            params = self._params_for_step(token)
            label = f"step{index}_{token.replace('+', '_pos').replace('-', '_neg')}"
            if not self._execute_trajectory(params, label):
                return 1
            if self.inter_step_pause_s > 0.0:
                time.sleep(self.inter_step_pause_s)

        land_goal = Land.Goal()
        land_goal.descent_velocity_mps = self.landing_descent_velocity_mps
        if not self._send_goal(self.land_client, land_goal, "land"):
            return 1

        self.get_logger().info("Step-response demo completed successfully.")
        return 0

    def _params_for_step(self, token: str) -> list[float]:
        dx = dy = dz = dyaw = 0.0
        if token == "x+":
            dx = self.lateral_step_m
        elif token == "x-":
            dx = -self.lateral_step_m
        elif token == "y+":
            dy = self.lateral_step_m
        elif token == "y-":
            dy = -self.lateral_step_m
        elif token == "z+":
            dz = self.vertical_step_m
        elif token == "z-":
            dz = -self.vertical_step_m
        elif token == "yaw+":
            dyaw = math.radians(self.yaw_step_deg)
        elif token == "yaw-":
            dyaw = -math.radians(self.yaw_step_deg)
        return [dx, dy, dz, dyaw, self.pre_step_hold_s, self.post_step_hold_s]

    def _execute_trajectory(self, params: list[float], label: str) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory_type = "step_response"
        goal.params = params
        return self._send_goal(self.execute_client, goal, label)

    def _wait_for_servers(self) -> bool:
        clients = [
            (self.takeoff_client, "uav_manager/takeoff"),
            (self.execute_client, "uav_manager/execute_trajectory"),
            (self.land_client, "uav_manager/land"),
        ]
        for client, name in clients:
            self.get_logger().info(f"Waiting for action server {name}")
            if not client.wait_for_server(timeout_sec=self.server_wait_s):
                self.get_logger().error(f"Action server unavailable: {name}")
                return False
        return True

    def _wait_for_preflight_ready(self) -> bool:
        self.get_logger().info(f"Waiting up to {self.preflight_wait_s:.1f}s for preflight readiness")
        deadline = time.monotonic() + self.preflight_wait_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            snapshot = self.latest_uav_state
            if snapshot is not None and snapshot.dependencies_ready:
                self.get_logger().info(f"Preflight ready: {snapshot.readiness_detail}")
                if self.post_ready_wait_s > 0.0:
                    self.get_logger().info(
                        f"Waiting an extra {self.post_ready_wait_s:.1f}s for PX4 pre-arm checks to settle"
                    )
                    time.sleep(self.post_ready_wait_s)
                return True

        if self.latest_uav_state is None:
            self.get_logger().error("Preflight readiness timeout: no uav_state received")
        else:
            self.get_logger().error(
                "Preflight readiness timeout: %s" % self.latest_uav_state.readiness_detail
            )
        return False

    def _on_uav_state(self, msg: UAVState) -> None:
        self.latest_uav_state = msg

    def _send_goal(self, client: ActionClient, goal_msg, label: str) -> bool:
        self.get_logger().info(f"Sending {label} goal")

        goal_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=self.action_timeout_s)
        if not goal_future.done():
            self.get_logger().error(f"{label} goal send timeout")
            return False

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{label} goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.action_timeout_s)
        if not result_future.done():
            self.get_logger().error(f"{label} result timeout")
            return False

        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error(f"{label} result is empty")
            return False

        result = wrapped_result.result
        success = bool(getattr(result, "success", False))
        message = str(getattr(result, "message", ""))
        if success:
            self.get_logger().info(f"{label} completed: {message}")
        else:
            self.get_logger().error(f"{label} failed: {message}")
        return success


def main() -> int:
    rclpy.init()
    node = StepResponseDemo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
