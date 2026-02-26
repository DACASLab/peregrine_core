#!/usr/bin/env python3
"""@file
@brief Demo mission runner for circle and figure-eight trajectories.

This script demonstrates how to interact with the peregrine UAV manager through
ROS2 action clients. It follows the standard action client lifecycle:
  1. wait_for_server() -- discover the action server via DDS
  2. send_goal_async() -- submit a goal and get a future for acceptance
  3. get_result_async() -- wait for the terminal result (success/failure)

The mission sequence is:
  - Wait for all three action servers (takeoff, execute_trajectory, land)
  - Wait for preflight readiness via UAVState.dependencies_ready
  - Execute the configured mission type (circle, figure8, or both)
  - Each mission type runs as: takeoff -> trajectory(s) -> land

Action goals target uav_manager (not trajectory_manager directly) because
uav_manager handles the full orchestration: arm -> offboard mode -> trajectory
execution -> land mode -> disarm, with safety checks at each step.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from peregrine_interfaces.action import ExecuteTrajectory, Land, Takeoff
from peregrine_interfaces.msg import UAVState


class CircleFigure8Demo(Node):
    """@brief Runs takeoff -> circle/figure8 -> land through uav_manager actions."""

    def __init__(self) -> None:
        super().__init__("circle_figure8_demo")

        self.mission_type = self.declare_parameter("mission_type", "circle_figure8").value
        self.takeoff_altitude_m = float(self.declare_parameter("takeoff_altitude_m", 5.0).value)
        self.climb_velocity_mps = float(self.declare_parameter("climb_velocity_mps", 1.0).value)
        self.landing_descent_velocity_mps = float(self.declare_parameter("landing_descent_velocity_mps", 0.8).value)
        self.circle_radius_m = float(self.declare_parameter("circle_radius_m", 2.0).value)
        self.circle_angular_velocity_radps = float(self.declare_parameter("circle_angular_velocity_radps", 0.6).value)
        self.circle_loops = float(self.declare_parameter("circle_loops", 1.0).value)
        self.figure8_radius_m = float(self.declare_parameter("figure8_radius_m", 2.0).value)
        self.figure8_angular_velocity_radps = float(self.declare_parameter("figure8_angular_velocity_radps", 0.6).value)
        self.figure8_loops = float(self.declare_parameter("figure8_loops", 1.0).value)
        self.preflight_wait_s = float(self.declare_parameter("preflight_wait_s", 30.0).value)
        self.server_wait_s = float(self.declare_parameter("server_wait_s", 20.0).value)
        self.action_timeout_s = float(self.declare_parameter("action_timeout_s", 240.0).value)

        # Single subscription to UAVState replaces 5 individual status subscriptions.
        # UAVState.dependencies_ready aggregates PX4 + all manager readiness from the
        # C++ HealthAggregator — no need to duplicate that logic here.
        self.latest_uav_state: UAVState | None = None
        self.create_subscription(UAVState, "uav_state", self._on_uav_state, 10)

        self.takeoff_client = ActionClient(self, Takeoff, "uav_manager/takeoff")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "uav_manager/execute_trajectory")
        self.land_client = ActionClient(self, Land, "uav_manager/land")

    def run(self) -> int:
        """@brief Executes the selected mission sequence."""
        if self.mission_type not in {"circle", "figure8", "circle_figure8", "circle_land_figure8"}:
            self.get_logger().error(
                "Invalid mission_type='%s'. Valid values: circle, figure8, circle_figure8, circle_land_figure8"
                % self.mission_type
            )
            return 1

        if not self._wait_for_servers():
            return 1

        if not self._wait_for_preflight_ready():
            return 1

        if self.mission_type == "circle_land_figure8":
            if not self._run_flight_cycle(run_circle=True, run_figure8=False, cycle_label="cycle1_circle"):
                return 1
            if not self._wait_for_preflight_ready():
                return 1
            if not self._run_flight_cycle(run_circle=False, run_figure8=True, cycle_label="cycle2_figure8"):
                return 1
        else:
            if not self._run_flight_cycle(
                run_circle=self.mission_type in {"circle", "circle_figure8"},
                run_figure8=self.mission_type in {"figure8", "circle_figure8"},
                cycle_label="cycle1",
            ):
                return 1

        self.get_logger().info("Demo mission completed successfully.")
        return 0

    def _wait_for_servers(self) -> bool:
        """@brief Waits for required uav_manager action servers."""
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
        """@brief Waits for UAVState.dependencies_ready before sending goals."""
        self.get_logger().info(f"Waiting up to {self.preflight_wait_s:.1f}s for preflight readiness")
        deadline = time.monotonic() + self.preflight_wait_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            s = self.latest_uav_state
            if s is not None and s.dependencies_ready:
                self.get_logger().info(f"Preflight ready: {s.readiness_detail}")
                return True

        if self.latest_uav_state is None:
            self.get_logger().error("Preflight readiness timeout: no uav_state received")
        else:
            self.get_logger().error(
                "Preflight readiness timeout: %s" % self.latest_uav_state.readiness_detail
            )
        return False

    def _on_uav_state(self, msg: UAVState) -> None:
        """@brief Stores latest UAVState snapshot."""
        self.latest_uav_state = msg

    def _run_flight_cycle(self, run_circle: bool, run_figure8: bool, cycle_label: str) -> bool:
        """@brief Runs one takeoff/trajectory/land cycle for FSM validation."""
        self.get_logger().info(f"Starting {cycle_label}")

        takeoff_goal = Takeoff.Goal()
        takeoff_goal.target_altitude_m = self.takeoff_altitude_m
        takeoff_goal.climb_velocity_mps = self.climb_velocity_mps
        if not self._send_goal(self.takeoff_client, takeoff_goal, f"{cycle_label}_takeoff"):
            return False

        time.sleep(1.0)
        if run_circle:
            if not self._execute_trajectory(
                "circle",
                [self.circle_radius_m, self.circle_angular_velocity_radps, self.circle_loops],
                f"{cycle_label}_circle",
            ):
                return False

        if run_figure8:
            if not self._execute_trajectory(
                "figure8",
                [self.figure8_radius_m, self.figure8_angular_velocity_radps, self.figure8_loops],
                f"{cycle_label}_figure8",
            ):
                return False

        time.sleep(1.0)
        land_goal = Land.Goal()
        land_goal.descent_velocity_mps = self.landing_descent_velocity_mps
        if not self._send_goal(self.land_client, land_goal, f"{cycle_label}_land"):
            return False

        return True

    def _execute_trajectory(self, trajectory_type: str, params: list[float], label: str) -> bool:
        """@brief Sends ExecuteTrajectory goal for the requested trajectory type."""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory_type = trajectory_type
        goal.params = params
        return self._send_goal(self.execute_client, goal, label)

    def _send_goal(self, client: ActionClient, goal_msg, label: str) -> bool:
        """@brief Sends one action goal and waits for the terminal result."""
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
    """@brief Entry point for circle/figure-eight demo mission."""
    rclpy.init()
    node = CircleFigure8Demo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
