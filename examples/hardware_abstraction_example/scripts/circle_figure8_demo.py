#!/usr/bin/env python3
"""@file
@brief Demo mission runner for circle and figure-eight trajectories.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from peregrine_interfaces.action import ExecuteTrajectory, Land, Takeoff
from peregrine_interfaces.msg import ManagerStatus, PX4Status, State


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

        self.latest_status: PX4Status | None = None
        self.latest_estimated_state: State | None = None
        self.latest_estimation_status: ManagerStatus | None = None
        self.latest_control_status: ManagerStatus | None = None
        self.latest_trajectory_status: ManagerStatus | None = None
        self.status_sub = self.create_subscription(PX4Status, "status", self._on_status, 10)
        self.estimated_state_sub = self.create_subscription(State, "estimated_state", self._on_estimated_state, 10)
        self.estimation_status_sub = self.create_subscription(
            ManagerStatus, "estimation_status", self._on_estimation_status, 10
        )
        self.control_status_sub = self.create_subscription(ManagerStatus, "control_status", self._on_control_status, 10)
        self.trajectory_status_sub = self.create_subscription(
            ManagerStatus, "trajectory_status", self._on_trajectory_status, 10
        )

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
        """@brief Waits for PX4 status to become ready before sending takeoff."""
        self.get_logger().info(f"Waiting up to {self.preflight_wait_s:.1f}s for preflight readiness")
        deadline = time.monotonic() + self.preflight_wait_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            status = self.latest_status
            if status is None:
                continue
            if (
                status.connected
                and not status.failsafe
                and status.nav_state != 0
                and self.latest_estimated_state is not None
                and self._manager_status_ready(self.latest_estimation_status)
                and self._manager_status_ready(self.latest_control_status)
                and self._manager_status_ready(self.latest_trajectory_status)
            ):
                self.get_logger().info(
                    f"Preflight ready: connected={status.connected}, nav_state={status.nav_state}, "
                    f"arming_state={status.arming_state}"
                )
                return True

        if self.latest_status is None:
            self.get_logger().error("Preflight readiness timeout: no status received")
        else:
            status = self.latest_status
            self.get_logger().error(
                "Preflight readiness timeout: connected=%s nav_state=%s arming_state=%s failsafe=%s "
                "estimated_state=%s estimation_status=%s control_status=%s trajectory_status=%s"
                % (
                    status.connected,
                    status.nav_state,
                    status.arming_state,
                    status.failsafe,
                    self.latest_estimated_state is not None,
                    self._manager_status_ready(self.latest_estimation_status),
                    self._manager_status_ready(self.latest_control_status),
                    self._manager_status_ready(self.latest_trajectory_status),
                )
            )
        return False

    def _on_status(self, msg: PX4Status) -> None:
        """@brief Stores latest PX4 status snapshot."""
        self.latest_status = msg

    def _on_estimated_state(self, msg: State) -> None:
        """@brief Stores latest estimated state snapshot."""
        self.latest_estimated_state = msg

    def _on_estimation_status(self, msg: ManagerStatus) -> None:
        """@brief Stores latest estimation manager status."""
        self.latest_estimation_status = msg

    def _on_control_status(self, msg: ManagerStatus) -> None:
        """@brief Stores latest control manager status."""
        self.latest_control_status = msg

    def _on_trajectory_status(self, msg: ManagerStatus) -> None:
        """@brief Stores latest trajectory manager status."""
        self.latest_trajectory_status = msg

    @staticmethod
    def _manager_status_ready(status: ManagerStatus | None) -> bool:
        """@brief Returns true when a manager status indicates active healthy output."""
        return status is not None and status.active and status.healthy

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
