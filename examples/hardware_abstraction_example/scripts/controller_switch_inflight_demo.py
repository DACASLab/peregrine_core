#!/usr/bin/env python3
"""@file
@brief In-flight multi-switch demo: swap controllers multiple times while airborne.

Default sequence:
  1. Take off with passthrough.
  2. Switch #1 to SE3, validate airborne continuity, fly circle.
  3. Switch #2 back to passthrough, validate airborne continuity, fly figure-8.
  4. Switch #3 to SE3 (optional), validate airborne continuity, fly circle.
  5. Land.

This isolates whether controller switching itself is stable in-flight across repeated
transitions, independent of takeoff-under-SE3 behavior.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from peregrine_interfaces.action import ExecuteTrajectory, Land, Takeoff
from peregrine_interfaces.msg import UAVState

PASSTHROUGH = "control_manager::Px4PassthroughController"
SE3 = "control_manager::Se3Controller"


class ControllerSwitchInflightDemo(Node):
    """@brief Performs repeated in-flight controller switches and trajectory checks."""

    def __init__(self) -> None:
        super().__init__("controller_switch_inflight_demo")

        self.takeoff_altitude_m = float(self.declare_parameter("takeoff_altitude_m", 5.0).value)
        self.climb_velocity_mps = float(self.declare_parameter("climb_velocity_mps", 1.0).value)
        self.landing_descent_velocity_mps = float(
            self.declare_parameter("landing_descent_velocity_mps", 0.8).value
        )
        self.circle_radius_m = float(self.declare_parameter("circle_radius_m", 2.0).value)
        self.circle_angular_velocity_radps = float(
            self.declare_parameter("circle_angular_velocity_radps", 0.6).value
        )
        self.circle_loops = float(self.declare_parameter("circle_loops", 1.0).value)
        self.figure8_radius_m = float(self.declare_parameter("figure8_radius_m", 2.0).value)
        self.figure8_angular_velocity_radps = float(
            self.declare_parameter("figure8_angular_velocity_radps", 0.6).value
        )
        self.figure8_loops = float(self.declare_parameter("figure8_loops", 1.0).value)
        self.preflight_wait_s = float(self.declare_parameter("preflight_wait_s", 30.0).value)
        self.server_wait_s = float(self.declare_parameter("server_wait_s", 20.0).value)
        self.action_timeout_s = float(self.declare_parameter("action_timeout_s", 240.0).value)
        self.lifecycle_timeout_s = float(self.declare_parameter("lifecycle_timeout_s", 10.0).value)
        self.post_takeoff_settle_s = float(self.declare_parameter("post_takeoff_settle_s", 2.0).value)
        self.post_switch_validate_s = float(
            self.declare_parameter("post_switch_validate_s", 8.0).value
        )
        self.switch_controller_1 = str(self.declare_parameter("switch_controller_1", SE3).value)
        self.switch_controller_2 = str(
            self.declare_parameter("switch_controller_2", PASSTHROUGH).value
        )
        self.enable_third_switch = bool(self.declare_parameter("enable_third_switch", True).value)
        self.switch_controller_3 = str(self.declare_parameter("switch_controller_3", SE3).value)
        self.require_airborne_after_switch = bool(
            self.declare_parameter("require_airborne_after_switch", True).value
        )

        self.latest_uav_state: UAVState | None = None
        self.create_subscription(UAVState, "uav_state", self._on_uav_state, 10)

        self.takeoff_client = ActionClient(self, Takeoff, "uav_manager/takeoff")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "uav_manager/execute_trajectory")
        self.land_client = ActionClient(self, Land, "uav_manager/land")

        self.change_state_client = self.create_client(ChangeState, "control_manager/change_state")
        self.set_params_client = self.create_client(SetParameters, "control_manager/set_parameters")

    def run(self) -> int:
        """@brief Runs takeoff, repeated in-flight switches, trajectories, then land."""
        if not self._wait_for_preflight_ready():
            return 1

        if not self._wait_for_servers():
            return 1

        takeoff_goal = Takeoff.Goal()
        takeoff_goal.target_altitude_m = self.takeoff_altitude_m
        takeoff_goal.climb_velocity_mps = self.climb_velocity_mps
        if not self._send_goal(self.takeoff_client, takeoff_goal, "takeoff_passthrough"):
            return 1

        if self.post_takeoff_settle_s > 0.0:
            self.get_logger().info("Settling for %.1fs before in-flight switch" % self.post_takeoff_settle_s)
            time.sleep(self.post_takeoff_settle_s)

        if not self._wait_for_airborne_stable(5.0, "pre_switch"):
            return 1

        if not self._switch_and_validate(self.switch_controller_1, "switch1"):
            return 1

        if not self._execute_trajectory(
            "circle",
            [self.circle_radius_m, self.circle_angular_velocity_radps, self.circle_loops],
            "after_switch1_circle",
        ):
            return 1

        if not self._switch_and_validate(self.switch_controller_2, "switch2"):
            return 1

        if not self._execute_trajectory(
            "figure8",
            [self.figure8_radius_m, self.figure8_angular_velocity_radps, self.figure8_loops],
            "after_switch2_figure8",
        ):
            return 1

        if self.enable_third_switch:
            if not self._switch_and_validate(self.switch_controller_3, "switch3"):
                return 1

            if not self._execute_trajectory(
                "circle",
                [self.circle_radius_m, self.circle_angular_velocity_radps, self.circle_loops],
                "after_switch3_circle",
            ):
                return 1

        land_goal = Land.Goal()
        land_goal.descent_velocity_mps = self.landing_descent_velocity_mps
        if not self._send_goal(self.land_client, land_goal, "land_after_switch"):
            return 1

        self.get_logger().info("In-flight multi-switch mission completed successfully.")
        return 0

    def _switch_and_validate(self, controller_type: str, phase: str) -> bool:
        """@brief Performs one in-flight switch and validates post-switch airborne status."""
        self.get_logger().info("=== %s: switching controller to %s ===" % (phase, controller_type))
        if not self._switch_controller(controller_type):
            return False

        if self.require_airborne_after_switch and not self._wait_for_airborne_stable(
            self.post_switch_validate_s, phase
        ):
            return False

        return True

    def _switch_controller(self, controller_type: str) -> bool:
        """@brief Drives control_manager lifecycle for controller swap."""
        transitions = [
            (Transition.TRANSITION_DEACTIVATE, "deactivate"),
            (Transition.TRANSITION_CLEANUP, "cleanup"),
        ]
        for transition_id, name in transitions:
            if not self._change_state(transition_id, name):
                return False

        if not self._set_controller_type(controller_type):
            return False

        transitions = [
            (Transition.TRANSITION_CONFIGURE, "configure"),
            (Transition.TRANSITION_ACTIVATE, "activate"),
        ]
        for transition_id, name in transitions:
            if not self._change_state(transition_id, name):
                return False

        self.get_logger().info("Controller switched to %s" % controller_type)
        return True

    def _change_state(self, transition_id: int, label: str) -> bool:
        """@brief Sends a lifecycle ChangeState request to control_manager."""
        if not self.change_state_client.wait_for_service(timeout_sec=self.lifecycle_timeout_s):
            self.get_logger().error("change_state service not available")
            return False

        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.lifecycle_timeout_s)

        if not future.done():
            self.get_logger().error("Lifecycle %s timed out" % label)
            return False

        result = future.result()
        if result is None or not result.success:
            self.get_logger().error("Lifecycle %s failed" % label)
            return False

        self.get_logger().info("Lifecycle %s succeeded" % label)
        return True

    def _set_controller_type(self, controller_type: str) -> bool:
        """@brief Sets the controller_type parameter on control_manager."""
        if not self.set_params_client.wait_for_service(timeout_sec=self.lifecycle_timeout_s):
            self.get_logger().error("set_parameters service not available")
            return False

        param = Parameter()
        param.name = "controller_type"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = controller_type

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.set_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.lifecycle_timeout_s)
        if not future.done():
            self.get_logger().error("set_parameters timed out")
            return False

        result = future.result()
        if result is None or not result.results[0].successful:
            reason = result.results[0].reason if result else "no response"
            self.get_logger().error("set_parameters failed: %s" % reason)
            return False

        self.get_logger().info("Set controller_type = %s" % controller_type)
        return True

    def _wait_for_servers(self) -> bool:
        """@brief Waits for required uav_manager action servers."""
        clients = [
            (self.takeoff_client, "uav_manager/takeoff"),
            (self.execute_client, "uav_manager/execute_trajectory"),
            (self.land_client, "uav_manager/land"),
        ]
        for client, name in clients:
            self.get_logger().info("Waiting for action server %s" % name)
            if not client.wait_for_server(timeout_sec=self.server_wait_s):
                self.get_logger().error("Action server unavailable: %s" % name)
                return False
        return True

    def _wait_for_preflight_ready(self) -> bool:
        """@brief Waits for UAVState.dependencies_ready before mission start."""
        self.get_logger().info("Waiting up to %.1fs for preflight readiness" % self.preflight_wait_s)
        deadline = time.monotonic() + self.preflight_wait_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            s = self.latest_uav_state
            if s is not None and s.dependencies_ready:
                self.get_logger().info("Preflight ready: %s" % s.readiness_detail)
                return True

        if self.latest_uav_state is None:
            self.get_logger().error("Preflight readiness timeout: no uav_state received")
        else:
            self.get_logger().error(
                "Preflight readiness timeout: %s" % self.latest_uav_state.readiness_detail
            )
        return False

    def _wait_for_airborne_stable(self, timeout_s: float, phase: str) -> bool:
        """@brief Verifies armed+offboard+connected and no failsafe within timeout."""
        deadline = time.monotonic() + timeout_s
        seen_state = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            s = self.latest_uav_state
            if s is None:
                continue
            seen_state = True
            if s.failsafe:
                self.get_logger().error(
                    "[%s] failsafe observed during switch validation: %s" % (phase, self._state_summary(s))
                )
                return False
            if s.connected and s.armed and s.offboard:
                self.get_logger().info("[%s] airborne stable: %s" % (phase, self._state_summary(s)))
                return True

        if not seen_state:
            self.get_logger().error("[%s] no uav_state received during airborne validation" % phase)
        else:
            self.get_logger().error(
                "[%s] airborne validation timeout, last state: %s"
                % (phase, self._state_summary(self.latest_uav_state))
            )
        return False

    def _execute_trajectory(self, trajectory_type: str, params: list[float], label: str) -> bool:
        """@brief Sends ExecuteTrajectory goal for the requested trajectory type."""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory_type = trajectory_type
        goal.params = params
        return self._send_goal(self.execute_client, goal, label)

    def _send_goal(self, client: ActionClient, goal_msg, label: str) -> bool:
        """@brief Sends one action goal and waits for terminal result."""
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

        result = wrapped_result.result
        success = bool(getattr(result, "success", False))
        message = str(getattr(result, "message", ""))
        if success:
            self.get_logger().info("%s completed: %s" % (label, message))
        else:
            self.get_logger().error("%s failed: %s" % (label, message))
        return success

    def _state_summary(self, s: UAVState) -> str:
        """@brief Compact UAV state summary for diagnostics."""
        return (
            "state=%d armed=%s offboard=%s connected=%s failsafe=%s mode=%s detail=%s"
            % (s.state, s.armed, s.offboard, s.connected, s.failsafe, s.mode, s.detail)
        )

    def _on_uav_state(self, msg: UAVState) -> None:
        """@brief Stores latest UAVState snapshot."""
        self.latest_uav_state = msg


def main() -> int:
    """@brief Entry point for in-flight controller-switch demo mission."""
    rclpy.init()
    node = ControllerSwitchInflightDemo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
