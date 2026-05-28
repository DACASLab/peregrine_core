#!/usr/bin/env python3
"""Demo: swap controllers multiple times while airborne.

Default: takeoff (passthrough) -> switch SE3 -> circle -> switch passthrough
-> figure8 -> optionally switch SE3 -> circle -> land.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from peregrine_client import FlightError, PeregrineClient
from peregrine_interfaces.msg import UAVState

PASSTHROUGH = "control_manager::Px4PassthroughController"
SE3 = "control_manager::Se3Controller"


class ControllerSwitchInflightDemo(Node):

    def __init__(self) -> None:
        super().__init__("controller_switch_inflight_demo")

        self.takeoff_altitude_m = float(self.declare_parameter("takeoff_altitude_m", 5.0).value)
        self.climb_velocity_mps = float(self.declare_parameter("climb_velocity_mps", 1.0).value)
        self.landing_descent_velocity_mps = float(
            self.declare_parameter("landing_descent_velocity_mps", 0.8).value
        )
        self.circle_radius_m = float(self.declare_parameter("circle_radius_m", 2.0).value)
        self.circle_angular_velocity_radps = float(
            self.declare_parameter("circle_angular_velocity_radps", 0.5).value
        )
        self.circle_loops = float(self.declare_parameter("circle_loops", 1.0).value)
        self.figure8_radius_m = float(self.declare_parameter("figure8_radius_m", 2.0).value)
        self.figure8_angular_velocity_radps = float(
            self.declare_parameter("figure8_angular_velocity_radps", 0.5).value
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

        self.change_state_client = self.create_client(ChangeState, "control_manager/change_state")
        self.set_params_client = self.create_client(SetParameters, "control_manager/set_parameters")

    def run(self) -> int:
        client = PeregrineClient(
            self, server_wait_s=self.server_wait_s, action_timeout_s=self.action_timeout_s
        )

        try:
            client.wait_ready(timeout_s=self.preflight_wait_s)
            client.takeoff(self.takeoff_altitude_m, self.climb_velocity_mps)

            if self.post_takeoff_settle_s > 0.0:
                self.get_logger().info(
                    "Settling for %.1fs before in-flight switch" % self.post_takeoff_settle_s
                )
                time.sleep(self.post_takeoff_settle_s)

            self._wait_airborne_stable(client, self.post_switch_validate_s, "pre_switch")

            self._switch_and_validate(client, self.switch_controller_1, "switch1")
            client.execute(
                "circle",
                [self.circle_radius_m, self.circle_angular_velocity_radps, self.circle_loops],
            )

            self._switch_and_validate(client, self.switch_controller_2, "switch2")
            client.execute(
                "figure8",
                [self.figure8_radius_m, self.figure8_angular_velocity_radps, self.figure8_loops],
            )

            if self.enable_third_switch:
                self._switch_and_validate(client, self.switch_controller_3, "switch3")
                client.execute(
                    "circle",
                    [self.circle_radius_m, self.circle_angular_velocity_radps, self.circle_loops],
                )

            client.land(self.landing_descent_velocity_mps)
        except (FlightError, RuntimeError) as e:
            self.get_logger().error(str(e))
            return 1

        self.get_logger().info("In-flight multi-switch mission completed successfully.")
        return 0

    # ── In-flight switch + validation ────────────────────────────────────

    def _switch_and_validate(
        self, client: PeregrineClient, controller_type: str, phase: str
    ) -> None:
        self.get_logger().info("=== %s: switching to %s ===" % (phase, controller_type))
        self._switch_controller(controller_type)
        if self.require_airborne_after_switch:
            self._wait_airborne_stable(client, self.post_switch_validate_s, phase)

    def _wait_airborne_stable(
        self, client: PeregrineClient, timeout_s: float, phase: str
    ) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            s = client.uav_state
            if s is None:
                continue
            if s.failsafe:
                raise RuntimeError(
                    "[%s] failsafe during validation: %s" % (phase, self._state_summary(s))
                )
            if s.connected and s.armed and s.offboard:
                self.get_logger().info("[%s] airborne stable: %s" % (phase, self._state_summary(s)))
                return
        raise RuntimeError("[%s] airborne validation timeout" % phase)

    @staticmethod
    def _state_summary(s: UAVState) -> str:
        return (
            "state=%d armed=%s offboard=%s connected=%s failsafe=%s mode=%s"
            % (s.state, s.armed, s.offboard, s.connected, s.failsafe, s.mode)
        )

    # ── Controller switching via lifecycle transitions ────────────────────

    def _switch_controller(self, controller_type: str) -> None:
        self._change_state(Transition.TRANSITION_DEACTIVATE, "deactivate")
        self._change_state(Transition.TRANSITION_CLEANUP, "cleanup")
        self._set_controller_type(controller_type)
        self._change_state(Transition.TRANSITION_CONFIGURE, "configure")
        self._change_state(Transition.TRANSITION_ACTIVATE, "activate")
        self.get_logger().info("Controller switched to %s" % controller_type)

    def _change_state(self, transition_id: int, label: str) -> None:
        if not self.change_state_client.wait_for_service(timeout_sec=self.lifecycle_timeout_s):
            raise RuntimeError("change_state service not available")
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.lifecycle_timeout_s)
        if not future.done():
            raise RuntimeError("Lifecycle %s timed out" % label)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError("Lifecycle %s failed" % label)
        self.get_logger().info("Lifecycle %s succeeded" % label)

    def _set_controller_type(self, controller_type: str) -> None:
        if not self.set_params_client.wait_for_service(timeout_sec=self.lifecycle_timeout_s):
            raise RuntimeError("set_parameters service not available")
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
            raise RuntimeError("set_parameters timed out")
        result = future.result()
        if result is None or not result.results[0].successful:
            reason = result.results[0].reason if result else "no response"
            raise RuntimeError("set_parameters failed: %s" % reason)
        self.get_logger().info("Set controller_type = %s" % controller_type)


def main() -> int:
    rclpy.init()
    node = ControllerSwitchInflightDemo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
