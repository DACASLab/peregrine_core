#!/usr/bin/env python3
"""Demo: swap control_manager plugin between flight cycles.

Runs three cycles (passthrough -> SE3 -> passthrough), switching the
controller while grounded between each cycle via lifecycle transitions.
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

PASSTHROUGH = "control_manager::Px4PassthroughController"
SE3 = "control_manager::Se3Controller"


class ControllerSwitchDemo(Node):

    def __init__(self) -> None:
        super().__init__("controller_switch_demo")

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

        self.change_state_client = self.create_client(ChangeState, "control_manager/change_state")
        self.set_params_client = self.create_client(SetParameters, "control_manager/set_parameters")

    def run(self) -> int:
        client = PeregrineClient(
            self, server_wait_s=self.server_wait_s, action_timeout_s=self.action_timeout_s
        )
        phases = [
            (PASSTHROUGH, "passthrough_1"),
            (SE3, "se3"),
            (PASSTHROUGH, "passthrough_2"),
        ]

        try:
            client.wait_ready(timeout_s=self.preflight_wait_s)

            for index, (controller_type, label) in enumerate(phases):
                if index > 0:
                    self.get_logger().info("=== Switching to %s ===" % controller_type)
                    self._switch_controller(controller_type)

                client.wait_ready(timeout_s=self.preflight_wait_s)
                self._run_flight_cycle(client, label)
        except (FlightError, RuntimeError) as e:
            self.get_logger().error(str(e))
            return 1

        self.get_logger().info("Controller-switch mission completed successfully.")
        return 0

    def _run_flight_cycle(self, client: PeregrineClient, label: str) -> None:
        self.get_logger().info("=== Starting flight cycle: %s ===" % label)
        client.takeoff(self.takeoff_altitude_m, self.climb_velocity_mps)
        time.sleep(1.0)

        client.execute(
            "circle",
            [self.circle_radius_m, self.circle_angular_velocity_radps, self.circle_loops],
        )
        client.execute(
            "figure8",
            [self.figure8_radius_m, self.figure8_angular_velocity_radps, self.figure8_loops],
        )

        time.sleep(1.0)
        client.land(self.landing_descent_velocity_mps)

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
    node = ControllerSwitchDemo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
