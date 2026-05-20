#!/usr/bin/env python3
"""Demo: takeoff -> configurable step-response sequence -> land."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from peregrine_client import FlightError, PeregrineClient


class StepResponseDemo(Node):

    _VALID_STEPS = {"x+", "x-", "y+", "y-", "z+", "z-", "yaw+", "yaw-"}

    def __init__(self) -> None:
        super().__init__("step_response_demo")

        self.takeoff_altitude_m = float(self.declare_parameter("takeoff_altitude_m", 5.0).value)
        self.climb_velocity_mps = float(self.declare_parameter("climb_velocity_mps", 1.0).value)
        self.landing_descent_velocity_mps = float(
            self.declare_parameter("landing_descent_velocity_mps", 0.8).value
        )
        raw = self.declare_parameter(
            "step_sequence", ["x+", "x-", "y+", "y-", "z+", "z-", "yaw+", "yaw-"]
        ).value
        self.step_sequence = (
            [t.strip() for t in raw.split(",") if t.strip()]
            if isinstance(raw, str) else list(raw)
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

    def run(self) -> int:
        invalid = [t for t in self.step_sequence if t not in self._VALID_STEPS]
        if invalid:
            self.get_logger().error("Invalid step_sequence entries: %s" % invalid)
            return 1

        client = PeregrineClient(
            self, server_wait_s=self.server_wait_s, action_timeout_s=self.action_timeout_s
        )

        try:
            client.wait_ready(timeout_s=self.preflight_wait_s, settle_s=self.post_ready_wait_s)
            client.takeoff(self.takeoff_altitude_m, self.climb_velocity_mps)

            time.sleep(1.0)
            for index, token in enumerate(self.step_sequence, start=1):
                label = "step%d_%s" % (index, token.replace("+", "_pos").replace("-", "_neg"))
                self.get_logger().info("Executing %s" % label)
                client.execute("step_response", self._params_for_step(token))
                if self.inter_step_pause_s > 0.0:
                    time.sleep(self.inter_step_pause_s)

            client.land(self.landing_descent_velocity_mps)
        except FlightError as e:
            self.get_logger().error(str(e))
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
