#!/usr/bin/env python3
"""Demo: takeoff -> circle and/or figure-eight trajectories -> land."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from peregrine_client import FlightError, PeregrineClient


class CircleFigure8Demo(Node):

    def __init__(self) -> None:
        super().__init__("circle_figure8_demo")

        self.mission_type = self.declare_parameter("mission_type", "circle_figure8").value
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
        self.post_ready_wait_s = float(self.declare_parameter("post_ready_wait_s", 5.0).value)
        self.server_wait_s = float(self.declare_parameter("server_wait_s", 20.0).value)
        self.action_timeout_s = float(self.declare_parameter("action_timeout_s", 240.0).value)

    def run(self) -> int:
        valid = {"circle", "figure8", "circle_figure8", "circle_land_figure8"}
        if self.mission_type not in valid:
            self.get_logger().error("Invalid mission_type='%s'. Valid: %s" % (self.mission_type, valid))
            return 1

        client = PeregrineClient(
            self, server_wait_s=self.server_wait_s, action_timeout_s=self.action_timeout_s
        )
        try:
            client.wait_ready(timeout_s=self.preflight_wait_s, settle_s=self.post_ready_wait_s)

            if self.mission_type == "circle_land_figure8":
                self._fly(client, run_circle=True, run_figure8=False)
                client.wait_ready(timeout_s=self.preflight_wait_s, settle_s=self.post_ready_wait_s)
                self._fly(client, run_circle=False, run_figure8=True)
            else:
                self._fly(
                    client,
                    run_circle=self.mission_type in {"circle", "circle_figure8"},
                    run_figure8=self.mission_type in {"figure8", "circle_figure8"},
                )
        except FlightError as e:
            self.get_logger().error(str(e))
            return 1

        self.get_logger().info("Demo mission completed successfully.")
        return 0

    def _fly(self, client: PeregrineClient, run_circle: bool, run_figure8: bool) -> None:
        client.takeoff(self.takeoff_altitude_m, self.climb_velocity_mps)
        time.sleep(1.0)

        if run_circle:
            client.execute(
                "circle",
                [self.circle_radius_m, self.circle_angular_velocity_radps, self.circle_loops],
            )

        if run_figure8:
            client.execute(
                "figure8",
                [self.figure8_radius_m, self.figure8_angular_velocity_radps, self.figure8_loops],
            )

        time.sleep(1.0)
        client.land(self.landing_descent_velocity_mps)


def main() -> int:
    rclpy.init()
    node = CircleFigure8Demo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
