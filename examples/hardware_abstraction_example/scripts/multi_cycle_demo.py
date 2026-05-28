#!/usr/bin/env python3
"""Demo: repeated takeoff/trajectory/land cycles to validate FSM recovery."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from peregrine_client import FlightError, PeregrineClient


class MultiCycleDemo(Node):

    def __init__(self) -> None:
        super().__init__("multi_cycle_demo")

        self.multi_cycle_sequence = str(
            self.declare_parameter("multi_cycle_sequence", "circle,figure8,circle,figure8").value
        )
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

    def run(self) -> int:
        segments = self._parse_sequence()
        if not segments:
            return 1

        client = PeregrineClient(
            self, server_wait_s=self.server_wait_s, action_timeout_s=self.action_timeout_s
        )

        try:
            for index, segment in enumerate(segments, start=1):
                client.wait_ready(timeout_s=self.preflight_wait_s)
                self.get_logger().info("Starting cycle %d: %s" % (index, segment))
                self._run_cycle(client, segment)
        except FlightError as e:
            self.get_logger().error(str(e))
            return 1

        self.get_logger().info("Multi-cycle mission completed successfully.")
        return 0

    def _parse_sequence(self) -> list[str]:
        allowed = {"circle", "figure8", "circle_figure8"}
        segments = [s.strip().lower() for s in self.multi_cycle_sequence.split(",") if s.strip()]
        if not segments:
            self.get_logger().error("multi_cycle_sequence is empty")
            return []
        invalid = [s for s in segments if s not in allowed]
        if invalid:
            self.get_logger().error("Invalid segments: %s" % ",".join(invalid))
            return []
        return segments

    def _run_cycle(self, client: PeregrineClient, segment: str) -> None:
        client.takeoff(self.takeoff_altitude_m, self.climb_velocity_mps)
        time.sleep(1.0)

        if segment in ("circle", "circle_figure8"):
            client.execute(
                "circle",
                [self.circle_radius_m, self.circle_angular_velocity_radps, self.circle_loops],
            )
        if segment in ("figure8", "circle_figure8"):
            client.execute(
                "figure8",
                [self.figure8_radius_m, self.figure8_angular_velocity_radps, self.figure8_loops],
            )

        time.sleep(1.0)
        client.land(self.landing_descent_velocity_mps)


def main() -> int:
    rclpy.init()
    node = MultiCycleDemo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
