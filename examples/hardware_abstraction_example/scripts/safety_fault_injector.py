#!/usr/bin/env python3
"""@file
@brief Deterministic fault injector for safety_monitor SITL validation.

Publishes synthetic overrides on manager-facing topics used by safety_monitor:
  - battery (sensor_msgs/BatteryState)
  - gps_status (peregrine_interfaces/GpsStatus)
  - estimated_state (peregrine_interfaces/State)

The injector runs in three phases:
  1) nominal pre-fault (start_delay_s)
  2) active fault (duration_s)
  3) nominal post-fault (until shutdown)

With high publish rates, this node dominates topic samples so safety checks can be
validated deterministically without modifying manager code.
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from peregrine_interfaces.msg import GpsStatus, State
from sensor_msgs.msg import BatteryState


_SCENARIOS = {
    "none",
    "gps_fix_critical",
    "gps_sats_critical",
    "gps_hdop_warning",
    "gps_vdop_warning",
    "gps_missing_warning",
    "battery_warning",
    "battery_critical",
    "battery_emergency",
    "battery_low_voltage_emergency",
    "battery_missing_warning",
    "geofence_radius_critical",
    "geofence_alt_high_critical",
    "geofence_alt_low_warning",
    "envelope_speed_critical",
    "envelope_tilt_warning",
}


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return quaternion (x, y, z, w) from roll/pitch/yaw."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class SafetyFaultInjector(Node):
    """Publishes controllable fault patterns for safety validation."""

    def __init__(self) -> None:
        super().__init__("safety_fault_injector")

        self.scenario = str(self.declare_parameter("scenario", "none").value).strip().lower()
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        self.start_delay_s = float(self.declare_parameter("start_delay_s", 15.0).value)
        self.duration_s = float(self.declare_parameter("duration_s", 10.0).value)
        self.shutdown_on_complete = bool(self.declare_parameter("shutdown_on_complete", False).value)
        self.battery_topic = str(self.declare_parameter("battery_topic", "battery").value)
        self.gps_status_topic = str(self.declare_parameter("gps_status_topic", "gps_status").value)
        self.estimated_state_topic = str(
            self.declare_parameter("estimated_state_topic", "estimated_state").value
        )

        if self.scenario not in _SCENARIOS:
            raise ValueError(
                f"Unsupported scenario '{self.scenario}'. Valid: {sorted(_SCENARIOS)}"
            )
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")
        if self.start_delay_s < 0.0:
            raise ValueError("start_delay_s must be >= 0")

        self.battery_pub = self.create_publisher(BatteryState, self.battery_topic, 10)
        self.gps_pub = self.create_publisher(GpsStatus, self.gps_status_topic, 10)
        self.state_pub = self.create_publisher(State, self.estimated_state_topic, 10)

        self._t0 = time.monotonic()
        self._phase = "init"
        self._done_logged = False

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "fault injector started: scenario=%s rate=%.1fHz delay=%.1fs duration=%.1fs shutdown=%s"
            % (
                self.scenario,
                self.publish_rate_hz,
                self.start_delay_s,
                self.duration_s,
                str(self.shutdown_on_complete).lower(),
            )
        )
        self.get_logger().info(
            "fault topics: battery=%s gps=%s state=%s"
            % (self.battery_topic, self.gps_status_topic, self.estimated_state_topic)
        )

    def _on_timer(self) -> None:
        elapsed = time.monotonic() - self._t0
        fault_active = elapsed >= self.start_delay_s and (
            self.duration_s <= 0.0 or elapsed < self.start_delay_s + self.duration_s
        )
        fault_done = self.duration_s > 0.0 and elapsed >= self.start_delay_s + self.duration_s

        phase = "fault" if fault_active else "nominal"
        if phase != self._phase:
            self._phase = phase
            self.get_logger().info(
                "phase=%s elapsed=%.1fs scenario=%s" % (phase, elapsed, self.scenario)
            )

        battery = self._build_nominal_battery()
        gps = self._build_nominal_gps()
        est = self._build_nominal_state()

        publish_battery = True
        publish_gps = True

        if fault_active:
            publish_battery, publish_gps = self._apply_fault(battery, gps, est)

        if publish_battery:
            self.battery_pub.publish(battery)
        if publish_gps:
            self.gps_pub.publish(gps)
        self.state_pub.publish(est)

        if fault_done and self.shutdown_on_complete and not self._done_logged:
            self._done_logged = True
            self.get_logger().info("fault window complete; shutting down injector")
            self.timer.cancel()
            rclpy.shutdown()

    def _build_nominal_battery(self) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = 0.80
        msg.voltage = 12.2
        msg.present = True
        return msg

    def _build_nominal_gps(self) -> GpsStatus:
        msg = GpsStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.fix_type = 3
        msg.hdop = 0.7
        msg.vdop = 1.1
        msg.eph = 0.9
        msg.epv = 1.8
        msg.satellites_used = 10
        return msg

    def _build_nominal_state(self) -> State:
        msg = State()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 2.0
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.source = "safety_fault_injector"
        msg.confidence = 1.0
        return msg

    def _apply_fault(self, battery: BatteryState, gps: GpsStatus, est: State) -> tuple[bool, bool]:
        publish_battery = True
        publish_gps = True

        if self.scenario == "none":
            return publish_battery, publish_gps

        if self.scenario == "gps_fix_critical":
            gps.fix_type = 2
        elif self.scenario == "gps_sats_critical":
            gps.satellites_used = 3
        elif self.scenario == "gps_hdop_warning":
            gps.hdop = 8.0
        elif self.scenario == "gps_vdop_warning":
            gps.vdop = 8.0
        elif self.scenario == "gps_missing_warning":
            publish_gps = False
        elif self.scenario == "battery_warning":
            battery.percentage = 0.20
        elif self.scenario == "battery_critical":
            battery.percentage = 0.12
        elif self.scenario == "battery_emergency":
            battery.percentage = 0.08
        elif self.scenario == "battery_low_voltage_emergency":
            battery.voltage = 9.5
        elif self.scenario == "battery_missing_warning":
            publish_battery = False
        elif self.scenario == "geofence_radius_critical":
            est.pose.pose.position.x = 600.0
            est.pose.pose.position.y = 0.0
        elif self.scenario == "geofence_alt_high_critical":
            est.pose.pose.position.z = 130.0
        elif self.scenario == "geofence_alt_low_warning":
            est.pose.pose.position.z = -10.0
        elif self.scenario == "envelope_speed_critical":
            est.twist.twist.linear.x = 20.0
        elif self.scenario == "envelope_tilt_warning":
            qx, qy, qz, qw = _quaternion_from_rpy(0.9, 0.0, 0.0)
            est.pose.pose.orientation.x = qx
            est.pose.pose.orientation.y = qy
            est.pose.pose.orientation.z = qz
            est.pose.pose.orientation.w = qw

        return publish_battery, publish_gps


def main() -> None:
    rclpy.init()
    node: SafetyFaultInjector | None = None
    try:
        node = SafetyFaultInjector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
