#!/usr/bin/env python3
"""@file
@brief End-to-end safety regression runner for Example 12.

Runs deterministic safety checks using live PX4/Gazebo telemetry only (no
synthetic `safety_test/*` sensor injection):
  1) battery critical after takeoff -> expect safety auto-land/disarm
  2) GPS critical before takeoff -> expect takeoff blocked/rejected
  3) GPS critical after takeoff -> expect safety auto-land/disarm
  4) geofence critical after takeoff -> command real motion beyond radius
"""

from __future__ import annotations

import os
import shutil
import subprocess
import time
from dataclasses import dataclass
from typing import Callable

import rclpy
from px4_msgs.msg import VehicleCommand, VehicleCommandAck
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from peregrine_interfaces.action import GoTo, Land, Takeoff
from peregrine_interfaces.msg import SafetyStatus, UAVState


@dataclass
class CaseResult:
    name: str
    success: bool
    detail: str


class SafetyRegressionDemo(Node):
    """Runs multi-case safety validation in one SITL launch."""

    def __init__(self) -> None:
        super().__init__("safety_regression_demo")

        self.takeoff_altitude_m = float(self.declare_parameter("takeoff_altitude_m", 4.0).value)
        self.climb_velocity_mps = float(self.declare_parameter("climb_velocity_mps", 1.0).value)
        self.landing_descent_velocity_mps = float(
            self.declare_parameter("landing_descent_velocity_mps", 0.8).value
        )

        self.preflight_wait_s = float(self.declare_parameter("preflight_wait_s", 60.0).value)
        self.server_wait_s = float(self.declare_parameter("server_wait_s", 20.0).value)
        self.action_timeout_s = float(self.declare_parameter("action_timeout_s", 180.0).value)
        self.recovery_wait_s = float(self.declare_parameter("recovery_wait_s", 90.0).value)
        self.pre_fault_hold_s = float(self.declare_parameter("pre_fault_hold_s", 3.0).value)
        self.post_clear_settle_s = float(self.declare_parameter("post_clear_settle_s", 2.0).value)

        self.geofence_breach_x_m = float(self.declare_parameter("geofence_breach_x_m", 40.0).value)
        self.geofence_breach_y_m = float(self.declare_parameter("geofence_breach_y_m", 0.0).value)
        self.go_to_velocity_mps = float(self.declare_parameter("go_to_velocity_mps", 3.0).value)
        self.go_to_acceptance_radius_m = float(
            self.declare_parameter("go_to_acceptance_radius_m", 0.8).value
        )

        self.takeoff_retry_count = int(self.declare_parameter("takeoff_retry_count", 2).value)
        self.takeoff_retry_delay_s = float(self.declare_parameter("takeoff_retry_delay_s", 8.0).value)
        self.offboard_confirm_timeout_s = float(
            self.declare_parameter("offboard_confirm_timeout_s", 6.0).value
        )
        self.post_disarm_rearm_delay_s = float(
            self.declare_parameter("post_disarm_rearm_delay_s", 15.0).value
        )
        self.cases_param = str(self.declare_parameter("cases", "all").value)

        self.px4_command_topic = str(
            self.declare_parameter("px4_command_topic", "auto").value
        ).strip()
        self.px4_command_ack_topic = str(
            self.declare_parameter("px4_command_ack_topic", "auto").value
        ).strip()
        self.px4_param_tool = str(
            self.declare_parameter(
                "px4_param_tool",
                "/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-param",
            ).value
        )
        self.px4_param_timeout_s = float(self.declare_parameter("px4_param_timeout_s", 4.0).value)
        self.gps_param_name = str(self.declare_parameter("gps_param_name", "SIM_GPS_USED").value)
        self.gps_nominal_satellites = int(
            self.declare_parameter("gps_nominal_satellites", 10).value
        )
        self.gps_fault_satellites = int(self.declare_parameter("gps_fault_satellites", 2).value)

        self.command_target_system = int(self.declare_parameter("command_target_system", 1).value)
        self.command_target_component = int(
            self.declare_parameter("command_target_component", 1).value
        )
        self.command_source_system = int(self.declare_parameter("command_source_system", 1).value)
        self.command_source_component = int(
            self.declare_parameter("command_source_component", 1).value
        )

        self.latest_uav_state: UAVState | None = None
        self.latest_safety_status: SafetyStatus | None = None
        self.last_safety_level = SafetyStatus.LEVEL_NOMINAL
        self.prev_armed: bool | None = None
        self.last_disarm_time_s: float | None = None
        self.latest_vehicle_command_ack: tuple[int, int, float] | None = None

        self.case_results: list[CaseResult] = []

        self.create_subscription(UAVState, "uav_state", self._on_uav_state, 10)
        self.create_subscription(SafetyStatus, "safety_status", self._on_safety_status, 10)

        self.takeoff_client = ActionClient(self, Takeoff, "uav_manager/takeoff")
        self.land_client = ActionClient(self, Land, "uav_manager/land")
        self.go_to_client = ActionClient(self, GoTo, "uav_manager/go_to")

        self.vehicle_command_pub = None
        self.vehicle_command_ack_sub = None

        self.get_logger().info(
            "safety regression demo started: cases=%s geofence_target=(%.1f, %.1f)"
            % (self.cases_param, self.geofence_breach_x_m, self.geofence_breach_y_m)
        )

    def run(self) -> int:
        if not self._wait_for_preflight_ready():
            return 1
        if not self._wait_for_action_servers():
            return 1
        if not self._configure_px4_command_io():
            return 1

        case_plan = self._resolve_case_plan()
        if not case_plan:
            self.get_logger().error("No valid safety cases selected. Requested='%s'" % self.cases_param)
            return 1

        if any("gps_" in name for name, _ in case_plan):
            if not self._set_gps_satellites(self.gps_nominal_satellites):
                self.get_logger().error("Failed to set nominal GPS satellites before GPS cases")
                return 1
            self._spin_for(1.0)

        all_ok = True
        for case_name, case_fn in case_plan:
            all_ok &= self._run_case(case_name, case_fn)

        # Always leave PX4 in a nominal simulation configuration.
        _ = self._set_battery_failure(False)
        _ = self._set_gps_satellites(self.gps_nominal_satellites)
        self._spin_for(self.post_clear_settle_s)

        if self._is_armed():
            self.get_logger().warn("Vehicle still armed at end of regression; sending land")
            if not self._send_land(expect_success=True):
                all_ok = False

        self.get_logger().info("Safety regression summary:")
        for item in self.case_results:
            status = "PASS" if item.success else "FAIL"
            self.get_logger().info("  [%s] %s: %s" % (status, item.name, item.detail))

        return 0 if all_ok else 1

    def _resolve_case_plan(self):
        available = [
            ("battery_post_takeoff_auto_land", self._case_battery_post_takeoff),
            ("gps_pre_takeoff_gate", self._case_gps_pre_takeoff),
            ("gps_post_takeoff_auto_land", self._case_gps_post_takeoff),
            ("geofence_post_takeoff_auto_land", self._case_geofence_post_takeoff),
        ]
        available_map = {name: fn for name, fn in available}

        requested = self.cases_param.strip().lower()
        if requested in ("all", "", "*"):
            self.get_logger().info("Selected safety cases: all")
            return available

        case_names = [token.strip() for token in self.cases_param.split(",") if token.strip()]
        selected = []
        unknown = []
        for name in case_names:
            fn = available_map.get(name)
            if fn is None:
                unknown.append(name)
                continue
            selected.append((name, fn))

        if unknown:
            self.get_logger().error(
                "Unknown safety case(s): %s. Valid: %s"
                % (", ".join(unknown), ", ".join(available_map.keys()))
            )
        if selected:
            self.get_logger().info("Selected safety cases: %s" % ", ".join(name for name, _ in selected))
        return selected

    def _run_case(self, name: str, fn: Callable[[], tuple[bool, str]]) -> bool:
        self.get_logger().info("=== CASE START: %s ===" % name)
        start = time.monotonic()
        try:
            ok, detail = fn()
        except Exception as exc:  # pragma: no cover - defensive path
            ok = False
            detail = "exception=%s" % str(exc)
        elapsed = time.monotonic() - start
        self.case_results.append(CaseResult(name=name, success=ok, detail=detail))
        if ok:
            self.get_logger().info("=== CASE PASS: %s (%.1fs) ===" % (name, elapsed))
        else:
            self.get_logger().error("=== CASE FAIL: %s (%.1fs) %s ===" % (name, elapsed, detail))
        return ok

    def _case_battery_post_takeoff(self) -> tuple[bool, str]:
        if not self._wait_dependencies_ready("pre-battery readiness"):
            return False, "dependencies_not_ready_before_battery_case"

        if not self._send_takeoff_with_retry():
            return False, "takeoff_failed_before_battery_fault"

        self._spin_for(self.pre_fault_hold_s)
        if not self._set_battery_failure(True):
            return False, "battery_fault_command_failed"

        if not self._wait_safety_level(
            SafetyStatus.LEVEL_CRITICAL,
            20.0,
            "battery critical trigger",
            reason_substrings=["battery_"],
        ):
            return False, "battery_fault_did_not_raise_critical"

        if not self._wait_disarmed("battery auto-land disarm", 70.0):
            return False, "battery_fault_did_not_disarm"

        _ = self._set_battery_failure(False)
        self._spin_for(self.post_clear_settle_s)

        if not self._wait_dependencies_ready("post-battery recovery"):
            return False, "dependencies_not_ready_after_battery_case"
        if not self._reconcile_supervisor_state():
            return False, "supervisor_not_recoverable_after_battery_case"

        return True, "PX4 battery failure triggered safety auto-land and recovery"

    def _case_gps_pre_takeoff(self) -> tuple[bool, str]:
        if not self._wait_dependencies_ready("pre-gps-preflight readiness"):
            return False, "dependencies_not_ready_before_gps_pre_case"

        if not self._set_gps_satellites(self.gps_fault_satellites):
            return False, "gps_fault_param_set_failed_pre"

        if not self._wait_safety_level(
            SafetyStatus.LEVEL_CRITICAL,
            20.0,
            "gps preflight critical trigger",
            reason_substrings=["fix_type=", "satellites="],
        ):
            return False, "gps_pre_fault_did_not_raise_critical"

        if not self._wait_for(
            lambda: self.latest_uav_state is not None and not self.latest_uav_state.dependencies_ready,
            20.0,
            "dependencies_not_ready_under_preflight_gps_fault",
        ):
            return False, "dependencies_did_not_drop_under_gps_pre_fault"

        if not self._send_takeoff(expect_success=False):
            return False, "takeoff_unexpectedly_succeeded_under_gps_pre_fault"

        _ = self._set_gps_satellites(self.gps_nominal_satellites)
        self._spin_for(self.post_clear_settle_s)

        if not self._wait_dependencies_ready("post-gps-preflight recovery"):
            return False, "dependencies_not_ready_after_gps_pre_case"

        return True, "takeoff blocked under preflight GPS fault"

    def _case_gps_post_takeoff(self) -> tuple[bool, str]:
        if not self._wait_dependencies_ready("pre-gps-post readiness"):
            return False, "dependencies_not_ready_before_gps_post_case"

        if not self._send_takeoff_with_retry():
            return False, "takeoff_failed_before_gps_post_fault"

        self._spin_for(self.pre_fault_hold_s)
        if not self._set_gps_satellites(self.gps_fault_satellites):
            return False, "gps_fault_param_set_failed_post"

        if not self._wait_safety_level(
            SafetyStatus.LEVEL_CRITICAL,
            20.0,
            "gps post-takeoff critical trigger",
            reason_substrings=["fix_type=", "satellites="],
        ):
            return False, "gps_post_fault_did_not_raise_critical"

        if not self._wait_disarmed("gps post-takeoff auto-land disarm", 70.0):
            return False, "gps_post_fault_did_not_disarm"

        _ = self._set_gps_satellites(self.gps_nominal_satellites)
        self._spin_for(self.post_clear_settle_s)

        if not self._wait_dependencies_ready("post-gps-post recovery"):
            return False, "dependencies_not_ready_after_gps_post_case"
        if not self._reconcile_supervisor_state():
            return False, "supervisor_not_recoverable_after_gps_post_case"

        return True, "in-air GPS fault triggered safety auto-land"

    def _case_geofence_post_takeoff(self) -> tuple[bool, str]:
        if not self._wait_dependencies_ready("pre-geofence readiness"):
            return False, "dependencies_not_ready_before_geofence_case"

        if not self._send_takeoff_with_retry():
            return False, "takeoff_failed_before_geofence_fault"

        self._spin_for(self.pre_fault_hold_s)
        if not self._send_go_to(
            target_x_m=self.geofence_breach_x_m,
            target_y_m=self.geofence_breach_y_m,
            target_z_m=self.takeoff_altitude_m,
            expect_success=False,
            timeout_s=min(self.action_timeout_s, 90.0),
        ):
            return False, "go_to_for_geofence_failed_to_execute"

        if not self._wait_safety_level(
            SafetyStatus.LEVEL_CRITICAL,
            30.0,
            "geofence critical trigger",
            reason_substrings=["horizontal_dist="],
        ):
            return False, "geofence_fault_did_not_raise_critical"

        if not self._wait_disarmed("geofence auto-land disarm", 70.0):
            return False, "geofence_fault_did_not_disarm"

        self._spin_for(self.post_clear_settle_s)

        if not self._wait_dependencies_ready("post-geofence recovery"):
            return False, "dependencies_not_ready_after_geofence_case"
        if not self._reconcile_supervisor_state():
            return False, "supervisor_not_recoverable_after_geofence_case"

        return True, "real go_to breach triggered geofence auto-land"

    def _configure_px4_command_io(self) -> bool:
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        cmd_topic = self.px4_command_topic
        if cmd_topic == "" or cmd_topic.lower() == "auto":
            cmd_topic = self._discover_topic(
                msg_type="px4_msgs/msg/VehicleCommand",
                name_fragment="/fmu/in/vehicle_command",
                timeout_s=self.server_wait_s,
            )
            if cmd_topic is None:
                self.get_logger().error("Unable to discover PX4 VehicleCommand topic")
                return False
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, cmd_topic, qos)

        ack_topic = self.px4_command_ack_topic
        if ack_topic == "" or ack_topic.lower() == "auto":
            ack_topic = self._discover_topic(
                msg_type="px4_msgs/msg/VehicleCommandAck",
                name_fragment="/fmu/out/vehicle_command_ack",
                timeout_s=3.0,
            )
        if ack_topic:
            self.vehicle_command_ack_sub = self.create_subscription(
                VehicleCommandAck, ack_topic, self._on_vehicle_command_ack, qos
            )
            self.get_logger().info(
                "PX4 command interface: cmd_topic=%s ack_topic=%s" % (cmd_topic, ack_topic)
            )
        else:
            self.get_logger().warn(
                "PX4 command ack topic not found; continuing without explicit command ACK checks"
            )
            self.get_logger().info("PX4 command interface: cmd_topic=%s" % cmd_topic)

        return True

    def _discover_topic(self, msg_type: str, name_fragment: str, timeout_s: float) -> str | None:
        deadline = time.monotonic() + timeout_s
        best_match = None
        while time.monotonic() < deadline:
            for topic_name, type_names in self.get_topic_names_and_types():
                if msg_type not in type_names:
                    continue
                if name_fragment in topic_name:
                    return topic_name
                if best_match is None:
                    best_match = topic_name
            rclpy.spin_once(self, timeout_sec=0.1)
        return best_match

    def _set_battery_failure(self, enabled: bool) -> bool:
        failure_type = (
            VehicleCommand.FAILURE_TYPE_OFF if enabled else VehicleCommand.FAILURE_TYPE_OK
        )
        ok = self._send_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_INJECT_FAILURE,
            param1=float(VehicleCommand.FAILURE_UNIT_SYSTEM_BATTERY),
            param2=float(failure_type),
            param3=0.0,
            label="battery_failure_%s" % ("on" if enabled else "clear"),
            retries=3,
        )
        if ok:
            self.get_logger().warn(
                "battery failure injection -> %s" % ("OFF(empty)" if enabled else "OK")
            )
        return ok

    def _set_gps_satellites(self, satellites: int) -> bool:
        if satellites < 0:
            self.get_logger().error("gps satellites must be >= 0")
            return False
        ok, out = self._run_px4_param(["set", self.gps_param_name, str(satellites)])
        if not ok:
            return False
        self.get_logger().warn("%s set to %d (%s)" % (self.gps_param_name, satellites, out))
        return True

    def _run_px4_param(self, args: list[str]) -> tuple[bool, str]:
        tool = self.px4_param_tool
        if not os.path.isfile(tool):
            candidate = shutil.which("px4-param")
            if candidate:
                tool = candidate
            else:
                self.get_logger().error("px4-param tool not found: %s" % self.px4_param_tool)
                return False, "tool_missing"

        cmd = [tool] + args
        try:
            completed = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.px4_param_timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error("Timeout running: %s" % " ".join(cmd))
            return False, "timeout"
        except Exception as exc:  # pragma: no cover - defensive path
            self.get_logger().error("Failed running '%s': %s" % (" ".join(cmd), str(exc)))
            return False, "exception"

        stdout = (completed.stdout or "").strip()
        stderr = (completed.stderr or "").strip()
        if completed.returncode != 0:
            self.get_logger().error(
                "px4-param failed rc=%d cmd='%s' stderr='%s' stdout='%s'"
                % (completed.returncode, " ".join(cmd), stderr, stdout)
            )
            return False, stdout if stdout else stderr

        output = stdout if stdout else "ok"
        return True, output

    def _send_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        label: str = "vehicle_command",
        retries: int = 2,
        ack_timeout_s: float = 1.5,
    ) -> bool:
        if self.vehicle_command_pub is None:
            self.get_logger().error("VehicleCommand publisher is not initialized")
            return False

        for attempt in range(1, max(1, retries) + 1):
            msg = VehicleCommand()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.param1 = float(param1)
            msg.param2 = float(param2)
            msg.param3 = float(param3)
            msg.param4 = float(param4)
            msg.param5 = float(param5)
            msg.param6 = float(param6)
            msg.param7 = float(param7)
            msg.command = int(command)
            msg.target_system = int(self.command_target_system)
            msg.target_component = int(self.command_target_component)
            msg.source_system = int(self.command_source_system)
            msg.source_component = int(self.command_source_component)
            msg.confirmation = 0
            msg.from_external = True

            sent_at = time.monotonic()
            self.vehicle_command_pub.publish(msg)

            if self._wait_vehicle_command_ack(command, sent_at, ack_timeout_s):
                return True

            self.get_logger().warn(
                "%s ack timeout attempt %d/%d" % (label, attempt, max(1, retries))
            )
            self._spin_for(0.15)

        self.get_logger().error("%s failed: no acceptable ack" % label)
        return False

    def _wait_vehicle_command_ack(
        self, command: int, sent_at_monotonic: float, timeout_s: float
    ) -> bool:
        if self.vehicle_command_ack_sub is None:
            return True

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_vehicle_command_ack is None:
                continue

            ack_cmd, ack_result, ack_rx_time = self.latest_vehicle_command_ack
            if ack_rx_time < sent_at_monotonic:
                continue
            if ack_cmd != command:
                continue

            if ack_result in (
                VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
                VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
            ):
                return True

            self.get_logger().error(
                "VehicleCommand ack rejected cmd=%d result=%d" % (ack_cmd, ack_result)
            )
            return False

        return False

    def _wait_for_preflight_ready(self) -> bool:
        return self._wait_for(
            lambda: self.latest_uav_state is not None and self.latest_uav_state.dependencies_ready,
            self.preflight_wait_s,
            "initial_preflight_ready",
        )

    def _wait_for_action_servers(self) -> bool:
        for client, name in (
            (self.takeoff_client, "uav_manager/takeoff"),
            (self.land_client, "uav_manager/land"),
            (self.go_to_client, "uav_manager/go_to"),
        ):
            self.get_logger().info("Waiting for action server %s" % name)
            if not client.wait_for_server(timeout_sec=self.server_wait_s):
                self.get_logger().error("Action server unavailable: %s" % name)
                return False
        return True

    def _wait_dependencies_ready(self, label: str) -> bool:
        return self._wait_for(
            lambda: self.latest_uav_state is not None and self.latest_uav_state.dependencies_ready,
            self.recovery_wait_s,
            label,
        )

    def _wait_disarmed(self, label: str, timeout_s: float) -> bool:
        return self._wait_for(
            lambda: self.latest_uav_state is not None and not self.latest_uav_state.armed,
            timeout_s,
            label,
        )

    def _wait_offboard_armed(self, label: str, timeout_s: float) -> bool:
        return self._wait_for(
            lambda: self.latest_uav_state is not None
            and self.latest_uav_state.armed
            and self.latest_uav_state.offboard,
            timeout_s,
            label,
        )

    def _wait_safety_level(
        self,
        min_level: int,
        timeout_s: float,
        label: str,
        reason_substrings: list[str] | None = None,
    ) -> bool:
        return self._wait_for(
            lambda: self._safety_level_at_least(min_level, reason_substrings),
            timeout_s,
            label,
        )

    def _safety_level_at_least(self, min_level: int, reason_substrings: list[str] | None) -> bool:
        if self.latest_safety_status is None:
            return False
        if self.latest_safety_status.level < min_level:
            return False
        if reason_substrings:
            reason = self.latest_safety_status.reason
            if not any(token in reason for token in reason_substrings):
                return False
        return True

    def _reconcile_supervisor_state(self) -> bool:
        if self.latest_uav_state is None:
            return False

        current_state = self.latest_uav_state.state

        if current_state == UAVState.STATE_EMERGENCY:
            # FSM is latched in EMERGENCY. The uav_manager auto-recovery logic
            # will fire EmergencyCleared once the vehicle is disarmed, PX4
            # failsafe clears, and safety level returns to nominal/warning for
            # the configured hold period. Wait for that to complete.
            self.get_logger().warn(
                "Reconciliation: supervisor in EMERGENCY, waiting for auto-clear"
            )
            return self._wait_for(
                lambda: self.latest_uav_state is not None
                and self.latest_uav_state.state != UAVState.STATE_EMERGENCY,
                self.recovery_wait_s,
                "emergency_auto_clear",
            )

        if current_state in (
            UAVState.STATE_ARMED,
            UAVState.STATE_HOVERING,
            UAVState.STATE_FLYING,
            UAVState.STATE_TAKING_OFF,
            UAVState.STATE_LANDING,
        ):
            # Safety-triggered PX4 landing may disarm externally without fully
            # advancing the supervisor FSM. Force one explicit land command.
            self.get_logger().warn(
                "Reconciliation land requested from state=%s armed=%s"
                % (self.latest_uav_state.mode, str(self.latest_uav_state.armed).lower())
            )
            if not self._send_land(expect_success=True):
                return False

        return self._wait_for(
            lambda: self.latest_uav_state is not None
            and self.latest_uav_state.state
            in (UAVState.STATE_IDLE, UAVState.STATE_ARMED, UAVState.STATE_LANDED),
            self.recovery_wait_s,
            "supervisor_reconciled",
        )

    def _send_takeoff(self, expect_success: bool) -> bool:
        goal = Takeoff.Goal()
        goal.target_altitude_m = self.takeoff_altitude_m
        goal.climb_velocity_mps = self.climb_velocity_mps
        return self._send_goal(self.takeoff_client, goal, "takeoff", expect_success)

    def _send_takeoff_with_retry(self) -> bool:
        attempts = max(1, self.takeoff_retry_count)
        for attempt in range(1, attempts + 1):
            self._wait_post_disarm_gap()
            if self._send_takeoff(expect_success=True):
                if not self._wait_offboard_armed(
                    "offboard_confirm_after_takeoff",
                    self.offboard_confirm_timeout_s,
                ):
                    self.get_logger().error(
                        "takeoff succeeded but vehicle did not report armed+offboard"
                    )
                else:
                    return True

            if attempt == attempts:
                return False

            self.get_logger().warn(
                "takeoff attempt %d/%d failed, reconciling before retry" % (attempt, attempts)
            )
            _ = self._reconcile_supervisor_state()
            self._spin_for(self.takeoff_retry_delay_s)
        return False

    def _send_land(self, expect_success: bool) -> bool:
        goal = Land.Goal()
        goal.descent_velocity_mps = self.landing_descent_velocity_mps
        return self._send_goal(self.land_client, goal, "land", expect_success)

    def _send_go_to(
        self,
        target_x_m: float,
        target_y_m: float,
        target_z_m: float,
        expect_success: bool,
        timeout_s: float,
    ) -> bool:
        goal = GoTo.Goal()
        goal.target_position.x = float(target_x_m)
        goal.target_position.y = float(target_y_m)
        goal.target_position.z = float(target_z_m)
        goal.target_yaw = 0.0
        goal.velocity_mps = float(self.go_to_velocity_mps)
        goal.acceptance_radius_m = float(self.go_to_acceptance_radius_m)
        return self._send_goal(
            self.go_to_client,
            goal,
            "go_to",
            expect_success,
            timeout_override_s=timeout_s,
        )

    def _send_goal(
        self,
        client: ActionClient,
        goal_msg,
        label: str,
        expect_success: bool,
        timeout_override_s: float | None = None,
    ) -> bool:
        timeout_s = timeout_override_s if timeout_override_s is not None else self.action_timeout_s

        self.get_logger().info("Sending %s goal (expect_success=%s)" % (label, str(expect_success)))
        goal_future = client.send_goal_async(goal_msg)
        if not self._wait_future(goal_future, timeout_s, "%s_goal_send" % label):
            return False

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            if expect_success:
                self.get_logger().error("%s goal rejected" % label)
                return False
            self.get_logger().info("%s goal rejected as expected" % label)
            return True

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_s, "%s_result" % label):
            return False

        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("%s result is empty" % label)
            return False

        success = wrapped.status == 4 and bool(wrapped.result.success)
        detail = "status=%d success=%s msg=%s" % (
            wrapped.status,
            str(bool(wrapped.result.success)).lower(),
            wrapped.result.message,
        )

        if expect_success and not success:
            self.get_logger().error("%s expected success but failed: %s" % (label, detail))
            return False
        if (not expect_success) and success:
            self.get_logger().error("%s expected failure but succeeded: %s" % (label, detail))
            return False

        if expect_success:
            self.get_logger().info("%s succeeded: %s" % (label, detail))
        else:
            self.get_logger().info("%s failed as expected: %s" % (label, detail))
        return True

    def _wait_future(self, future, timeout_s: float, label: str) -> bool:
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            self.get_logger().error("Timeout waiting for %s" % label)
            return False
        return True

    def _wait_for(self, predicate: Callable[[], bool], timeout_s: float, label: str) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        self._log_state_snapshot("timeout_%s" % label)
        self.get_logger().error("Timeout waiting for %s" % label)
        return False

    def _spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_post_disarm_gap(self) -> None:
        if self.last_disarm_time_s is None:
            return
        ready_time = self.last_disarm_time_s + self.post_disarm_rearm_delay_s
        now = time.monotonic()
        if now >= ready_time:
            return
        wait_s = ready_time - now
        self.get_logger().info("post-disarm arm guard: waiting %.1fs before re-arm" % wait_s)
        self._spin_for(wait_s)

    def _is_armed(self) -> bool:
        return self.latest_uav_state is not None and self.latest_uav_state.armed

    def _log_state_snapshot(self, tag: str) -> None:
        u = self.latest_uav_state
        s = self.latest_safety_status
        self.get_logger().error(
            "[%s] uav_state=%s armed=%s offboard=%s deps_ready=%s detail=%s safety_level=%s safety_reason=%s"
            % (
                tag,
                "none" if u is None else u.mode,
                "none" if u is None else str(u.armed).lower(),
                "none" if u is None else str(u.offboard).lower(),
                "none" if u is None else str(u.dependencies_ready).lower(),
                "none" if u is None else u.readiness_detail,
                "none" if s is None else str(s.level),
                "none" if s is None else s.reason,
            )
        )

    def _on_uav_state(self, msg: UAVState) -> None:
        if self.prev_armed is True and not msg.armed:
            self.last_disarm_time_s = time.monotonic()
        self.prev_armed = msg.armed
        self.latest_uav_state = msg

    def _on_safety_status(self, msg: SafetyStatus) -> None:
        self.latest_safety_status = msg
        if msg.level != self.last_safety_level:
            self.last_safety_level = msg.level
            self.get_logger().warn("safety level changed -> %d reason=%s" % (msg.level, msg.reason))

    def _on_vehicle_command_ack(self, msg: VehicleCommandAck) -> None:
        self.latest_vehicle_command_ack = (int(msg.command), int(msg.result), time.monotonic())


def main() -> None:
    rclpy.init()
    node = SafetyRegressionDemo()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
