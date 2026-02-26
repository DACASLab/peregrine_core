#!/usr/bin/env python3
"""@file
@brief End-to-end safety regression runner for Example 12.

Runs a deterministic sequence against `uav_manager` + `safety_monitor`:
  1) battery critical after takeoff -> expect safety land/disarm
  2) GPS critical before takeoff -> expect takeoff rejection/abort
  3) GPS critical after takeoff -> expect safety land/disarm
  4) geofence critical after takeoff -> expect safety land/disarm

This node continuously publishes synthetic safety inputs to isolated
`safety_test/*` topics so safety behavior can be validated without races against
live telemetry topics.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from peregrine_interfaces.action import Land, Takeoff
from peregrine_interfaces.msg import GpsStatus, SafetyStatus, State, UAVState
from sensor_msgs.msg import BatteryState


@dataclass
class CaseResult:
    name: str
    success: bool
    detail: str


class SafetyRegressionDemo(Node):
    """Runs multi-case safety validation in one SITL launch."""

    def __init__(self) -> None:
        super().__init__("safety_regression_demo")

        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 25.0).value)
        self.battery_topic = str(self.declare_parameter("battery_topic", "safety_test/battery").value)
        self.gps_status_topic = str(
            self.declare_parameter("gps_status_topic", "safety_test/gps_status").value
        )
        self.estimated_state_topic = str(
            self.declare_parameter("estimated_state_topic", "safety_test/estimated_state").value
        )

        self.takeoff_altitude_m = float(self.declare_parameter("takeoff_altitude_m", 4.0).value)
        self.climb_velocity_mps = float(self.declare_parameter("climb_velocity_mps", 1.0).value)
        self.landing_descent_velocity_mps = float(
            self.declare_parameter("landing_descent_velocity_mps", 0.8).value
        )

        self.preflight_wait_s = float(self.declare_parameter("preflight_wait_s", 60.0).value)
        self.server_wait_s = float(self.declare_parameter("server_wait_s", 20.0).value)
        self.action_timeout_s = float(self.declare_parameter("action_timeout_s", 180.0).value)
        self.recovery_wait_s = float(self.declare_parameter("recovery_wait_s", 60.0).value)
        self.pre_fault_hold_s = float(self.declare_parameter("pre_fault_hold_s", 3.0).value)
        self.post_clear_settle_s = float(self.declare_parameter("post_clear_settle_s", 2.0).value)
        self.geofence_breach_x_m = float(self.declare_parameter("geofence_breach_x_m", 80.0).value)
        self.takeoff_retry_count = int(self.declare_parameter("takeoff_retry_count", 2).value)
        self.takeoff_retry_delay_s = float(self.declare_parameter("takeoff_retry_delay_s", 8.0).value)
        self.offboard_confirm_timeout_s = float(
            self.declare_parameter("offboard_confirm_timeout_s", 6.0).value
        )
        self.post_disarm_rearm_delay_s = float(
            self.declare_parameter("post_disarm_rearm_delay_s", 12.0).value
        )
        self.cases_param = str(self.declare_parameter("cases", "all").value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        self.latest_uav_state: UAVState | None = None
        self.latest_safety_status: SafetyStatus | None = None
        self.last_safety_level = SafetyStatus.LEVEL_NOMINAL
        self.prev_armed: bool | None = None
        self.last_disarm_time_s: float | None = None

        self.active_fault = "none"
        self.case_results: list[CaseResult] = []

        self.battery_pub = self.create_publisher(BatteryState, self.battery_topic, 10)
        self.gps_pub = self.create_publisher(GpsStatus, self.gps_status_topic, 10)
        self.state_pub = self.create_publisher(State, self.estimated_state_topic, 10)

        self.create_subscription(UAVState, "uav_state", self._on_uav_state, 10)
        self.create_subscription(SafetyStatus, "safety_status", self._on_safety_status, 10)

        self.takeoff_client = ActionClient(self, Takeoff, "uav_manager/takeoff")
        self.land_client = ActionClient(self, Land, "uav_manager/land")

        self.publish_timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_inputs)

        self.get_logger().info(
            "safety regression demo started: rate=%.1fHz topics=[%s, %s, %s]"
            % (
                self.publish_rate_hz,
                self.battery_topic,
                self.gps_status_topic,
                self.estimated_state_topic,
            )
        )

    def run(self) -> int:
        if not self._wait_for_preflight_ready():
            return 1
        if not self._wait_for_action_servers():
            return 1

        case_plan = self._resolve_case_plan()
        if not case_plan:
            self.get_logger().error(
                "No valid safety cases selected. Requested='%s'" % self.cases_param
            )
            return 1

        all_ok = True
        for case_name, case_fn in case_plan:
            all_ok &= self._run_case(case_name, case_fn)

        self._set_fault("none")
        self._spin_for(self.post_clear_settle_s)

        # Leave stack in a conservative state for the next run.
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
            self.get_logger().info(
                "Selected safety cases: %s" % ", ".join(name for name, _ in selected)
            )
        return selected

    def _run_case(self, name: str, fn) -> bool:
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
        self._set_fault("battery_critical")

        if not self._wait_safety_level(
            SafetyStatus.LEVEL_CRITICAL, 15.0, "battery critical trigger", "battery_pct="
        ):
            return False, "battery_fault_did_not_raise_critical"

        if not self._wait_disarmed("battery auto-land disarm", 60.0):
            return False, "battery_fault_did_not_disarm"

        self._set_fault("none")
        self._spin_for(self.post_clear_settle_s)

        if not self._wait_dependencies_ready("post-battery recovery"):
            return False, "dependencies_not_ready_after_battery_case"
        if not self._reconcile_supervisor_state():
            return False, "supervisor_not_recoverable_after_battery_case"

        return True, "auto-land observed and readiness recovered"

    def _case_gps_pre_takeoff(self) -> tuple[bool, str]:
        if not self._wait_dependencies_ready("pre-gps-preflight readiness"):
            return False, "dependencies_not_ready_before_gps_pre_case"

        self._set_fault("gps_fix_critical")

        if not self._wait_for(
            lambda: self.latest_uav_state is not None and not self.latest_uav_state.dependencies_ready,
            15.0,
            "dependencies_not_ready_under_preflight_gps_fault",
        ):
            return False, "dependencies_did_not_drop_under_gps_pre_fault"

        if not self._wait_safety_level(
            SafetyStatus.LEVEL_CRITICAL, 15.0, "gps preflight critical trigger", "fix_type="
        ):
            return False, "gps_pre_fault_did_not_raise_critical"

        if not self._send_takeoff(expect_success=False):
            return False, "takeoff_unexpectedly_succeeded_under_gps_pre_fault"

        self._set_fault("none")
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
        self._set_fault("gps_fix_critical")

        if not self._wait_safety_level(
            SafetyStatus.LEVEL_CRITICAL, 15.0, "gps post-takeoff critical trigger", "fix_type="
        ):
            return False, "gps_post_fault_did_not_raise_critical"

        if not self._wait_disarmed("gps post-takeoff auto-land disarm", 60.0):
            return False, "gps_post_fault_did_not_disarm"

        self._set_fault("none")
        self._spin_for(self.post_clear_settle_s)

        if not self._wait_dependencies_ready("post-gps-post recovery"):
            return False, "dependencies_not_ready_after_gps_post_case"
        if not self._reconcile_supervisor_state():
            return False, "supervisor_not_recoverable_after_gps_post_case"

        return True, "gps fault in-air triggered auto-land"

    def _case_geofence_post_takeoff(self) -> tuple[bool, str]:
        if not self._wait_dependencies_ready("pre-geofence readiness"):
            return False, "dependencies_not_ready_before_geofence_case"

        if not self._send_takeoff_with_retry():
            return False, "takeoff_failed_before_geofence_fault"

        self._spin_for(self.pre_fault_hold_s)
        self._set_fault("geofence_radius_critical")

        if not self._wait_safety_level(
            SafetyStatus.LEVEL_CRITICAL, 15.0, "geofence critical trigger", "horizontal_dist="
        ):
            return False, "geofence_fault_did_not_raise_critical"

        if not self._wait_disarmed("geofence auto-land disarm", 60.0):
            return False, "geofence_fault_did_not_disarm"

        self._set_fault("none")
        self._spin_for(self.post_clear_settle_s)

        if not self._wait_dependencies_ready("post-geofence recovery"):
            return False, "dependencies_not_ready_after_geofence_case"
        if not self._reconcile_supervisor_state():
            return False, "supervisor_not_recoverable_after_geofence_case"

        return True, "geofence breach triggered auto-land"

    def _publish_inputs(self) -> None:
        battery = self._build_nominal_battery()
        gps = self._build_nominal_gps()
        state = self._build_nominal_state()

        publish_battery = True
        publish_gps = True

        if self.active_fault == "battery_critical":
            battery.percentage = 0.12
        elif self.active_fault == "gps_fix_critical":
            gps.fix_type = 2
        elif self.active_fault == "gps_missing_warning":
            publish_gps = False
        elif self.active_fault == "geofence_radius_critical":
            state.pose.pose.position.x = self.geofence_breach_x_m
            state.pose.pose.position.y = 0.0

        if publish_battery:
            self.battery_pub.publish(battery)
        if publish_gps:
            self.gps_pub.publish(gps)
        self.state_pub.publish(state)

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
        msg.pose.pose.position.z = 2.0 if self._is_armed() else 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.source = "safety_regression_demo"
        msg.confidence = 1.0
        return msg

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
        self, min_level: int, timeout_s: float, label: str, reason_substring: str | None = None
    ) -> bool:
        return self._wait_for(
            lambda: self._safety_level_at_least(min_level, reason_substring),
            timeout_s,
            label,
        )

    def _safety_level_at_least(self, min_level: int, reason_substring: str | None) -> bool:
        if self.latest_safety_status is None:
            return False
        if self.latest_safety_status.level < min_level:
            return False
        if reason_substring and reason_substring not in self.latest_safety_status.reason:
            return False
        return True

    def _reconcile_supervisor_state(self) -> bool:
        if self.latest_uav_state is None:
            return False

        current_state = self.latest_uav_state.state
        if current_state in (
            UAVState.STATE_ARMED,
            UAVState.STATE_HOVERING,
            UAVState.STATE_FLYING,
            UAVState.STATE_TAKING_OFF,
            UAVState.STATE_LANDING,
        ):
            # Safety-triggered PX4 landing may disarm externally without advancing
            # uav_manager FSM to Landed/Idle. Issue a land action to reconcile.
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
                "takeoff attempt %d/%d failed, reconciling before retry"
                % (attempt, attempts)
            )
            _ = self._reconcile_supervisor_state()
            self._spin_for(self.takeoff_retry_delay_s)
        return False

    def _send_land(self, expect_success: bool) -> bool:
        goal = Land.Goal()
        goal.descent_velocity_mps = self.landing_descent_velocity_mps
        return self._send_goal(self.land_client, goal, "land", expect_success)

    def _send_goal(self, client: ActionClient, goal_msg, label: str, expect_success: bool) -> bool:
        self.get_logger().info("Sending %s goal (expect_success=%s)" % (label, str(expect_success)))
        goal_future = client.send_goal_async(goal_msg)
        if not self._wait_future(goal_future, self.action_timeout_s, "%s_goal_send" % label):
            return False

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            if expect_success:
                self.get_logger().error("%s goal rejected" % label)
                return False
            self.get_logger().info("%s goal rejected as expected" % label)
            return True

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, self.action_timeout_s, "%s_result" % label):
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

    def _wait_for(self, predicate, timeout_s: float, label: str) -> bool:
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

    def _set_fault(self, fault_name: str) -> None:
        if fault_name == self.active_fault:
            return
        self.active_fault = fault_name
        self.get_logger().warn("fault_mode=%s" % fault_name)

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
