from __future__ import annotations

import time
from typing import Callable, Optional, TypeVar

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from peregrine_interfaces.action import ExecuteTrajectory, GoTo, Land, Takeoff
from peregrine_interfaces.msg import SafetyStatus, UAVState
from peregrine_interfaces.srv import Arm, ClearEmergency, SetMode

from peregrine_client.exceptions import (
    ActionFailed,
    ActionRejected,
    ActionTimeout,
    FlightError,
    ReadinessTimeout,
    ServiceError,
)

T = TypeVar("T")


class PeregrineClient:
    """Blocking Python client for the peregrine flight stack.

    Wraps all public ROS 2 actions and services exposed by uav_manager,
    trajectory_manager, and hardware_abstraction into a single object
    with blocking call semantics. Raises FlightError subclasses on failure.
    """

    def __init__(
        self,
        node: Node,
        *,
        server_wait_s: float = 20.0,
        action_timeout_s: float = 240.0,
    ) -> None:
        self._node = node
        self._server_wait_s = server_wait_s
        self._action_timeout_s = action_timeout_s

        self._uav_state: Optional[UAVState] = None
        self._safety_status: Optional[SafetyStatus] = None

        self._uav_state_sub = node.create_subscription(
            UAVState, "uav_state", self._on_uav_state, 10
        )
        self._safety_status_sub = node.create_subscription(
            SafetyStatus, "safety_status", self._on_safety_status, 10
        )

        self._takeoff_client = ActionClient(node, Takeoff, "uav_manager/takeoff")
        self._land_client = ActionClient(node, Land, "uav_manager/land")
        self._execute_client = ActionClient(
            node, ExecuteTrajectory, "trajectory_manager/execute_trajectory"
        )
        self._go_to_client = ActionClient(node, GoTo, "trajectory_manager/go_to")

        self._arm_client = node.create_client(Arm, "uav_manager/arm")
        self._clear_emergency_client = node.create_client(
            ClearEmergency, "uav_manager/clear_emergency"
        )
        self._set_mode_client = node.create_client(SetMode, "hardware_abstraction/set_mode")

        self._wait_for_action_servers()

    # ── State observation ────────────────────────────────────────────────

    @property
    def uav_state(self) -> Optional[UAVState]:
        return self._uav_state

    @property
    def safety_status(self) -> Optional[SafetyStatus]:
        return self._safety_status

    def wait_ready(self, timeout_s: float = 30.0, settle_s: float = 0.0) -> None:
        """Block until UAVState.dependencies_ready is True."""
        detail = ""

        def ready() -> bool:
            nonlocal detail
            s = self._uav_state
            if s is not None:
                detail = s.readiness_detail
                return bool(s.dependencies_ready)
            return False

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.2)
            if ready():
                self._log_info("Preflight ready: %s" % detail)
                if settle_s > 0.0:
                    self._log_info("Settling for %.1fs" % settle_s)
                    self._spin_for(settle_s)
                return

        raise ReadinessTimeout(timeout_s, detail)

    def wait_for(
        self,
        predicate: Callable[[], bool],
        timeout_s: float,
        label: str = "",
    ) -> None:
        """Block until predicate returns True, spinning the node."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.2)
            if predicate():
                return
        raise FlightError(
            label or "wait_for",
            "timed out after %.1fs" % timeout_s,
        )

    # ── Flight lifecycle — uav_manager actions ───────────────────────────

    def takeoff(
        self,
        altitude_m: float,
        climb_velocity_mps: float = 1.0,
        *,
        timeout_s: Optional[float] = None,
    ) -> Takeoff.Result:
        goal = Takeoff.Goal()
        goal.target_altitude_m = float(altitude_m)
        goal.climb_velocity_mps = float(climb_velocity_mps)
        return self._send_action(self._takeoff_client, goal, "takeoff", timeout_s)

    def land(
        self,
        descent_velocity_mps: float = 0.8,
        *,
        timeout_s: Optional[float] = None,
    ) -> Land.Result:
        goal = Land.Goal()
        goal.descent_velocity_mps = float(descent_velocity_mps)
        return self._send_action(self._land_client, goal, "land", timeout_s)

    # ── Trajectory execution — trajectory_manager actions ────────────────

    def execute(
        self,
        trajectory_type: str,
        params: list[float],
        *,
        timeout_s: Optional[float] = None,
    ) -> ExecuteTrajectory.Result:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory_type = trajectory_type
        goal.params = [float(p) for p in params]
        return self._send_action(self._execute_client, goal, "execute(%s)" % trajectory_type, timeout_s)

    def go_to(
        self,
        x: float,
        y: float,
        z: float,
        *,
        yaw: float = 0.0,
        velocity_mps: float = 2.0,
        timeout_s: Optional[float] = None,
    ) -> GoTo.Result:
        goal = GoTo.Goal()
        goal.target_position.x = float(x)
        goal.target_position.y = float(y)
        goal.target_position.z = float(z)
        goal.target_yaw = float(yaw)
        goal.velocity_mps = float(velocity_mps)
        return self._send_action(self._go_to_client, goal, "go_to", timeout_s)

    # ── Services ─────────────────────────────────────────────────────────

    def arm(self, arm: bool = True) -> None:
        req = Arm.Request()
        req.arm = arm
        label = "arm" if arm else "disarm"
        resp = self._call_service(self._arm_client, req, label)
        if not resp.success:
            raise ServiceError(label, resp.message)

    def clear_emergency(self) -> None:
        req = ClearEmergency.Request()
        resp = self._call_service(self._clear_emergency_client, req, "clear_emergency")
        if not resp.success:
            raise ServiceError("clear_emergency", resp.message)

    def set_mode(self, mode: str) -> None:
        req = SetMode.Request()
        req.mode = mode
        resp = self._call_service(self._set_mode_client, req, "set_mode(%s)" % mode)
        if not resp.success:
            raise ServiceError("set_mode", resp.message)

    # ── Internal ─────────────────────────────────────────────────────────

    def _send_action(self, client: ActionClient, goal, operation: str, timeout_s: Optional[float]) -> T:
        timeout = timeout_s if timeout_s is not None else self._action_timeout_s

        self._log_info("Sending %s goal" % operation)

        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, goal_future, timeout_sec=timeout)
        if not goal_future.done():
            raise ActionTimeout(operation, "goal send")

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise ActionRejected(operation)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout)
        if not result_future.done():
            raise ActionTimeout(operation, "result")

        wrapped = result_future.result()
        if wrapped is None:
            raise ActionFailed(operation, "empty result")

        result = wrapped.result
        success = bool(getattr(result, "success", False))
        message = str(getattr(result, "message", ""))

        if not success:
            raise ActionFailed(operation, message, result=result)

        self._log_info("%s completed: %s" % (operation, message))
        return result

    def _call_service(self, client, request, operation: str):
        if not client.wait_for_service(timeout_sec=self._server_wait_s):
            raise ServiceError(operation, "service not available")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._server_wait_s)
        if not future.done():
            raise ServiceError(operation, "call timed out")

        result = future.result()
        if result is None:
            raise ServiceError(operation, "empty response")
        return result

    def _wait_for_action_servers(self) -> None:
        servers = [
            (self._takeoff_client, "uav_manager/takeoff"),
            (self._land_client, "uav_manager/land"),
            (self._execute_client, "trajectory_manager/execute_trajectory"),
            (self._go_to_client, "trajectory_manager/go_to"),
        ]
        for client, name in servers:
            self._log_info("Waiting for action server %s" % name)
            if not client.wait_for_server(timeout_sec=self._server_wait_s):
                raise FlightError("init", "action server unavailable: %s" % name)

    def _spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def _log_info(self, msg: str) -> None:
        self._node.get_logger().info(msg)

    def _on_uav_state(self, msg: UAVState) -> None:
        self._uav_state = msg

    def _on_safety_status(self, msg: SafetyStatus) -> None:
        self._safety_status = msg
