#!/usr/bin/env python3
"""@file
@brief Deterministic lifecycle bringup orchestrator for manager-chain startup.
"""

import sys
import time
from typing import Iterable, Optional

import rclpy
from lifecycle_msgs.msg import State as LifecycleState, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from peregrine_interfaces.msg import ManagerStatus, PX4Status, State
from rclpy.node import Node


class LifecycleBringupOrchestrator(Node):
    """@brief Configures and activates lifecycle nodes in deterministic phases."""

    def __init__(self) -> None:
        super().__init__("lifecycle_bringup_orchestrator")

        # Two-phase architecture: phase_1 configures/activates the data-processing
        # managers (estimation, control, trajectory). phase_2 configures/activates
        # uav_manager only after the data plane is confirmed ready, so first-goal
        # behavior is deterministic.
        self.declare_parameter(
            "phase_1_nodes", ["estimation_manager", "control_manager", "trajectory_manager"]
        )
        self.declare_parameter("phase_2_nodes", ["uav_manager"])
        self.declare_parameter("service_wait_timeout_s", 10.0)
        self.declare_parameter("state_wait_timeout_s", 15.0)
        self.declare_parameter("poll_period_s", 0.2)
        self.declare_parameter("wait_for_data_readiness", True)
        self.declare_parameter("data_readiness_timeout_s", 30.0)
        self.declare_parameter("data_freshness_timeout_s", 1.5)
        self.declare_parameter("phase_2_retry_timeout_s", 20.0)
        self.declare_parameter("phase_2_retry_backoff_s", 0.5)

        self.phase_1_nodes = list(self.get_parameter("phase_1_nodes").value)
        self.phase_2_nodes = list(self.get_parameter("phase_2_nodes").value)
        self.service_wait_timeout_s = float(self.get_parameter("service_wait_timeout_s").value)
        self.state_wait_timeout_s = float(self.get_parameter("state_wait_timeout_s").value)
        self.poll_period_s = float(self.get_parameter("poll_period_s").value)
        self.wait_for_data_readiness = bool(self.get_parameter("wait_for_data_readiness").value)
        self.data_readiness_timeout_s = float(self.get_parameter("data_readiness_timeout_s").value)
        self.data_freshness_timeout_s = float(self.get_parameter("data_freshness_timeout_s").value)
        self.phase_2_retry_timeout_s = float(self.get_parameter("phase_2_retry_timeout_s").value)
        self.phase_2_retry_backoff_s = float(self.get_parameter("phase_2_retry_backoff_s").value)

        self.latest_status: PX4Status | None = None
        self.latest_estimated_state: State | None = None
        self.latest_estimation_status: ManagerStatus | None = None
        self.latest_control_status: ManagerStatus | None = None
        self.latest_trajectory_status: ManagerStatus | None = None

        self.status_rx_s = 0.0
        self.estimated_state_rx_s = 0.0
        self.estimation_status_rx_s = 0.0
        self.control_status_rx_s = 0.0
        self.trajectory_status_rx_s = 0.0

        self.create_subscription(PX4Status, "status", self._on_status, 10)
        self.create_subscription(State, "estimated_state", self._on_estimated_state, 10)
        self.create_subscription(ManagerStatus, "estimation_status", self._on_estimation_status, 10)
        self.create_subscription(ManagerStatus, "control_status", self._on_control_status, 10)
        self.create_subscription(ManagerStatus, "trajectory_status", self._on_trajectory_status, 10)

    def run(self) -> bool:
        """@brief Executes bringup phases and returns True on success."""
        # The orchestrator exists as a separate node because ROS2 lifecycle
        # transitions must be initiated externally -- a lifecycle node cannot
        # trigger its own configure/activate. An alternative to this node is
        # invoking `ros2 lifecycle set <node> configure` via CLI commands.
        self.get_logger().info("Starting deterministic lifecycle bringup")

        if not self._activate_phase(self.phase_1_nodes, phase_name="phase_1_managers"):
            return False

        if self.wait_for_data_readiness and not self._wait_for_data_readiness():
            return False

        if not self._activate_phase_with_retry(self.phase_2_nodes, phase_name="phase_2_supervisor"):
            return False

        self.get_logger().info("Lifecycle bringup completed successfully")
        return True

    def _activate_phase(self, nodes: Iterable[str], phase_name: str) -> bool:
        """@brief Configures then activates each node in the provided phase."""
        for node_name in nodes:
            # State-aware idempotence keeps retries deterministic: if a node is
            # already ACTIVE from a prior attempt, this phase still succeeds.
            current_state = self._get_state(node_name)
            if current_state is None:
                return False

            if current_state == LifecycleState.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(f"[{phase_name}] '{node_name}' already ACTIVE")
                continue

            # Configure only from UNCONFIGURED. A retry may see INACTIVE if the
            # previous configure succeeded but activate failed.
            if current_state == LifecycleState.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info(f"[{phase_name}] configuring '{node_name}'")
                if not self._change_state(node_name, Transition.TRANSITION_CONFIGURE):
                    self.get_logger().error(f"[{phase_name}] configure failed for '{node_name}'")
                    return False
                if not self._wait_for_state(node_name, LifecycleState.PRIMARY_STATE_INACTIVE):
                    self.get_logger().error(f"[{phase_name}] '{node_name}' did not reach INACTIVE")
                    return False
                current_state = LifecycleState.PRIMARY_STATE_INACTIVE

            if current_state != LifecycleState.PRIMARY_STATE_INACTIVE:
                self.get_logger().error(
                    f"[{phase_name}] '{node_name}' in unsupported lifecycle state id={current_state}"
                )
                return False

            self.get_logger().info(f"[{phase_name}] activating '{node_name}'")
            if not self._change_state(node_name, Transition.TRANSITION_ACTIVATE):
                self.get_logger().error(f"[{phase_name}] activate failed for '{node_name}'")
                return False
            if not self._wait_for_state(node_name, LifecycleState.PRIMARY_STATE_ACTIVE):
                self.get_logger().error(f"[{phase_name}] '{node_name}' did not reach ACTIVE")
                return False

        return True

    def _activate_phase_with_retry(self, nodes: Iterable[str], phase_name: str) -> bool:
        """@brief Retries phase activation with bounded timeout/backoff."""
        if self.phase_2_retry_timeout_s <= 0.0:
            return self._activate_phase(nodes, phase_name)

        deadline = time.monotonic() + self.phase_2_retry_timeout_s
        attempt = 1
        while rclpy.ok():
            if self._activate_phase(nodes, phase_name):
                return True

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.get_logger().error(
                    f"[{phase_name}] failed after retry budget {self.phase_2_retry_timeout_s:.1f}s"
                )
                return False

            sleep_s = min(max(self.phase_2_retry_backoff_s, 0.1), remaining)
            self.get_logger().warn(
                f"[{phase_name}] activation attempt {attempt} failed; retrying in {sleep_s:.2f}s"
            )
            time.sleep(sleep_s)
            attempt += 1

        return False

    def _change_state(self, node_name: str, transition_id: int) -> bool:
        """@brief Calls `<node>/change_state` and verifies transition acceptance."""
        # Lifecycle transitions are exposed as ROS2 services on each managed
        # node. Creating a new client per call is acceptable here because the
        # orchestrator only runs at startup (not performance-critical).
        service_name = f"{node_name}/change_state"
        client = self.create_client(ChangeState, service_name)

        if not client.wait_for_service(timeout_sec=self.service_wait_timeout_s):
            self.get_logger().error(f"Service unavailable: {service_name}")
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id
        # call_async + spin_until_future_complete is the rclpy pattern for
        # making a synchronous service call from within a node. call_async
        # dispatches the request, and spin_until_future_complete processes
        # callbacks (including the service response) until the future resolves.
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_wait_timeout_s)

        if not future.done() or future.result() is None:
            self.get_logger().error(f"No response from {service_name}")
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(f"Transition {transition_id} rejected by {node_name}")
            return False

        return True

    def _get_state(self, node_name: str) -> int | None:
        """@brief Retrieves current lifecycle state id from `<node>/get_state`."""
        service_name = f"{node_name}/get_state"
        client = self.create_client(GetState, service_name)
        if not client.wait_for_service(timeout_sec=self.service_wait_timeout_s):
            self.get_logger().error(f"Service unavailable: {service_name}")
            return None

        request = GetState.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_wait_timeout_s)
        if not future.done() or future.result() is None:
            self.get_logger().error(f"No response from {service_name}")
            return None

        return int(future.result().current_state.id)

    def _wait_for_state(self, node_name: str, expected_state: int) -> bool:
        """@brief Polls `<node>/get_state` until expected primary state is reached."""
        # After requesting a state change, the node may take time to complete
        # the transition (e.g. on_configure blocks while waiting for upstream
        # topics to appear). We poll get_state at a fixed cadence until the
        # expected state is reached or the timeout expires.
        service_name = f"{node_name}/get_state"
        client = self.create_client(GetState, service_name)

        if not client.wait_for_service(timeout_sec=self.service_wait_timeout_s):
            self.get_logger().error(f"Service unavailable: {service_name}")
            return False

        deadline = time.monotonic() + self.state_wait_timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            # Poll at fixed cadence to keep startup timing deterministic.
            request = GetState.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.poll_period_s)

            if future.done() and future.result() is not None:
                current = future.result().current_state.id
                if current == expected_state:
                    return True

            time.sleep(self.poll_period_s)

        return False

    def _wait_for_data_readiness(self) -> bool:
        """@brief Waits for fresh, healthy data-plane dependencies before phase-2 activation."""
        self.get_logger().info(
            f"Waiting up to {self.data_readiness_timeout_s:.1f}s for data readiness before phase_2"
        )
        deadline = time.monotonic() + self.data_readiness_timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self.poll_period_s)
            ready, detail = self._data_ready(time.monotonic())
            if ready:
                self.get_logger().info(f"Data readiness satisfied: {detail}")
                return True

        ready, detail = self._data_ready(time.monotonic())
        if not ready:
            self.get_logger().error(f"Data readiness timeout: {detail}")
        return ready

    def _is_fresh(self, received_at_s: float, now_s: float) -> bool:
        """@brief Returns True if sample exists and age is within freshness timeout."""
        return received_at_s > 0.0 and (now_s - received_at_s) <= self.data_freshness_timeout_s

    @staticmethod
    def _manager_ready(status: ManagerStatus | None) -> tuple[bool, str]:
        """@brief Evaluates active/healthy flags from ManagerStatus."""
        if status is None:
            return False, "MISSING"
        if not status.active:
            return False, "INACTIVE"
        if not status.healthy:
            return False, "UNHEALTHY"
        return True, "OK"

    def _data_ready(self, now_s: float) -> tuple[bool, str]:
        """@brief Evaluates PX4 + manager + estimator readiness with reason tokens."""
        if self.latest_status is None:
            status_reason = "MISSING"
            status_ok = False
        elif not self._is_fresh(self.status_rx_s, now_s):
            status_reason = "STALE"
            status_ok = False
        elif not self.latest_status.connected:
            status_reason = "DISCONNECTED"
            status_ok = False
        elif self.latest_status.failsafe:
            status_reason = "FAILSAFE"
            status_ok = False
        elif self.latest_status.nav_state == 0:
            status_reason = "NAV_UNKNOWN"
            status_ok = False
        else:
            status_reason = "OK"
            status_ok = True

        if self.latest_estimated_state is None:
            estimated_reason = "MISSING"
            estimated_ok = False
        elif not self._is_fresh(self.estimated_state_rx_s, now_s):
            estimated_reason = "STALE"
            estimated_ok = False
        else:
            estimated_reason = "OK"
            estimated_ok = True

        estimation_flags_ok, estimation_flags_reason = self._manager_ready(self.latest_estimation_status)
        control_flags_ok, control_flags_reason = self._manager_ready(self.latest_control_status)
        trajectory_flags_ok, trajectory_flags_reason = self._manager_ready(self.latest_trajectory_status)

        estimation_ok = estimation_flags_ok and self._is_fresh(self.estimation_status_rx_s, now_s)
        control_ok = control_flags_ok and self._is_fresh(self.control_status_rx_s, now_s)
        trajectory_ok = trajectory_flags_ok and self._is_fresh(self.trajectory_status_rx_s, now_s)

        estimation_reason = estimation_flags_reason if estimation_flags_reason != "OK" else (
            "OK" if estimation_ok else "STALE"
        )
        control_reason = control_flags_reason if control_flags_reason != "OK" else (
            "OK" if control_ok else "STALE"
        )
        trajectory_reason = trajectory_flags_reason if trajectory_flags_reason != "OK" else (
            "OK" if trajectory_ok else "STALE"
        )

        ready = status_ok and estimated_ok and estimation_ok and control_ok and trajectory_ok
        detail = (
            f"status={status_reason} "
            f"estimated_state={estimated_reason} "
            f"estimation={estimation_reason} "
            f"control={control_reason} "
            f"trajectory={trajectory_reason}"
        )
        return ready, detail

    def _on_status(self, msg: PX4Status) -> None:
        """@brief Caches latest PX4 status with receive timestamp."""
        self.latest_status = msg
        self.status_rx_s = time.monotonic()

    def _on_estimated_state(self, msg: State) -> None:
        """@brief Caches latest estimated state with receive timestamp."""
        self.latest_estimated_state = msg
        self.estimated_state_rx_s = time.monotonic()

    def _on_estimation_status(self, msg: ManagerStatus) -> None:
        """@brief Caches latest estimation manager status."""
        self.latest_estimation_status = msg
        self.estimation_status_rx_s = time.monotonic()

    def _on_control_status(self, msg: ManagerStatus) -> None:
        """@brief Caches latest control manager status."""
        self.latest_control_status = msg
        self.control_status_rx_s = time.monotonic()

    def _on_trajectory_status(self, msg: ManagerStatus) -> None:
        """@brief Caches latest trajectory manager status."""
        self.latest_trajectory_status = msg
        self.trajectory_status_rx_s = time.monotonic()


def main(argv: Optional[list[str]] = None) -> int:
    """@brief Entry point for lifecycle bringup orchestration utility."""
    rclpy.init(args=argv)
    node = LifecycleBringupOrchestrator()

    success = False
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
