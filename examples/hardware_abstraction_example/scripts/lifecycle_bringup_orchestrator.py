#!/usr/bin/env python3
"""@file
@brief Deterministic lifecycle bringup orchestrator for manager-chain startup.
"""

import sys
import time
from typing import Iterable, Optional

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class LifecycleBringupOrchestrator(Node):
    """@brief Configures and activates lifecycle nodes in deterministic phases."""

    def __init__(self) -> None:
        super().__init__("lifecycle_bringup_orchestrator")

        # Two-phase architecture: phase_1 configures/activates the data-processing
        # managers (estimation, control, trajectory) that have no inter-dependencies.
        # phase_2 configures/activates uav_manager, which depends on all phase_1
        # managers being active and healthy before it can complete on_configure.
        self.declare_parameter(
            "phase_1_nodes", ["estimation_manager", "control_manager", "trajectory_manager"]
        )
        self.declare_parameter("phase_2_nodes", ["uav_manager"])
        self.declare_parameter("service_wait_timeout_s", 10.0)
        self.declare_parameter("state_wait_timeout_s", 15.0)
        self.declare_parameter("poll_period_s", 0.2)

        self.phase_1_nodes = list(self.get_parameter("phase_1_nodes").value)
        self.phase_2_nodes = list(self.get_parameter("phase_2_nodes").value)
        self.service_wait_timeout_s = float(self.get_parameter("service_wait_timeout_s").value)
        self.state_wait_timeout_s = float(self.get_parameter("state_wait_timeout_s").value)
        self.poll_period_s = float(self.get_parameter("poll_period_s").value)

    def run(self) -> bool:
        """@brief Executes bringup phases and returns True on success."""
        # The orchestrator exists as a separate node because ROS2 lifecycle
        # transitions must be initiated externally -- a lifecycle node cannot
        # trigger its own configure/activate. An alternative to this node is
        # invoking `ros2 lifecycle set <node> configure` via CLI commands.
        self.get_logger().info("Starting deterministic lifecycle bringup")

        if not self._activate_phase(self.phase_1_nodes, phase_name="phase_1_managers"):
            return False

        if not self._activate_phase(self.phase_2_nodes, phase_name="phase_2_supervisor"):
            return False

        self.get_logger().info("Lifecycle bringup completed successfully")
        return True

    def _activate_phase(self, nodes: Iterable[str], phase_name: str) -> bool:
        """@brief Configures then activates each node in the provided phase."""
        # ROS2 lifecycle requires configure (allocate resources, subscribe to
        # topics) before activate (start processing). We wait for each state
        # transition to complete before moving to the next node, ensuring
        # deterministic ordering within the phase.
        for node_name in nodes:
            self.get_logger().info(f"[{phase_name}] configuring '{node_name}'")
            if not self._change_state(node_name, Transition.TRANSITION_CONFIGURE):
                self.get_logger().error(f"[{phase_name}] configure failed for '{node_name}'")
                return False
            if not self._wait_for_state(node_name, State.PRIMARY_STATE_INACTIVE):
                self.get_logger().error(f"[{phase_name}] '{node_name}' did not reach INACTIVE")
                return False

            self.get_logger().info(f"[{phase_name}] activating '{node_name}'")
            if not self._change_state(node_name, Transition.TRANSITION_ACTIVATE):
                self.get_logger().error(f"[{phase_name}] activate failed for '{node_name}'")
                return False
            if not self._wait_for_state(node_name, State.PRIMARY_STATE_ACTIVE):
                self.get_logger().error(f"[{phase_name}] '{node_name}' did not reach ACTIVE")
                return False

        return True

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
