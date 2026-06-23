"""GCS trigger for trajectory BT missions (circle, figure-8, multi-trajectory).

Usage:
  Both UAVs (default):
    python3 -m peregrine_client.missions.trigger_trajectory

  Single UAV:
    python3 -m peregrine_client.missions.trigger_trajectory --uavs uav1

  Custom tree:
    python3 -m peregrine_client.missions.trigger_trajectory --tree TakeoffHoverLand

  Pick specific UAVs:
    python3 -m peregrine_client.missions.trigger_trajectory --uavs uav1,uav3

  List available trees:
    python3 -m peregrine_client.missions.trigger_trajectory --list-trees
"""

from __future__ import annotations

import argparse
import sys
from threading import Event, Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from btcpp_ros2_interfaces.action import ExecuteTree


def _wait_future(future, timeout_sec: float):
    """Block until a rclpy future completes, driven by a separately-spinning executor.

    Returns the future result, or None on timeout. Avoids spin_until_future_complete so
    several UAVs can run concurrently without each grabbing the global executor (whose
    shared wait set races -> 'wait set index too big').
    """
    done = Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout_sec):
        return None
    return future.result()


def _spin_executor(executor: SingleThreadedExecutor) -> None:
    try:
        executor.spin()
    except Exception:  # noqa: BLE001 - executor.spin() raises on shutdown; expected.
        pass


def run_trees(uav_list: list[str], tree_name: str, timeout_s: float) -> dict[str, bool]:
    """Dispatch `tree_name` to every UAV concurrently from one spinning executor.

    All trigger nodes share a single SingleThreadedExecutor on one background thread, so there
    is exactly one wait set (no cross-thread race). Goals are sent up front, then we block on
    each UAV's result future. Scales to any number of UAVs.
    """
    nodes = {ns: Node("trajectory_trigger_%s" % ns, namespace=ns) for ns in uav_list}
    executor = SingleThreadedExecutor()
    for node in nodes.values():
        executor.add_node(node)

    spin_thread = Thread(target=_spin_executor, args=(executor,), daemon=True)
    spin_thread.start()

    ok: dict[str, bool] = {}
    result_futures: dict[str, object] = {}
    try:
        for ns in uav_list:
            node = nodes[ns]
            logger = node.get_logger()
            client = ActionClient(node, ExecuteTree, "execute_tree")
            if not client.wait_for_server(timeout_sec=30.0):
                logger.error("Action server execute_tree not available for %s" % ns)
                ok[ns] = False
                continue

            goal = ExecuteTree.Goal()
            goal.target_tree = tree_name
            logger.info("Sending ExecuteTree: %s -> %s" % (ns, tree_name))
            goal_handle = _wait_future(client.send_goal_async(goal), 10.0)
            if goal_handle is None or not goal_handle.accepted:
                logger.error("Goal rejected or send timed out for %s" % ns)
                ok[ns] = False
                continue

            logger.info("Goal accepted for %s, waiting for result" % ns)
            result_futures[ns] = goal_handle.get_result_async()

        for ns, fut in result_futures.items():
            result = _wait_future(fut, timeout_s)
            ok[ns] = result is not None and result.status == 4
    finally:
        executor.shutdown()
        for node in nodes.values():
            node.destroy_node()

    return ok


KNOWN_TREES = {
    "MultiTrajectory": "Preflight -> takeoff(3m) -> circle(6m@0.5m/s) -> figure8(6m@0.5m/s) -> land",
    "TakeoffHoverLand": "Preflight -> takeoff(3m) -> hover 10s -> land",
    "SingleUavSurveillance": "Single-agent surveillance mission",
    "MultiUavSurveillanceUav1": "Multi-agent surveillance (UAV 1 pattern)",
    "MultiUavSurveillanceUav2": "Multi-agent surveillance (UAV 2 pattern)",
    "TestFallbackLand": "Test tree with fallback land",
}


def main():
    parser = argparse.ArgumentParser(description="Trigger trajectory BT missions from GCS")
    parser.add_argument(
        "--tree",
        default="MultiTrajectory",
        help="BT tree ID to execute (default: MultiTrajectory)",
    )
    parser.add_argument(
        "--uavs",
        default="uav1,uav2",
        help="Comma-separated UAV namespaces (default: uav1,uav2)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="Per-UAV result timeout in seconds (default: 300)",
    )
    parser.add_argument(
        "--list-trees",
        action="store_true",
        help="List available BT trees and exit",
    )
    args = parser.parse_args()

    if args.list_trees:
        print("Available BT trees:")
        for name, desc in KNOWN_TREES.items():
            print("  %-30s %s" % (name, desc))
        return 0

    uav_list = [u.strip() for u in args.uavs.split(",") if u.strip()]
    if not uav_list:
        print("No UAVs specified")
        return 1

    rclpy.init()
    try:
        results = run_trees(uav_list, args.tree, args.timeout)
    finally:
        rclpy.shutdown()

    for ns in uav_list:
        ok = results.get(ns, False)
        print("%s: %s" % (ns, "OK" if ok else "FAILED"))

    return 0 if all(results.get(ns, False) for ns in uav_list) else 1


if __name__ == "__main__":
    raise SystemExit(main())
