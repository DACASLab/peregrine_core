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
from threading import Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from btcpp_ros2_interfaces.action import ExecuteTree


def send_execute_tree(
    node: Node,
    action_name: str,
    tree_name: str,
    timeout_s: float = 300.0,
) -> bool:
    client = ActionClient(node, ExecuteTree, action_name)
    logger = node.get_logger()

    logger.info("Waiting for action server %s" % action_name)
    if not client.wait_for_server(timeout_sec=30.0):
        logger.error("Action server %s not available" % action_name)
        return False

    goal = ExecuteTree.Goal()
    goal.target_tree = tree_name

    logger.info("Sending ExecuteTree: %s -> %s" % (action_name, tree_name))
    goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, goal_future, timeout_sec=5.0)

    if not goal_future.done():
        logger.error("Goal send timed out")
        return False

    goal_handle = goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        logger.error("Goal rejected")
        return False

    logger.info("Goal accepted, waiting for result")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_s)

    if not result_future.done():
        logger.error("Result timed out")
        return False

    result = result_future.result()
    logger.info(
        "Tree completed: status=%s msg=%s"
        % (result.result.node_status, result.result.return_message)
    )
    return result.status == 4


def _run_uav(namespace: str, tree_name: str, timeout_s: float, results: dict[str, bool]):
    node = Node("trajectory_trigger_%s" % namespace, namespace=namespace)
    try:
        results[namespace] = send_execute_tree(
            node, "execute_tree", tree_name, timeout_s=timeout_s
        )
    finally:
        node.destroy_node()


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
        results: dict[str, bool] = {}
        threads = [
            Thread(target=_run_uav, args=(ns, args.tree, args.timeout, results))
            for ns in uav_list
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    finally:
        rclpy.shutdown()

    for ns in uav_list:
        ok = results.get(ns, False)
        print("%s: %s" % (ns, "OK" if ok else "FAILED"))

    return 0 if all(results.get(ns, False) for ns in uav_list) else 1


if __name__ == "__main__":
    raise SystemExit(main())
