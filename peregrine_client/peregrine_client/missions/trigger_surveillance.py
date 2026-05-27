"""Minimal GCS trigger for surveillance BT missions.

This script ONLY sends ExecuteTree action goals — all flight logic lives
in the BT trees running on each UAV.

Usage:
  Single-agent:
    python3 -m peregrine_client.missions.trigger_surveillance --mode single

  Multi-agent (2 UAVs):
    python3 -m peregrine_client.missions.trigger_surveillance --mode multi
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
    rclpy.spin_until_future_complete(node, goal_future, timeout_sec=30.0)

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
    return True


def run_single():
    node = Node("surveillance_trigger_single")
    try:
        send_execute_tree(node, "execute_tree", "SingleUavSurveillance")
    finally:
        node.destroy_node()


def _run_uav(namespace: str, tree_name: str):
    node = Node("surveillance_trigger_%s" % namespace, namespace=namespace)
    try:
        send_execute_tree(node, "execute_tree", tree_name, timeout_s=600.0)
    finally:
        node.destroy_node()


def run_multi():
    t1 = Thread(
        target=_run_uav,
        args=("uav1", "MultiUavSurveillanceUav1"),
    )
    t2 = Thread(
        target=_run_uav,
        args=("uav2", "MultiUavSurveillanceUav2"),
    )

    t1.start()
    t2.start()
    t1.join()
    t2.join()


def main():
    parser = argparse.ArgumentParser(description="Trigger surveillance BT missions")
    parser.add_argument(
        "--mode",
        choices=["single", "multi"],
        default="single",
        help="single = 1 UAV, multi = 2 UAVs with opposite sweep patterns",
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        if args.mode == "single":
            run_single()
        else:
            run_multi()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
