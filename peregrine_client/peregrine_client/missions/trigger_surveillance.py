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
from threading import Event, Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from btcpp_ros2_interfaces.action import ExecuteTree


def _wait_future(future, timeout_sec: float):
    """Block until a rclpy future completes, driven by a separately-spinning executor.

    Returns the future result, or None on timeout. Used instead of
    spin_until_future_complete so several missions can run concurrently without each
    grabbing the global executor (whose shared wait set races -> 'wait set index too big').
    """
    done = Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout_sec):
        return None
    return future.result()


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


def run_single(namespace: str):
    node = Node("surveillance_trigger_single", namespace=namespace)
    try:
        return send_execute_tree(node, "execute_tree", "SingleUavSurveillance")
    finally:
        node.destroy_node()


def run_multi(timeout_s: float = 600.0) -> bool:
    """Dispatch each UAV's surveillance tree concurrently from one spinning executor.

    All trigger nodes share a single SingleThreadedExecutor spun on one background thread, so
    there is exactly one wait set (no cross-thread race). Goals are sent up front, then we
    block on every UAV's result future in parallel. Generalizes to N UAVs by extending `specs`.
    """
    specs = [
        ("uav1", "MultiUavSurveillanceUav1"),
        ("uav2", "MultiUavSurveillanceUav2"),
    ]

    nodes = [Node("surveillance_trigger_%s" % ns, namespace=ns) for ns, _ in specs]
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    spin_thread = Thread(target=_spin_executor, args=(executor,), daemon=True)
    spin_thread.start()

    ok: dict[str, bool] = {}
    result_futures: dict[str, object] = {}
    try:
        for (ns, tree_name), node in zip(specs, nodes):
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
        for node in nodes:
            node.destroy_node()

    return all(ok.get(ns, False) for ns, _ in specs)


def _spin_executor(executor: SingleThreadedExecutor) -> None:
    try:
        executor.spin()
    except Exception:  # noqa: BLE001 - executor.spin() raises on shutdown; expected.
        pass


def main():
    parser = argparse.ArgumentParser(description="Trigger surveillance BT missions")
    parser.add_argument(
        "--mode",
        choices=["single", "multi"],
        default="single",
        help="single = 1 UAV, multi = 2 UAVs with opposite sweep patterns",
    )
    parser.add_argument(
        "--namespace",
        default="uav1",
        help="UAV namespace for single mode, for example uav1. Use '' for root namespace.",
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        if args.mode == "single":
            success = run_single(args.namespace)
        else:
            success = run_multi()
    finally:
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
