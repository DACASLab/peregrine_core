"""@file
@brief Launch the BT TreeExecutionServer alongside the core flight stack.

Includes core_stack.launch.py and launches PeregrineTreeServer. Missions are
triggered by sending ExecuteTree action goals to the "execute_tree" action
server (e.g. from the TUI, GCS, or CLI).

Usage:
  # Full stack + BT server (SITL):
  ros2 launch peregrine_bringup bt_mission.launch.py

  # BT server only (core stack already running):
  ros2 launch peregrine_bringup bt_mission.launch.py start_core_stack:=false

  # Trigger a mission:
  ros2 action send_goal /execute_tree btcpp_ros2_interfaces/action/ExecuteTree \
      "{target_tree: 'TakeoffHoverLand', payload: ''}"
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_core_stack = LaunchConfiguration("start_core_stack")
    use_sim_time = LaunchConfiguration("use_sim_time")

    core_stack_launch = PathJoinSubstitution(
        [FindPackageShare("peregrine_bringup"), "launch", "core_stack.launch.py"]
    )

    tree_server_config = PathJoinSubstitution(
        [FindPackageShare("peregrine_bt"), "config", "tree_server.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "uav_namespace",
                default_value="",
                description="ROS namespace for this UAV stack.",
            ),
            DeclareLaunchArgument(
                "start_core_stack",
                default_value="true",
                description="Include core_stack.launch.py (set false if stack is already running).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(core_stack_launch),
                condition=IfCondition(start_core_stack),
                launch_arguments={
                    "uav_namespace": uav_namespace,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            Node(
                package="peregrine_bt",
                executable="peregrine_tree_server",
                name="bt_action_server",
                namespace=uav_namespace,
                output="screen",
                parameters=[
                    tree_server_config,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
