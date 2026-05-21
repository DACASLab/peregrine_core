"""@file
@brief Launch a BT mission alongside the core flight stack.

Includes core_stack.launch.py and launches the BTExecutorNode as a separate
process. The BT executor is a lifecycle node — it must be configured and
activated after launch (e.g. via ros2 lifecycle set).

Usage:
  # Full stack + BT (SITL):
  ros2 launch peregrine_bringup bt_mission.launch.py \
      tree_file:=/ros2_ws/install/peregrine_bt/share/peregrine_bt/trees/takeoff_hover_land.xml

  # BT only (core stack already running, e.g. hardware):
  ros2 launch peregrine_bringup bt_mission.launch.py \
      tree_file:=<path> start_core_stack:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    tree_file = LaunchConfiguration("tree_file")
    tick_rate_hz = LaunchConfiguration("tick_rate_hz")
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_core_stack = LaunchConfiguration("start_core_stack")
    use_sim_time = LaunchConfiguration("use_sim_time")

    core_stack_launch = PathJoinSubstitution(
        [FindPackageShare("peregrine_bringup"), "launch", "core_stack.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "tree_file",
                description="Absolute path to the BT XML tree file.",
            ),
            DeclareLaunchArgument(
                "tick_rate_hz",
                default_value="2.0",
                description="BT tick frequency in Hz.",
            ),
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
            LifecycleNode(
                package="peregrine_bt",
                executable="bt_executor_node",
                name="bt_executor",
                namespace=uav_namespace,
                output="screen",
                parameters=[
                    {
                        "tree_file": tree_file,
                        "tick_rate_hz": tick_rate_hz,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
