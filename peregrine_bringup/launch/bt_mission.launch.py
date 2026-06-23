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
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    px4_namespace = LaunchConfiguration("px4_namespace")
    target_system_id = LaunchConfiguration("target_system_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    groot2_port = LaunchConfiguration("groot2_port")
    enable_avoidance = LaunchConfiguration("enable_avoidance")

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
                "start_microxrce_agent",
                default_value="true",
                description="Forwarded to core_stack.launch.py when start_core_stack is true.",
            ),
            DeclareLaunchArgument(
                "microxrce_port",
                default_value="8888",
                description="Forwarded to core_stack.launch.py when start_core_stack is true.",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Forwarded to core_stack.launch.py when start_core_stack is true.",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="0",
                description="Forwarded to core_stack.launch.py when start_core_stack is true.",
            ),
            DeclareLaunchArgument(
                "px4_namespace",
                default_value="",
                description="Forwarded to core_stack.launch.py when start_core_stack is true.",
            ),
            DeclareLaunchArgument(
                "target_system_id",
                default_value="1",
                description="Forwarded to core_stack.launch.py when start_core_stack is true.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "groot2_port",
                default_value="1667",
                description="Port used by the Groot2 publisher for this BT server.",
            ),
            DeclareLaunchArgument(
                "enable_avoidance",
                default_value="true",
                description="Enable inter-UAV BVC collision avoidance (forwarded to core_stack). "
                "On by default; set false for an avoidance-off baseline.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(core_stack_launch),
                condition=IfCondition(start_core_stack),
                launch_arguments={
                    "uav_namespace": uav_namespace,
                    "start_microxrce_agent": start_microxrce_agent,
                    "microxrce_port": microxrce_port,
                    "ros_localhost_only": ros_localhost_only,
                    "ros_domain_id": ros_domain_id,
                    "px4_namespace": px4_namespace,
                    "target_system_id": target_system_id,
                    "use_sim_time": use_sim_time,
                    "enable_avoidance": enable_avoidance,
                }.items(),
            ),
            Node(
                package="peregrine_bt",
                executable="peregrine_tree_server",
                name="bt_action_server",
                namespace=uav_namespace,
                output="screen",
                # The BT server can abort (SIGABRT) if the transport delivers a
                # duplicate ExecuteTree goal request: rclcpp_action throws
                # "Failed to accept new goal" from inside the executor thread,
                # which is upstream of handle_goal() so the execution_active_
                # guard cannot catch it. Without respawn the action server stays
                # dead and /execute_tree silently disappears until a manual
                # relaunch. Respawn so the action server recovers within seconds.
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    tree_server_config,
                    {
                        # YAML node keys do not match once this node is placed
                        # in a UAV namespace, so keep the critical tree server
                        # parameters inline as namespace-safe overrides.
                        "action_name": "execute_tree",
                        "tick_frequency": 10,
                        "groot2_port": groot2_port,
                        "ros_plugins_timeout": 1000,
                        "behavior_trees": [
                            "peregrine_bt/trees",
                            "peregrine_bt/trees/subtrees",
                        ],
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ]
    )
