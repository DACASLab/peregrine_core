"""@file
@brief Example 14 launch: N-domain mission clients for multi-container SITL.

Runs N independent circle_figure8_demo mission nodes, one per ROS domain.
Each mission executes: takeoff -> circle -> figure-8 -> land.

This launch does NOT start SITL or the manager stack. It is intended for the
multi-container setup where:
  - sim container runs shared Gazebo + PX4 instances
  - uav containers already run single_uav.launch.py

Usage:
  ros2 launch hardware_abstraction_example example14_multi_uav_circle_figure8.launch.py
  ros2 launch hardware_abstraction_example example14_multi_uav_circle_figure8.launch.py num_uavs:=4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_namespace(prefix: str, uav_id: int) -> str:
    """Build namespace from optional prefix and UAV index."""
    normalized = prefix.strip()
    if not normalized:
        return ""
    normalized = normalized.rstrip("/")
    if not normalized.startswith("/"):
        normalized = f"/{normalized}"
    return f"{normalized}{uav_id}"


def _launch_setup(context, *args, **kwargs):
    num_uavs = int(context.launch_configurations["num_uavs"])
    if num_uavs < 1:
        raise RuntimeError("num_uavs must be >= 1")

    mission_type = context.launch_configurations["mission_type"]
    ros_localhost_only = context.launch_configurations["ros_localhost_only"]
    base_domain_id = int(context.launch_configurations["base_domain_id"])
    namespace_prefix = context.launch_configurations["uav_namespace_prefix"]
    inter_uav_start_delay_s = float(context.launch_configurations["inter_uav_start_delay_s"])
    base_takeoff_altitude_m = float(context.launch_configurations["base_takeoff_altitude_m"])
    takeoff_altitude_step_m = float(context.launch_configurations["takeoff_altitude_step_m"])

    mission_yaml = PathJoinSubstitution(
        [
            FindPackageShare("hardware_abstraction_example"),
            "config",
            "multi_uav_circle_figure8_mission.yaml",
        ]
    )

    actions = []
    for i in range(num_uavs):
        uav_id = i + 1
        domain_id = str(base_domain_id + i)
        namespace = _resolve_namespace(namespace_prefix, uav_id)
        takeoff_altitude_m = base_takeoff_altitude_m + i * takeoff_altitude_step_m

        node_actions = [
            LogInfo(
                msg=(
                    f"[example14] Starting UAV{uav_id} mission client "
                    f"(domain={domain_id}, namespace='{namespace}', "
                    f"takeoff_altitude_m={takeoff_altitude_m:.2f})"
                )
            ),
            Node(
                package="hardware_abstraction_example",
                executable="circle_figure8_demo.py",
                name=f"circle_figure8_demo_uav{uav_id}",
                namespace=namespace,
                output="screen",
                parameters=[
                    mission_yaml,
                    {
                        "mission_type": mission_type,
                        "takeoff_altitude_m": takeoff_altitude_m,
                    },
                ],
                additional_env={
                    "ROS_DOMAIN_ID": domain_id,
                    "ROS_LOCALHOST_ONLY": ros_localhost_only,
                },
            ),
        ]

        start_delay_s = i * inter_uav_start_delay_s
        if start_delay_s > 0.0:
            actions.append(TimerAction(period=start_delay_s, actions=node_actions))
        else:
            actions.extend(node_actions)

    return actions


def generate_launch_description() -> LaunchDescription:
    """Start one mission client per UAV/domain for synchronized demo flights."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_type",
                default_value="circle_figure8",
                description="Mission sequence: circle, figure8, circle_figure8, or circle_land_figure8.",
            ),
            DeclareLaunchArgument(
                "num_uavs",
                default_value="2",
                description="Number of UAV mission clients to launch.",
            ),
            DeclareLaunchArgument(
                "base_domain_id",
                default_value="1",
                description="ROS domain ID for UAV1; UAV i uses base_domain_id + i - 1.",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Set ROS_LOCALHOST_ONLY for all mission clients.",
            ),
            DeclareLaunchArgument(
                "uav_namespace_prefix",
                default_value="uav",
                description="Optional namespace prefix for mission nodes (e.g. 'uav' -> /uav1, /uav2).",
            ),
            DeclareLaunchArgument(
                "base_takeoff_altitude_m",
                default_value="4.0",
                description="Takeoff altitude for UAV1.",
            ),
            DeclareLaunchArgument(
                "takeoff_altitude_step_m",
                default_value="1.5",
                description="Takeoff altitude increment per UAV index.",
            ),
            DeclareLaunchArgument(
                "inter_uav_start_delay_s",
                default_value="1.5",
                description="Delay between launching consecutive UAV mission clients.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
