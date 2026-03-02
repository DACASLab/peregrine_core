"""@file
@brief Example 15 launch: N-UAV circle + figure-8 mission client bundle.

Wraps example14_multi_uav_circle_figure8.launch.py with defaults tuned for
running N UAV mission clients (domains 1..N), where N defaults to the
NUM_UAVS environment variable set in docker/.env.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch N mission clients (from NUM_UAVS env) for multi-container SITL."""
    base_launch = PathJoinSubstitution(
        [
            FindPackageShare("hardware_abstraction_example"),
            "launch",
            "example14_multi_uav_circle_figure8.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_uavs",
                default_value=os.environ.get("NUM_UAVS", "9"),
                description="Number of UAV mission clients to launch (defaults to NUM_UAVS env var).",
            ),
            DeclareLaunchArgument(
                "base_domain_id",
                default_value="1",
                description="ROS domain ID for UAV1.",
            ),
            DeclareLaunchArgument(
                "mission_type",
                default_value="circle_figure8",
                description="Mission sequence type.",
            ),
            DeclareLaunchArgument(
                "inter_uav_start_delay_s",
                default_value="1.0",
                description="Delay between launching consecutive UAV mission clients.",
            ),
            DeclareLaunchArgument(
                "base_takeoff_altitude_m",
                default_value="4.0",
                description="Takeoff altitude for UAV1.",
            ),
            DeclareLaunchArgument(
                "takeoff_altitude_step_m",
                default_value="1.0",
                description="Takeoff altitude increment per UAV index.",
            ),
            DeclareLaunchArgument(
                "uav_namespace_prefix",
                default_value="uav",
                description="Optional namespace prefix (e.g. 'uav' -> /uav1, /uav2).",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Set ROS_LOCALHOST_ONLY for all mission clients.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "num_uavs": LaunchConfiguration("num_uavs"),
                    "base_domain_id": LaunchConfiguration("base_domain_id"),
                    "mission_type": LaunchConfiguration("mission_type"),
                    "inter_uav_start_delay_s": LaunchConfiguration(
                        "inter_uav_start_delay_s"
                    ),
                    "base_takeoff_altitude_m": LaunchConfiguration(
                        "base_takeoff_altitude_m"
                    ),
                    "takeoff_altitude_step_m": LaunchConfiguration(
                        "takeoff_altitude_step_m"
                    ),
                    "uav_namespace_prefix": LaunchConfiguration("uav_namespace_prefix"),
                    "ros_localhost_only": LaunchConfiguration("ros_localhost_only"),
                }.items(),
            ),
        ]
    )
