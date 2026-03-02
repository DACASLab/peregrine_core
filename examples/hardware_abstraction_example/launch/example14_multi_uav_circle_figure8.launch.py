"""@file
@brief Example 14 launch: two-domain mission clients for multi-container SITL.

Runs two independent circle_figure8_demo mission nodes, one per ROS domain:
  - UAV1 mission client -> domain 1
  - UAV2 mission client -> domain 2

This launch does NOT start SITL or the manager stack. It is intended for the
multi-container setup where:
  - sim container runs shared Gazebo + PX4 instances
  - uav1/uav2 containers already run single_uav.launch.py

Usage:
  ros2 launch hardware_abstraction_example example14_multi_uav_circle_figure8.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Starts one mission client per UAV domain for synchronized demo flights."""
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    mission_type = LaunchConfiguration("mission_type")
    uav1_domain_id = LaunchConfiguration("uav1_domain_id")
    uav2_domain_id = LaunchConfiguration("uav2_domain_id")
    uav1_namespace = LaunchConfiguration("uav1_namespace")
    uav2_namespace = LaunchConfiguration("uav2_namespace")
    uav1_takeoff_altitude_m = LaunchConfiguration("uav1_takeoff_altitude_m")
    uav2_takeoff_altitude_m = LaunchConfiguration("uav2_takeoff_altitude_m")
    uav2_start_delay_s = LaunchConfiguration("uav2_start_delay_s")

    mission_yaml = PathJoinSubstitution(
        [
            FindPackageShare("hardware_abstraction_example"),
            "config",
            "multi_uav_circle_figure8_mission.yaml",
        ]
    )

    uav1_demo = Node(
        package="hardware_abstraction_example",
        executable="circle_figure8_demo.py",
        name="circle_figure8_demo_uav1",
        namespace=uav1_namespace,
        output="screen",
        parameters=[
            mission_yaml,
            {
                "mission_type": mission_type,
                "takeoff_altitude_m": ParameterValue(uav1_takeoff_altitude_m, value_type=float),
            },
        ],
        additional_env={
            "ROS_DOMAIN_ID": uav1_domain_id,
            "ROS_LOCALHOST_ONLY": ros_localhost_only,
        },
    )

    uav2_demo = Node(
        package="hardware_abstraction_example",
        executable="circle_figure8_demo.py",
        name="circle_figure8_demo_uav2",
        namespace=uav2_namespace,
        output="screen",
        parameters=[
            mission_yaml,
            {
                "mission_type": mission_type,
                "takeoff_altitude_m": ParameterValue(uav2_takeoff_altitude_m, value_type=float),
            },
        ],
        additional_env={
            "ROS_DOMAIN_ID": uav2_domain_id,
            "ROS_LOCALHOST_ONLY": ros_localhost_only,
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_type",
                default_value="circle_figure8",
                description="Mission sequence: circle, figure8, circle_figure8, or circle_land_figure8.",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Set ROS_LOCALHOST_ONLY for both mission clients.",
            ),
            DeclareLaunchArgument(
                "uav1_domain_id",
                default_value="1",
                description="ROS domain for UAV1 mission client.",
            ),
            DeclareLaunchArgument(
                "uav2_domain_id",
                default_value="2",
                description="ROS domain for UAV2 mission client.",
            ),
            DeclareLaunchArgument(
                "uav1_namespace",
                default_value="",
                description="Namespace for UAV1 mission client topics/actions.",
            ),
            DeclareLaunchArgument(
                "uav2_namespace",
                default_value="",
                description="Namespace for UAV2 mission client topics/actions.",
            ),
            DeclareLaunchArgument(
                "uav1_takeoff_altitude_m",
                default_value="4.0",
                description="Takeoff altitude for UAV1 mission.",
            ),
            DeclareLaunchArgument(
                "uav2_takeoff_altitude_m",
                default_value="6.0",
                description="Takeoff altitude for UAV2 mission.",
            ),
            DeclareLaunchArgument(
                "uav2_start_delay_s",
                default_value="2.0",
                description="Delay before starting UAV2 mission client (seconds).",
            ),
            LogInfo(msg="[example14] Starting UAV1 mission client."),
            uav1_demo,
            TimerAction(
                period=uav2_start_delay_s,
                actions=[
                    LogInfo(msg="[example14] Starting UAV2 mission client."),
                    uav2_demo,
                ],
            ),
        ]
    )
