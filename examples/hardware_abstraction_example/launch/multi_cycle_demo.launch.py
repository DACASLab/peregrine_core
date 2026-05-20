"""@file
@brief Repeated takeoff/trajectory/land multi-cycle demo.

Each segment in the comma-separated multi_cycle_sequence triggers a full
takeoff -> trajectory -> land cycle with preflight re-check between cycles.
Validates FSM return-to-idle and re-arm behavior.

Expects PX4 SITL or hardware to be running externally.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    uav_namespace = LaunchConfiguration("uav_namespace")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    multi_cycle_sequence = LaunchConfiguration("multi_cycle_sequence")

    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction_example"), "config", "multi_cycle_mission.yaml"]
    )

    core_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("peregrine_bringup"), "launch", "core_stack.launch.py"]
            )
        ),
        launch_arguments={
            "uav_namespace": uav_namespace,
            "ros_localhost_only": ros_localhost_only,
            "ros_domain_id": ros_domain_id,
        }.items(),
    )

    demo_node = Node(
        package="hardware_abstraction_example",
        executable="multi_cycle_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[mission_yaml, {"multi_cycle_sequence": multi_cycle_sequence}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_namespace", default_value=""),
            DeclareLaunchArgument("ros_localhost_only", default_value="1"),
            DeclareLaunchArgument("ros_domain_id", default_value="42"),
            DeclareLaunchArgument(
                "multi_cycle_sequence",
                default_value="circle,figure8,circle,figure8",
                description="Comma-separated cycle list: circle, figure8, circle_figure8.",
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            core_stack,
            demo_node,
        ]
    )
