"""@file
@brief Circle/figure-eight trajectory demo.

Launches the core peregrine stack plus a mission node that executes:
  arm -> takeoff -> circle/figure8 trajectory -> land

Expects PX4 SITL or hardware to be running externally (e.g. via Docker).
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
    mission_type = LaunchConfiguration("mission_type")

    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction_example"), "config", "circle_figure8_mission.yaml"]
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
        executable="circle_figure8_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[mission_yaml, {"mission_type": mission_type}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_namespace", default_value=""),
            DeclareLaunchArgument("ros_localhost_only", default_value="1"),
            DeclareLaunchArgument("ros_domain_id", default_value="42"),
            DeclareLaunchArgument(
                "mission_type",
                default_value="circle_figure8",
                description="Mission sequence: circle, figure8, circle_figure8, or circle_land_figure8.",
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            core_stack,
            demo_node,
        ]
    )
