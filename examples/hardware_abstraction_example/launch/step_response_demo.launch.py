"""@file
@brief Step-response trajectory demo.

Launches the core peregrine stack plus a mission node that executes:
  arm -> takeoff -> position/yaw step-response sequence -> land

Expects PX4 SITL or hardware to be running externally.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    uav_namespace = LaunchConfiguration("uav_namespace")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    step_sequence = LaunchConfiguration("step_sequence")

    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction_example"), "config", "step_response_mission.yaml"]
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
        executable="step_response_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[mission_yaml, {"step_sequence": step_sequence}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_namespace", default_value=""),
            DeclareLaunchArgument("ros_localhost_only", default_value="1"),
            DeclareLaunchArgument("ros_domain_id", default_value="42"),
            DeclareLaunchArgument(
                "step_sequence",
                default_value="x+,x-,y+,y-,z+,z-,yaw+,yaw-",
                description="Comma-separated sequence of step tokens.",
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            core_stack,
            demo_node,
        ]
    )
