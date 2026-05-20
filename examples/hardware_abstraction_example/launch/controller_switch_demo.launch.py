"""@file
@brief Runtime controller switching demo with pluginlib.

Runs three flight cycles switching the control_manager's controller plugin:
  Cycle 1: passthrough (takeoff -> circle -> figure8 -> land)
  Cycle 2: SE3        (takeoff -> circle -> figure8 -> land)
  Cycle 3: passthrough (takeoff -> circle -> figure8 -> land)

Between cycles the demo drives control_manager through lifecycle transitions
to swap the controller backend without restarting any process.

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
    config_overrides = LaunchConfiguration("config_overrides")

    default_overrides = PathJoinSubstitution(
        [FindPackageShare("control_manager"), "config", "se3_tuned.yaml"]
    )
    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction_example"), "config", "controller_switch_mission.yaml"]
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
            "config_overrides": config_overrides,
        }.items(),
    )

    demo_node = Node(
        package="hardware_abstraction_example",
        executable="controller_switch_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[mission_yaml],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_namespace", default_value=""),
            DeclareLaunchArgument("ros_localhost_only", default_value="1"),
            DeclareLaunchArgument("ros_domain_id", default_value="42"),
            DeclareLaunchArgument(
                "config_overrides",
                default_value=default_overrides,
                description="Override YAML applied after per-package defaults.",
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            core_stack,
            demo_node,
        ]
    )
