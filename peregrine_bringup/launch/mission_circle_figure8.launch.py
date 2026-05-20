"""@file
@brief Hardware mission: circle and/or figure-eight trajectories.

Launches ONLY the mission demo node. The full peregrine stack must already be
running (via tmuxinator `flight` session or manual `core_stack.launch.py`).

This is the hardware counterpart to example10_circle_figure8_demo.launch.py,
which launches its own manager container (for SITL). On hardware the managers
are already running, so we only need the mission script.

Usage (from the shell pane in the tmuxinator flight session):

  ros2 launch peregrine_bringup mission_circle_figure8.launch.py
  ros2 launch peregrine_bringup mission_circle_figure8.launch.py mission_type:=circle
  ros2 launch peregrine_bringup mission_circle_figure8.launch.py mission_type:=figure8
  ros2 launch peregrine_bringup mission_circle_figure8.launch.py \\
      mission_type:=circle_figure8 takeoff_altitude_m:=3.0 circle_radius_m:=1.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Start mission demo node against an already-running stack."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    mission_type = LaunchConfiguration("mission_type")

    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("peregrine_bringup"), "config", "hardware_mission.yaml"]
    )

    demo_node = Node(
        package="hardware_abstraction_example",
        executable="circle_figure8_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[
            mission_yaml,
            {"mission_type": mission_type},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "uav_namespace",
                default_value="",
                description="Must match the namespace of the running stack.",
            ),
            DeclareLaunchArgument(
                "mission_type",
                default_value="circle_figure8",
                description="circle, figure8, circle_figure8, or circle_land_figure8.",
            ),
            demo_node,
        ]
    )
