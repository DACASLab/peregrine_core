"""@file
@brief Example 13 launch: focused multi-cycle demo with monitoring tools.

This launch wraps Example 11 (multi-cycle mission) and adds optional monitoring
processes:
  - tui_status (`tui_status_node`)
  - RViz flight visualizer (`rviz_plugins/flight_visualizer_node`)
  - RViz2 with default flight visualization profile

Use this when you want repeated takeoff/land loops with observability in one
entrypoint.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Launch mission demo plus optional TUI and RViz visualization tools."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")

    multi_cycle_sequence = LaunchConfiguration("multi_cycle_sequence")

    start_tui = LaunchConfiguration("start_tui")
    start_flight_visualizer = LaunchConfiguration("start_flight_visualizer")
    start_rviz = LaunchConfiguration("start_rviz")
    fixed_frame = LaunchConfiguration("fixed_frame")
    rviz_config = LaunchConfiguration("rviz_config")

    wrapped_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hardware_abstraction_example"),
                    "launch",
                    "example11_multi_cycle_demo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "uav_namespace": uav_namespace,
            "start_microxrce_agent": start_microxrce_agent,
            "microxrce_port": microxrce_port,
            "ros_localhost_only": ros_localhost_only,
            "ros_domain_id": ros_domain_id,
            "multi_cycle_sequence": multi_cycle_sequence,
        }.items(),
    )

    tui_node = Node(
        condition=IfCondition(start_tui),
        package="tui_status",
        executable="tui_status_node",
        name="tui_status",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tui_status"), "config", "defaults.yaml"]
            ),
            {"uav_namespace": uav_namespace, "use_sim_time": True},
        ],
    )

    visualizer_node = Node(
        condition=IfCondition(start_flight_visualizer),
        package="rviz_plugins",
        executable="flight_visualizer_node",
        name="flight_visualizer",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("rviz_plugins"), "config", "defaults.yaml"]
            ),
            {
                "uav_namespace": uav_namespace,
                "fixed_frame": fixed_frame,
                "use_sim_time": True,
            },
        ],
    )

    rviz_node = Node(
        condition=IfCondition(start_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=uav_namespace,
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_namespace", default_value=""),
            DeclareLaunchArgument("start_microxrce_agent", default_value="true"),
            DeclareLaunchArgument("microxrce_port", default_value="8888"),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Set ROS_LOCALHOST_ONLY (use 1 with PX4 UXRCE_DDS_PTCFG=1).",
            ),
            DeclareLaunchArgument("ros_domain_id", default_value="42"),
            DeclareLaunchArgument(
                "multi_cycle_sequence",
                default_value="circle,figure8,circle,figure8",
                description="Forwarded to example11 multi-cycle mission.",
            ),
            DeclareLaunchArgument(
                "start_tui",
                default_value="false",
                description="Start tui_status node (best run in dedicated terminal).",
            ),
            DeclareLaunchArgument(
                "start_flight_visualizer",
                default_value="true",
                description="Start rviz_plugins flight visualizer node.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz2 with flight visualization profile.",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="map",
                description="Fixed frame for visualization outputs.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("rviz_plugins"), "rviz", "flight_visualization.rviz"]
                ),
                description="RViz config file path.",
            ),
            wrapped_demo_launch,
            tui_node,
            visualizer_node,
            rviz_node,
        ]
    )
