"""@file
@brief Example 11 launch: repeated takeoff/trajectory/land multi-cycle demo.

Similar to example10 but uses multi_cycle_demo.py instead of circle_figure8_demo.py.
The key difference is that this demo exercises repeated flight cycles: each segment
in the comma-separated multi_cycle_sequence triggers a full takeoff -> trajectory ->
land cycle, with a preflight readiness re-check between cycles.

This validates the FSM's ability to return to Idle after landing and successfully
re-arm/re-takeoff for subsequent missions.

Default sequence: "circle,figure8,circle,figure8" (4 cycles).

Architecture is identical to example10:
  bridge_container (example8 base) + manager_container (auto_start) + demo node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Starts composed manager stack plus multi-cycle demo with YAML-configured params."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    multi_cycle_sequence = LaunchConfiguration("multi_cycle_sequence")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hardware_abstraction_example"),
                    "launch",
                    "example8_px4_sitl_single_uav.launch.py",
                ]
            )
        ),
        launch_arguments={
            "uav_namespace": uav_namespace,
            "start_microxrce_agent": start_microxrce_agent,
            "microxrce_port": microxrce_port,
            "ros_localhost_only": ros_localhost_only,
            "ros_domain_id": ros_domain_id,
        }.items(),
    )

    estimation_yaml = PathJoinSubstitution(
        [FindPackageShare("estimation_manager"), "config", "defaults.yaml"]
    )
    control_yaml = PathJoinSubstitution(
        [FindPackageShare("control_manager"), "config", "defaults.yaml"]
    )
    trajectory_yaml = PathJoinSubstitution(
        [FindPackageShare("trajectory_manager"), "config", "defaults.yaml"]
    )
    uav_yaml = PathJoinSubstitution(
        [FindPackageShare("uav_manager"), "config", "defaults.yaml"]
    )
    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction_example"), "config", "multi_cycle_mission.yaml"]
    )
    # Example-level overrides loaded AFTER package defaults — ROS2 last-file-wins
    # semantics mean only the keys present here replace the defaults.
    example_managers_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction_example"), "config", "example11_managers.yaml"]
    )

    manager_container = ComposableNodeContainer(
        name="manager_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="estimation_manager",
                plugin="estimation_manager::EstimationManagerNode",
                name="estimation_manager",
                namespace=uav_namespace,
                parameters=[estimation_yaml, example_managers_yaml],
            ),
            ComposableNode(
                package="control_manager",
                plugin="control_manager::ControlManagerNode",
                name="control_manager",
                namespace=uav_namespace,
                parameters=[control_yaml, example_managers_yaml],
            ),
            ComposableNode(
                package="trajectory_manager",
                plugin="trajectory_manager::TrajectoryManagerNode",
                name="trajectory_manager",
                namespace=uav_namespace,
                parameters=[trajectory_yaml, example_managers_yaml],
            ),
            ComposableNode(
                package="uav_manager",
                plugin="uav_manager::UavManagerNode",
                name="uav_manager",
                namespace=uav_namespace,
                parameters=[uav_yaml, example_managers_yaml],
            ),
        ],
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
                description="Comma-separated cycle list. Allowed segments: circle, figure8, circle_figure8.",
            ),
            base_launch,
            manager_container,
            demo_node,
        ]
    )
