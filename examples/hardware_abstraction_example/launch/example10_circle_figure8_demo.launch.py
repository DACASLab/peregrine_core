"""@file
@brief Example 10 launch: manager-chain demo with circle/figure-eight trajectories.

Launches the full peregrine stack for one UAV plus an automated demo mission:

  Layer 1 (base_launch = example8):
    MicroXRCEAgent + PX4HardwareAbstraction + FrameTransformer

  Layer 2 (manager_container):
    EstimationManagerNode, ControlManagerNode, TrajectoryManagerNode, UavManagerNode
    All four are lifecycle nodes loaded into a single component_container_mt.
    With auto_start=true (default), they self-transition to ACTIVE on startup.

  Layer 3 (demo mission):
    circle_figure8_demo.py waits for UAVState.dependencies_ready, then sends
    action goals to uav_manager: takeoff -> circle/figure8 trajectory -> land.

Parameters are loaded from YAML defaults per package. Only structural/deployment
args and the most commonly overridden mission param remain as launch arguments.

Usage:
  ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py
  ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py mission_type:=circle
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Starts composed manager stack plus demo node with YAML-configured params."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    mission_type = LaunchConfiguration("mission_type")

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
        [FindPackageShare("hardware_abstraction_example"), "config", "circle_figure8_mission.yaml"]
    )
    # Example-level overrides loaded AFTER package defaults — ROS2 last-file-wins
    # semantics mean only the keys present here replace the defaults.
    example_managers_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction_example"), "config", "example10_managers.yaml"]
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
        executable="circle_figure8_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[mission_yaml, {"mission_type": mission_type}],
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
                "mission_type",
                default_value="circle_figure8",
                description="Mission sequence: circle, figure8, circle_figure8, or circle_land_figure8.",
            ),
            base_launch,
            manager_container,
            demo_node,
        ]
    )
