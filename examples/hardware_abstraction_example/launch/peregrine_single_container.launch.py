"""@file
@brief Single-container launch: all peregrine nodes in one process.

Merges the bridge_container and manager_container into a single
component_container_mt process. All composable nodes share one process and
can leverage intra-process zero-copy communication.

Benefits over the two-container approach:
  - One fewer process (lower memory, simpler process tree)
  - Intra-process transport between hardware_abstraction and managers
  - Ideal for resource-constrained SBCs (Jetson, RPi, etc.)

All nodes load parameters from their package YAML defaults. Managers use
auto_start=true to self-transition through configure -> activate.

Usage:
  ros2 launch hardware_abstraction_example peregrine_single_container.launch.py

Override auto_start for manual lifecycle control:
  ros2 launch hardware_abstraction_example peregrine_single_container.launch.py \\
    -p estimation_manager:auto_start:=false \\
    -p control_manager:auto_start:=false \\
    -p trajectory_manager:auto_start:=false \\
    -p uav_manager:auto_start:=false

Override safety/uav parameter profiles:
  ros2 launch hardware_abstraction_example peregrine_single_container.launch.py \\
    safety_params_file:=/path/to/safety_profile.yaml \\
    uav_params_file:=/path/to/uav_profile.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Launches entire peregrine stack in a single component container."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    safety_params_file = LaunchConfiguration("safety_params_file")
    uav_params_file = LaunchConfiguration("uav_params_file")

    hardware_abstraction_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction"), "config", "defaults.yaml"]
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
    safety_yaml = PathJoinSubstitution(
        [FindPackageShare("safety_monitor"), "config", "defaults.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "uav_namespace",
                default_value="",
                description="ROS namespace for the UAV stack.",
            ),
            DeclareLaunchArgument(
                "start_microxrce_agent",
                default_value="true",
                description="Start MicroXRCEAgent inside this launch.",
            ),
            DeclareLaunchArgument(
                "microxrce_port",
                default_value="8888",
                description="UDP port for MicroXRCEAgent.",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Set ROS_LOCALHOST_ONLY (use 1 with PX4 UXRCE_DDS_PTCFG=1).",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="42",
                description="ROS domain used by this launch.",
            ),
            DeclareLaunchArgument(
                "safety_params_file",
                default_value=safety_yaml,
                description="Safety monitor parameters file path.",
            ),
            DeclareLaunchArgument(
                "uav_params_file",
                default_value=uav_yaml,
                description="UAV manager parameters file path.",
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            ExecuteProcess(
                condition=IfCondition(start_microxrce_agent),
                cmd=["MicroXRCEAgent", "udp4", "-p", microxrce_port],
                output="screen",
                name="microxrce_agent",
            ),
            ComposableNodeContainer(
                name="peregrine_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="hardware_abstraction",
                        plugin="hardware_abstraction::PX4HardwareAbstraction",
                        name="px4_hardware_abstraction",
                        namespace=uav_namespace,
                        parameters=[hardware_abstraction_yaml],
                    ),
                    ComposableNode(
                        package="frame_transforms",
                        plugin="frame_transforms::FrameTransformer",
                        name="frame_transformer",
                        namespace=uav_namespace,
                        parameters=[
                            {
                                "frame_prefix": "",
                                "odometry_topic": "odometry",
                                "publish_rate_hz": 100.0,
                                "home_lat_deg": 13.018509,
                                "home_lon_deg": 77.565088,
                                "gps_min_fix_type": 3,
                                "gps_min_satellites": 6,
                                "gps_max_hdop": 5.0,
                                "gps_max_vdop": 5.0,
                                "gps_freshness_timeout_s": 2.0,
                                "home_init_timeout_s": 60.0,
                            }
                        ],
                    ),
                    ComposableNode(
                        package="estimation_manager",
                        plugin="estimation_manager::EstimationManagerNode",
                        name="estimation_manager",
                        namespace=uav_namespace,
                        parameters=[estimation_yaml],
                    ),
                    ComposableNode(
                        package="control_manager",
                        plugin="control_manager::ControlManagerNode",
                        name="control_manager",
                        namespace=uav_namespace,
                        parameters=[control_yaml],
                    ),
                    ComposableNode(
                        package="trajectory_manager",
                        plugin="trajectory_manager::TrajectoryManagerNode",
                        name="trajectory_manager",
                        namespace=uav_namespace,
                        parameters=[trajectory_yaml],
                    ),
                    ComposableNode(
                        package="safety_monitor",
                        plugin="safety_monitor::SafetyMonitorNode",
                        name="safety_monitor",
                        namespace=uav_namespace,
                        parameters=[safety_params_file],
                    ),
                    ComposableNode(
                        package="uav_manager",
                        plugin="uav_manager::UavManagerNode",
                        name="uav_manager",
                        namespace=uav_namespace,
                        parameters=[uav_params_file],
                    ),
                ],
            ),
        ]
    )
