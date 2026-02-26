"""@file
@brief Example 10 launch: manager-chain demo with circle/figure-eight trajectories.

Launches the full peregrine stack for one UAV plus an automated demo mission:

  Layer 1 (base_launch = example8):
    MicroXRCEAgent + PX4HardwareAbstraction + FrameTransformer

  Layer 2 (manager_container):
    EstimationManagerNode, ControlManagerNode, TrajectoryManagerNode, UavManagerNode
    All four are lifecycle nodes loaded into a single component_container_mt.
    They start in UNCONFIGURED state and remain inactive until orchestrated.

  Layer 3 (lifecycle orchestrator):
    lifecycle_bringup_orchestrator.py configures/activates managers in two phases:
      Phase 1: estimation_manager, control_manager, trajectory_manager (parallel-safe)
      Phase 2: uav_manager (depends on Phase 1 managers being active)

  Layer 4 (demo mission):
    circle_figure8_demo.py waits for preflight readiness, then sends action goals
    to uav_manager: takeoff -> circle/figure8 trajectory -> land.

The two-container architecture (bridge_container + manager_container) keeps the
non-lifecycle bridge nodes separate from the lifecycle-managed manager nodes.

Usage:
  ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py
  ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py mission_type:=circle
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Starts composed manager stack plus optional orchestrator and demo node."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    px4_namespace = LaunchConfiguration("px4_namespace")
    frame_prefix = LaunchConfiguration("frame_prefix")
    sensor_gps_topic_suffix = LaunchConfiguration("sensor_gps_topic_suffix")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    offboard_rate_hz = LaunchConfiguration("offboard_rate_hz")
    status_rate_hz = LaunchConfiguration("status_rate_hz")
    tf_publish_rate_hz = LaunchConfiguration("tf_publish_rate_hz")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    use_lifecycle_orchestrator = LaunchConfiguration("use_lifecycle_orchestrator")
    lifecycle_service_timeout_s = LaunchConfiguration("lifecycle_service_timeout_s")
    lifecycle_state_timeout_s = LaunchConfiguration("lifecycle_state_timeout_s")
    lifecycle_data_readiness_timeout_s = LaunchConfiguration("lifecycle_data_readiness_timeout_s")
    lifecycle_data_freshness_timeout_s = LaunchConfiguration("lifecycle_data_freshness_timeout_s")
    lifecycle_phase_2_retry_timeout_s = LaunchConfiguration("lifecycle_phase_2_retry_timeout_s")
    lifecycle_phase_2_retry_backoff_s = LaunchConfiguration("lifecycle_phase_2_retry_backoff_s")

    mission_type = LaunchConfiguration("mission_type")
    takeoff_altitude_m = LaunchConfiguration("takeoff_altitude_m")
    climb_velocity_mps = LaunchConfiguration("climb_velocity_mps")
    landing_descent_velocity_mps = LaunchConfiguration("landing_descent_velocity_mps")
    circle_radius_m = LaunchConfiguration("circle_radius_m")
    circle_angular_velocity_radps = LaunchConfiguration("circle_angular_velocity_radps")
    circle_loops = LaunchConfiguration("circle_loops")
    figure8_radius_m = LaunchConfiguration("figure8_radius_m")
    figure8_angular_velocity_radps = LaunchConfiguration("figure8_angular_velocity_radps")
    figure8_loops = LaunchConfiguration("figure8_loops")
    preflight_wait_s = LaunchConfiguration("preflight_wait_s")
    server_wait_s = LaunchConfiguration("server_wait_s")
    action_timeout_s = LaunchConfiguration("action_timeout_s")

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
            "px4_namespace": px4_namespace,
            "frame_prefix": frame_prefix,
            "sensor_gps_topic_suffix": sensor_gps_topic_suffix,
            "start_microxrce_agent": start_microxrce_agent,
            "microxrce_port": microxrce_port,
            "offboard_rate_hz": offboard_rate_hz,
            "status_rate_hz": status_rate_hz,
            "tf_publish_rate_hz": tf_publish_rate_hz,
            "ros_localhost_only": ros_localhost_only,
            "ros_domain_id": ros_domain_id,
        }.items(),
    )

    demo_parameters = [
        {
            "mission_type": mission_type,
            "takeoff_altitude_m": takeoff_altitude_m,
            "climb_velocity_mps": climb_velocity_mps,
            "landing_descent_velocity_mps": landing_descent_velocity_mps,
            "circle_radius_m": circle_radius_m,
            "circle_angular_velocity_radps": circle_angular_velocity_radps,
            "circle_loops": circle_loops,
            "figure8_radius_m": figure8_radius_m,
            "figure8_angular_velocity_radps": figure8_angular_velocity_radps,
            "figure8_loops": figure8_loops,
            "preflight_wait_s": preflight_wait_s,
            "server_wait_s": server_wait_s,
            "action_timeout_s": action_timeout_s,
        }
    ]

    orchestrator_node = Node(
        condition=IfCondition(use_lifecycle_orchestrator),
        package="hardware_abstraction_example",
        executable="lifecycle_bringup_orchestrator.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[
            {
                "phase_1_nodes": [
                    "estimation_manager",
                    "control_manager",
                    "trajectory_manager",
                ],
                "phase_2_nodes": ["uav_manager"],
                "service_wait_timeout_s": lifecycle_service_timeout_s,
                "state_wait_timeout_s": lifecycle_state_timeout_s,
                "data_readiness_timeout_s": lifecycle_data_readiness_timeout_s,
                "data_freshness_timeout_s": lifecycle_data_freshness_timeout_s,
                "phase_2_retry_timeout_s": lifecycle_phase_2_retry_timeout_s,
                "phase_2_retry_backoff_s": lifecycle_phase_2_retry_backoff_s,
            }
        ],
    )

    demo_node_direct = Node(
        condition=UnlessCondition(use_lifecycle_orchestrator),
        package="hardware_abstraction_example",
        executable="circle_figure8_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=demo_parameters,
    )

    demo_node_after_orchestrator = Node(
        package="hardware_abstraction_example",
        executable="circle_figure8_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=demo_parameters,
    )

    def _on_orchestrator_exit(event, _context):
        if event.returncode == 0:
            return [demo_node_after_orchestrator]
        return [
            LogInfo(
                msg=(
                    "lifecycle_bringup_orchestrator exited with non-zero status; "
                    "demo mission will not start."
                )
            ),
            Shutdown(reason="Lifecycle bringup failed"),
        ]

    orchestrator_exit_handler = RegisterEventHandler(
        OnProcessExit(target_action=orchestrator_node, on_exit=_on_orchestrator_exit)
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_namespace", default_value=""),
            DeclareLaunchArgument("px4_namespace", default_value=""),
            DeclareLaunchArgument("frame_prefix", default_value=""),
            DeclareLaunchArgument("sensor_gps_topic_suffix", default_value="/fmu/out/sensor_gps"),
            DeclareLaunchArgument("start_microxrce_agent", default_value="true"),
            DeclareLaunchArgument("microxrce_port", default_value="8888"),
            DeclareLaunchArgument("offboard_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("status_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("tf_publish_rate_hz", default_value="100.0"),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Set ROS_LOCALHOST_ONLY (use 1 with PX4 UXRCE_DDS_PTCFG=1).",
            ),
            DeclareLaunchArgument("ros_domain_id", default_value="42"),
            DeclareLaunchArgument(
                "use_lifecycle_orchestrator",
                default_value="true",
                description=(
                    "If true, configure/activate lifecycle managers in deterministic order."
                ),
            ),
            DeclareLaunchArgument("lifecycle_service_timeout_s", default_value="10.0"),
            DeclareLaunchArgument("lifecycle_state_timeout_s", default_value="15.0"),
            DeclareLaunchArgument("lifecycle_data_readiness_timeout_s", default_value="30.0"),
            DeclareLaunchArgument("lifecycle_data_freshness_timeout_s", default_value="1.5"),
            DeclareLaunchArgument("lifecycle_phase_2_retry_timeout_s", default_value="20.0"),
            DeclareLaunchArgument("lifecycle_phase_2_retry_backoff_s", default_value="0.5"),
            DeclareLaunchArgument(
                "mission_type",
                default_value="circle_figure8",
                description=(
                    "Mission sequence: circle, figure8, circle_figure8, or "
                    "circle_land_figure8."
                ),
            ),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="5.0"),
            DeclareLaunchArgument("climb_velocity_mps", default_value="1.0"),
            DeclareLaunchArgument("landing_descent_velocity_mps", default_value="0.8"),
            DeclareLaunchArgument("circle_radius_m", default_value="2.0"),
            DeclareLaunchArgument("circle_angular_velocity_radps", default_value="0.6"),
            DeclareLaunchArgument("circle_loops", default_value="1.0"),
            DeclareLaunchArgument("figure8_radius_m", default_value="2.0"),
            DeclareLaunchArgument("figure8_angular_velocity_radps", default_value="0.6"),
            DeclareLaunchArgument("figure8_loops", default_value="1.0"),
            DeclareLaunchArgument("preflight_wait_s", default_value="30.0"),
            DeclareLaunchArgument("server_wait_s", default_value="20.0"),
            DeclareLaunchArgument("action_timeout_s", default_value="240.0"),
            base_launch,
            # All four managers are loaded into one container so they share a process and
            # can use intra-process communication. component_container_mt (MultiThreadedExecutor)
            # is required by uav_manager, which uses multiple callback groups for concurrent
            # action handling and timer callbacks. The managers start in UNCONFIGURED state;
            # the lifecycle orchestrator (below) transitions them through configure -> activate.
            ComposableNodeContainer(
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
                    ),
                    ComposableNode(
                        package="control_manager",
                        plugin="control_manager::ControlManagerNode",
                        name="control_manager",
                        namespace=uav_namespace,
                    ),
                    ComposableNode(
                        package="trajectory_manager",
                        plugin="trajectory_manager::TrajectoryManagerNode",
                        name="trajectory_manager",
                        namespace=uav_namespace,
                    ),
                    # uav_manager requires MultiThreadedExecutor (component_container_mt).
                    ComposableNode(
                        package="uav_manager",
                        plugin="uav_manager::UavManagerNode",
                        name="uav_manager",
                        namespace=uav_namespace,
                    ),
                ],
            ),
            # Start orchestrator first; demo starts only if it exits cleanly.
            orchestrator_node,
            orchestrator_exit_handler,
            demo_node_direct,
        ]
    )
