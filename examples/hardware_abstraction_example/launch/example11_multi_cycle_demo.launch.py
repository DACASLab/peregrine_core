"""@file
@brief Example 11 launch: repeated takeoff/trajectory/land multi-cycle demo.

Similar to example10 but uses multi_cycle_demo.py instead of circle_figure8_demo.py.
The key difference is that this demo exercises repeated flight cycles: each segment
in the comma-separated multi_cycle_sequence triggers a full takeoff -> trajectory ->
land cycle, with a preflight readiness re-check between cycles.

This validates the FSM's ability to return to Idle after landing and successfully
re-arm/re-takeoff for subsequent missions -- a critical path for autonomous multi-
sortie operations.

Default sequence: "circle,figure8,circle,figure8" (4 cycles).

Architecture is identical to example10:
  bridge_container (example8 base) + manager_container + orchestrator + demo node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Starts composed manager stack plus optional orchestrator and multi-cycle demo."""
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

    multi_cycle_sequence = LaunchConfiguration("multi_cycle_sequence")
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
            DeclareLaunchArgument(
                "multi_cycle_sequence",
                default_value="circle,figure8,circle,figure8",
                description=(
                    "Comma-separated cycle list. Allowed segments: circle, figure8, circle_figure8."
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
            # Same manager container as example10. The dependency_startup_timeout_s
            # parameter on uav_manager controls how long on_configure() waits for
            # upstream manager status topics to appear before failing the transition.
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
                    ComposableNode(
                        package="uav_manager",
                        plugin="uav_manager::UavManagerNode",
                        name="uav_manager",
                        namespace=uav_namespace,
                        parameters=[{"dependency_startup_timeout_s": 10.0}],
                    ),
                ],
            ),
            Node(
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
                    }
                ],
            ),
            Node(
                package="hardware_abstraction_example",
                executable="multi_cycle_demo.py",
                namespace=uav_namespace,
                output="screen",
                parameters=[
                    {
                        "multi_cycle_sequence": multi_cycle_sequence,
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
                ],
            ),
        ]
    )
