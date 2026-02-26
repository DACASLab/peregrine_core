"""@file
@brief Example 12 launch: repeatable safety-validation pipeline.

Starts the full single-container stack plus helper nodes for deterministic
safety validation:
  1) `safety_regression_demo.py` (default on) for multi-case end-to-end checks
  2) `safety_takeoff_hold_demo.py` (optional manual helper)
  3) `safety_fault_injector.py` (optional manual helper)

Default profile routes safety inputs to isolated `safety_test/*` topics, enables
auto-land, requires external safety in `uav_manager`, and uses a small geofence
for explicit geofence breach validation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Launch full stack and optional safety test helpers."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")

    safety_params_file = LaunchConfiguration("safety_params_file")
    uav_params_file = LaunchConfiguration("uav_params_file")

    start_takeoff_hold_demo = LaunchConfiguration("start_takeoff_hold_demo")
    hold_time_s = LaunchConfiguration("hold_time_s")

    start_fault_injector = LaunchConfiguration("start_fault_injector")
    fault_scenario = LaunchConfiguration("fault_scenario")
    fault_start_delay_s = LaunchConfiguration("fault_start_delay_s")
    fault_duration_s = LaunchConfiguration("fault_duration_s")
    fault_publish_rate_hz = LaunchConfiguration("fault_publish_rate_hz")
    fault_battery_topic = LaunchConfiguration("fault_battery_topic")
    fault_gps_status_topic = LaunchConfiguration("fault_gps_status_topic")
    fault_estimated_state_topic = LaunchConfiguration("fault_estimated_state_topic")

    start_safety_regression_demo = LaunchConfiguration("start_safety_regression_demo")
    regression_publish_rate_hz = LaunchConfiguration("regression_publish_rate_hz")
    regression_geofence_breach_x_m = LaunchConfiguration("regression_geofence_breach_x_m")
    regression_cases = LaunchConfiguration("regression_cases")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hardware_abstraction_example"),
                    "launch",
                    "peregrine_single_container.launch.py",
                ]
            )
        ),
        launch_arguments={
            "uav_namespace": uav_namespace,
            "start_microxrce_agent": start_microxrce_agent,
            "microxrce_port": microxrce_port,
            "ros_localhost_only": ros_localhost_only,
            "ros_domain_id": ros_domain_id,
            "safety_params_file": safety_params_file,
            "uav_params_file": uav_params_file,
        }.items(),
    )

    takeoff_hold_demo = Node(
        condition=IfCondition(start_takeoff_hold_demo),
        package="hardware_abstraction_example",
        executable="safety_takeoff_hold_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[
            {
                "hold_time_s": hold_time_s,
            }
        ],
    )

    fault_injector = Node(
        condition=IfCondition(start_fault_injector),
        package="hardware_abstraction_example",
        executable="safety_fault_injector.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[
            {
                "scenario": fault_scenario,
                "start_delay_s": fault_start_delay_s,
                "duration_s": fault_duration_s,
                "publish_rate_hz": fault_publish_rate_hz,
                "shutdown_on_complete": False,
                "battery_topic": fault_battery_topic,
                "gps_status_topic": fault_gps_status_topic,
                "estimated_state_topic": fault_estimated_state_topic,
            }
        ],
    )

    safety_regression_demo = Node(
        condition=IfCondition(start_safety_regression_demo),
        package="hardware_abstraction_example",
        executable="safety_regression_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[
            {
                "publish_rate_hz": regression_publish_rate_hz,
                "battery_topic": fault_battery_topic,
                "gps_status_topic": fault_gps_status_topic,
                "estimated_state_topic": fault_estimated_state_topic,
                "geofence_breach_x_m": regression_geofence_breach_x_m,
                "cases": regression_cases,
            }
        ],
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
                "safety_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("hardware_abstraction_example"),
                        "config",
                        "safety_regression.yaml",
                    ]
                ),
                description="Safety monitor parameter profile file.",
            ),
            DeclareLaunchArgument(
                "uav_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("hardware_abstraction_example"),
                        "config",
                        "uav_require_external_safety.yaml",
                    ]
                ),
                description="UAV manager parameter profile file.",
            ),
            DeclareLaunchArgument(
                "start_safety_regression_demo",
                default_value="true",
                description=(
                    "Run multi-case safety regression "
                    "(battery post-takeoff, GPS pre/post, geofence post)."
                ),
            ),
            DeclareLaunchArgument(
                "regression_publish_rate_hz",
                default_value="25.0",
                description="Synthetic safety input publish rate for regression runner.",
            ),
            DeclareLaunchArgument(
                "regression_geofence_breach_x_m",
                default_value="80.0",
                description="Injected x-position used to breach geofence radius.",
            ),
            DeclareLaunchArgument(
                "regression_cases",
                default_value="all",
                description=(
                    "Comma-separated case names or 'all'. "
                    "Valid: battery_post_takeoff_auto_land, gps_pre_takeoff_gate, "
                    "gps_post_takeoff_auto_land, geofence_post_takeoff_auto_land"
                ),
            ),
            DeclareLaunchArgument("start_takeoff_hold_demo", default_value="false"),
            DeclareLaunchArgument(
                "hold_time_s",
                default_value="35.0",
                description="Hover hold window for fault injection.",
            ),
            DeclareLaunchArgument("start_fault_injector", default_value="false"),
            DeclareLaunchArgument(
                "fault_scenario",
                default_value="battery_critical",
                description=(
                    "Fault scenario: none, gps_fix_critical, gps_sats_critical, "
                    "gps_hdop_warning, gps_vdop_warning, gps_missing_warning, "
                    "battery_warning, battery_critical, battery_emergency, "
                    "battery_low_voltage_emergency, battery_missing_warning, "
                    "geofence_radius_critical, geofence_alt_high_critical, "
                    "geofence_alt_low_warning, envelope_speed_critical, envelope_tilt_warning"
                ),
            ),
            DeclareLaunchArgument("fault_start_delay_s", default_value="18.0"),
            DeclareLaunchArgument("fault_duration_s", default_value="12.0"),
            DeclareLaunchArgument("fault_publish_rate_hz", default_value="25.0"),
            DeclareLaunchArgument(
                "fault_battery_topic",
                default_value="safety_test/battery",
                description="Battery topic used by fault injector (must match safety monitor profile).",
            ),
            DeclareLaunchArgument(
                "fault_gps_status_topic",
                default_value="safety_test/gps_status",
                description="GPS topic used by fault injector (must match safety monitor profile).",
            ),
            DeclareLaunchArgument(
                "fault_estimated_state_topic",
                default_value="safety_test/estimated_state",
                description="State topic used by fault injector (must match safety monitor profile).",
            ),
            base_launch,
            safety_regression_demo,
            takeoff_hold_demo,
            fault_injector,
        ]
    )
