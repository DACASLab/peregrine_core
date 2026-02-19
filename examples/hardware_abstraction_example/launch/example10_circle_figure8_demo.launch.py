"""Example 10: single-UAV manager-chain demo with circle/figure-eight trajectories."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
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
            DeclareLaunchArgument("ros_domain_id", default_value="0"),
            DeclareLaunchArgument(
                "mission_type",
                default_value="circle_figure8",
                description="Mission sequence: circle, figure8, circle_figure8, or circle_land_figure8.",
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
            Node(
                package="estimation_manager",
                executable="estimation_manager_node",
                namespace=uav_namespace,
                output="screen",
            ),
            Node(
                package="control_manager",
                executable="control_manager_node",
                namespace=uav_namespace,
                output="screen",
            ),
            Node(
                package="trajectory_manager",
                executable="trajectory_manager_node",
                namespace=uav_namespace,
                output="screen",
            ),
            Node(
                package="uav_manager",
                executable="uav_manager_node",
                namespace=uav_namespace,
                output="screen",
            ),
            Node(
                package="hardware_abstraction_example",
                executable="circle_figure8_demo.py",
                namespace=uav_namespace,
                output="screen",
                parameters=[
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
                ],
            ),
        ]
    )
