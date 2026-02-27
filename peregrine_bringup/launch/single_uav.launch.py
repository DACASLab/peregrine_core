"""@file
@brief Single-UAV PEREGRINE bringup in one composable container.

Launches hardware_abstraction, frame_transforms, estimation_manager,
control_manager, trajectory_manager, safety_monitor, and uav_manager inside a
single `component_container_mt` process.

This launch expects PX4 + MicroXRCE connectivity to be available, unless
`start_microxrce_agent:=true` is used.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Launch full single-UAV stack (no SITL process management)."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    safety_params_file = LaunchConfiguration("safety_params_file")
    uav_params_file = LaunchConfiguration("uav_params_file")
    config_overrides = LaunchConfiguration("config_overrides")
    start_visualizer = LaunchConfiguration("start_visualizer")
    start_rviz = LaunchConfiguration("start_rviz")
    fixed_frame = LaunchConfiguration("fixed_frame")
    rviz_config = LaunchConfiguration("rviz_config")

    bringup_default_overrides = PathJoinSubstitution(
        [FindPackageShare("peregrine_bringup"), "config", "default.yaml"]
    )

    hardware_yaml = PathJoinSubstitution(
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
    safety_yaml = PathJoinSubstitution(
        [FindPackageShare("safety_monitor"), "config", "defaults.yaml"]
    )
    uav_yaml = PathJoinSubstitution(
        [FindPackageShare("uav_manager"), "config", "defaults.yaml"]
    )
    rviz_yaml = PathJoinSubstitution(
        [FindPackageShare("rviz_plugins"), "config", "defaults.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("rviz_plugins"), "rviz", "flight_visualization.rviz"]
    )

    frame_defaults = {
        "frame_prefix": "",
        "odometry_topic": "odometry",
        "publish_rate_hz": 100.0,
        "home_lat_deg": 47.397742,
        "home_lon_deg": 8.545594,
        "gps_min_fix_type": 3,
        "gps_min_satellites": 6,
        "gps_max_hdop": 5.0,
        "gps_max_vdop": 5.0,
        "gps_freshness_timeout_s": 2.0,
        "home_init_timeout_s": 60.0,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "uav_namespace",
                default_value="",
                description="ROS namespace for this UAV stack.",
            ),
            DeclareLaunchArgument(
                "start_microxrce_agent",
                default_value="true",
                description="Start MicroXRCEAgent in this launch.",
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
                description="ROS_DOMAIN_ID for this stack.",
            ),
            DeclareLaunchArgument(
                "safety_params_file",
                default_value=safety_yaml,
                description="Safety monitor parameter profile.",
            ),
            DeclareLaunchArgument(
                "uav_params_file",
                default_value=uav_yaml,
                description="UAV manager parameter profile.",
            ),
            DeclareLaunchArgument(
                "config_overrides",
                default_value=bringup_default_overrides,
                description="Optional bringup override YAML (applied after per-package defaults).",
            ),
            DeclareLaunchArgument(
                "start_visualizer",
                default_value="false",
                description="Start rviz_plugins flight visualizer node.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
                description="Start RViz2 with flight visualization config.",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="map",
                description="Fixed frame for RViz and visualization fallback.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file to load when start_rviz:=true.",
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
                        parameters=[hardware_yaml, config_overrides],
                    ),
                    ComposableNode(
                        package="frame_transforms",
                        plugin="frame_transforms::FrameTransformer",
                        name="frame_transformer",
                        namespace=uav_namespace,
                        parameters=[frame_defaults, config_overrides],
                    ),
                    ComposableNode(
                        package="estimation_manager",
                        plugin="estimation_manager::EstimationManagerNode",
                        name="estimation_manager",
                        namespace=uav_namespace,
                        parameters=[estimation_yaml, config_overrides],
                    ),
                    ComposableNode(
                        package="control_manager",
                        plugin="control_manager::ControlManagerNode",
                        name="control_manager",
                        namespace=uav_namespace,
                        parameters=[control_yaml, config_overrides],
                    ),
                    ComposableNode(
                        package="trajectory_manager",
                        plugin="trajectory_manager::TrajectoryManagerNode",
                        name="trajectory_manager",
                        namespace=uav_namespace,
                        parameters=[trajectory_yaml, config_overrides],
                    ),
                    ComposableNode(
                        package="safety_monitor",
                        plugin="safety_monitor::SafetyMonitorNode",
                        name="safety_monitor",
                        namespace=uav_namespace,
                        parameters=[safety_params_file, config_overrides],
                    ),
                    ComposableNode(
                        package="uav_manager",
                        plugin="uav_manager::UavManagerNode",
                        name="uav_manager",
                        namespace=uav_namespace,
                        parameters=[uav_params_file, config_overrides],
                    ),
                ],
            ),
            Node(
                condition=IfCondition(start_visualizer),
                package="rviz_plugins",
                executable="flight_visualizer_node",
                name="flight_visualizer",
                output="screen",
                parameters=[
                    rviz_yaml,
                    {
                        "uav_namespace": uav_namespace,
                        "fixed_frame": fixed_frame,
                        "use_sim_time": True,
                    },
                ],
            ),
            Node(
                condition=IfCondition(start_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                namespace=uav_namespace,
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
