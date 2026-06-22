"""@file
@brief Core PEREGRINE flight stack in one composable container.

Single source of truth for launching the peregrine node composition. All launch
files that need the flight stack should IncludeLaunchDescription this file rather
than defining their own ComposableNodeContainer.

Nodes launched (all composable, single process):
  hardware_abstraction, frame_transforms, estimation_manager, control_manager,
  trajectory_manager, safety_monitor, uav_manager, compute_monitor

Optional extras (via launch args):
  MicroXRCEAgent, Gazebo clock bridge, flight visualizer, RViz2
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Launch full single-UAV stack (no SITL process management)."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    px4_namespace = LaunchConfiguration("px4_namespace")
    target_system_id = LaunchConfiguration("target_system_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    safety_params_file = LaunchConfiguration("safety_params_file")
    uav_params_file = LaunchConfiguration("uav_params_file")
    config_overrides = LaunchConfiguration("config_overrides")
    start_visualizer = LaunchConfiguration("start_visualizer")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_avoidance = LaunchConfiguration("enable_avoidance")

    bringup_default_overrides = PathJoinSubstitution(
        [FindPackageShare("peregrine_bringup"), "config", "default.yaml"]
    )

    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("rviz_plugins"), "rviz", "flight_visualization.rviz"]
    )

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
                default_value="0",
                description="ROS_DOMAIN_ID for this stack.",
            ),
            DeclareLaunchArgument(
                "px4_namespace",
                default_value="",
                description="PX4 topic namespace prefix (e.g. /px4_1 for multi-instance SITL).",
            ),
            DeclareLaunchArgument(
                "target_system_id",
                default_value="1",
                description="MAV_SYS_ID of the PX4 instance this stack commands (instance N → N+1).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use Gazebo /clock as time source (set true for SITL).",
            ),
            DeclareLaunchArgument(
                "safety_params_file",
                default_value=bringup_default_overrides,
                description="Optional safety monitor parameter override YAML.",
            ),
            DeclareLaunchArgument(
                "uav_params_file",
                default_value=bringup_default_overrides,
                description="Optional UAV manager parameter override YAML.",
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
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file to load when start_rviz:=true.",
            ),
            DeclareLaunchArgument(
                "enable_avoidance",
                default_value="true",
                description="Enable inter-UAV BVC collision avoidance (multi-UAV only). "
                "On by default; set false for an avoidance-off baseline.",
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            ExecuteProcess(
                condition=IfCondition(start_microxrce_agent),
                cmd=["MicroXRCEAgent", "udp4", "-p", microxrce_port],
                output="screen",
                name="microxrce_agent",
            ),
            # Bridge Gazebo /clock → ROS 2 so all nodes share simulation time.
            # See: https://docs.px4.io/main/en/ros2/user_guide#ros-gazebo-and-px4-time-synchronization
            Node(
                condition=IfCondition(use_sim_time),
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_clock_bridge",
                output="screen",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
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
                        parameters=[config_overrides, {
                            "px4_namespace": px4_namespace,
                            "target_system_id": target_system_id,
                            "frame_prefix": uav_namespace,
                            "use_sim_time": use_sim_time,
                        }],
                    ),
                    ComposableNode(
                        package="frame_transforms",
                        plugin="frame_transforms::FrameTransformer",
                        name="frame_transformer",
                        namespace=uav_namespace,
                        parameters=[config_overrides, {
                            "frame_prefix": uav_namespace,
                            "use_sim_time": use_sim_time,
                        }],
                    ),
                    ComposableNode(
                        package="estimation_manager",
                        plugin="estimation_manager::EstimationManagerNode",
                        name="estimation_manager",
                        namespace=uav_namespace,
                        parameters=[config_overrides, {"use_sim_time": use_sim_time}],
                    ),
                    ComposableNode(
                        package="control_manager",
                        plugin="control_manager::ControlManagerNode",
                        name="control_manager",
                        namespace=uav_namespace,
                        parameters=[
                            config_overrides,
                            {"frame_prefix": uav_namespace, "use_sim_time": use_sim_time},
                        ],
                    ),
                    ComposableNode(
                        package="trajectory_manager",
                        plugin="trajectory_manager::TrajectoryManagerNode",
                        name="trajectory_manager",
                        namespace=uav_namespace,
                        # Output is intercepted by multi_agent_coordinator, which
                        # republishes the BVC-projected setpoint on trajectory_setpoint.
                        remappings=[("trajectory_setpoint", "trajectory_setpoint_raw")],
                        parameters=[config_overrides, {"use_sim_time": use_sim_time}],
                    ),
                    ComposableNode(
                        package="multi_agent_coordinator",
                        plugin="multi_agent_coordinator::MultiAgentCoordinatorNode",
                        name="multi_agent_coordinator",
                        namespace=uav_namespace,
                        parameters=[config_overrides, {
                            "uav_id": ParameterValue(uav_namespace, value_type=str),
                            "enabled": ParameterValue(enable_avoidance, value_type=bool),
                            "use_sim_time": use_sim_time,
                        }],
                    ),
                    ComposableNode(
                        package="safety_monitor",
                        plugin="safety_monitor::SafetyMonitorNode",
                        name="safety_monitor",
                        namespace=uav_namespace,
                        parameters=[safety_params_file, config_overrides, {"use_sim_time": use_sim_time}],
                    ),
                    ComposableNode(
                        package="uav_manager",
                        plugin="uav_manager::UavManagerNode",
                        name="uav_manager",
                        namespace=uav_namespace,
                        parameters=[uav_params_file, config_overrides, {"use_sim_time": use_sim_time}],
                    ),
                    ComposableNode(
                        package="compute_monitor",
                        plugin="compute_monitor::ComputeMonitorNode",
                        name="compute_monitor",
                        namespace=uav_namespace,
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
                    {
                        "uav_namespace": uav_namespace,
                        "fixed_frame": "map",
                        "use_sim_time": use_sim_time,
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
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
