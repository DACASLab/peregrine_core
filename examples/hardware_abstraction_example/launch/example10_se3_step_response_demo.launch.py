"""@file
@brief Example 10 SE3 launch: PX4 SITL + minimal SE(3) step-response demo.

This launch mirrors the SE(3) circle/figure-8 bringup path, but swaps the demo
node for a step-response mission runner and optional step-response evaluator.
It is intended for classical tuning using rise time, overshoot, settling, and
steady-state error metrics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    start_step_evaluator = LaunchConfiguration("start_step_evaluator")
    evaluator_output_path = LaunchConfiguration("evaluator_output_path")
    mission_config_file = LaunchConfiguration("mission_config_file")
    manager_overrides_file = LaunchConfiguration("manager_overrides_file")

    start_px4_sitl = LaunchConfiguration("start_px4_sitl")
    clean_before_start = LaunchConfiguration("clean_before_start")
    sitl_start_delay_s = LaunchConfiguration("sitl_start_delay_s")
    stack_start_delay_s = LaunchConfiguration("stack_start_delay_s")

    px4_autopilot_dir = LaunchConfiguration("px4_autopilot_dir")
    px4_gz_world = LaunchConfiguration("px4_gz_world")
    headless = LaunchConfiguration("headless")

    px4_param_uxrce_dds_ptcfg = LaunchConfiguration("px4_param_uxrce_dds_ptcfg")
    px4_param_sys_failure_en = LaunchConfiguration("px4_param_sys_failure_en")
    px4_param_sim_gps_used = LaunchConfiguration("px4_param_sim_gps_used")

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
    default_mission_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction_example"), "config", "step_response_se3_mission.yaml"]
    )
    default_se3_managers_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction_example"), "config", "example10_se3_managers.yaml"]
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
                parameters=[estimation_yaml, manager_overrides_file],
            ),
            ComposableNode(
                package="control_manager",
                plugin="control_manager::ControlManagerNode",
                name="control_manager",
                namespace=uav_namespace,
                parameters=[control_yaml, manager_overrides_file],
            ),
            ComposableNode(
                package="trajectory_manager",
                plugin="trajectory_manager::TrajectoryManagerNode",
                name="trajectory_manager",
                namespace=uav_namespace,
                parameters=[trajectory_yaml, manager_overrides_file],
            ),
            ComposableNode(
                package="uav_manager",
                plugin="uav_manager::UavManagerNode",
                name="uav_manager",
                namespace=uav_namespace,
                parameters=[uav_yaml, manager_overrides_file],
            ),
        ],
    )

    demo_node = Node(
        package="hardware_abstraction_example",
        executable="step_response_demo.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[mission_config_file],
    )

    evaluator_node = Node(
        package="hardware_abstraction_example",
        executable="step_response_evaluator.py",
        namespace=uav_namespace,
        output="screen",
        parameters=[mission_config_file, {"output_path": evaluator_output_path}],
        condition=IfCondition(start_step_evaluator),
    )

    cleanup = ExecuteProcess(
        condition=IfCondition(clean_before_start),
        cmd=[
            "bash",
            "-lc",
            (
                "pkill -f '[p]x4.*gz_x500' || true; "
                "pkill -f '[m]ake px4_sitl' || true; "
                "pkill -f '[/]PX4-Autopilot/.*/bin/px4' || true; "
                "pkill -f '[M]icroXRCEAgent' || true; "
                "pkill -f '[g]z sim' || true; "
                "pkill -f '[c]omponent_container_mt' || true; "
                "sleep 1"
            ),
        ],
        output="screen",
        name="bringup_clean_start",
    )

    px4_sitl = ExecuteProcess(
        condition=IfCondition(start_px4_sitl),
        cmd=[
            "bash",
            "-lc",
            [
                "cd ",
                px4_autopilot_dir,
                " && export PX4_GZ_WORLD=",
                px4_gz_world,
                " && ROS_DOMAIN_ID=",
                ros_domain_id,
                " ROS_LOCALHOST_ONLY=",
                ros_localhost_only,
                " HEADLESS=",
                headless,
                " PX4_PARAM_UXRCE_DDS_PTCFG=",
                px4_param_uxrce_dds_ptcfg,
                " PX4_PARAM_UXRCE_DDS_SYNCT=0"
                " PX4_PARAM_SYS_FAILURE_EN=",
                px4_param_sys_failure_en,
                " PX4_PARAM_SIM_GPS_USED=",
                px4_param_sim_gps_used,
                " make px4_sitl gz_x500",
            ],
        ],
        output="screen",
        name="px4_sitl",
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
                "start_step_evaluator",
                default_value="false",
                description="Start the step-response evaluator node and write metrics to evaluator_output_path.",
            ),
            DeclareLaunchArgument(
                "evaluator_output_path",
                default_value="/tmp/step_response_eval.json",
                description="Output path for step-response metrics.",
            ),
            DeclareLaunchArgument(
                "mission_config_file",
                default_value=default_mission_yaml,
                description="Mission config YAML to load for the step-response demo node.",
            ),
            DeclareLaunchArgument(
                "manager_overrides_file",
                default_value=default_se3_managers_yaml,
                description="Manager override YAML loaded after package defaults.",
            ),
            DeclareLaunchArgument(
                "start_px4_sitl",
                default_value="true",
                description="Start PX4 SITL + Gazebo from this launch.",
            ),
            DeclareLaunchArgument(
                "clean_before_start",
                default_value="true",
                description="Kill stale PX4/GZ/XRCE/container processes before launch.",
            ),
            DeclareLaunchArgument("sitl_start_delay_s", default_value="1.0"),
            DeclareLaunchArgument("stack_start_delay_s", default_value="9.0"),
            DeclareLaunchArgument(
                "px4_autopilot_dir",
                default_value="/opt/PX4-Autopilot",
                description="Path to PX4-Autopilot checkout.",
            ),
            DeclareLaunchArgument("px4_gz_world", default_value="default"),
            DeclareLaunchArgument(
                "headless",
                default_value="1",
                description="Set HEADLESS for PX4 Gazebo launch (1=headless).",
            ),
            DeclareLaunchArgument("px4_param_uxrce_dds_ptcfg", default_value="1"),
            DeclareLaunchArgument("px4_param_sys_failure_en", default_value="1"),
            DeclareLaunchArgument("px4_param_sim_gps_used", default_value="10"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            cleanup,
            TimerAction(
                period=sitl_start_delay_s,
                actions=[px4_sitl],
                condition=IfCondition(start_px4_sitl),
            ),
            TimerAction(
                period=stack_start_delay_s,
                actions=[base_launch, manager_container, demo_node, evaluator_node],
                condition=IfCondition(start_px4_sitl),
            ),
            TimerAction(
                period=0.0,
                actions=[base_launch, manager_container, demo_node, evaluator_node],
                condition=UnlessCondition(start_px4_sitl),
            ),
        ]
    )
