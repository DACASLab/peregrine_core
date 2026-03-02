"""@file
@brief Single-UAV PX4 SITL + Gazebo + PEREGRINE bringup.

This launch file:
  1) Optionally cleans stale simulator/bridge/container processes.
  2) Starts PX4 SITL against Gazebo.
  3) Launches the single-UAV composable stack.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _make_stack_include(context_vars):
    """Create an IncludeLaunchDescription for single_uav.launch.py."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("peregrine_bringup"),
                    "launch",
                    "single_uav.launch.py",
                ]
            )
        ),
        launch_arguments={
            "uav_namespace": context_vars["uav_namespace"],
            "start_microxrce_agent": context_vars["start_microxrce_agent"],
            "microxrce_port": context_vars["microxrce_port"],
            "ros_localhost_only": context_vars["ros_localhost_only"],
            "ros_domain_id": context_vars["ros_domain_id"],
            "px4_namespace": context_vars["px4_namespace"],
            "target_system_id": context_vars["target_system_id"],
            "use_sim_time": context_vars["use_sim_time"],
            "safety_params_file": context_vars["safety_params_file"],
            "uav_params_file": context_vars["uav_params_file"],
            "config_overrides": context_vars["config_overrides"],
        }.items(),
    )


def generate_launch_description() -> LaunchDescription:
    """@brief Launch single-UAV SITL and stack."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    px4_namespace = LaunchConfiguration("px4_namespace")
    target_system_id = LaunchConfiguration("target_system_id")
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    safety_params_file = LaunchConfiguration("safety_params_file")
    uav_params_file = LaunchConfiguration("uav_params_file")
    config_overrides = LaunchConfiguration("config_overrides")

    simulation_overrides = PathJoinSubstitution(
        [FindPackageShare("peregrine_bringup"), "config", "simulation.yaml"]
    )

    # Dict passed to _make_stack_include to avoid sharing a single
    # IncludeLaunchDescription object between two TimerAction branches.
    stack_vars = {
        "uav_namespace": uav_namespace,
        "start_microxrce_agent": start_microxrce_agent,
        "microxrce_port": microxrce_port,
        "ros_localhost_only": ros_localhost_only,
        "ros_domain_id": ros_domain_id,
        "px4_namespace": px4_namespace,
        "target_system_id": target_system_id,
        "use_sim_time": use_sim_time,
        "safety_params_file": safety_params_file,
        "uav_params_file": uav_params_file,
        "config_overrides": config_overrides,
    }

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
            DeclareLaunchArgument(
                "start_microxrce_agent",
                default_value="true",
                description="Start MicroXRCEAgent from single_uav.launch.py",
            ),
            DeclareLaunchArgument("microxrce_port", default_value="8888"),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Set ROS_LOCALHOST_ONLY (use 1 with PX4 UXRCE_DDS_PTCFG=1).",
            ),
            DeclareLaunchArgument("ros_domain_id", default_value="0"),
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
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo /clock as time source (default true for SITL).",
            ),
            DeclareLaunchArgument("px4_param_uxrce_dds_ptcfg", default_value="1"),
            DeclareLaunchArgument("px4_param_sys_failure_en", default_value="1"),
            DeclareLaunchArgument("px4_param_sim_gps_used", default_value="10"),
            DeclareLaunchArgument(
                "safety_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("safety_monitor"), "config", "defaults.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "uav_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("uav_manager"), "config", "defaults.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "config_overrides",
                default_value=simulation_overrides,
                description="Bringup override YAML applied after per-package defaults.",
            ),
            cleanup,
            TimerAction(
                period=sitl_start_delay_s,
                actions=[px4_sitl],
                condition=IfCondition(start_px4_sitl),
            ),
            # When running SITL, delay stack start to let PX4+Gazebo initialize.
            # When not running SITL (external PX4), start immediately.
            TimerAction(
                period=stack_start_delay_s,
                actions=[_make_stack_include(stack_vars)],
                condition=IfCondition(start_px4_sitl),
            ),
            TimerAction(
                period=0.0,
                actions=[_make_stack_include(stack_vars)],
                condition=UnlessCondition(start_px4_sitl),
            ),
        ]
    )
