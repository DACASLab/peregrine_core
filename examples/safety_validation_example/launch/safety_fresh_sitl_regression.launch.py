"""@file
@brief Fresh-start safety regression launcher.

This wrapper launch is intentionally dedicated to safety testing runs. It:
  1) Cleans up stale PX4/GZ/XRCE/ROS safety processes.
  2) Starts a fresh PX4 SITL + Gazebo instance.
  3) Starts the full ROS stack and safety regression runner (example12 pipeline).

Use this when repeated runs from stale simulator state were causing flaky behavior.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Launch a clean SITL-backed safety regression run."""
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    start_px4_sitl = LaunchConfiguration("start_px4_sitl")
    clean_before_start = LaunchConfiguration("clean_before_start")
    sitl_start_delay_s = LaunchConfiguration("sitl_start_delay_s")
    stack_start_delay_s = LaunchConfiguration("stack_start_delay_s")
    px4_autopilot_dir = LaunchConfiguration("px4_autopilot_dir")
    px4_gz_world = LaunchConfiguration("px4_gz_world")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    uav_namespace = LaunchConfiguration("uav_namespace")
    regression_cases = LaunchConfiguration("regression_cases")
    regression_gps_fault_satellites = LaunchConfiguration("regression_gps_fault_satellites")
    regression_px4_param_tool = LaunchConfiguration("regression_px4_param_tool")
    px4_param_uxrce_dds_ptcfg = LaunchConfiguration("px4_param_uxrce_dds_ptcfg")
    px4_param_sys_failure_en = LaunchConfiguration("px4_param_sys_failure_en")
    px4_param_sim_gps_used = LaunchConfiguration("px4_param_sim_gps_used")

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
                "pkill -f '[s]afety_regression_demo.py' || true; "
                "pkill -f '[s]afety_fault_injector.py' || true; "
                "pkill -f '[s]afety_takeoff_hold_demo.py' || true; "
                "sleep 1"
            ),
        ],
        output="screen",
        name="safety_clean_start",
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
                " PX4_PARAM_UXRCE_DDS_PTCFG=",
                px4_param_uxrce_dds_ptcfg,
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

    example12_regression = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("hardware_abstraction_example"),
                    "launch",
                    "example12_safety_validation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "uav_namespace": uav_namespace,
            "ros_domain_id": ros_domain_id,
            "ros_localhost_only": ros_localhost_only,
            "start_microxrce_agent": start_microxrce_agent,
            "microxrce_port": microxrce_port,
            "start_safety_regression_demo": "true",
            "start_takeoff_hold_demo": "false",
            "start_fault_injector": "false",
            "regression_cases": regression_cases,
            "regression_gps_fault_satellites": regression_gps_fault_satellites,
            "regression_px4_param_tool": regression_px4_param_tool,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_namespace", default_value=""),
            DeclareLaunchArgument(
                "px4_autopilot_dir",
                default_value="/opt/PX4-Autopilot",
                description="PX4-Autopilot repo path used to start SITL.",
            ),
            DeclareLaunchArgument(
                "px4_gz_world",
                default_value="default",
                description="Gazebo world for PX4 SITL.",
            ),
            DeclareLaunchArgument(
                "start_px4_sitl",
                default_value="true",
                description="Start a fresh PX4 SITL + Gazebo process from this launch.",
            ),
            DeclareLaunchArgument(
                "clean_before_start",
                default_value="true",
                description="Kill stale PX4/GZ/XRCE/ROS safety processes before launch.",
            ),
            DeclareLaunchArgument(
                "sitl_start_delay_s",
                default_value="1.0",
                description="Delay before SITL launch (lets cleanup finish).",
            ),
            DeclareLaunchArgument(
                "stack_start_delay_s",
                default_value="9.0",
                description="Delay before starting ROS stack when this launch starts SITL.",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description="Set ROS_LOCALHOST_ONLY (use 1 with PX4 UXRCE_DDS_PTCFG=1).",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="42",
                description="ROS domain for both PX4 uXRCE and ROS 2 stack.",
            ),
            DeclareLaunchArgument("start_microxrce_agent", default_value="true"),
            DeclareLaunchArgument("microxrce_port", default_value="8888"),
            DeclareLaunchArgument(
                "px4_param_uxrce_dds_ptcfg",
                default_value="1",
                description="Value exported as PX4_PARAM_UXRCE_DDS_PTCFG for SITL startup.",
            ),
            DeclareLaunchArgument(
                "px4_param_sys_failure_en",
                default_value="1",
                description="Value exported as PX4_PARAM_SYS_FAILURE_EN for SITL startup.",
            ),
            DeclareLaunchArgument(
                "px4_param_sim_gps_used",
                default_value="10",
                description="Nominal SIM_GPS_USED exported at SITL startup.",
            ),
            DeclareLaunchArgument(
                "regression_cases",
                default_value="all",
                description=(
                    "Comma-separated safety case names (or 'all') forwarded to "
                    "safety_regression_demo."
                ),
            ),
            DeclareLaunchArgument(
                "regression_gps_fault_satellites",
                default_value="2",
                description="SIM_GPS_USED value used by regression node for GPS fault cases.",
            ),
            DeclareLaunchArgument(
                "regression_px4_param_tool",
                default_value="/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-param",
                description="Path forwarded to safety_regression_demo for runtime PX4 param updates.",
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", ros_localhost_only),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            cleanup,
            TimerAction(
                period=sitl_start_delay_s,
                actions=[px4_sitl],
            ),
            TimerAction(
                period=stack_start_delay_s,
                actions=[example12_regression],
                condition=IfCondition(start_px4_sitl),
            ),
            TimerAction(
                period=0.0,
                actions=[example12_regression],
                condition=UnlessCondition(start_px4_sitl),
            ),
        ]
    )
