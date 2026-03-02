"""@file
@brief Example 8 launch: single-UAV PX4 SITL end-to-end validation.

Launches the minimal "data-plane" layer for one UAV:
  1. MicroXRCEAgent  -- UDP bridge between PX4 and the ROS2 DDS domain
  2. PX4HardwareAbstraction  -- translates PX4 uORB messages to peregrine
                                ROS2 interfaces (NED->ENU, FRD->FLU, etc.)
  3. FrameTransformer  -- publishes TF frames (map->odom->base_link) from
                          the ENU odometry output of hardware_abstraction

Both composable nodes are loaded into a single component_container_mt.
Parameters are loaded from YAML defaults; only structural/deployment args remain.

Usage (standalone, without managers):
  ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """@brief Brings up MicroXRCE agent and a composed bridge/TF container for one UAV."""
    uav_namespace = LaunchConfiguration("uav_namespace")
    start_microxrce_agent = LaunchConfiguration("start_microxrce_agent")
    microxrce_port = LaunchConfiguration("microxrce_port")
    ros_localhost_only = LaunchConfiguration("ros_localhost_only")
    ros_domain_id = LaunchConfiguration("ros_domain_id")

    hardware_abstraction_yaml = PathJoinSubstitution(
        [FindPackageShare("hardware_abstraction"), "config", "defaults.yaml"]
    )
    frame_yaml = PathJoinSubstitution(
        [FindPackageShare("frame_transforms"), "config", "defaults.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "uav_namespace",
                default_value="",
                description="ROS namespace for the UAV stack (empty for single-UAV global topics).",
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
                description="ROS domain used by this launch (must match PX4 uXRCE-DDS domain).",
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
                name="bridge_container",
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
                        parameters=[hardware_abstraction_yaml, {"frame_prefix": uav_namespace}],
                    ),
                    ComposableNode(
                        package="frame_transforms",
                        plugin="frame_transforms::FrameTransformer",
                        name="frame_transformer",
                        namespace=uav_namespace,
                        parameters=[frame_yaml],
                    ),
                ],
            ),
        ]
    )
