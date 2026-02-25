"""@file
@brief Example 8 launch: single-UAV PX4 SITL end-to-end validation.

Launches the minimal "data-plane" layer for one UAV:
  1. MicroXRCEAgent  -- UDP bridge between PX4 and the ROS2 DDS domain
  2. PX4HardwareAbstraction  -- translates PX4 uORB messages to peregrine
                                ROS2 interfaces (NED->ENU, FRD->FLU, etc.)
  3. FrameTransformer  -- publishes TF frames (map->odom->base_link) from
                          the ENU odometry output of hardware_abstraction

Both composable nodes are loaded into a single component_container_mt
(MultiThreadedExecutor container). This is required because PX4's uXRCE-DDS
topics use best-effort QoS, which needs the DDS middleware to be in the same
process for reliable intra-process transport.

Key environment variables:
  ROS_LOCALHOST_ONLY=1  -- restricts DDS discovery to localhost; must match
                           PX4's UXRCE_DDS_PTCFG=1 setting in SITL
  ROS_DOMAIN_ID=42      -- isolates this ROS2 domain from other sessions;
                           must match PX4's uXRCE-DDS client domain

Usage (standalone, without managers):
  ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py

This launch file is also included by example10 and example11 as a base layer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    """@brief Brings up MicroXRCE agent and a composed bridge/TF container for one UAV."""
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "uav_namespace",
                default_value="",
                description="ROS namespace for the UAV stack (empty for single-UAV global topics).",
            ),
            DeclareLaunchArgument(
                "px4_namespace",
                default_value="",
                description="Namespace prefix for PX4 /fmu topics.",
            ),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value="",
                description="TF frame prefix used by frame_transformer.",
            ),
            DeclareLaunchArgument(
                "sensor_gps_topic_suffix",
                default_value="/fmu/out/sensor_gps",
                description="PX4 SensorGps topic suffix (for compatibility with different PX4 topic aliases).",
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
                "offboard_rate_hz",
                default_value="20.0",
                description="OffboardControlMode publication rate.",
            ),
            DeclareLaunchArgument(
                "status_rate_hz",
                default_value="5.0",
                description="PX4 status publication rate.",
            ),
            DeclareLaunchArgument(
                "tf_publish_rate_hz",
                default_value="100.0",
                description="Dynamic TF publication rate.",
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
            # MicroXRCEAgent bridges PX4's uORB messages over UDP to the ROS2 DDS domain.
            # In SITL, PX4 starts its own XRCE-DDS client that connects to this agent.
            # The agent must be running BEFORE PX4 boots for reliable initial discovery.
            ExecuteProcess(
                condition=IfCondition(start_microxrce_agent),
                cmd=["MicroXRCEAgent", "udp4", "-p", microxrce_port],
                output="screen",
                name="microxrce_agent",
            ),
            # component_container_mt uses a MultiThreadedExecutor, allowing concurrent
            # callback execution across the loaded composable nodes. This is necessary
            # for hardware_abstraction, which has both timer-driven publishers and
            # subscription callbacks that must not block each other.
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
                        parameters=[
                            {
                                "px4_namespace": px4_namespace,
                                "sensor_gps_topic_suffix": sensor_gps_topic_suffix,
                                "offboard_rate_hz": offboard_rate_hz,
                                "status_rate_hz": status_rate_hz,
                            }
                        ],
                    ),
                    ComposableNode(
                        package="frame_transforms",
                        plugin="frame_transforms::FrameTransformer",
                        name="frame_transformer",
                        namespace=uav_namespace,
                        parameters=[
                            {
                                "frame_prefix": frame_prefix,
                                "odometry_topic": "odometry",
                                "publish_rate_hz": tf_publish_rate_hz,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
