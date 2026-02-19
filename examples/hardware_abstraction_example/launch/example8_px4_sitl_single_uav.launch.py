"""Example 8: single-UAV PX4 SITL end-to-end validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
                "ros_domain_id",
                default_value="0",
                description="ROS domain used by this launch (must match PX4 uXRCE-DDS domain).",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            ExecuteProcess(
                condition=IfCondition(start_microxrce_agent),
                cmd=["MicroXRCEAgent", "udp4", "-p", microxrce_port],
                output="screen",
                name="microxrce_agent",
            ),
            Node(
                package="hardware_abstraction",
                executable="hardware_abstraction_node",
                namespace=uav_namespace,
                output="screen",
                parameters=[
                    {
                        "px4_namespace": px4_namespace,
                        "sensor_gps_topic_suffix": sensor_gps_topic_suffix,
                        "offboard_rate_hz": offboard_rate_hz,
                        "status_rate_hz": status_rate_hz,
                    }
                ],
            ),
            Node(
                package="frame_transforms",
                executable="frame_transformer_node",
                namespace=uav_namespace,
                output="screen",
                parameters=[
                    {
                        "frame_prefix": frame_prefix,
                        "odometry_topic": "odometry",
                        "publish_rate_hz": tf_publish_rate_hz,
                    }
                ],
            ),
        ]
    )
