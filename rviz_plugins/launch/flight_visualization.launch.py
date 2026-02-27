"""Launch RViz visualization node and optionally RViz2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    defaults_yaml = PathJoinSubstitution(
        [FindPackageShare("rviz_plugins"), "config", "defaults.yaml"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("rviz_plugins"), "rviz", "flight_visualization.rviz"]
    )

    uav_namespace = LaunchConfiguration("uav_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    fixed_frame = LaunchConfiguration("fixed_frame")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "uav_namespace",
                default_value="",
                description="UAV namespace to monitor.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock for simulation time.",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="map",
                description="RViz fixed frame and visualization fallback frame.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 alongside visualization node.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config path.",
            ),
            Node(
                package="rviz_plugins",
                executable="flight_visualizer_node",
                name="flight_visualizer",
                output="screen",
                parameters=[
                    defaults_yaml,
                    {
                        "uav_namespace": uav_namespace,
                        "fixed_frame": fixed_frame,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                namespace=uav_namespace,
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
