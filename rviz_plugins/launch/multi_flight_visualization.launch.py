"""Launch one flight visualizer node per UAV and a shared RViz2 instance."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_uav_namespace(prefix: str, uav_index: int) -> str:
    normalized = prefix.strip().strip("/")
    if not normalized:
        return ""
    return f"/{normalized}{uav_index}"


def _launch_setup(context, *args, **kwargs):
    num_uavs = int(context.launch_configurations["num_uavs"])
    if num_uavs < 1:
        raise RuntimeError("num_uavs must be >= 1")

    namespace_prefix = context.launch_configurations["uav_namespace_prefix"]
    fixed_frame = context.launch_configurations["fixed_frame"]
    use_sim_time_raw = context.launch_configurations["use_sim_time"]
    use_sim_time = use_sim_time_raw.strip().lower() in ("1", "true", "yes", "on")

    start_visualizers = LaunchConfiguration("start_visualizers")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    defaults_yaml = PathJoinSubstitution(
        [FindPackageShare("rviz_plugins"), "config", "defaults.yaml"]
    )

    actions = []
    for i in range(num_uavs):
        uav_id = i + 1
        uav_namespace = _resolve_uav_namespace(namespace_prefix, uav_id)
        actions.append(
            Node(
                condition=IfCondition(start_visualizers),
                package="rviz_plugins",
                executable="flight_visualizer_node",
                name=f"flight_visualizer_uav{uav_id}",
                output="screen",
                parameters=[
                    defaults_yaml,
                    {
                        "uav_namespace": uav_namespace,
                        "fixed_frame": fixed_frame,
                        "use_sim_time": use_sim_time,
                    },
                ],
            )
        )

    actions.append(
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("rviz_plugins"), "rviz", "flight_visualization.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_uavs",
                default_value="2",
                description="Number of UAV visualizer nodes to launch.",
            ),
            DeclareLaunchArgument(
                "uav_namespace_prefix",
                default_value="uav",
                description="Namespace prefix; UAV i uses /<prefix><i>.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="map",
                description="Fixed frame used by visualizer overlays.",
            ),
            DeclareLaunchArgument(
                "start_visualizers",
                default_value="true",
                description="Start per-UAV flight visualizer nodes.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config file path.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
