from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    tree_file = LaunchConfiguration("tree_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    uav_namespace = LaunchConfiguration("uav_namespace")

    default_tree = PathJoinSubstitution(
        [FindPackageShare("mission_executor"), "trees", "example_mission.xml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("tree_file", default_value=default_tree),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("uav_namespace", default_value=""),
        Node(
            package="mission_executor",
            executable="mission_executor_node",
            namespace=uav_namespace,
            output="screen",
            parameters=[{"tree_file": tree_file, "use_sim_time": use_sim_time}],
        ),
    ])
