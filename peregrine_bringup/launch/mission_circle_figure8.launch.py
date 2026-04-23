"""Launch single UAV stack plus mission_executor BT mission."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    uav_namespace = LaunchConfiguration("uav_namespace")
    tree_file = LaunchConfiguration("tree_file")

    single_uav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("peregrine_bringup"), "launch", "single_uav.launch.py"])
        ),
        launch_arguments={
            "uav_namespace": uav_namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mission_executor"), "launch", "mission_executor.launch.py"])
        ),
        launch_arguments={
            "uav_namespace": uav_namespace,
            "use_sim_time": use_sim_time,
            "tree_file": tree_file,
        }.items(),
    )

    default_tree = PathJoinSubstitution([FindPackageShare("mission_executor"), "trees", "example_mission.xml"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("uav_namespace", default_value=""),
        DeclareLaunchArgument("tree_file", default_value=default_tree),
        single_uav,
        mission,
    ])
