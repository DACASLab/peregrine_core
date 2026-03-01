"""@file
@brief Multi-UAV PX4 SITL launcher for shared Gazebo simulation.

Spawns N PX4 SITL instances in one shared Gazebo world, each with its own
MicroXRCE-DDS Agent. Does NOT launch the peregrine stack — each UAV container
does that separately via single_uav.launch.py.

Usage:
  ros2 launch peregrine_bringup multi_uav_sitl.launch.py num_uavs:=2
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """@brief Launch multi-UAV SITL (PX4 instances + XRCE agents, no stack)."""
    num_uavs = LaunchConfiguration("num_uavs")
    px4_autopilot_dir = LaunchConfiguration("px4_autopilot_dir")
    px4_gz_world = LaunchConfiguration("px4_gz_world")
    headless = LaunchConfiguration("headless")
    clean_before_start = LaunchConfiguration("clean_before_start")

    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_uavs",
                default_value="2",
                description="Number of PX4 SITL instances to spawn.",
            ),
            DeclareLaunchArgument(
                "px4_autopilot_dir",
                default_value="/opt/PX4-Autopilot",
                description="Path to PX4-Autopilot checkout.",
            ),
            DeclareLaunchArgument(
                "px4_gz_world",
                default_value="default",
                description="Gazebo world to load.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="0",
                description="Run Gazebo headless (1=headless, 0=GUI).",
            ),
            DeclareLaunchArgument(
                "clean_before_start",
                default_value="true",
                description="Kill stale PX4/GZ/XRCE processes before launch.",
            ),
        ]
    )

    # We generate actions for a fixed max (hardcoded to 2 for now, matching
    # the plan). LaunchConfiguration doesn't support runtime loops, so we
    # use OpaqueFunction for dynamic instance count.
    _generate_instances(ld)
    return ld


def _generate_instances(ld: LaunchDescription) -> None:
    """Generate PX4 SITL + XRCE agent actions for each UAV instance."""
    from launch.actions import OpaqueFunction

    def _launch_setup(context, *args, **kwargs):
        n = int(context.launch_configurations["num_uavs"])
        px4_dir = context.launch_configurations["px4_autopilot_dir"]
        gz_world = context.launch_configurations["px4_gz_world"]
        headless_val = context.launch_configurations["headless"]
        clean = context.launch_configurations["clean_before_start"]

        actions = []

        # Optional cleanup
        if clean.lower() in ("true", "1", "yes"):
            actions.append(
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-lc",
                        (
                            "pkill -f '[p]x4_sitl_default/bin/px4' || true; "
                            "pkill -f '[M]icroXRCEAgent' || true; "
                            "pkill -f '[g]z sim' || true; "
                            "sleep 1"
                        ),
                    ],
                    output="screen",
                    name="multi_sitl_cleanup",
                )
            )

        for i in range(n):
            instance_id = i
            domain_id = i + 1  # UAV 1 → domain 1, UAV 2 → domain 2
            xrce_port = 8888 + 2 * i  # 8888, 8890, 8892, ...
            x_pos = i * 3  # 0m, 3m, 6m, ...
            delay = 2.0 + i * 5.0  # stagger: 2s, 7s, 12s, ...

            # For instance 0, we let PX4 spawn Gazebo (GZ_MODEL_POSE creates
            # the world). For instance 1+, we set PX4_GZ_MODEL_NAME to avoid
            # spawning a second Gazebo server.
            #
            # PX4 multi-vehicle uses instance index (-i) to offset ports.
            # Instance 0: SITL UDP 14540, MAVLink 14550, XRCE 8888
            # Instance 1: SITL UDP 14541, MAVLink 14551, XRCE 8890

            env_vars = (
                f"PX4_SYS_AUTOSTART=4001 "
                f"PX4_GZ_MODEL_POSE='{x_pos},0,0,0,0,0' "
                f"PX4_GZ_WORLD={gz_world} "
                f"HEADLESS={headless_val} "
                f"ROS_DOMAIN_ID={domain_id} "
                f"ROS_LOCALHOST_ONLY=1 "
                f"PX4_PARAM_UXRCE_DDS_PTCFG=1 "
                f"PX4_PARAM_UXRCE_DDS_DOM_ID={domain_id} "
                f"PX4_PARAM_SYS_FAILURE_EN=1 "
                f"PX4_PARAM_SIM_GPS_USED=10"
            )

            # First instance spawns the Gazebo world; subsequent instances
            # attach to the existing Gazebo server.
            if i == 0:
                px4_cmd = (
                    f"cd {px4_dir} && "
                    f"{env_vars} "
                    f"./build/px4_sitl_default/bin/px4 "
                    f"-i {instance_id} "
                    f"-d ./build/px4_sitl_default/etc "
                    f"-s etc/init.d-posix/rcS "
                    f"-w sitl_instance_{instance_id}"
                )
            else:
                # Skip world creation for subsequent instances
                px4_cmd = (
                    f"cd {px4_dir} && "
                    f"{env_vars} "
                    f"PX4_GZ_MODEL=x500 "
                    f"./build/px4_sitl_default/bin/px4 "
                    f"-i {instance_id} "
                    f"-d ./build/px4_sitl_default/etc "
                    f"-s etc/init.d-posix/rcS "
                    f"-w sitl_instance_{instance_id}"
                )

            px4_action = ExecuteProcess(
                cmd=["bash", "-lc", px4_cmd],
                output="screen",
                name=f"px4_sitl_{instance_id}",
            )

            xrce_action = ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", str(xrce_port)],
                output="screen",
                name=f"microxrce_agent_{instance_id}",
            )

            log_action = LogInfo(
                msg=f"[multi_uav_sitl] Launching PX4 instance {instance_id}: "
                f"domain_id={domain_id}, xrce_port={xrce_port}, "
                f"pose=({x_pos},0,0)"
            )

            # Stagger PX4 instances; XRCE agent starts 2s after its PX4
            actions.append(
                TimerAction(
                    period=delay,
                    actions=[log_action, px4_action],
                )
            )
            actions.append(
                TimerAction(
                    period=delay + 2.0,
                    actions=[xrce_action],
                )
            )

        return actions

    ld.add_action(OpaqueFunction(function=_launch_setup))
