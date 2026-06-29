"""@file
@brief Multi-UAV PX4 SITL launcher for shared Gazebo simulation.

Spawns N PX4 SITL instances in one shared Gazebo world, each with its own
MicroXRCE-DDS Agent. Does NOT launch the peregrine stack — each UAV container
does that separately via core_stack.launch.py.

Usage:
  ros2 launch peregrine_bringup multi_uav_sitl.launch.py num_uavs:=2
"""

import math
import platform
import re
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)

SITL_HOME_LAT = "13.015581"
SITL_HOME_LON = "77.561373"


def generate_launch_description() -> LaunchDescription:
    """@brief Launch multi-UAV SITL (PX4 instances + XRCE agents, no stack)."""
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
                "uav_spawn_spacing_m",
                default_value="5.0",
                description="Grid spacing between UAV spawn points (meters).",
            ),
            DeclareLaunchArgument(
                "px4_base_delay",
                default_value="8.0",
                description="Seconds before launching first PX4 instance.",
            ),
            DeclareLaunchArgument(
                "px4_stagger_delay",
                default_value="5.0",
                description="Additional seconds between consecutive PX4 instances.",
            ),
            DeclareLaunchArgument(
                "xrce_offset",
                default_value="2.0",
                description="Seconds after each PX4 launch before starting its XRCE agent.",
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

    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld


def _launch_setup(context, *args, **kwargs):
    """Generate Gazebo + PX4 SITL + XRCE agent actions for each UAV instance."""
    n = int(context.launch_configurations["num_uavs"])
    px4_dir = context.launch_configurations["px4_autopilot_dir"]
    gz_world = context.launch_configurations["px4_gz_world"]
    spacing_m = float(context.launch_configurations["uav_spawn_spacing_m"])
    base_delay = float(context.launch_configurations["px4_base_delay"])
    stagger_delay = float(context.launch_configurations["px4_stagger_delay"])
    xrce_offset = float(context.launch_configurations["xrce_offset"])
    headless_val = context.launch_configurations["headless"]
    clean = context.launch_configurations["clean_before_start"]

    actions = []

    # Optional cleanup of stale processes.
    if clean.lower() in ("true", "1", "yes"):
        actions.append(
            ExecuteProcess(
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
                name="multi_sitl_cleanup",
            )
        )

    # ── Patch Gazebo world GPS home ────────────────────────────────────
    gz_world_path = f"{px4_dir}/Tools/simulation/gz/worlds/{gz_world}.sdf"
    sdf = Path(gz_world_path)
    if sdf.exists():
        text = sdf.read_text(encoding="utf-8")
        text = re.sub(r"<latitude_deg>[^<]+</latitude_deg>",
                       f"<latitude_deg>{SITL_HOME_LAT}</latitude_deg>", text)
        text = re.sub(r"<longitude_deg>[^<]+</longitude_deg>",
                       f"<longitude_deg>{SITL_HOME_LON}</longitude_deg>", text)
        sdf.write_text(text, encoding="utf-8")

    # ── Shared Gazebo world ──────────────────────────────────────────────
    gz_model_path = f"{px4_dir}/Tools/simulation/gz/models"
    gz_server_config = f"{px4_dir}/Tools/simulation/gz/server.config"
    gz_build_dir = f"{px4_dir}/build/px4_sitl_default/build_gz"

    arch = platform.machine()  # e.g. x86_64, aarch64
    gz_physics_path = f"/usr/lib/{arch}-linux-gnu/gz-physics-7/engine-plugins"

    is_headless = headless_val.lower() in ("true", "1", "yes")
    gz_flags = "-v2 -r -s" if is_headless else "-v2 -r"
    gz_cmd = (
        f"GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:{gz_model_path} "
        f"GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:{gz_build_dir} "
        f"GZ_SIM_PHYSICS_ENGINE_PATH=$GZ_SIM_PHYSICS_ENGINE_PATH:{gz_physics_path} "
        f"GZ_SIM_SERVER_CONFIG_PATH={gz_server_config} "
        f"gz sim {gz_flags} {gz_world_path}"
    )
    actions.append(
        TimerAction(
            period=1.0,
            actions=[
                LogInfo(msg=f"[multi_uav_sitl] Launching shared Gazebo world: {gz_world}"),
                ExecuteProcess(
                    cmd=["bash", "-lc", gz_cmd],
                    output="screen",
                    name="gz_sim_world",
                ),
            ],
        )
    )

    # ── Per-instance PX4 SITL + XRCE agent ───────────────────────────────
    grid_cols = math.ceil(math.sqrt(n))
    grid_rows = math.ceil(n / grid_cols)
    x_center = 0.5 * (grid_cols - 1)
    y_center = 0.5 * (grid_rows - 1)

    for i in range(n):
        domain_id = i + 1              # UAV 1 → domain 1, UAV 2 → domain 2
        xrce_port = 8888 + 2 * i       # 8888, 8890, 8892, ...
        col = i % grid_cols
        row = i // grid_cols
        x_pos = (col - x_center) * spacing_m
        y_pos = (row - y_center) * spacing_m
        delay = base_delay + i * stagger_delay

        env_vars = (
            f"PX4_GZ_STANDALONE=1 "
            f"PX4_SYS_AUTOSTART=4001 "
            f"PX4_GZ_MODEL_POSE='{x_pos},{y_pos},0,0,0,0' "
            f"PX4_GZ_WORLD={gz_world} "
            f"HEADLESS={headless_val} "
            f"ROS_DOMAIN_ID={domain_id} "
            f"ROS_LOCALHOST_ONLY=1 "
            f"PX4_SIM_MODEL=gz_x500 "
            f"PX4_PARAM_UXRCE_DDS_PTCFG=1 "
            f"PX4_PARAM_UXRCE_DDS_DOM_ID={domain_id} "
            f"PX4_PARAM_SYS_FAILURE_EN=1 "
            f"PX4_PARAM_SIM_GPS_USED=10 "
            f"PX4_PARAM_UXRCE_DDS_SYNCT=0"
        )

        # Multi-vehicle SITL requires per-instance runtime isolation and IDs.
        # We launch the PX4 binary from rootfs/instance_i with `-i i` so each
        # vehicle has separate state/params/logs while all instances connect to
        # one shared Gazebo server (PX4_GZ_STANDALONE=1), instead of each
        # `make px4_sitl gz_x500` invocation starting its own Gazebo process.
        px4_cmd = (
            f"cd {px4_dir}/build/px4_sitl_default/rootfs && "
            f"mkdir -p instance_{i} && "
            f"cd instance_{i} && "
            f"{env_vars} "
            f"../../bin/px4 "
            f"-i {i} "
            f"-d ../../etc"
        )

        log_action = LogInfo(
            msg=f"[multi_uav_sitl] Launching PX4 instance {i}: "
            f"domain_id={domain_id}, xrce_port={xrce_port}, "
            f"pose=({x_pos},{y_pos},0)"
        )

        # Stagger PX4 instances; XRCE agent starts xrce_offset seconds later.
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    log_action,
                    ExecuteProcess(
                        cmd=["bash", "-lc", px4_cmd],
                        output="screen",
                        name=f"px4_sitl_{i}",
                    ),
                ],
            )
        )
        actions.append(
            TimerAction(
                period=delay + xrce_offset,
                actions=[
                    ExecuteProcess(
                        cmd=["MicroXRCEAgent", "udp4", "-p", str(xrce_port)],
                        output="screen",
                        name=f"microxrce_agent_{i}",
                    ),
                ],
            )
        )

    return actions
