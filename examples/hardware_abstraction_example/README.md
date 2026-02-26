# hardware_abstraction_example

Standalone launch package for validating the single-UAV hardware abstraction
and frame transform pipeline against PX4 SITL.

## Launch file

`ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py`

## Circle / Figure-8 Demo Launch

`ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py`

## Multi-Cycle Demo Launch

`ros2 launch hardware_abstraction_example example11_multi_cycle_demo.launch.py`

This launch starts:

- `bridge_container` (composed nodes)
  - `px4_hardware_abstraction`
  - `frame_transformer`
- `manager_container` (composed lifecycle nodes)
  - `estimation_manager`
  - `control_manager`
  - `trajectory_manager`
  - `uav_manager`
- `circle_figure8_demo.py` mission runner
- optional `lifecycle_bringup_orchestrator.py` (enabled by default)

Mission selection:

- `mission_type:=circle`
- `mission_type:=figure8`
- `mission_type:=circle_figure8` (default)
- `mission_type:=circle_land_figure8` (two full FSM cycles: takeoff/circle/land, then takeoff/figure8/land)

## What it starts

- `bridge_container` (executable: `component_container_mt`)
  - `px4_hardware_abstraction`
  - `frame_transformer`
- Optional `MicroXRCEAgent` process (enabled by default)

## Validated SITL Runbook (2026-02-26)

### 0) Clean stale processes

```bash
pkill -f "ros2 launch hardware_abstraction_example|component_container_mt|MicroXRCEAgent|/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4|gz sim" || true
```

### 1) Start PX4 SITL

```bash
cd /opt/PX4-Autopilot
ROS_DOMAIN_ID=42 HEADLESS=1 make px4_sitl gz_x500
```

### 2) Verify PX4 uXRCE params in `pxh`

```bash
param show UXRCE_DDS_DOM_ID
param show UXRCE_DDS_PTCFG
```

Expected values:

- `UXRCE_DDS_DOM_ID = 42`
- `UXRCE_DDS_PTCFG = 1`

### 3) Launch Example 8 (stack-only sanity check)

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py
```

### 4) Launch Example 10 (circle + figure-8)

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
mkdir -p /tmp/ros_logs
export ROS_LOG_DIR=/tmp/ros_logs
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  timeout 600s ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py \
  > /tmp/example10_redo.log 2>&1
```

### 5) Launch Example 11 (multi-cycle)

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
mkdir -p /tmp/ros_logs
export ROS_LOG_DIR=/tmp/ros_logs
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  timeout 900s ros2 launch hardware_abstraction_example example11_multi_cycle_demo.launch.py \
  > /tmp/example11_redo2.log 2>&1
```

### 6) Quick result checks

```bash
rg -n "Data readiness satisfied|Lifecycle bringup completed successfully|Demo mission completed successfully" /tmp/example10_redo.log
rg -n "Data readiness satisfied|Lifecycle bringup completed successfully|Multi-cycle mission completed successfully|cycle4_figure8_land completed" /tmp/example11_redo2.log
```

## PX4 uXRCE-DDS Notes

- SITL startup script maps `ROS_DOMAIN_ID` to `UXRCE_DDS_DOM_ID` at boot.
- If `ROS_DOMAIN_ID` is not exported when SITL starts, rcS will set `UXRCE_DDS_DOM_ID` to `0`.
- PX4 SITL does not auto-map `ROS_LOCALHOST_ONLY`; set once and save:
  - `param set UXRCE_DDS_PTCFG 1`
  - `param save`
- Restart SITL after any uXRCE param changes.
