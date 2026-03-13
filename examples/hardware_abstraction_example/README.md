# hardware_abstraction_example

Standalone launch package for validating the single-UAV hardware abstraction
and frame transform pipeline against PX4 SITL.

## Launch file

`ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py`

## Circle / Figure-8 Demo Launch

`ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py`

## Circle / Figure-8 Demo Launch (SE3)

`ros2 launch hardware_abstraction_example example10_se3_circle_figure8_demo.launch.py`

This SE(3) variant starts PX4 SITL itself and uses the minimal tuning path:

- `example8` bridge stack (`MicroXRCEAgent`, `px4_hardware_abstraction`, `frame_transformer`)
- SE(3) manager chain (`estimation_manager`, `control_manager`, `trajectory_manager`, `uav_manager`)
- `circle_figure8_demo.py`

It intentionally does not include `safety_monitor`.

Accepted SITL tuning values for the default x500 validation path:

- `se3.mass=2.0643`
- `se3.max_thrust_N=28.0`
- `se3.k_p=[8.0, 8.0, 8.0]`
- `se3.k_v=[5.0, 5.0, 5.0]`
- `se3.k_i=[0.2, 0.2, 0.6]`
- `se3.k_R=[5.0, 5.0, 2.0]`
- `se3.k_omega=[0.9, 0.9, 0.4]`
- `se3.k_Ri=[0.05, 0.05, 0.03]`
- `post_ready_wait_s=5.0`

Trajectory improvements in this path:

- circle and figure-8 publish acceleration feedforward
- circle publishes yaw-rate feedforward
- figure-8 publishes yaw-rate from path curvature

Validated outcome:

- takeoff
- circle
- figure-8
- land

Measured circle-only tracking at `2.0 m` radius and `0.6 rad/s`:

- accepted tuning: `rms_xy ~= 0.31 m`, `max_xy ~= 0.59 m`

## Multi-Cycle Demo Launch

`ros2 launch hardware_abstraction_example example11_multi_cycle_demo.launch.py`

## Safety Validation Launch

`ros2 launch hardware_abstraction_example example12_safety_validation.launch.py`

## Focused Monitoring Launch

`ros2 launch hardware_abstraction_example example13_monitoring_demo.launch.py`

## Multi-UAV Container Mission Launch

`ros2 launch hardware_abstraction_example example14_multi_uav_circle_figure8.launch.py`

Use this with the multi-container stack already running.
It launches one mission client per UAV domain:

- UAVi mission: takeoff -> circle -> figure-8 -> land
- `num_uavs:=N` launches N clients on domains `base_domain_id..base_domain_id+N-1`
- Default namespace prefix is `uav` (`/uav1`, `/uav2`, ...)
- Default altitude split: `base_takeoff_altitude_m:=4.0`, `takeoff_altitude_step_m:=1.5`
- Large-pattern defaults in `multi_uav_circle_figure8_mission.yaml`:
  - `circle_radius_m=5.0`
  - `figure8_radius_m=5.0`

Example for 4 UAVs:

```bash
ros2 launch hardware_abstraction_example example14_multi_uav_circle_figure8.launch.py \
  num_uavs:=4 base_domain_id:=1
```

## 9-UAV Multi-Container Mission Launch

`ros2 launch hardware_abstraction_example example15_multi_uav_circle_figure8_9uav.launch.py`

This wraps Example 14 with 9-UAV defaults:

- `num_uavs:=9`
- `base_domain_id:=1` (domains 1..9)
- `inter_uav_start_delay_s:=1.0`
- `base_takeoff_altitude_m:=4.0`
- `takeoff_altitude_step_m:=1.0`

Example:

```bash
ros2 launch hardware_abstraction_example example15_multi_uav_circle_figure8_9uav.launch.py
```

## In-Flight Controller Switch Demo Launch

`ros2 launch hardware_abstraction_example example17_controller_switch_inflight_demo.launch.py`

Purpose:

- Take off with passthrough controller.
- Switch to SE3 while airborne and fly a trajectory.
- Switch back to passthrough in-flight and fly another trajectory.
- Optional third switch back to SE3, then land.

This is a targeted diagnosis flow to separate in-flight switch behavior from
SE3 takeoff behavior.

This wraps Example 11 (multi-cycle mission) and adds optional monitoring:

- `start_flight_visualizer:=true|false` (default `true`)
- `start_rviz:=true|false` (default `true`)
- `start_tui:=true|false` (default `false`)

Common usage:

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  ros2 launch hardware_abstraction_example example13_monitoring_demo.launch.py
```

Enable TUI from this launch if desired:

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  ros2 launch hardware_abstraction_example example13_monitoring_demo.launch.py start_tui:=true
```

Default behavior:

- full single-container stack (`peregrine_single_container.launch.py`)
- `safety_regression_demo.py` (multi-case sequence)
- safety monitor + regression node on live stack topics (`battery`, `gps_status`, `estimated_state`)
- `safety_params_file:=.../safety_regression.yaml`
- `uav_params_file:=.../uav_require_external_safety.yaml`

Regression sequence:

- battery critical after takeoff -> expect safety-triggered land/disarm
- GPS critical before takeoff -> expect takeoff blocked/rejected
- GPS critical after takeoff -> expect safety-triggered land/disarm
- geofence critical after takeoff -> expect safety-triggered land/disarm

Useful overrides:

- diagnostics only (no auto-land command from safety monitor):
  - `safety_params_file:=/ros2_ws/src/peregrine_core/examples/hardware_abstraction_example/config/safety_diag_only.yaml`
- disable regression runner:
  - `start_safety_regression_demo:=false`
- keep old manual helpers disabled by default:
  - `start_takeoff_hold_demo:=false`
  - `start_fault_injector:=false`
- geofence breach target used by regression:
  - `regression_geofence_breach_x_m:=40.0`
- GPS fault strength used by regression:
  - `regression_gps_fault_satellites:=2`
- runtime PX4 param tool path used by regression:
  - `regression_px4_param_tool:=/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-param`

Fault scenarios (`safety_fault_injector.py`, optional helper):

- `none`
- `gps_fix_critical`, `gps_sats_critical`, `gps_hdop_warning`, `gps_vdop_warning`, `gps_missing_warning`
- `battery_warning`, `battery_critical`, `battery_emergency`, `battery_low_voltage_emergency`, `battery_missing_warning`
- `geofence_radius_critical`, `geofence_alt_high_critical`, `geofence_alt_low_warning`
- `envelope_speed_critical`, `envelope_tilt_warning`

This launch starts:

- `peregrine_container` (single `component_container_mt` process)
  - `px4_hardware_abstraction`
  - `frame_transformer`
  - `estimation_manager`
  - `control_manager`
  - `trajectory_manager`
  - `safety_monitor`
  - `uav_manager`
- `safety_regression_demo.py` (enabled by default)
- optional `safety_takeoff_hold_demo.py` and `safety_fault_injector.py` manual helpers

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

### 6) Launch Example 12 (multi-safety regression)

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
mkdir -p /tmp/ros_logs
export ROS_LOG_DIR=/tmp/ros_logs
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  timeout 1200s ros2 launch hardware_abstraction_example example12_safety_validation.launch.py \
  > /tmp/example12_safety.log 2>&1
```

### 7) Quick result checks

```bash
rg -n "Data readiness satisfied|Lifecycle bringup completed successfully|Demo mission completed successfully" /tmp/example10_redo.log
rg -n "Data readiness satisfied|Lifecycle bringup completed successfully|Multi-cycle mission completed successfully|cycle4_figure8_land completed" /tmp/example11_redo2.log
rg -n "CASE PASS|CASE FAIL|Safety regression summary|battery_post_takeoff_auto_land|gps_pre_takeoff_gate|gps_post_takeoff_auto_land|geofence_post_takeoff_auto_land" /tmp/example12_safety.log
```

## PX4 uXRCE-DDS Notes

- SITL startup script maps `ROS_DOMAIN_ID` to `UXRCE_DDS_DOM_ID` at boot.
- If `ROS_DOMAIN_ID` is not exported when SITL starts, rcS will set `UXRCE_DDS_DOM_ID` to `0`.
- PX4 SITL does not auto-map `ROS_LOCALHOST_ONLY`; set once and save:
  - `param set UXRCE_DDS_PTCFG 1`
  - `param save`
- Restart SITL after any uXRCE param changes.
