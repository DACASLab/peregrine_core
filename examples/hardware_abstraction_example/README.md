# hardware_abstraction_example

Standalone launch package for validating the single-UAV hardware abstraction
and frame transform pipeline against PX4 SITL.

## Launch file

`ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py`

## Circle / Figure-8 Demo Launch

`ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py`

## Multi-Cycle Demo Launch

`ros2 launch hardware_abstraction_example example11_multi_cycle_demo.launch.py`

## Safety Validation Launch

`ros2 launch hardware_abstraction_example example12_safety_validation.launch.py`

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
