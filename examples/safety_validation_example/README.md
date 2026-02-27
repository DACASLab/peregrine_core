# safety_validation_example

Fresh-start safety regression for PX4 SITL + Gazebo using live telemetry faults.

## What It Runs

`safety_fresh_sitl_regression.launch.py` does the following:

1. Kills stale PX4/GZ/XRCE/ROS safety processes.
2. Starts fresh PX4 SITL (`gz_x500`).
3. Starts Example 12 safety regression (`safety_regression_demo.py`) with:
   - Battery fault via PX4 `VEHICLE_CMD_INJECT_FAILURE`.
   - GPS fault via runtime `px4-param set SIM_GPS_USED`.
   - Geofence fault via real `go_to` motion beyond radius.

## Build

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hardware_abstraction_example safety_validation_example --symlink-install
source install/setup.bash
```

## Run (Full Regression)

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
mkdir -p /tmp/ros_logs
export ROS_LOG_DIR=/tmp/ros_logs

ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  timeout 1500s ros2 launch safety_validation_example safety_fresh_sitl_regression.launch.py \
  > /tmp/safety_native_all.log 2>&1
```

## Run (Single Case)

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  ros2 launch safety_validation_example safety_fresh_sitl_regression.launch.py \
  regression_cases:=gps_pre_takeoff_gate
```

Valid `regression_cases` values:

- `battery_post_takeoff_auto_land`
- `gps_pre_takeoff_gate`
- `gps_post_takeoff_auto_land`
- `geofence_post_takeoff_auto_land`
- `all`

## PX4 DDS Param Checks

With SITL running, verify DDS settings:

```bash
/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-param show UXRCE_DDS_PTCFG
/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-param show UXRCE_DDS_DOM_ID
```

Expected:

- `UXRCE_DDS_PTCFG : 1`
- `UXRCE_DDS_DOM_ID : 42` (if `ROS_DOMAIN_ID=42` at SITL launch)

## Log Checks

```bash
rg -n "CASE START|CASE PASS|CASE FAIL|Safety regression summary|\[PASS\]|\[FAIL\]" /tmp/safety_native_all.log
```

## Latest Observed Outcome (2026-02-26)

Command used:

```bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  timeout 1500s ros2 launch safety_validation_example safety_fresh_sitl_regression.launch.py \
  > /tmp/safety_native_all.log 2>&1
```

Summary lines from log:

- `[FAIL] battery_post_takeoff_auto_land: supervisor_not_recoverable_after_battery_case`
- `[PASS] gps_pre_takeoff_gate: takeoff blocked under preflight GPS fault`
- `[FAIL] gps_post_takeoff_auto_land: takeoff_failed_before_gps_post_fault`
- `[FAIL] geofence_post_takeoff_auto_land: takeoff_failed_before_geofence_fault`

Interpretation:

- PX4-native battery failure and GPS satellite fault injection are working.
- Main blocker is supervisor recovery: `uav_manager` stays latched in `EMERGENCY` after the first critical event, so later takeoff-dependent cases cannot start in the same run.

Single-case sanity run (same day):

- `regression_cases:=gps_pre_takeoff_gate` passed end-to-end (`/tmp/safety_native_gps_pre.log`).
