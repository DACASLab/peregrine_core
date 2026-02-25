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

## Suggested usage

1. Terminal A (PX4 SITL):
   `cd /opt/PX4-Autopilot && ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=42 HEADLESS=1 make px4_sitl gz_x500`
2. Terminal B (ROS stack):
   `cd /ros2_ws && source install/setup.bash && ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py ros_localhost_only:=1 ros_domain_id:=42`
3. Terminal C (checks):
   - `ros2 topic echo --once /status`
   - `ros2 topic hz /odometry`
   - `ros2 run tf2_ros tf2_echo odom base_link`

For the autonomous trajectory demo, replace step 2 with:

`cd /ros2_ws && source install/setup.bash && ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py mission_type:=circle_figure8 ros_localhost_only:=1 ros_domain_id:=42`

For the FSM cycle validation requested here:

`cd /ros2_ws && source install/setup.bash && ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py mission_type:=circle_land_figure8 ros_localhost_only:=1 ros_domain_id:=42`

For repeated takeoff/land cycles in a dedicated example:

`cd /ros2_ws && source install/setup.bash && ros2 launch hardware_abstraction_example example11_multi_cycle_demo.launch.py multi_cycle_sequence:=circle,figure8,circle,figure8 ros_localhost_only:=1 ros_domain_id:=42`

## PX4 uXRCE-DDS Notes

- PX4 SITL startup already maps `ROS_DOMAIN_ID` to `UXRCE_DDS_DOM_ID`.
- PX4 SITL does not auto-map `ROS_LOCALHOST_ONLY`; set this once in PX4 shell and save:
  - `param set UXRCE_DDS_PTCFG 1`
  - `param save`
- Reboot/restart SITL after changing these PX4 parameters.
