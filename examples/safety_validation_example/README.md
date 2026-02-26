# safety_validation_example

Dedicated safety-regression example package with fresh SITL startup.

## What This Launch Does

`safety_fresh_sitl_regression.launch.py` performs a clean run by default:

1. kills stale PX4/GZ/XRCE/ROS safety processes
2. starts fresh PX4 SITL (`gz_x500`) + Gazebo
3. starts the ROS stack + safety regression pipeline (via Example 12)

## Usage

Build:

```bash
cd /ros2_ws
colcon build --packages-select safety_validation_example hardware_abstraction_example --symlink-install
source install/setup.bash
```

Run (fresh SITL + full safety regression):

```bash
ros2 launch safety_validation_example safety_fresh_sitl_regression.launch.py
```

If PX4 SITL is already running externally:

```bash
ros2 launch safety_validation_example safety_fresh_sitl_regression.launch.py \
  start_px4_sitl:=false stack_start_delay_s:=0.0
```

## Notes

- This launch assumes PX4 source is at `/opt/PX4-Autopilot` (override via `px4_autopilot_dir`).
- For localhost-only transport, PX4 must use `UXRCE_DDS_PTCFG=1`.
- ROS and PX4 domain are aligned via `ros_domain_id` (default `42`).
