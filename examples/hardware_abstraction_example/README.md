# hardware_abstraction_example

Standalone launch package for validating the single-UAV hardware abstraction
and frame transform pipeline against PX4 SITL.

## Launch file

`ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py`

## What it starts

- `hardware_abstraction_node`
- `frame_transformer_node`
- Optional `MicroXRCEAgent` process (enabled by default)

## Suggested usage

1. Terminal A (PX4 SITL):
   `cd /opt/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500`
2. Terminal B (ROS stack):
   `cd /ros2_ws && source install/setup.bash && ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py`
3. Terminal C (checks):
   - `ros2 topic echo --once /status`
   - `ros2 topic hz /odometry`
   - `ros2 run tf2_ros tf2_echo odom base_link`
