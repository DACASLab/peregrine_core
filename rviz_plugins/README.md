# rviz_plugins

RViz visualization utilities for the PEREGRINE stack.

## What it provides

- `flight_visualizer_node`: publishes RViz-native geometry
  - `viz/actual_path` (`nav_msgs/Path`) from `estimated_state`
  - `viz/reference_path` (`nav_msgs/Path`) from `trajectory_setpoint`
  - `viz/markers` (`visualization_msgs/MarkerArray`) for:
    - vehicle body and heading
    - active reference setpoint and velocity vector
    - safety text overlay
    - geofence volume (cylindrical envelope)
- `launch/flight_visualization.launch.py`: starts visualizer and optional `rviz2`
- `rviz/flight_visualization.rviz`: default RViz display profile

## Build

```bash
colcon build --packages-select rviz_plugins
```

## Run (visualizer + RViz)

```bash
ros2 launch rviz_plugins flight_visualization.launch.py
```

## Run against namespaced UAV topics

```bash
ros2 launch rviz_plugins flight_visualization.launch.py uav_namespace:=uav1
```

## Notes

- TF already comes from `frame_transforms`; RViz `TF` display should be enabled with `Fixed Frame: map`.
- RViz config uses relative topics (`viz/...`). When `uav_namespace:=uav1`, launch runs RViz in that namespace so displays resolve to `/uav1/viz/...`.
- Geofence visualization is parameterized in this node. Keep the geofence params aligned with `safety_monitor` config for accurate overlays.
