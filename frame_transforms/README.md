# frame_transforms

Utilities for ROS ENU/FLU and PX4 NED/FRD conversions plus a TF broadcaster component.

## Library API

`include/frame_transforms/conversions.hpp` provides:

- ENU <-> NED position/velocity transforms.
- FLU <-> FRD body-frame transforms.
- Orientation and yaw conversion helpers.

## Component

`frame_transformer_node` subscribes to odometry and publishes:

- Static frames: `world -> map`, `map -> odom`, `base_link -> base_link_frd`.
- Dynamic frame: `odom -> base_link` from incoming odometry.
