# Anchors the per-UAV ROS frame tree to PX4's estimator references.
# Published by hardware_abstraction from VehicleLocalPosition; consumed by
# frame_transforms (TF) and any consumer that needs the world<->PX4-local relationship.
# See docs/investigations/world-frame-anchoring-plan.md.

std_msgs/Header header

# map -> odom transform = INVERSE of the accumulated PX4 EKF local-estimate jumps.
# Steps only on a *_reset_counter change; constant otherwise. Keeps map/world pose
# continuous when the raw PX4 odom jumps. (map_to_odom_enu = -sum(nedToEnu(delta_xy,delta_z)))
geometry_msgs/Vector3 map_to_odom_enu
float64 map_to_odom_yaw                   # ENU yaw of map->odom = -sum(delta_heading)

# Global placement of the EKF local origin (world -> map source). Latched once from the
# first valid sample; subsequent origin moves are carried by map_to_odom (not by re-latching).
float64 ref_lat                           # deg
float64 ref_lon                           # deg
float64 ref_alt                           # AMSL meters
bool ref_valid                            # xy_global && z_global
uint64 ref_timestamp                      # PX4 ref_timestamp (origin change marker)

# Vertical ground datum so "height above ground" is meaningful. estimator-frame Z taken as
# ground: captured at arm/first-valid, or from a downward range finder (dist_bottom).
float64 ground_datum_z
bool ground_datum_valid
