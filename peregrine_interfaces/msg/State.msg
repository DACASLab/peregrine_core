# Fused vehicle state estimate.
#
# Frame conventions:
#   header.frame_id = odom frame (e.g. "odom" or "<prefix>/odom").
#   Pose (position + orientation) is expressed in the odom frame (ENU/FLU).
#   Twist (linear + angular velocity) is in the body frame (base_link/FLU),
#     consistent with nav_msgs/Odometry convention.
#   linear_acceleration is in the ENU world frame.
std_msgs/Header header

# Pose is in odom frame (ENU). Twist is in body frame (base_link/FLU).
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

# Optional linear acceleration estimate in ENU.
geometry_msgs/Vector3 linear_acceleration

# Metadata for estimator provenance.
string source
float32 confidence
