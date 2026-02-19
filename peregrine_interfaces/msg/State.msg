std_msgs/Header header

# Pose and twist are expressed in ROS ENU/FLU conventions.
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

# Optional linear acceleration estimate in ENU.
geometry_msgs/Vector3 linear_acceleration

# Metadata for estimator provenance.
string source
float32 confidence
