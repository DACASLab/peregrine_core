# Desired trajectory setpoint for the controller.
#
# Frame conventions:
#   header.frame_id indicates the reference frame for position/velocity/acceleration:
#     - "odom" (or "<prefix>/odom"): world-frame setpoints (default)
#     - "base_link" (or "<prefix>/base_link"): body-frame velocity commands
#       (position fields should not be used in body frame)
#   Yaw and yaw_rate always use ENU convention (0 = East, CCW positive),
#   regardless of frame_id.
std_msgs/Header header

bool use_position
bool use_velocity
bool use_acceleration
bool use_yaw
bool use_yaw_rate

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

float64 yaw
float64 yaw_rate
