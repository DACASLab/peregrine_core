std_msgs/Header header

uint8 control_mode
uint8 MODE_TRAJECTORY = 0
uint8 MODE_BODY_RATE = 1
uint8 MODE_ATTITUDE = 2
uint8 MODE_DIRECT_ACTUATOR = 3

# Trajectory control fields (ENU/FLU side).
bool use_position
bool use_velocity
bool use_acceleration
bool use_yaw
bool use_yaw_rate
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float32 yaw
float32 yaw_rate

# Attitude / rate control fields (FLU body frame).
geometry_msgs/Quaternion attitude
geometry_msgs/Vector3 body_rates
float32 thrust

# Direct normalized actuator commands in [-1, 1].
float32[] motor_commands
