# Trajectory setpoint message.
# The setpoints are in the frame of reference provided in the header.
# Setting a value to NaN indicates that state should not be controlled.

std_msgs/Header header

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

float32 yaw
float32 yaw_speed