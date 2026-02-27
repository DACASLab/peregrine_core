std_msgs/Header header

bool connected
bool armed
bool offboard
bool failsafe

uint8 nav_state
uint8 arming_state
uint16 failure_detector_status

float32 battery_remaining
float32 battery_voltage

# Normalized motor outputs (0.0 to 1.0), 4 motors for quadrotor
float32[4] motor_output
