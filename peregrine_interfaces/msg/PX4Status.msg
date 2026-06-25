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

# Position-estimate validity, mirrored from PX4's EKF (vehicle_local_position) so that the
# stack gates on the SAME signal PX4 uses to allow OFFBOARD. PX4 refuses an OFFBOARD switch
# whose OffboardControlMode requests position control while local_position_invalid is set
# (see PX4 commander offboardCheck.cpp / estimatorCheck.cpp). Raw GPS quality (gps_status)
# is an upstream proxy and does NOT match this gate, so we surface the EKF view directly.
#
# local_position_valid mirrors PX4: xy_valid && eph < threshold && fresh (with hysteresis).
# This is the field the takeoff pre-arm gate and the TUI position badge consume.
bool local_position_valid
bool xy_valid              # raw EKF flag: horizontal position estimate usable
bool z_valid              # raw EKF flag: vertical position estimate usable
bool v_xy_valid           # raw EKF flag: horizontal velocity estimate usable
bool dead_reckoning       # true when position is propagated without absolute aiding (GPS lost)
float32 eph               # std-dev of horizontal position error (m); NaN if unknown
float32 evh               # std-dev of horizontal velocity error (m/s); NaN if unknown
