std_msgs/Header header

uint8 state
uint8 STATE_IDLE = 0
uint8 STATE_ARMED = 1
uint8 STATE_TAKING_OFF = 2
uint8 STATE_HOVERING = 3
uint8 STATE_FLYING = 4
uint8 STATE_LANDING = 5
uint8 STATE_LANDED = 6
uint8 STATE_EMERGENCY = 7

bool armed
bool offboard
bool connected
bool failsafe

string mode
string detail
