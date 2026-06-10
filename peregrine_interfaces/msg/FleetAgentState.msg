# Lightweight inter-UAV broadcast for multi-agent collision avoidance.
#
# Each UAV's multi_agent_coordinator publishes this for its peers (over the Zenoh
# fleet bus) and consumes its neighbors'. Everything is expressed in the SHARED
# fleet frame (header.frame_id, normally "map") so neighbors need no cross-agent
# TF: the owner resolves its own pose/velocity into the fleet frame before sending.
std_msgs/Header header          # stamp = sample time; frame_id = fleet frame (e.g. "map")

string uav_id                   # e.g. "uav1" (namespace without leading slash)

geometry_msgs/Point position    # fleet-frame ENU position (x, y, z)
geometry_msgs/Vector3 velocity  # fleet-frame ENU velocity (for forward prediction)
float64 yaw                     # ENU heading (0 = East, CCW positive)

# Mirrors peregrine_interfaces/UAVState.state (IDLE/ARMED/.../FLYING/...). Lets
# neighbors reason about intent (e.g. ignore agents that are landed/idle).
uint8 mode

geometry_msgs/Point goal        # current setpoint target in fleet frame (intent; optional)

bool avoidance_active           # true while this agent is actively applying BVC
