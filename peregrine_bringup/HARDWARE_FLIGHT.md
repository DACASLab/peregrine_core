# Hardware Flight Rundown

## 1. One-Time Setup (per drone)

On the companion computer, create `docker/.env.local`:

```bash
DRONE_ID=1
ZENOH_PORT=7447
# XRCE_DEVICE=/dev/ttyTHS1   # Jetson default, uncomment to override
```

PX4 flight controller params (via QGroundControl or pxh):

```
param set UXRCE_DDS_PTCFG 1
param set UXRCE_DDS_DOM_ID 1    # Must match DRONE_ID
param set UXRCE_DDS_SYNCT 0
param save
reboot
```

Update `peregrine_bringup/config/default.yaml` with your exact test field GPS coordinates.

## 2. Start the Stack

```bash
# Option A: manual
cd docker && make jetson   # or make rpi5

# Option B: auto on boot
make install-service PLATFORM=jetson
make enable-flight
```

The container starts and the entrypoint runs:
1. MicroXRCE agent connects to PX4 over UART
2. Zenoh bridge starts on port 7447
3. Tmuxinator opens two panes:

| Pane | What's running |
|---|---|
| **stack** | `ros2 launch peregrine_bringup core_stack.launch.py start_microxrce_agent:=false` |
| **shell** | Empty bash — run missions from here |

## 3. Run a Mission (from the shell pane)

```bash
# Circle + figure-8 (conservative hardware defaults: 3m alt, 1.5m radius, slow)
ros2 launch peregrine_bringup mission_circle_figure8.launch.py

# Circle only
ros2 launch peregrine_bringup mission_circle_figure8.launch.py mission_type:=circle

# Figure-8 only
ros2 launch peregrine_bringup mission_circle_figure8.launch.py mission_type:=figure8

# Override params inline
ros2 launch peregrine_bringup mission_circle_figure8.launch.py \
  mission_type:=circle takeoff_altitude_m:=5.0 circle_radius_m:=2.0
```

The mission script waits for `uav_state.dependencies_ready`, then executes:
**arm -> offboard -> takeoff -> trajectory -> land -> disarm**

### Hardware mission defaults (`config/hardware_mission.yaml`)

| Parameter | Hardware | SITL |
|---|---|---|
| takeoff_altitude_m | 3.0 | 5.0 |
| circle_radius_m | 1.5 | 2.0 |
| angular_velocity_radps | 0.4 | 0.5 |
| climb_velocity_mps | 0.5 | 1.0 |
| descent_velocity_mps | 0.5 | 0.8 |

### Manual action goals (alternative to launch file)

```bash
# Takeoff
ros2 action send_goal /uav_manager/takeoff peregrine_interfaces/action/Takeoff \
  "{target_altitude_m: 3.0, climb_velocity_mps: 0.5}"

# Circle (radius, angular_vel, num_loops)
ros2 action send_goal /uav_manager/execute_trajectory \
  peregrine_interfaces/action/ExecuteTrajectory \
  "{trajectory_type: 'circle', params: [1.5, 0.4, 1.0]}"

# Figure-8 (radius, angular_vel, num_loops)
ros2 action send_goal /uav_manager/execute_trajectory \
  peregrine_interfaces/action/ExecuteTrajectory \
  "{trajectory_type: 'figure8', params: [1.5, 0.4, 1.0]}"

# Land
ros2 action send_goal /uav_manager/land peregrine_interfaces/action/Land \
  "{descent_velocity_mps: 0.5}"
```

## 4. GCS Monitoring (from your laptop)

### Setup

Create `docker/.env.local` on the GCS laptop:

```bash
GCS_MODE=hardware
GCS_UAV_IPS=192.168.1.10   # companion computer IP (comma-separated for multi)
```

### Start

```bash
cd docker
make gcs
```

Attach to the tmux session:

```bash
make tmux-gcs
```

### GCS tmux windows

| Window | Contents |
|---|---|
| **ground_station** | One TUI pane per UAV — ncurses dashboard |
| **rviz** | Actual path, reference path, vehicle markers, geofence |
| **topics** | `ros2 topic list` for quick diagnostics |

Navigate windows: `Ctrl+b` then `n`/`p` or window number (`0`, `1`, `2`).

### TUI shows (per UAV)

- Connection status (PX4 link up/down)
- Battery percentage + voltage
- FSM state (Idle -> Armed -> TakingOff -> Hovering -> Flying -> Landing -> Landed)
- GPS fix quality (satellites, HDOP)
- Safety level (NOMINAL / WARNING / CRITICAL / EMERGENCY)
- All manager health (estimation, control, trajectory)
- Alert history (state transitions, warnings)

## 5. Monitoring from the Drone (shell pane)

```bash
# Vehicle state
ros2 topic echo /estimated_state

# Control output (thrust + body rates)
ros2 topic echo /control_output

# Safety status
ros2 topic echo /safety_status

# UAV FSM state
ros2 topic echo /uav_state

# Manager health
ros2 topic echo /control_status
```

## 6. Network Architecture

```
Companion Computer (domain 1)          GCS Laptop (domain 99)
+----------------------------+         +----------------------+
| PX4 <-> XRCE Agent (UART) |         |                      |
| Peregrine Stack            |         |  TUI (ncurses)       |
| Zenoh Bridge :7447  -------+--wifi-->|  Zenoh Bridge :7459  |
|                            |         |  RViz                |
+----------------------------+         +----------------------+
```

All ROS traffic stays local on each machine (`ROS_LOCALHOST_ONLY=1`).
Only the Zenoh bridges talk over the network, selectively forwarding
status and telemetry topics.

## 7. Safety Notes

- Battery auto-lands at 15%, emergency at 10%
- Geofence: 500m radius, 120m altitude (configurable in `safety_monitor/config/defaults.yaml`)
- PX4 exits offboard if heartbeat lost for >500ms (reverts to position hold)
- Always have RC transmitter with kill switch as backup
- Start with conservative params and increase gradually
