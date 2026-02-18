# tui_status

**Package Type:** ROS2 Node Package  
**Dependencies:** peregrine_interfaces, rclcpp, ncurses  

---

## Overview

`tui_status` provides a **terminal-based user interface** for monitoring UAV status during development, testing, and flight operations. Uses ncurses for a rich, colored display.

---

## Responsibilities

1. **Real-time status display** - state, mode, position, attitude
2. **Health monitoring** - battery, connections, safety status
3. **Manager status** - active estimator, controller, trajectory
4. **Alert display** - warnings, errors with timestamps
5. **Multi-UAV view** - compact view of fleet status
6. **Color coding** - visual indication of status severity

---

## Display Layout

### Single UAV View

```
┌──────────────────────────────────────────────────────────────────────┐
│ PEREGRINE Flight Stack - UAV1 (Alpha)                      [HOVERING]   │
├────────────────────────────────────┬─────────────────────────────────┤
│ STATE                              │ POSITION (ENU)                  │
│   Mode:     OFFBOARD               │   X:     12.45 m   (East)       │
│   Armed:    YES                    │   Y:     -3.21 m   (North)      │
│   Uptime:   00:05:23               │   Z:     10.02 m   (Up)         │
│                                    │   Yaw:   045.2°                 │
├────────────────────────────────────┼─────────────────────────────────┤
│ VELOCITY (m/s)                     │ ATTITUDE                        │
│   Vx:      0.12                    │   Roll:   1.2°                  │
│   Vy:     -0.05                    │   Pitch: -0.8°                  │
│   Vz:      0.02                    │   Yaw:   45.2°                  │
│   |V|:     0.13                    │   Tilt:   1.4°                  │
├────────────────────────────────────┼─────────────────────────────────┤
│ BATTERY                            │ CONNECTIONS                     │
│   Voltage:  22.8 V                 │   PX4:    ████████ OK           │
│   Percent:  78%  ████████░░        │   GCS:    ████████ OK           │
│   Status:   OK                     │   Est:    ████████ OK           │
│   Current:  12.3 A                 │   Ctrl:   ████████ OK           │
├────────────────────────────────────┼─────────────────────────────────┤
│ MANAGERS                           │ SAFETY                          │
│   Estimator:   px4_passthrough     │   Status:    NOMINAL            │
│   Controller:  position_pid        │   Geofence:  OK (42m to edge)   │
│   Trajectory:  waypoint_linear     │   Battery:   OK                 │
│   Traj Prog:   45.2%               │   Neighbors: 3 OK               │
├──────────────────────────────────────────────────────────────────────┤
│ ALERTS                                                               │
│   [INFO]  12:34:56  Takeoff complete, entering HOVERING              │
│   [WARN]  12:34:12  GPS accuracy degraded (HDOP: 2.1)               │
│                                                                      │
├──────────────────────────────────────────────────────────────────────┤
│ [Q]uit  [R]efresh  [S]witch UAV  [C]lear alerts  [H]elp             │
└──────────────────────────────────────────────────────────────────────┘
```

### Multi-UAV Compact View

```
┌──────────────────────────────────────────────────────────────────────┐
│ PEREGRINE Fleet Status                                    4 UAVs Active │
├────────┬─────────┬──────────┬────────────┬─────────┬────────────────┤
│  ID    │  State  │ Position │   Battery  │ Safety  │    Status      │
├────────┼─────────┼──────────┼────────────┼─────────┼────────────────┤
│  UAV1  │ HOVER   │ 12, -3,10│    78%     │ NOMINAL │ Traj: 45%      │
│  UAV2  │ FLYING  │  8,  5,12│    82%     │ NOMINAL │ Traj: 67%      │
│  UAV3  │ HOVER   │ -2, 10, 8│    75%     │ WARNING │ GPS degraded   │
│  UAV4  │ LANDING │  0,  0, 2│    71%     │ NOMINAL │ Landing...     │
├────────┴─────────┴──────────┴────────────┴─────────┴────────────────┤
│ Fleet: 4 total, 4 connected, 0 errors                               │
│ [1-4] Select UAV  [A]ll view  [Q]uit                                │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Color Scheme

| Element | Condition | Color |
|---------|-----------|-------|
| State | NOMINAL | Green |
| State | WARNING | Yellow |
| State | ERROR/EMERGENCY | Red |
| Battery | > 50% | Green |
| Battery | 20-50% | Yellow |
| Battery | < 20% | Red |
| Connection | OK | Green |
| Connection | STALE | Yellow |
| Connection | LOST | Red |
| Geofence | OK | Green |
| Geofence | WARNING | Yellow |
| Geofence | VIOLATION | Red |
| Value | Normal | White |
| Label | Always | Cyan |
| Border | Always | Blue |
| Alert INFO | Always | White |
| Alert WARN | Always | Yellow |
| Alert ERROR | Always | Red |

---

## Implementation Details

### ncurses Setup

```
Key ncurses functions to use:
  initscr()          - Initialize screen
  start_color()      - Enable colors
  init_pair()        - Define color pairs
  curs_set(0)        - Hide cursor
  noecho()           - Don't echo input
  nodelay()          - Non-blocking input
  keypad()           - Enable special keys
  
  mvwprintw()        - Print at position
  wattron/wattroff() - Color attributes
  box()              - Draw borders
  refresh()          - Update display
```

### Refresh Strategy

```
Display update loop:
  rate = 10 Hz (configurable)
  
  while running:
    check_keyboard_input()    # Non-blocking
    update_data_from_topics()
    render_display()
    napms(100)                # Sleep 100ms
```

### Data Subscriptions

```
Subscribe to:
  /uavN/uav_manager/status           → Comprehensive status
  /uavN/estimator_manager/state      → Current state
  /uavN/safety_monitor/safety_status → Safety info
  /uavN/safety_monitor/alerts        → Alert stream
  /uavN/trajectory_manager/status    → Trajectory progress
  /uavN/hardware_abstraction/battery → Battery state

For multi-UAV:
  Subscribe to all /uav*/... topics using wildcard
  or configure explicit list of UAV IDs
```

### Keyboard Handling

```
Key bindings:
  q, Q    - Quit
  r, R    - Force refresh
  s, S    - Switch to next UAV (single view)
  1-9     - Switch to UAV N (multi view)
  a, A    - Toggle all/single view
  c, C    - Clear alert history
  h, H    - Show help
  ↑↓      - Scroll alert history
```

### Alert Buffer

```
Alert buffer:
  - Fixed size ring buffer (e.g., 100 entries)
  - Each entry: timestamp, severity, message
  - Display most recent N that fit in window
  - Scroll support for history
```

---

## Configuration

```yaml
tui_status:
  ros__parameters:
    # Display settings
    refresh_rate_hz: 10.0
    default_view: "single"           # single, multi
    default_uav: "uav1"
    
    # Multi-UAV
    uav_ids: ["uav1", "uav2", "uav3", "uav4"]
    
    # Alert buffer
    max_alerts: 100
    
    # Display options
    show_velocity: true
    show_attitude: true
    show_trajectory_progress: true
    
    # Units
    position_unit: "m"               # m, ft
    velocity_unit: "m/s"             # m/s, km/h, mph
    angle_unit: "deg"                # deg, rad
```

---

## Build Notes

### CMakeLists.txt

```cmake
find_package(Curses REQUIRED)

add_executable(tui_status_node
  src/tui_status_node.cpp
  src/display_renderer.cpp
  src/main.cpp
)

target_link_libraries(tui_status_node
  ${CURSES_LIBRARIES}
)
```

### Terminal Requirements

- Minimum 80x24 terminal
- 256-color support recommended
- UTF-8 support for box drawing characters

---

## Major Cautions

| Issue | Mitigation |
|-------|------------|
| **Terminal resize** | Handle SIGWINCH, redraw on resize |
| **Thread safety** | ncurses is not thread-safe, use mutex |
| **Cleanup on exit** | Always call endwin() to restore terminal |
| **Blocking input** | Use non-blocking mode (nodelay) |
| **High CPU** | Don't refresh faster than necessary |
| **Color support** | Check has_colors(), fallback to monochrome |

---

## File Structure

```
tui_status/
├── include/tui_status/
│   ├── tui_status_node.hpp
│   ├── display_renderer.hpp
│   ├── alert_buffer.hpp
│   └── color_scheme.hpp
├── src/
│   ├── tui_status_node.cpp
│   ├── display_renderer.cpp
│   ├── alert_buffer.cpp
│   └── main.cpp
├── config/
│   └── tui_status.yaml
└── launch/
    └── tui_status.launch.py
```
