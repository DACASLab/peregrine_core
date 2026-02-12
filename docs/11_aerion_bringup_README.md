# aerion_bringup

**Package Type:** ROS2 Launch/Configuration Package  
**Dependencies:** All AERION packages, launch_ros, ros2launch  

---

## Overview

`aerion_bringup` provides **launch files, configurations, and deployment infrastructure** for the AERION flight stack. This is the primary entry point for starting the system.

---

## Responsibilities

1. **Launch file organization** - single UAV, multi-UAV, simulation
2. **Configuration management** - environment-specific configs
3. **Startup orchestration** - proper node ordering and dependencies
4. **Simulation setup** - Gazebo Harmonic integration
5. **Deployment support** - systemd services, Jetson setup

---

## Launch File Structure

### Core Launch Files

| Launch File | Purpose |
|-------------|---------|
| `single_uav.launch.py` | Launch complete stack for one UAV |
| `multi_uav.launch.py` | Launch stack for N UAVs |
| `simulation.launch.py` | Start Gazebo + SITL + stack |
| `hardware.launch.py` | Real hardware deployment |
| `minimal.launch.py` | Minimal stack for testing |

### Component Launch Files

| Launch File | Purpose |
|-------------|---------|
| `hardware_abstraction.launch.py` | Just hardware_abstraction |
| `managers.launch.py` | All manager nodes |
| `safety.launch.py` | Safety monitor only |
| `tui.launch.py` | TUI status display |

---

## Single UAV Launch

### Launch File

```python
# launch/single_uav.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    namespace = LaunchConfiguration('namespace', default='uav1')
    config_file = LaunchConfiguration('config', default='default.yaml')
    environment = LaunchConfiguration('environment', default='gps')
    
    # Load config
    config_path = PathJoinSubstitution([
        FindPackageShare('aerion_bringup'),
        'config', config_file
    ])
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('namespace', default_value='uav1'),
        DeclareLaunchArgument('config', default_value='default.yaml'),
        DeclareLaunchArgument('environment', default_value='gps'),
        
        # Hardware Abstraction (first - connects to PX4)
        Node(
            package='hardware_abstraction',
            executable='hardware_abstraction_node',
            name='hardware_abstraction',
            namespace=namespace,
            parameters=[config_path],
            output='screen',
        ),
        
        # Frame Transforms (TF broadcaster)
        Node(
            package='frame_transforms',
            executable='tf_broadcaster_node',
            name='tf_broadcaster',
            namespace=namespace,
            parameters=[config_path],
            output='screen',
        ),
        
        # Estimator Manager
        Node(
            package='estimator_manager',
            executable='estimator_manager_node',
            name='estimator_manager',
            namespace=namespace,
            parameters=[config_path],
            output='screen',
        ),
        
        # Controller Manager
        Node(
            package='controller_manager',
            executable='controller_manager_node',
            name='controller_manager',
            namespace=namespace,
            parameters=[config_path],
            output='screen',
        ),
        
        # Trajectory Manager
        Node(
            package='trajectory_manager',
            executable='trajectory_manager_node',
            name='trajectory_manager',
            namespace=namespace,
            parameters=[config_path],
            output='screen',
        ),
        
        # Safety Monitor
        Node(
            package='safety_monitor',
            executable='safety_monitor_node',
            name='safety_monitor',
            namespace=namespace,
            parameters=[config_path],
            output='screen',
        ),
        
        # UAV Manager (last - coordinates others)
        Node(
            package='uav_manager',
            executable='uav_manager_node',
            name='uav_manager',
            namespace=namespace,
            parameters=[config_path],
            output='screen',
        ),
    ])
```

### Usage

```bash
# Basic launch
ros2 launch aerion_bringup single_uav.launch.py

# With custom namespace and config
ros2 launch aerion_bringup single_uav.launch.py \
    namespace:=uav1 \
    config:=outdoor_config.yaml \
    environment:=gps
```

---

## Multi-UAV Launch

### Launch File

```python
# launch/multi_uav.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def launch_uav(context, namespace, config_path):
    """Generate nodes for a single UAV."""
    nodes = []
    
    # Add all nodes with namespace
    # (same as single_uav but in a function)
    
    return nodes

def generate_launch_description():
    num_uavs = LaunchConfiguration('num_uavs', default='4')
    
    def launch_setup(context):
        n = int(context.launch_configurations['num_uavs'])
        all_nodes = []
        
        for i in range(1, n + 1):
            namespace = f'uav{i}'
            # Launch each UAV's stack
            all_nodes.extend(launch_uav(context, namespace, config_path))
            
            # Add multi-agent coordinator for each
            all_nodes.append(Node(
                package='multi_agent_coordinator',
                executable='multi_agent_coordinator_node',
                name='multi_agent_coordinator',
                namespace=namespace,
                parameters=[{'fleet_size': n}],
            ))
        
        return all_nodes
    
    return LaunchDescription([
        DeclareLaunchArgument('num_uavs', default_value='4'),
        OpaqueFunction(function=launch_setup),
    ])
```

### Usage

```bash
# Launch 4 UAVs
ros2 launch aerion_bringup multi_uav.launch.py num_uavs:=4

# With custom config
ros2 launch aerion_bringup multi_uav.launch.py \
    num_uavs:=4 \
    config:=fleet_config.yaml
```

---

## Simulation Launch

### Gazebo Harmonic + PX4 SITL

```python
# launch/simulation.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    num_uavs = LaunchConfiguration('num_uavs', default='1')
    world = LaunchConfiguration('world', default='empty.sdf')
    
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )
    
    # Start PX4 SITL instances
    # Each instance needs unique MAV_SYS_ID and ports
    px4_instances = []
    for i in range(int(num_uavs)):
        px4 = ExecuteProcess(
            cmd=[
                'px4',
                f'-i {i}',
                '-d',
                f'INSTANCE={i}',
            ],
            cwd=os.environ.get('PX4_HOME', '~/PX4-Autopilot'),
            output='screen'
        )
        px4_instances.append(px4)
    
    # Start uXRCE-DDS agents
    dds_agents = []
    for i in range(int(num_uavs)):
        agent = ExecuteProcess(
            cmd=[
                'MicroXRCEAgent', 'udp4',
                '-p', str(8888 + i)
            ],
            output='screen'
        )
        dds_agents.append(agent)
    
    # ROS-Gazebo bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Add other bridged topics
        ],
        output='screen'
    )
    
    # Include multi-UAV launch
    aerion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('aerion_bringup'),
            '/launch/multi_uav.launch.py'
        ]),
        launch_arguments={'num_uavs': num_uavs}.items()
    )
    
    return LaunchDescription([
        gazebo,
        *px4_instances,
        *dds_agents,
        gz_bridge,
        aerion_launch,
    ])
```

### Usage

```bash
# Single UAV simulation
ros2 launch aerion_bringup simulation.launch.py

# 4-UAV simulation
ros2 launch aerion_bringup simulation.launch.py num_uavs:=4

# Custom world
ros2 launch aerion_bringup simulation.launch.py \
    world:=/path/to/custom_world.sdf
```

---

## Configuration Files

### Directory Structure

```
config/
├── default.yaml              # Default configuration
├── environments/
│   ├── gps.yaml              # GPS/outdoor specific
│   ├── mocap.yaml            # MoCap/indoor specific
│   └── simulation.yaml       # Simulation specific
├── uav_types/
│   ├── quadrotor_x.yaml      # Quadrotor X config
│   ├── quadrotor_plus.yaml   # Quadrotor + config
│   └── hexarotor.yaml        # Hexarotor config
├── missions/
│   ├── inspection.yaml       # Inspection mission params
│   ├── survey.yaml           # Survey mission params
│   └── formation.yaml        # Formation flight params
└── fleet/
    ├── 4_uav_diamond.yaml    # 4-UAV diamond formation
    └── 6_uav_line.yaml       # 6-UAV line formation
```

### Default Configuration

```yaml
# config/default.yaml

# Global settings
global:
  environment: "gps"               # gps, mocap
  log_level: "info"

# Hardware Abstraction
hardware_abstraction:
  ros__parameters:
    px4_namespace: ""
    offboard_rate_hz: 10.0

# Estimator Manager
estimator_manager:
  ros__parameters:
    publish_rate_hz: 250.0
    default_estimator: "px4_passthrough"
    plugins: ["px4_passthrough"]

# Controller Manager
controller_manager:
  ros__parameters:
    control_rate_hz: 250.0
    default_controller: "position_pid"
    plugins: ["position_pid", "velocity_controller"]

# Trajectory Manager
trajectory_manager:
  ros__parameters:
    publish_rate_hz: 50.0
    default_generator: "waypoint_linear"

# UAV Manager
uav_manager:
  ros__parameters:
    default_takeoff_altitude_m: 2.0
    default_climb_rate_mps: 1.0

# Safety Monitor
safety_monitor:
  ros__parameters:
    check_rate_hz: 100.0
    geofence:
      max_radius_m: 100.0
      max_altitude_m: 50.0

# Multi-Agent Coordinator
multi_agent_coordinator:
  ros__parameters:
    safety_buffer_m: 2.0
    broadcast_rate_hz: 20.0
```

---

## Deployment to Jetson

### Systemd Service

```ini
# /etc/systemd/system/aerion.service

[Unit]
Description=AERION Flight Stack
After=network.target

[Service]
Type=simple
User=aerion
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
ExecStart=/opt/ros/humble/bin/ros2 launch aerion_bringup hardware.launch.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### Installation Commands

```bash
# Install service
sudo cp aerion.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable aerion.service

# Start/Stop
sudo systemctl start aerion
sudo systemctl stop aerion

# Check status
sudo systemctl status aerion
journalctl -u aerion -f  # Follow logs
```

### Jetson Optimization

```bash
# Set Jetson to max performance
sudo nvpmodel -m 0              # Max power mode
sudo jetson_clocks              # Max clock speeds

# Real-time priority setup
sudo setcap cap_sys_nice+ep /opt/ros/humble/lib/*/hardware_abstraction_node
sudo setcap cap_sys_nice+ep /opt/ros/humble/lib/*/controller_manager_node
```

---

## Major Cautions

| Issue | Mitigation |
|-------|------------|
| **Node startup order** | Use launch events for dependencies |
| **Config file errors** | Validate YAML before flight |
| **Namespace conflicts** | Verify unique namespaces per UAV |
| **Simulation time** | Use use_sim_time parameter consistently |
| **Resource limits** | Monitor CPU/memory on Jetson |
| **Network configuration** | Set ROS_DOMAIN_ID, DDS config |

---

## File Structure

```
aerion_bringup/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── single_uav.launch.py
│   ├── multi_uav.launch.py
│   ├── simulation.launch.py
│   ├── hardware.launch.py
│   ├── minimal.launch.py
│   └── components/
│       ├── hardware_abstraction.launch.py
│       ├── managers.launch.py
│       └── safety.launch.py
├── config/
│   ├── default.yaml
│   ├── environments/
│   ├── uav_types/
│   ├── missions/
│   └── fleet/
├── scripts/
│   ├── setup_jetson.sh
│   ├── install_service.sh
│   └── check_system.sh
├── worlds/
│   └── test_world.sdf
└── models/
    └── x500/
        ├── model.sdf
        └── model.config
```
