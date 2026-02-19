# hardware_abstraction

Single PX4 boundary package for PEREGRINE.

## Inputs

- `control_output` (`peregrine_interfaces/msg/ControlOutput`)

## Outputs

- `state` (`peregrine_interfaces/msg/State`)
- `status` (`peregrine_interfaces/msg/PX4Status`)
- `odometry` (`nav_msgs/msg/Odometry`)
- `battery` (`sensor_msgs/msg/BatteryState`)
- `gnss` (`sensor_msgs/msg/NavSatFix`)

## Services

- `arm` (`peregrine_interfaces/srv/Arm`)
- `set_mode` (`peregrine_interfaces/srv/SetMode`)

## PX4 Interface

Subscribes from `/fmu/out/*` and publishes to `/fmu/in/*` under optional `px4_namespace` parameter.
