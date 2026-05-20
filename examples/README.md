# examples

Standalone example packages for validating and demonstrating the peregrine stack.

## Packages

- `hardware_abstraction_example`: single-UAV and multi-UAV demo missions
- `safety_validation_example`: safety regression launcher (clean stale processes,
  start fresh PX4 SITL/GZ, run safety regression pipeline)

## Demo launch files

All demos use `core_stack.launch.py` from `peregrine_bringup` as the single
source of truth for the flight stack container. Examples expect PX4 to be
running externally (e.g. via Docker entrypoint or `make px4_sitl gz_x500`).

For SITL, start PX4+Gazebo separately first (e.g. `make px4_sitl gz_x500`
inside the Docker container), then launch the example.

### Trajectory demos

```bash
ros2 launch hardware_abstraction_example circle_figure8_demo.launch.py
ros2 launch hardware_abstraction_example multi_cycle_demo.launch.py
```

### Controller demos

```bash
ros2 launch hardware_abstraction_example controller_switch_demo.launch.py
ros2 launch hardware_abstraction_example controller_switch_inflight_demo.launch.py
```

### Safety and monitoring

```bash
ros2 launch hardware_abstraction_example example12_safety_validation.launch.py
ros2 launch hardware_abstraction_example example13_monitoring_demo.launch.py
ros2 launch safety_validation_example safety_fresh_sitl_regression.launch.py
```

## SE3 controller gains

Tuned and conservative SE3 gain profiles live in `control_manager/config/`:
- `se3_tuned.yaml` — aggressive gains validated in SITL step-response tests
- `se3_conservative.yaml` — softer gains suitable for first flights
- `defaults.yaml` — package defaults (uses Px4PassthroughController by default)

Override via `config_overrides` launch arg:
```bash
ros2 launch hardware_abstraction_example circle_figure8_demo.launch.py \
  config_overrides:=$(ros2 pkg prefix control_manager)/share/control_manager/config/se3_tuned.yaml
```
