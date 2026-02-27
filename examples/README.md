# examples

Top-level container for standalone example packages.

## Packages

- `hardware_abstraction_example`: single-UAV PX4 SITL validation flow for
  `hardware_abstraction` + `frame_transforms`.
- `safety_validation_example`: dedicated fresh-start safety regression launcher
  (clean stale processes, start fresh PX4 SITL/GZ, run safety regression pipeline).

## Usage

Build the example package you want, then launch from that package directly.
For the hardware abstraction flow:

`ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py`

For autonomous circle/figure-eight demo on the manager chain:

`ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py mission_type:=circle_figure8`

For focused multi-cycle demo with RViz/TUI monitoring hooks:

`ros2 launch hardware_abstraction_example example13_monitoring_demo.launch.py`

For a fresh SITL safety-regression run:

`ros2 launch safety_validation_example safety_fresh_sitl_regression.launch.py`
