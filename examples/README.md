# examples

Top-level container for standalone example packages.

## Packages

- `hardware_abstraction_example`: single-UAV PX4 SITL validation flow for
  `hardware_abstraction` + `frame_transforms`.

## Usage

Build the example package you want, then launch from that package directly.
For the hardware abstraction flow:

`ros2 launch hardware_abstraction_example example8_px4_sitl_single_uav.launch.py`

For autonomous circle/figure-eight demo on the manager chain:

`ros2 launch hardware_abstraction_example example10_circle_figure8_demo.launch.py mission_type:=circle_figure8`
