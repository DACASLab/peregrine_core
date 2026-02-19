# peregrine_interfaces

Core ROS 2 interfaces for the PEREGRINE stack.

## Messages

- `State.msg`: ENU/FLU state estimate for manager-level consumers.
- `ControlOutput.msg`: controller command envelope consumed by hardware abstraction.
- `PX4Status.msg`: PX4 connection, arming, mode, and battery status.

## Services

- `Arm.srv`: arm or disarm the vehicle.
- `SetMode.srv`: request a high-level PX4 mode by name.
