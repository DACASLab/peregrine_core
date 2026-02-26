# References

## ROS2 Lifecycle Design

- **ROS2 Lifecycle Design Article** — Foundational design document for managed (lifecycle) nodes.
  https://design.ros2.org/articles/node_lifecycle.html

- **`trigger_transition()` API** — Self-transition mechanism on `rclcpp_lifecycle::LifecycleNode`.
  Allows a node to drive its own configure/activate transitions without external service calls.
  https://github.com/ros2/rclcpp/blob/rolling/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp

- **ROS2 Managed Nodes Demo** — Official lifecycle patterns and examples from the ros2/demos repo.
  https://github.com/ros2/demos/tree/rolling/lifecycle

## Lifecycle Management Alternatives

- **nav2_lifecycle_manager** — Navigation2's approach to lifecycle orchestration.
  A heavier alternative that manages multiple lifecycle nodes from a single external manager.
  The `auto_start` approach used in this codebase is lighter but compatible — set `auto_start: false`
  and point nav2_lifecycle_manager at the nodes if the heavier approach is ever needed.
  https://github.com/ros-planning/navigation2/tree/main/nav2_lifecycle_manager

## ROS2 YAML Parameter Loading

- **ROS2 Parameter YAML Format** — Standard format used by nav2, moveit2, and other ROS2 stacks
  for loading node parameters from config files instead of inline launch arguments.
  https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html

- **ROS2 Launch System — Loading Parameters** — How `PathJoinSubstitution` + `FindPackageShare`
  resolve installed YAML files at launch time.
  https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html

## ROS2 Composition (Composable Nodes)

- **ROS2 Composition Tutorial** — Loading multiple nodes into a single process via
  `component_container` / `component_container_mt` for intra-process zero-copy communication.
  https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html

- **Intra-Process Communication** — Zero-copy message passing between composable nodes
  sharing the same process, avoiding serialization overhead.
  https://docs.ros.org/en/rolling/Tutorials/Demos/Intra-Process-Communication.html

## PX4 Integration

- **PX4 uXRCE-DDS** — Micro XRCE-DDS bridge between PX4 autopilot and ROS2.
  https://docs.px4.io/main/en/middleware/uxrce_dds.html

- **PX4 Offboard Control** — Requirements for offboard mode (setpoint stream cadence,
  mode transitions, arming prerequisites).
  https://docs.px4.io/main/en/flight_modes/offboard.html

## ROS2 Actions

- **ROS2 Action Design** — Two-phase goal handshake (send_goal → get_result), feedback
  streaming, and cancellation protocol used by all peregrine action interfaces.
  https://design.ros2.org/articles/actions.html

- **rclcpp_action API** — C++ action server/client implementation used by trajectory_manager
  and uav_manager.
  https://github.com/ros2/rclcpp/tree/rolling/rclcpp_action

## Threading & Callback Groups

- **ROS2 Executors** — MultiThreadedExecutor and callback group semantics (MutuallyExclusive
  vs Reentrant) that govern concurrent callback dispatch in the peregrine manager stack.
  https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html
