# PEREGRINE Architecture

## Layer separation

- **Mission / application layer:** `mission_executor`
  - Owns mission intent, sequencing, branching, retries, and recovery logic via Behavior Trees.
  - Consumes `uav_state` and `safety_status`.
  - Calls only executive/motion APIs.
  - Never publishes `trajectory_setpoint` or `control_output`.

- **Executive layer:** `uav_manager`
  - Owns authoritative executive state and fault latch.
  - Publishes authoritative `uav_state` with executive gate fields:
    - `fault_latched`
    - `motion_authorized`
    - `takeoff_authorized`
    - `landing_authorized`
    - `executive_reason`
  - Exposes executive vehicle actions (`takeoff`, `land`) and `clear_emergency` service.
  - Does **not** expose generic mission motion proxy actions.

- **Motion layer:** `trajectory_manager`
  - Owns in-flight motion primitives (`go_to`, `execute_trajectory`).
  - Subscribes to authoritative `uav_state`.
  - Rejects/aborts motion unless executive authority allows motion.

- **Lower layers:**
  - `estimation_manager`: state estimation
  - `control_manager`: control law application
  - `hardware_abstraction`: PX4 bridge boundary (OFFBOARD gate)
  - `safety_monitor`: independent safety assessment/watchdog

## Safety and authority rules

1. BT owns mission intent, not executive authority.
2. `uav_manager` is the single executive authority.
3. `trajectory_manager` cannot bypass `uav_manager` authority.
4. Fault/emergency state is latched until explicit `clear_emergency`.
5. Startup readiness is non-blocking in lifecycle `on_configure()` for estimator/control/trajectory managers.
