# mission_executor

`mission_executor` is the application/mission layer for Peregrine. It owns mission sequencing and recovery logic via Behavior Trees.

## Responsibilities
- Read executive state (`uav_state`) and safety (`safety_status`).
- Run mission intent logic (sequence, retry, fallback).
- Invoke only executive/motion APIs:
  - `uav_manager/takeoff`
  - `uav_manager/land`
  - `uav_manager/clear_emergency`
  - `trajectory_manager/go_to`
  - `trajectory_manager/execute_trajectory`

## Non-responsibilities
- Never publish `trajectory_setpoint`.
- Never publish `control_output`.
- Never call hardware arm/set_mode services directly.
