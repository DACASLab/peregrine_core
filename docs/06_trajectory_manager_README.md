# trajectory_manager

`trajectory_manager` owns in-flight motion primitives and trajectory generation.

## Public actions
- `trajectory_manager/go_to`
- `trajectory_manager/execute_trajectory`

## Executive gating (required)
`trajectory_manager` subscribes to authoritative `uav_state` and gates both goal acceptance and active-goal continuation.

A motion goal is rejected unless all are true:
- lifecycle active
- estimated state present
- no active goal
- executive state present
- `uav_state.motion_authorized == true`
- `uav_state.fault_latched == false`
- `uav_state.failsafe == false`
- `uav_state.dependencies_ready == true`

If authority is lost during an active goal, the goal is canceled/aborted and the manager returns to hold mode with explicit reason codes (`EXECUTIVE_MOTION_REVOKED`, `EXECUTIVE_FAULT_LATCHED`, `EXECUTIVE_NOT_READY`).

## Scope boundary
`trajectory_manager` motion API is for in-flight primitives. Vehicle-executive behaviors (`takeoff`, `land`) are not accepted through public `execute_trajectory`.
