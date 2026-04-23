# trajectory_manager

Motion primitive and trajectory service layer.

## Responsibilities
- Generate and publish trajectory setpoints.
- Serve in-flight motion actions (`go_to`, `execute_trajectory`).
- Enforce executive gate from `uav_state` for acceptance and continued execution.

## Safety contract
- Reject motion when executive authority is absent/revoked.
- Abort active motion and return to hold if authority is revoked in-flight.
- Keep timer-driven setpoint publication model.
