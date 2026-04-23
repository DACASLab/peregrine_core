# uav_manager

`uav_manager` is the executive authority for vehicle-level state and safety gating.

## Public APIs
- `uav_manager/takeoff` (action)
- `uav_manager/land` (action)
- `uav_manager/clear_emergency` (service)

`uav_manager` no longer provides public generic mission motion actions (`go_to`, `execute_trajectory`).
Mission/application clients should call motion primitives from `trajectory_manager` directly, while relying on executive gating via `uav_state`.

## Authoritative state
`uav_manager` publishes `uav_state` as the authoritative executive contract, including:
- `fault_latched`
- `motion_authorized`
- `takeoff_authorized`
- `landing_authorized`
- `executive_reason`

## Emergency semantics
- Emergency/fault remains latched.
- No automatic emergency clear timer.
- Clear is explicit via `clear_emergency` and guard-checked by FSM policy.
