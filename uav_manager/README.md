# uav_manager

Executive authority node for Peregrine.

## Role
- Authoritative executive/fault state publisher (`uav_state`).
- Vehicle-level executive actions (`takeoff`, `land`).
- Explicit fault clear API (`clear_emergency`).

## Not this node's role
- No public generic mission motion proxy (`go_to`, `execute_trajectory`).
- No hidden auto-clear recovery behavior.
- No forced mode manipulation to make arm requests pass.

Mission sequencing and branching now belong to `mission_executor`.
