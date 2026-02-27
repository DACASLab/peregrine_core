# Safety Monitor + Home-Frame Refactor Plan (Final)

## Summary
Implement an extensible **external safety layer** (`safety_monitor`) and a **fleet-home anchored frame initialization** in `frame_transforms`, while keeping:
- `hardware_abstraction` as a thin PX4 bridge (no new GPS interpretation interfaces),
- `uav_manager` focused on internal readiness/supervisor logic, plus external-safety gating.

Key decisions locked:
1. External safety policy: **WARN then LAND**, with **per-rule grace**.
2. External safety command path: **`safety_monitor` calls `set_mode("land")` directly**.
3. External safety clear policy: **auto-clear on sustained healthy**.
4. Home frame: **single fleet-wide configured home**.
5. GPS/geodetic computations: **in `frame_transforms`**, not `hardware_abstraction`.
6. No new `ReferenceFrameStatus.msg`; use diagnostics/TF state instead.

---

## Goals and Success Criteria

## Goals
1. Add modular external safety rules (GPS quality, battery, geofence) that are easy to extend.
2. Enforce deterministic safety actions with explicit reason codes.
3. Initialize map alignment to configured home GPS once quality is sufficient.
4. Gate arm/takeoff on both internal readiness and external safety/frame readiness.

## Success Criteria
1. External safety violations trigger WARN then LAND according to configured grace.
2. `uav_manager` rejects arm/takeoff when external safety is not OK.
3. `frame_transforms` computes home-anchored `map->odom` once valid GPS quality is present.
4. Existing nominal SITL examples still pass when safety conditions are healthy.
5. New safety rules can be added by implementing one rule class + config entry only.

---

## Package-Level Implementation Plan

## 1) New Package: `safety_monitor`
Create lifecycle component with pure-core architecture.

### 1.1 Structure
- `safety_monitor/include/safety_monitor/...`
- `safety_monitor/src/...`
- `safety_monitor_core` logic kept ROS-free within package (`rules`, `engine`, `models` namespaces).

### 1.2 Core Abstractions (pure C++)
1. `SafetySnapshot`
   - Inputs normalized from ROS: GPS fix/sats/hdop/vdop/freshness, battery, position in map, PX4 mode/armed/failsafe, geofence context.
2. `SafetyRule` interface
   - `evaluate(snapshot, now) -> RuleEvaluation`.
3. `RuleEvaluation`
   - fields: `level` (`OK|WARN|CRITICAL`), `reason_code`, `detail`, `recommended_action`.
4. `RuleEngine`
   - Runs all enabled rules.
   - Tracks per-rule timers for WARN->LAND transition.
   - Implements auto-clear after `healthy_hold_s`.
5. `SafetyActionExecutor`
   - Non-blocking, timeout-bounded `set_mode("land")` orchestration.
   - Explicit reason codes: `LAND_CMD_TIMEOUT`, `LAND_CMD_REJECTED`, etc.

### 1.3 V1 Rules
1. `GpsQualityRule`
   - Thresholds: min fix quality, min satellites, max HDOP, max VDOP, freshness.
2. `BatteryRule`
   - Warn + critical thresholds (percent, optional voltage).
3. `GeofenceRule`
   - Polygon in map XY + min/max altitude checks.

### 1.4 ROS Node (`SafetyMonitorNode`, LifecycleNode)
Subscriptions:
- `gnss` (`sensor_msgs/NavSatFix`) from `hardware_abstraction`.
- `status` (`peregrine_interfaces/PX4Status`) for battery/failsafe/nav context.
- `estimated_state` (`peregrine_interfaces/State`) for position/geofence checks.
- diagnostics from `frame_transforms` (home init + GPS quality status).

Publisher:
- `safety_status` (`diagnostic_msgs/DiagnosticArray` in V1; optional typed msg later).

Client:
- `set_mode` service client (`peregrine_interfaces/SetMode`).

Lifecycle:
- `on_configure`: load rules/config, setup subs/pubs/client.
- `on_activate`: enable evaluation timer + action execution.
- `on_deactivate`: stop timers, suppress commands.
- cleanup/shutdown/error: deterministic stop.

---

## 2) Update: `frame_transforms`
Add home-origin + GPS-quality initialization logic.

### 2.1 New Parameters
- `use_home_origin` (bool)
- `home_lat_deg` (double)
- `home_lon_deg` (double)
- `require_gps_for_home_init` (bool)
- GPS quality thresholds:
  - `gps_min_fix_type`
  - `gps_min_satellites`
  - `gps_max_hdop`
  - `gps_max_vdop`
  - `gps_freshness_timeout_s`
- `home_init_timeout_s` (optional warn timeout)

### 2.2 New Inputs
- subscribe to `gnss` (`sensor_msgs/NavSatFix`).

### 2.3 Initialization Logic
1. Before init:
   - keep current behavior for `odom->base_link` publishing.
   - publish diagnostics indicating `home_initialized=false`.
2. When GPS quality passes:
   - convert current GNSS to ENU relative to configured home lat/lon.
   - compute and latch one-time `map->odom` transform offset.
   - set `home_initialized=true`.
3. After init:
   - keep `map->odom` fixed in V1 (no continuous global correction).
   - continue `odom->base_link` updates normally.

### 2.4 Outputs
- Existing TF tree unchanged in shape.
- Add diagnostics (`diagnostic_msgs/DiagnosticArray`) containing:
  - `home_initialized`
  - `gps_quality_ok`
  - `reason_code`
  - configured home lat/lon

No new custom frame-status interface.

---

## 3) `hardware_abstraction` Changes
Minimal/no logic expansion.

### Keep as-is
- Publish `gnss` (`sensor_msgs/NavSatFix`).
- Publish `status` (`peregrine_interfaces/PX4Status`) with battery/failsafe/nav fields.
- No geodetic conversion or HDOP/VDOP policy logic added here.

Optional small hardening only:
- Ensure NavSatFix covariance/quality fields are consistently filled from PX4 source.
- Add comments documenting that GPS policy is owned by frame/safety layers.

---

## 4) `uav_manager` Integration
Internal supervisor remains primary for internal readiness.

### 4.1 Add external safety intake
- Subscribe to `safety_status` diagnostics (or adapted compact status stream).
- Track `external_safety_ok` + latest reason code.

### 4.2 Gate actions
- In `onTakeoffGoal` and arm path:
  - reject when `external_safety_ok=false`.
  - reason code propagation (`EXTERNAL_SAFETY_WARN`, `EXTERNAL_SAFETY_CRITICAL`, `HOME_NOT_INITIALIZED`, etc.).
- Keep existing internal guard model unchanged (estimation/control/trajectory/PX4 readiness).

### 4.3 Status surfacing
- Append external safety reason to `UAVState.readiness_detail`.
- No takeover of external-policy thresholds into `uav_manager`.

---

## 5) Launch/Bringup Sequencing
Update example/bringup launch so dependencies are deterministic:

1. `hardware_abstraction` + `frame_transforms` start.
2. manager lifecycle nodes start (`estimation/control/trajectory`).
3. `safety_monitor` starts and becomes active.
4. `uav_manager` activates.
5. Arm/takeoff only accepted if:
   - internal dependencies ready,
   - external safety OK,
   - home frame initialized.

---

## Important Public Interface Changes

## Added
1. New package `safety_monitor`.
2. New topic from safety monitor:
   - `safety_status` (`diagnostic_msgs/DiagnosticArray`) with stable key/value reason codes.
3. `frame_transforms` diagnostics output:
   - home init and GPS quality diagnostics.

## Not Added
1. No `ReferenceFrameStatus.msg`.
2. No new GPS-quality message in `hardware_abstraction` in V1.
3. No changes to existing `PX4Status.msg`, `UAVState.msg`, or service names in V1.

---

## Config Specification (Decision-Complete)

## `safety_monitor` YAML
- `rules.gps.enabled`
- `rules.gps.warn_grace_s`
- `rules.gps.critical_grace_s`
- `rules.gps.min_fix_type`
- `rules.gps.min_satellites`
- `rules.gps.max_hdop`
- `rules.gps.max_vdop`
- `rules.gps.freshness_timeout_s`

- `rules.battery.enabled`
- `rules.battery.warn_percent`
- `rules.battery.critical_percent`
- `rules.battery.warn_voltage_v` (optional)
- `rules.battery.critical_voltage_v` (optional)
- `rules.battery.warn_grace_s`
- `rules.battery.critical_grace_s`
- `rules.battery.freshness_timeout_s`

- `rules.geofence.enabled`
- `rules.geofence.vertices_xy` (polygon in map frame)
- `rules.geofence.min_alt_m`
- `rules.geofence.max_alt_m`
- `rules.geofence.warn_grace_s`
- `rules.geofence.critical_grace_s`

- `global.healthy_auto_clear_s`
- `global.land_command_timeout_s`
- `global.land_command_retry_count`

## `frame_transforms` YAML
- `use_home_origin`
- `home_lat_deg`
- `home_lon_deg`
- `require_gps_for_home_init`
- `gps_min_fix_type`
- `gps_min_satellites`
- `gps_max_hdop`
- `gps_max_vdop`
- `gps_freshness_timeout_s`
- `home_init_timeout_s`

---

## Test Plan

## Unit Tests
1. `GpsQualityRule`
   - fix drop, sats drop, hdop/vdop violations, stale GPS.
2. `BatteryRule`
   - warn/critical transitions, recovery behavior.
3. `GeofenceRule`
   - inside/outside polygon, altitude bound violations.
4. `RuleEngine`
   - per-rule grace timing, highest-severity merge, auto-clear window.
5. `Frame home init math`
   - ENU conversion from configured home, deterministic map->odom latching.

## Integration Tests (ROS2)
1. Startup with bad GPS quality:
   - frame not initialized, safety not OK, takeoff rejected.
2. GPS becomes valid:
   - frame initializes, safety transitions to OK.
3. Battery critical:
   - WARN then LAND after configured grace.
4. Geofence breach:
   - WARN then LAND after configured grace.
5. Recovery:
   - sustained healthy -> auto-clear -> takeoff allowed.
6. Existing mission demos:
   - Example 10 and 11 pass in healthy conditions.

## SITL Validation Scenarios
1. Inject GPS degradation (sat count + hdop/vdop) and confirm LAND behavior.
2. Inject low battery and confirm LAND behavior.
3. Drive beyond polygon bounds and confirm LAND behavior.
4. Confirm no manual workaround required for nominal launch/mission flow.

---

## Rollout Plan

1. Phase 1: introduce `safety_monitor` package + diagnostics-only output (no LAND command), verify rule evaluations.
2. Phase 2: enable LAND command path with per-rule grace.
3. Phase 3: enable `uav_manager` external safety gating.
4. Phase 4: tune thresholds for SITL vs hardware profiles.

Feature flags:
- `safety_monitor.enabled`
- `safety_monitor.command_land_enabled`
- `uav_manager.require_external_safety`

---

## Assumptions and Defaults
1. External safety owns battery/GPS/geofence.
2. Internal safety remains manager readiness/freshness/FSM legality.
3. LAND is conservative default for critical external faults.
4. Fleet uses one configured home origin.
5. Home altitude semantics are deferred; V1 anchors by configured lat/lon and local map-alt limits.
6. Diagnostics are sufficient for readiness/status exchange in V1; typed status messages can be added later if needed.
