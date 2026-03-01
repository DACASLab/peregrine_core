# Plan: Multi-UAV Containerized Stack with Zenoh Bridging

## Context

The peregrine stack currently runs as a single UAV in one Docker container. The goal is to run **2 UAVs**, each in its own container, with a shared Gazebo simulation and **Zenoh for inter-container topic bridging**. This is infrastructure for future multi-agent coordination — NO coordination code needed yet, just prove N isolated UAV stacks can run, and a ground station TUI can observe both.

### What Already Exists
- `docker/docker/Dockerfile.simulation` — full sim image (CUDA, Gazebo, PX4 SITL, ROS2)
- `docker/compose/docker-compose.simulation.yml` — single UAV compose
- `docker/config/entrypoint.sh` — maps `DRONE_ID` → `ROS_DOMAIN_ID`, optional XRCE agent start
- `docker/.env` — central config (versions, DRONE_ID=1, ROS_LOCALHOST_ONLY=1)
- `docker/Makefile` — build/run targets
- `peregrine_bringup/launch/single_uav.launch.py` — stack-only (no SITL)
- `peregrine_bringup/launch/single_uav_sitl.launch.py` — full SITL + stack

### Architecture Target

```
┌──────────────────────────────────────────────────────────┐
│  Container: sim                                          │
│  Shared Gazebo + 2x PX4 SITL instances                  │
│  PX4 inst 0: domain_id=1, XRCE port 8888, pos=(0,0)    │
│  PX4 inst 1: domain_id=2, XRCE port 8890, pos=(3,0)    │
│  MicroXRCE Agent x2 (one per PX4 instance)              │
└──────────┬──────────────────────────┬────────────────────┘
           │ DDS domain 1             │ DDS domain 2
┌──────────▼──────────┐    ┌──────────▼──────────┐
│  Container: uav1    │    │  Container: uav2    │
│  DOMAIN_ID=1        │    │  DOMAIN_ID=2        │
│  LOCALHOST_ONLY=1   │    │  LOCALHOST_ONLY=1   │
│  peregrine stack    │    │  peregrine stack    │
│  zenoh-bridge       │    │  zenoh-bridge       │
│  (exports 8 topics) │    │  (exports 8 topics) │
└──────────┬──────────┘    └──────────┬──────────┘
           │ Zenoh                    │ Zenoh
┌──────────▼──────────────────────────▼──────────┐
│  Container: gcs                                │
│  DOMAIN_ID=99                                  │
│  zenoh-bridge (imports all /uav* topics)       │
│  tui_status x2 (one per UAV, tmux panes)       │
└────────────────────────────────────────────────┘
```

All containers use `network_mode: host` so Zenoh TCP can reach across them while DDS stays isolated via separate `ROS_DOMAIN_ID` + `ROS_LOCALHOST_ONLY=1`.

---

## Implementation Steps

### Step 1: Multi-instance PX4 SITL launch file

**New file:** `peregrine_bringup/launch/multi_uav_sitl.launch.py`

Spawns N PX4 SITL instances in one shared Gazebo world. Does NOT run the peregrine stack — each UAV container does that separately.

Per PX4 instance:
- `PX4_SYS_AUTOSTART=4001` (gz_x500 airframe)
- `PX4_GZ_MODEL_POSE="X,0"` (spread apart, e.g., 0m and 3m)
- `PX4_PARAM_UXRCE_DDS_PTCFG=1` (localhost DDS)
- `PX4_PARAM_UXRCE_DDS_DOM_ID=<N>` (match container's ROS_DOMAIN_ID)
- PX4 instance index via `-i <N>` (changes ports: 8888+2*N)
- Staggered startup (instance 0 first, instance 1 after 5s delay)

Also starts one MicroXRCE Agent per PX4 instance on the corresponding UDP port.

**Reference:** PX4 multi-vehicle SITL docs — uses `Tools/simulation/gz/sitl_multiple_run.sh` pattern but as a ROS2 launch file for better integration.

### Step 2: Install zenoh-bridge-ros2dds in Dockerfile

**Modified file:** `docker/docker/Dockerfile.simulation`

Add zenoh-bridge-ros2dds binary. Use the official release binary for x86_64:
```dockerfile
RUN curl -fsSL <zenoh-bridge release URL> -o /tmp/zenoh.zip && \
    unzip /tmp/zenoh.zip -d /usr/local/bin/ && \
    chmod +x /usr/local/bin/zenoh-bridge-ros2dds && \
    rm /tmp/zenoh.zip
```

Check latest compatible version for ROS2 Humble + CycloneDDS at implementation time.

### Step 3: Zenoh bridge configs

**New file:** `docker/config/zenoh/uav_bridge.json5`

Per-UAV config — bridges local DDS topics to Zenoh with namespace prefix:
- Allowlist: only the 8 status topics (uav_state, estimated_state, safety_status, estimation_status, control_status, trajectory_status, status, gps_status)
- Zenoh key prefix: `/uav{DRONE_ID}/` (set via env var substitution or template)
- Mode: peer (connects to Zenoh router or directly to other bridges)
- DDS domain: reads from `ROS_DOMAIN_ID` env var

**New file:** `docker/config/zenoh/gcs_bridge.json5`

GCS config — subscribes to all `/uav*/**` Zenoh keys, republishes on local DDS domain 99:
- Allowlist subscribe: `uav*/uav_state`, `uav*/estimated_state`, etc.
- DDS domain: 99 (GCS-only)
- Republishes with namespace so TUI sees `/uav1/uav_state`, `/uav2/uav_state`

### Step 4: Update entrypoint.sh

**Modified file:** `docker/config/entrypoint.sh`

Add optional Zenoh bridge startup before exec:
```bash
if [ -n "${ZENOH_BRIDGE_CONFIG}" ]; then
    echo "[entrypoint] Starting zenoh-bridge-ros2dds..."
    zenoh-bridge-ros2dds -c "${ZENOH_BRIDGE_CONFIG}" &
    sleep 1  # brief settle
fi
```

### Step 5: Multi-UAV Docker Compose

**New file:** `docker/compose/docker-compose.multi-sitl.yml`

Four services:

**`sim`** — shared Gazebo + PX4 SITL instances:
- Same image as simulation
- Runs `multi_uav_sitl.launch.py` (or shell script that starts 2 PX4 instances)
- GPU access for Gazebo rendering
- No Zenoh bridge needed (PX4 talks DDS directly to UAV containers)

**`uav1`** — peregrine stack for UAV 1:
- `DRONE_ID=1` → `ROS_DOMAIN_ID=1`
- `ZENOH_BRIDGE_CONFIG=/ros2_ws/docker/config/zenoh/uav_bridge.json5`
- `XRCE_PORT=8888`
- Command: `ros2 launch peregrine_bringup single_uav.launch.py start_microxrce_agent:=false`
- (XRCE agent runs in sim container, not here)

**`uav2`** — peregrine stack for UAV 2:
- `DRONE_ID=2` → `ROS_DOMAIN_ID=2`
- `ZENOH_BRIDGE_CONFIG=/ros2_ws/docker/config/zenoh/uav_bridge.json5`
- `XRCE_PORT=8890`
- Command: same launch, different domain

**`gcs`** — ground station:
- `DRONE_ID=99` → `ROS_DOMAIN_ID=99`
- `ZENOH_BRIDGE_CONFIG=/ros2_ws/docker/config/zenoh/gcs_bridge.json5`
- Runs 2x tui_status_node in tmux (one per UAV namespace)

### Step 6: GCS tmux layout

**New file:** `docker/config/tmuxinator/gcs.yml`

Tmux layout for ground station:
- Top pane: `tui_status_node --ros-args -p uav_namespace:=/uav1`
- Bottom pane: `tui_status_node --ros-args -p uav_namespace:=/uav2`
- (Or side-by-side for wider terminals)

### Step 7: Makefile + .env updates

**Modified file:** `docker/.env`
- Add `NUM_UAVS=2`

**Modified file:** `docker/Makefile`
- Add `multi-sitl` target: `docker compose -f compose/docker-compose.multi-sitl.yml up`
- Add `multi-sitl-down` target

---

## Files Summary

| File | Action | Purpose |
|------|--------|---------|
| `peregrine_bringup/launch/multi_uav_sitl.launch.py` | New | Multi-PX4 SITL spawner |
| `docker/compose/docker-compose.multi-sitl.yml` | New | 4-service compose |
| `docker/config/zenoh/uav_bridge.json5` | New | Per-UAV Zenoh allowlist |
| `docker/config/zenoh/gcs_bridge.json5` | New | GCS Zenoh subscriber |
| `docker/config/tmuxinator/gcs.yml` | New | GCS tmux layout |
| `docker/docker/Dockerfile.simulation` | Modify | Add zenoh-bridge binary |
| `docker/config/entrypoint.sh` | Modify | Optional Zenoh startup |
| `docker/.env` | Modify | Add NUM_UAVS |
| `docker/Makefile` | Modify | Add multi-sitl targets |

---

## Key Decisions

1. **MicroXRCE Agent runs in sim container**, not UAV containers — because PX4 SITL and the agent need to be on the same UDP port/host. UAV containers just join the DDS domain.
2. **`network_mode: host` on all containers** — simplest for Zenoh peer discovery and DDS domain isolation. No Docker networking complexity.
3. **Zenoh in peer mode** (no dedicated router) — for 2-3 UAVs on localhost, peer-to-peer TCP is sufficient. Add a router later if scaling to 10+.
4. **GCS gets its own DDS domain (99)** — completely isolated, only sees what Zenoh bridges in. Prevents accidental command injection.

---

## Verification

```bash
# Build image with Zenoh bridge
cd docker && make build-sim

# Start everything
make multi-sitl

# Verify DDS isolation (inside uav1 container):
ros2 topic list  # Should see ONLY uav1's topics, no uav2

# Verify DDS isolation (inside uav2 container):
ros2 topic list  # Should see ONLY uav2's topics, no uav1

# Verify GCS (inside gcs container):
ros2 topic list  # Should see /uav1/* AND /uav2/* topics

# Verify TUI:
# GCS tmux should show two TUI panes, each showing one UAV's data
# Both should have position data at different coordinates

# Kill one PX4 instance:
# Corresponding TUI should show STALE indicators
# Other UAV continues normally
```

---

## Future Extensions (not in scope now)

- **multi_agent_coordinator node** — BVC collision avoidance, state sharing via Zenoh
- **Zenoh router** — for scaling beyond 3-4 UAVs or cross-network deployments
- **Per-UAV Dockerfile variants** — Jetson/RPi5 containers joining the same Zenoh mesh
- **Command bridging** — GCS sending arm/takeoff/land commands back to individual UAVs via Zenoh
- **Multi-UAV TUI mode** — single TUI showing all UAVs in a dashboard view
