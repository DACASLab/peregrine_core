# Hardware Wi-Fi Demo Runbook

Verified working on 2026-04-08 for:
- CubeOrange+ running PX4
- Orin companion running the `peregrine` systemd service
- Kahuna MAVLink Wi-Fi bridge
- Host laptop running the GCS container over Wi-Fi only

## 1. Working Topology

```text
CubeOrange+ (PX4)
  |  TELEM2, 921600, uXRCE DDS
  v
CP2102 USB-UART adapter on Orin
  |  /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
  v
Orin companion
  - Wi-Fi AP SSID: falinks
  - AP IP: 192.168.1.10
  - Zenoh bridge port: 7447
  - ROS domain: 1

Kahuna MAVLink Wi-Fi bridge
  - Station mode on falinks
  - Static IP: 192.168.1.20
  - UDP host port: 14550
  - UDP client port: 14555
  - UART baud: 57600

Host laptop
  - Connects to SSID falinks using profile falinks-host
  - Gets DHCP on 192.168.1.x
  - Runs GCS Zenoh bridge on ROS domain 99
```

## 2. Network Model

There are two different Wi-Fi profiles with similar names:

- `falinks-ap`
  - Lives on the Orin.
  - Makes the Orin behave as an access point.
  - Uses `ipv4.method shared`.
  - Owns the fixed AP-side address `192.168.1.10/24`.

- `falinks-host`
  - Lives on the host laptop.
  - Makes the laptop join the Orin AP as a client.
  - Uses DHCP, so the host address is not fixed.
  - Example working lease: `192.168.1.110`.

Important:
- Do not rely on the host always being `192.168.1.110`.
- Do rely on the Orin being `192.168.1.10`.
- Do rely on Kahuna being `192.168.1.20`.
- Keep the Orin AP on 2.4 GHz. Kahuna is ESP8266-based and should be treated as 2.4 GHz only.

## 3. Known-Good Orin Config

Orin NetworkManager profile:

```bash
nmcli connection show falinks-ap
```

Expected important fields:

```text
connection.id: falinks-ap
802-11-wireless.ssid: falinks
802-11-wireless.mode: ap
ipv4.method: shared
ipv4.addresses: 192.168.1.10/24
connection.autoconnect: yes
```

Working Jetson override file on the Orin:

`~/robotics/peregrine/docker/.env.local`

```bash
USER_UID=1001
USER_GID=1001
DRONE_ID=1
ROS_LOCALHOST_ONLY=1
START_XRCE_AGENT=true
XRCE_DEVICE=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
XRCE_BAUD=921600
```

Source-of-truth service launch files:

- `docker/config/start_flight_stack.sh`
- `docker/config/tmuxinator/flight.yml`
- `docker/compose/docker-compose.jetson.yml`

Important behavior:
- the aircraft ROS graph runs in domain `DRONE_ID`
- the launch wrapper forces `uav_namespace:=/uav${DRONE_ID}`
- with `DRONE_ID=1`, the aircraft topics are `/uav1/...`

## 4. Known-Good PX4 and Kahuna Config

PX4 side:
- `TELEM2` is used for uXRCE DDS
- `SER_TEL2_BAUD=921600`
- `UXRCE_DDS_CFG=102` for `TELEM2`
- `UXRCE_DDS_DOM_ID=1`
- `UXRCE_DDS_KEY=1`
- `UXRCE_DDS_PTCFG=1`
- `MAV_1_CONFIG=0` so `TELEM2` is not being used for MAVLink

Kahuna side:

```text
WIFI_MODE=STA
WIFI_CHANNEL=11
WIFI_UDP_HPORT=14550
WIFI_UDP_CPORT=14555
WIFI_IPADDRESS=192.168.1.20
WIFI_SSIDSTA1=falinks
WIFI_PWDSTA1=falinks3
WIFI_IPSTA=192.168.1.20
WIFI_GATEWAYSTA=192.168.1.10
WIFI_SUBNET_STA=255.255.255.0
UART_BAUDRATE=57600
```

Kahuna fallback AP:
- SSID: `DEXTER_3`
- Password: `Dexter@3`

## 5. Known-Good Host GCS Behavior

Host Wi-Fi profile:

```bash
nmcli connection show falinks-host
```

Expected important fields:

```text
connection.id: falinks-host
802-11-wireless.ssid: falinks
ipv4.method: auto
connection.autoconnect: yes
connection.autoconnect-priority: 100
```

Host GCS behavior:
- host GCS ROS graph runs in domain `99`
- aircraft ROS graph runs in domain `1`
- Zenoh bridges move selected `/uav1/...` topics across the network

This means:
- local checks on the Orin use `ROS_DOMAIN_ID=1`
- local checks in the host GCS container use `ROS_DOMAIN_ID=99`

## 6. Normal Demo Startup

1. Power the UAV from batteries.
2. Wait for SSID `falinks` to appear.
3. Connect the host laptop to `falinks`.
4. Verify Orin reachability:

```bash
ping -c 2 192.168.1.10
ssh falinks@192.168.1.10
```

5. Verify Orin flight service:

```bash
ssh falinks@192.168.1.10 'systemctl is-active peregrine'
ssh falinks@192.168.1.10 'docker ps --format "table {{.Names}}\t{{.Status}}"'
```

6. Verify aircraft ROS on the Orin:

```bash
ssh falinks@192.168.1.10 \
  'docker exec ros2-px4-flight-aircraft-1 bash -lc "
    export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source /ros2_ws/install/setup.bash >/dev/null 2>&1
    ros2 topic echo --once /uav1/estimated_state --no-daemon
  "'
```

7. Start or restart the host GCS:

```bash
cd /scratch/robotics/peregrine/docker
python3 scripts/generate_gcs_config.py --mode hardware --uav-ips 192.168.1.10
GCS_MODE=hardware GCS_UAV_IPS=192.168.1.10 \
  docker compose --env-file .env --env-file .env.local \
  -f compose/docker-compose.gcs.yml up -d gcs
```

8. Verify bridged ROS from the host:

```bash
docker exec ros2-px4-flight-gcs bash -lc '
  export ROS_DOMAIN_ID=99
  source /opt/ros/humble/setup.bash >/dev/null 2>&1
  source /ros2_ws/install/setup.bash >/dev/null 2>&1
  ros2 topic echo --once /uav1/estimated_state --no-daemon
'
```

9. Verify Kahuna MAVLink from the host:

```bash
python3 - <<'PY'
from pymavlink import mavutil
import time
recv = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=252, source_component=190)
send = mavutil.mavlink_connection('udpout:192.168.1.20:14555', source_system=252, source_component=190)
for _ in range(5):
    send.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0,0,0)
    time.sleep(0.2)
for _ in range(8):
    msg = recv.recv_match(blocking=True, timeout=1)
    if msg:
        print(msg.get_type())
PY
```

## 7. Restart Commands

Restart the host GCS only:

```bash
cd /scratch/robotics/peregrine/docker
docker compose --env-file .env --env-file .env.local \
  -f compose/docker-compose.gcs.yml restart gcs
```

Restart the Orin flight service:

```bash
ssh falinks@192.168.1.10 'echo falinks | sudo -S systemctl restart peregrine'
```

Recreate the Orin aircraft container if the service restart is not enough:

```bash
ssh falinks@192.168.1.10 '
  echo falinks | sudo -S systemctl stop peregrine &&
  docker rm -f ros2-px4-flight-aircraft-1 >/dev/null 2>&1 || true &&
  echo falinks | sudo -S systemctl start peregrine
'
```

## 8. Tomorrow Emergency Recovery

If the demo network is not healthy, use this order and stop as soon as it recovers.

### 1. Force the host back to `falinks`

```bash
nmcli connection up falinks-host
ping -c 2 192.168.1.10
```

Expected:
- host Wi-Fi is `falinks-host`
- Orin answers at `192.168.1.10`

### 2. If `falinks` is missing, force the Orin AP back up

If you still have some path into the Orin, run:

```bash
echo falinks | sudo -S nmcli connection up falinks-ap
nmcli connection show falinks-ap
```

Expected:
- SSID `falinks`
- `ipv4.method shared`
- Orin AP IP `192.168.1.10/24`

### 3. If the Orin booted on lab Wi-Fi instead of `falinks`

Use the lab address:

```bash
ssh falinks@192.168.0.202
echo falinks | sudo -S nmcli connection up falinks-ap
```

Then reconnect the host:

```bash
nmcli connection up falinks-host
```

### 4. Restart the flight stack on the Orin

```bash
ssh falinks@192.168.1.10 'echo falinks | sudo -S systemctl restart peregrine'
ssh falinks@192.168.1.10 'systemctl is-active peregrine'
ssh falinks@192.168.1.10 'docker ps --format "table {{.Names}}\t{{.Status}}"'
```

### 5. Quick ROS health check on the Orin

```bash
ssh falinks@192.168.1.10 \
  'docker exec ros2-px4-flight-aircraft-1 bash -lc "
    export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source /ros2_ws/install/setup.bash >/dev/null 2>&1
    ros2 topic echo --once /uav1/status --no-daemon
    echo ---
    ros2 topic echo --once /uav1/uav_state --no-daemon
  "'
```

Expected:
- `connected: true` in `/uav1/status`
- `/uav1/uav_state` returns a message

### 6. Bring up the Orin TUI

```bash
ssh falinks@192.168.1.10
docker exec -it ros2-px4-flight-aircraft-1 bash -lc 'tmux resize-window -t flight:2 -x 120 -y 40; tmux select-window -t flight:2; tmux attach -t flight'
```

Inside tmux:
- `Ctrl+b`, then `2` for TUI
- `Ctrl+b`, then `0` for stack logs
- `Ctrl+b`, then `d` to detach

### 7. Kahuna quick check

```bash
ping -c 3 192.168.1.20
python3 - <<'PY'
from pymavlink import mavutil
recv = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=252, source_component=190)
for _ in range(6):
    msg = recv.recv_match(blocking=True, timeout=1)
    if msg:
        print(msg.get_type())
PY
```

Expected:
- Kahuna answers at `192.168.1.20`
- MAVLink types such as `HEARTBEAT`, `ATTITUDE`, `GPS_RAW_INT` appear

## 9. Network Troubleshooting

### A. Host is on the wrong Wi-Fi

Symptom:
- `falinks` is visible, but the host is still on lab Wi-Fi
- `ping 192.168.1.10` fails

Fix:

```bash
nmcli connection up falinks-host
```

Recommended persistent setting:

```bash
nmcli connection modify falinks-host connection.autoconnect yes
nmcli connection modify falinks-host connection.autoconnect-priority 100
```

### B. `falinks` SSID is not visible

Symptom:
- the host cannot even see the Orin AP

Checks:
- Orin is powered
- Orin finished booting
- `falinks-ap` still exists and has `autoconnect=yes`

If you have another way into the Orin:

```bash
sudo nmcli connection up falinks-ap
nmcli connection show falinks-ap
```

### C. Orin is reachable but `peregrine` is not healthy

Checks:

```bash
systemctl is-active peregrine
docker ps
docker logs --tail 120 ros2-px4-flight-aircraft-1
```

If ROS looks empty on the Orin, always query with:

```bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1
```

### D. Host GCS looks empty

Most common cause:
- you queried the GCS container with the wrong ROS domain

Host GCS queries must use:

```bash
export ROS_DOMAIN_ID=99
```

If you use `ROS_DOMAIN_ID=1` inside `ros2-px4-flight-gcs`, the graph will look empty even when the bridge is working.

### E. Kahuna does not rejoin `falinks`

What we observed:
- this can happen after a power cycle
- on the Orin AP, `ip neigh show dev wlP1p1s0` then shows `192.168.1.20 FAILED`
- `iw dev wlP1p1s0 station dump` only shows the host laptop, not Kahuna

Useful checks:

```bash
ssh falinks@192.168.1.10 'iw dev wlP1p1s0 station dump'
ssh falinks@192.168.1.10 'ip neigh show dev wlP1p1s0'
ping -c 3 192.168.1.20
```

Things to verify:
- Orin AP is still `falinks`
- Orin AP is still 2.4 GHz
- Kahuna station credentials are still:
  - SSID `falinks`
  - password `falinks3`
  - static IP `192.168.1.20`
  - gateway `192.168.1.10`

Recovery path:
1. Wait up to about a minute after boot.
2. Scan for fallback AP `DEXTER_3`.
3. If `DEXTER_3` appears, Kahuna did not join `falinks`.
4. Connect to `DEXTER_3` using `Dexter@3`.
5. Open the Kahuna web UI and restore the station-mode values above.
6. Reboot Kahuna and reconnect the host back to `falinks`.

### F. Host USB to Cube changes device name

If you use host USB for PX4 maintenance:
- do not rely on `/dev/ttyACM0`
- after reboot, the Cube may return as `/dev/ttyACM1`

Use the stable by-id path instead:

```bash
/dev/serial/by-id/usb-CubePilot_CubeOrange+_0-if00
```

Also:
- close QGroundControl before doing raw serial tests
- QGC can hold the USB port open

## 10. Quick Acceptance Checklist

The demo is ready when all of these are true:

- host is connected to `falinks`
- `ping 192.168.1.10` succeeds
- `ssh falinks@192.168.1.10` works
- Orin `peregrine` service is `active`
- aircraft container publishes `/uav1/estimated_state`
- host GCS container receives `/uav1/estimated_state`
- Kahuna is reachable at `192.168.1.20`
- host receives MAVLink on `udpin:0.0.0.0:14550`
