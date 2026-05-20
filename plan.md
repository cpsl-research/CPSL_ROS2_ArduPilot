# CPSL_ROS2_ArduPilot — Implementation Plan

## Platform Summary

| Component | Detail |
|---|---|
| UAV | Aurelia X6 (hexacopter, HEXA/X frame) |
| Flight Controller | CubePilot CubeOrange+ |
| Firmware | ArduCopter V4.5.4 (MAVLink 2) |
| Companion PC OS | Ubuntu 24.04 |
| ROS2 Distro | Jazzy |
| Physical Link | USB micro-USB → `/dev/ttyACM0` (baud 115200) |

---

## Goals

Provide a bare-bones ROS2 package that:
1. Connects to the ArduCopter flight controller via MAVROS over USB
2. Exposes odometry (x, y, z, roll, pitch, yaw, linear and angular velocities) as a standard `nav_msgs/Odometry` topic
3. Publishes a TF tree that mirrors the existing PX4 convention:
   - `odom → base_link → base_footprint` (local/EKF odometry)
   - `map → odom` (only when GPS global position is available)
4. Supports robot namespacing (e.g. `cpsl_uav_1`) so downstream sensor and nav2 packages work unchanged

Control (arming, takeoff, landing, setpoints) is **out of scope** for this phase.

---

## Repository Layout

```
CPSL_ROS2_ArduPilot/
└── src/
    └── aurelia_x6_mavlink/          # sole ROS2 package
        ├── package.xml
        ├── setup.py
        ├── config/
        │   └── mavros_params.yaml   # MAVROS node configuration
        ├── launch/
        │   ├── mavros.launch.py     # launches only the MAVROS node
        │   └── aurelia_x6.launch.py # entry-point: MAVROS + odom node
        ├── aurelia_x6_mavlink/
        │   ├── __init__.py
        │   ├── odom_tf_node.py      # thin republisher + TF broadcaster
        │   └── diagnostics_node.py  # optional connection health logger
        └── README.md
```

---

## Pre-Flight: ArduPilot Parameter Changes

Set these via Mission Planner or QGroundControl **before** running the ROS2 package.
They increase stream rates on the USB port from the slow defaults to robotics-usable rates.
All changes are non-destructive and reversible.

| Parameter | Current | Target | Reason |
|---|---|---|---|
| `SR0_POSITION` | 2 Hz | 10 Hz | Local position for odometry |
| `SR0_EXTRA1` | 4 Hz | 10 Hz | Attitude (roll/pitch/yaw) |
| `SR0_EXTRA2` | 4 Hz | 10 Hz | VFR HUD / airspeed |
| `SR0_EXT_STAT` | 2 Hz | 5 Hz | Extended status |
| `SR0_RAW_SENS` | 2 Hz | 10 Hz | Raw IMU |

Do **not** modify `SERIAL0_PROTOCOL` (MAVLink2) or `SERIAL0_BAUD` (115200).

---

## Software Dependencies

Install before building the workspace:

```bash
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

Python dependencies (handled via `rosdep`):
- `scipy` — quaternion and rotation math (same as `px4_controller`)

---

## Package: `aurelia_x6_mavlink`

### Build type
`ament_python` (Python package, consistent with `px4_controller`)

### `package.xml` dependencies
```xml
<depend>mavros</depend>
<depend>mavros_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>tf2_ros</depend>
<depend>python3-scipy</depend>
```

---

## Step 1 — MAVROS Configuration (`config/mavros_params.yaml`)

MAVROS is configured to use the **APM plugin set** (not px4) which handles
ArduCopter-specific MAVLink quirks. Only the plugins needed for odometry and
diagnostics are enabled to keep the node lightweight.

Key parameters:
```yaml
mavros:
  ros__parameters:
    fcu_url: "/dev/ttyACM0:115200"
    gcs_url: ""              # set to "udp://@14550" if QGC must coexist
    system_id: 255
    component_id: 240
    target_system_id: 1
    target_component_id: 1
    plugin_allowlist:
      - sys_status
      - sys_time
      - imu
      - local_position
      - global_position
```

`gcs_url` can be overridden at launch time to forward MAVLink to QGroundControl
over UDP so that only MAVROS holds the USB serial port (see launch section below).

---

## Step 2 — Launch Files

### `launch/mavros.launch.py`
Launches only the MAVROS node. Accepts two arguments:
- `fcu_url` — default `/dev/ttyACM0:115200`; can be overridden to the stable
  symlink `/dev/serial/by-id/usb-CubePilot_CubeOrange+_4B003B000651323138363132-if00:115200`
- `gcs_url` — default `""` (no forwarding); set to `"udp://@14550"` to let QGC
  coexist on the same USB connection

### `launch/aurelia_x6.launch.py` (entry point)
Includes `mavros.launch.py` and additionally launches:
- `odom_tf_node` — the thin republisher/TF broadcaster
- `diagnostics_node` — connection health logger (optional, enabled by arg)

Launch arguments:
| Argument | Default | Description |
|---|---|---|
| `namespace` | `''` | Robot namespace (e.g. `cpsl_uav_1`) |
| `fcu_url` | `/dev/ttyACM0:115200` | Flight controller device path |
| `gcs_url` | `""` | Optional QGC UDP forward address |
| `diagnostics_enable` | `true` | Launch the diagnostics node |

---

## Step 3 — Odom / TF Node (`odom_tf_node.py`)

This node is the core deliverable. It mirrors the TF pattern in `px4_control_node.py`
but without any control logic.

### Subscriptions
| Topic | Type | Notes |
|---|---|---|
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | MAVROS already converts NED→ENU |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | Used to determine GPS fix quality |

### Publications
| Topic | Type | Notes |
|---|---|---|
| `odom` | `nav_msgs/Odometry` | Namespace-relative (e.g. `cpsl_uav_1/odom`) |

### TF Broadcasts
Three transforms are broadcast at 10 Hz:

```
[map] ──(when GPS fix)──► [odom]
[odom] ──────────────────► [base_link]      (full 6DOF from EKF position + orientation)
[odom] ──────────────────► [base_footprint] (yaw-only, z=0 — same as PX4 convention)
```

All frame IDs are prefixed with the namespace when one is set
(e.g. `cpsl_uav_1/odom`, `cpsl_uav_1/base_link`, `cpsl_uav_1/base_footprint`).

### Coordinate Frame Note
MAVROS's `local_position/odom` is already in **ENU** with `map` as the parent
frame (ArduPilot NED→ENU is handled inside MAVROS). No manual NED→ENU conversion
is needed in `odom_tf_node.py` — unlike `px4_control_node.py` which had to
convert from raw PX4 NED messages.

The `odom` frame used in the ROS TF tree corresponds to the EKF's local origin
(where the FC was powered on / EKF initialized). The `map` frame is linked to
`odom` by a static or near-static transform; initially it will be identity
(map ≡ odom). When a GPS fix is available, the `map→odom` broadcast can be
populated from `/mavros/global_position/local` in a future phase.

### `base_footprint` derivation
Identical to the PX4 pattern:
- Position: x and y from `base_link`, z = 0
- Orientation: yaw-only quaternion (roll and pitch zeroed out)

---

## Step 4 — Diagnostics Node (`diagnostics_node.py`)

Lightweight health-check node. Subscribes to:
- `/mavros/state` — logs connection status and armed/guided flags at 1 Hz
- `odom` (local, within namespace) — logs x, y, z position at 1 Hz
- `/mavros/global_position/global` — logs GPS fix type at 0.2 Hz

Emits a `WARN` log if no heartbeat is received from `/mavros/state` within 5 s.
This node has no side effects and can be launched or killed at any time.

---

## Topics Exposed After Launch

| Topic | Type | Frame | Rate |
|---|---|---|---|
| `<ns>/odom` | `nav_msgs/Odometry` | `<ns>/odom` → `<ns>/base_link` | 10 Hz |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | `map` → `base_link` | 10 Hz |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `map` | 10 Hz |
| `/mavros/imu/data` | `sensor_msgs/Imu` | body | 10 Hz |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | WGS84 | 5 Hz |
| `/mavros/state` | `mavros_msgs/State` | — | 1 Hz |
| `/mavros/battery` | `sensor_msgs/BatteryState` | — | 2 Hz |

---

## Known Constraints

1. **USB only** — no UART companion link is wired. `/dev/ttyACM0` is the only
   connection. A future hardware phase will route a UART cable to SERIAL5 for
   an onboard companion computer (verify physical connector availability first).

2. **QGC conflict** — if QGroundControl holds `/dev/ttyACM0` simultaneously,
   MAVROS cannot connect. Either close QGC first, or set `gcs_url:=udp://@14550`
   so MAVROS holds the port and forwards telemetry to QGC over UDP.

3. **Firmware is signed** — Aurelia signs their ArduCopter builds from v4.5.0.
   Do not attempt to flash custom firmware or enable AP_DDS (requires a custom
   build). Work within MAVLink2 + MAVROS only.

4. **GPS indoors** — the FC will show pre-arm GPS warnings when USB-only and
   indoors. This does not block MAVLink communication or MAVROS topic publishing.
   The `map→odom` TF broadcast should be treated as identity (or suppressed)
   when `/mavros/global_position/global` reports `NO_FIX`.

5. **Device node stability** — `/dev/ttyACM0` is assigned dynamically at boot.
   For production, use the stable symlink:
   `/dev/serial/by-id/usb-CubePilot_CubeOrange+_4B003B000651323138363132-if00`

---

## Build & Run

```bash
# one-time workspace setup
cd ~/Documents/CPSL_ROS2_ArduPilot
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash   # or setup.zsh

# launch (with namespace)
ros2 launch aurelia_x6_mavlink aurelia_x6.launch.py namespace:=cpsl_uav_1

# launch with QGC coexistence
ros2 launch aurelia_x6_mavlink aurelia_x6.launch.py \
    namespace:=cpsl_uav_1 \
    gcs_url:=udp://@14550
```

---

## Verification Checklist

```bash
# 1. Device is present and readable
ls -la /dev/ttyACM0

# 2. User has dialout permission
groups $USER | grep dialout
# If missing: sudo usermod -aG dialout $USER  (then log out/in)

# 3. MAVROS heartbeat is live
ros2 topic echo /mavros/state --once

# 4. Local odometry is publishing at expected rate
ros2 topic hz /mavros/local_position/odom

# 5. Namespace-remapped odom topic is live
ros2 topic hz /cpsl_uav_1/odom

# 6. TF tree is publishing correct frames
ros2 run tf2_tools view_frames
# Expect: odom → base_link, odom → base_footprint (namespace prefixed)

# 7. GPS (outdoors or near window)
ros2 topic echo /mavros/global_position/global --once
```

---

## Future Phases (out of scope here)

| Phase | Description |
|---|---|
| 2 | UART companion link via SERIAL5 once physically wired |
| 3 | `map→odom` transform driven by a real GPS/EKF2 global estimate |
| 4 | Control node (arming, takeoff, velocity setpoints) — modelled on `px4_control_node.py` |
| 5 | Integration with nav2 and sensor bringup launches |
| 6 | AP_DDS native integration if Aurelia confirms `DDS_ENABLE` in firmware |

---

## Helpful Links

- [MAVROS APM Plugin Documentation](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md)
- [ArduCopter MAVLink Parameter Reference](https://ardupilot.org/copter/docs/parameters.html)
- [MAVROS ROS2 Migration Guide](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md#ros2)
- [ArduPilot Companion Computer Setup](https://ardupilot.org/dev/docs/companion-computers.html)
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [CubePilot CubeOrange+ Documentation](https://docs.cubepilot.org/user-guides/autopilot/the-cube-orange)
