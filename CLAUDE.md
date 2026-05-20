# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**CPSL_ROS2_ArduPilot** is a ROS2 package providing telemetry integration between an ArduCopter flight controller (CubePilot CubeOrange+ on Aurelia X6 hexacopter) and ROS2. The package bridges MAVLink communication via MAVROS to expose odometry, transform frames, and diagnostics.

### Key Characteristics
- **Target Platform**: Aurelia X6 hexacopter with CubePilot CubeOrange+ (ArduCopter V4.5.4)
- **ROS2 Distro**: Jazzy
- **Companion OS**: Ubuntu 24.04
- **Physical Interface**: USB micro-USB connection (`/dev/ttyACM0`) at 115200 baud
- **Build System**: `ament_python` (Python-based ROS2 package)
- **Scope**: Telemetry only (odometry, TF frames, diagnostics). Control is out of scope.

## Repository Structure

The project follows the standard ROS2 workspace layout:
```
CPSL_ROS2_ArduPilot/
├── src/
│   └── aurelia_x6_mavlink/          # Single ROS2 package
│       ├── package.xml              # Package metadata and dependencies
│       ├── setup.py                 # Python package configuration
│       ├── config/
│       │   └── mavros_params.yaml   # MAVROS node parameter file (APM plugin set)
│       ├── launch/
│       │   ├── mavros.launch.py     # MAVROS standalone launcher
│       │   └── aurelia_x6.launch.py # Entry point: MAVROS + odom + diagnostics
│       ├── aurelia_x6_mavlink/
│       │   ├── __init__.py
│       │   ├── odom_tf_node.py      # Core node: odometry republisher + TF broadcaster
│       │   └── diagnostics_node.py  # Health check logger
│       └── README.md
└── plan.md                          # Detailed implementation specification
```

## Build Commands

```bash
# Install system dependencies (one-time)
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Install workspace dependencies (from repository root)
rosdep install -i --from-path src --rosdistro jazzy -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash   # or setup.zsh for zsh shells
```

## Running the Package

### Basic Launch
```bash
# Without namespace
ros2 launch aurelia_x6_mavlink aurelia_x6.launch.py

# With namespace (typical usage)
ros2 launch aurelia_x6_mavlink aurelia_x6.launch.py namespace:=cpsl_uav_1

# With QGroundControl coexistence (MAVROS forwards MAVLink to UDP)
ros2 launch aurelia_x6_mavlink aurelia_x6.launch.py \
    namespace:=cpsl_uav_1 \
    gcs_url:=udp://@14550
```

### Launch Arguments
- `namespace` (default: `''`) — Robot namespace for topic remapping (e.g., `cpsl_uav_1`)
- `fcu_url` (default: `/dev/ttyACM0:115200`) — Flight controller device path; use stable symlink for production: `/dev/serial/by-id/usb-CubePilot_CubeOrange+_4B003B000651323138363132-if00:115200`
- `gcs_url` (default: `""`) — QGroundControl UDP forward address (set to `udp://@14550` if QGC must coexist)
- `diagnostics_enable` (default: `true`) — Enable the optional diagnostics health-check node

### Verification Checklist
```bash
# 1. Device is present and readable
ls -la /dev/ttyACM0

# 2. User has dialout permission
groups $USER | grep dialout
# If missing: sudo usermod -aG dialout $USER  (then log out/in)

# 3. MAVROS heartbeat is live
ros2 topic echo /mavros/state --once

# 4. Local odometry publishing at expected rate
ros2 topic hz /mavros/local_position/odom

# 5. Namespace-remapped odom topic is live
ros2 topic hz /cpsl_uav_1/odom

# 6. TF tree is correct
ros2 run tf2_tools view_frames
# Expected: odom → base_link, odom → base_footprint (namespace prefixed)

# 7. GPS connectivity (if outdoors or near window)
ros2 topic echo /mavros/global_position/global --once
```

## Core Architecture

### 1. MAVROS Configuration (`config/mavros_params.yaml`)
- Uses **APM plugin set** (not px4) for ArduCopter-specific MAVLink handling
- Only essential plugins enabled: `sys_status`, `sys_time`, `imu`, `local_position`, `global_position`
- Stream rate parameters must be set on flight controller **before** launching:
  - `SR0_POSITION`: 2 Hz → **10 Hz** (local position for odometry)
  - `SR0_EXTRA1`: 4 Hz → **10 Hz** (attitude: roll/pitch/yaw)
  - `SR0_EXTRA2`: 4 Hz → **10 Hz** (airspeed)
  - `SR0_EXT_STAT`: 2 Hz → **5 Hz** (extended status)
  - `SR0_RAW_SENS`: 2 Hz → **10 Hz** (raw IMU)

### 2. Odom/TF Node (`aurelia_x6_mavlink/odom_tf_node.py`)
The core deliverable. Republishes and exposes TF frames at 10 Hz.

**Subscriptions:**
- `/mavros/local_position/odom` (`nav_msgs/Odometry`) — Already in ENU (MAVROS handles NED→ENU conversion)
- `/mavros/global_position/global` (`sensor_msgs/NavSatFix`) — GPS fix quality for `map→odom` broadcasting

**Publications:**
- `<namespace>/odom` (`nav_msgs/Odometry`) — Namespace-relative odometry republish

**TF Broadcasts (10 Hz):**
```
[map] ──(when GPS available)──► [odom]       (identity initially, GPS-driven in future)
[odom] ────────────────────────► [base_link]      (full 6DOF: position + orientation from EKF)
[odom] ────────────────────────► [base_footprint] (yaw-only, z=0; mirrors PX4 convention)
```

All frame IDs are namespace-prefixed (e.g., `cpsl_uav_1/odom`, `cpsl_uav_1/base_link`).

**Coordinate Frames:**
- `odom` = EKF's local origin (where FC was powered on)
- `map` = Global/inertial frame (identity to `odom` until GPS provides correction)
- `base_link` = Vehicle center with full 6DOF from EKF position + attitude
- `base_footprint` = Vehicle projection with yaw-only rotation and z=0

### 3. Diagnostics Node (`aurelia_x6_mavlink/diagnostics_node.py`)
Optional health-check logger. Subscribes to:
- `/mavros/state` (1 Hz) — Connection status, armed/guided flags
- `<namespace>/odom` (1 Hz) — Position (x, y, z)
- `/mavros/global_position/global` (0.2 Hz) — GPS fix type

Emits `WARN` log if no heartbeat from `/mavros/state` within 5 seconds. No side effects; can be launched/killed at runtime.

## Topics Exposed After Launch

| Topic | Type | Frame Parent | Rate |
|-------|------|--------------|------|
| `<ns>/odom` | `nav_msgs/Odometry` | `<ns>/odom` → `<ns>/base_link` | 10 Hz |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | `map` → `base_link` | 10 Hz |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `map` | 10 Hz |
| `/mavros/imu/data` | `sensor_msgs/Imu` | body | 10 Hz |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | WGS84 | 5 Hz |
| `/mavros/state` | `mavros_msgs/State` | — | 1 Hz |
| `/mavros/battery` | `sensor_msgs/BatteryState` | — | 2 Hz |

## Package Dependencies

**ROS2 dependencies** (in `package.xml`):
- `mavros` — Flight controller bridging
- `mavros_msgs` — ArduPilot-specific message types
- `nav_msgs` — Odometry and standard navigation messages
- `geometry_msgs` — Transforms and poses
- `sensor_msgs` — IMU and GPS data
- `tf2_ros` — Transform tree broadcasting
- `python3-scipy` — Quaternion/rotation mathematics (same as `px4_controller`)

## Important Constraints & Pre-flight Requirements

1. **Flight Controller Parameters**: Must be set via Mission Planner or QGroundControl **before** launching ROS2 (see "MAVROS Configuration" section above). These control MAVLink stream rates on the USB port.

2. **Device Permission**: User must be in `dialout` group to access `/dev/ttyACM0`:
   ```bash
   sudo usermod -aG dialout $USER
   # Log out and back in to take effect
   ```

3. **USB-Only Connection**: No UART companion link is currently wired. Future phase will route UART to SERIAL5 once physical wiring is complete.

4. **QGroundControl Conflict**: If QGC holds `/dev/ttyACM0` simultaneously, MAVROS cannot connect. Either:
   - Close QGC first, OR
   - Set `gcs_url:=udp://@14550` so MAVROS holds the port and forwards telemetry to QGC over UDP

5. **Firmware is Signed**: Aurelia signs ArduCopter builds from v4.5.0. Do not attempt custom firmware or AP_DDS (requires custom build). Work within MAVLink2 + MAVROS only.

6. **GPS Indoors**: FC shows pre-arm GPS warnings when USB-only and indoors. This does not block MAVLink communication or MAVROS publishing. Treat `map→odom` as identity (or suppress) when `/mavros/global_position/global` reports `NO_FIX`.

7. **Device Node Stability**: `/dev/ttyACM0` is assigned dynamically at boot. For production, use stable symlink: `/dev/serial/by-id/usb-CubePilot_CubeOrange+_4B003B000651323138363132-if00`

## Planned Future Phases

1. **Phase 2**: UART companion link via SERIAL5 (pending physical wiring)
2. **Phase 3**: GPS-driven `map→odom` transform from global position estimate
3. **Phase 4**: Control node (arming, takeoff, velocity setpoints; modeled on `px4_control_node.py`)
4. **Phase 5**: Integration with nav2 and sensor bringup launches
5. **Phase 6**: AP_DDS native integration (if Aurelia enables `DDS_ENABLE` in firmware)

## Python Conventions

- Uses `scipy.spatial.transform.Rotation` for quaternion and rotation mathematics (consistent with `px4_controller`)
- Follows ROS2 Python naming conventions
- Namespace handling passed as launch argument, applied to all topics and frame IDs
