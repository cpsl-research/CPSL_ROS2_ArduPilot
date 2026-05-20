# aurelia_x6_mavlink

ROS2 telemetry bridge for the Aurelia X6 hexacopter (CubePilot CubeOrange+, ArduCopter V4.5.4).

See the top-level [CLAUDE.md](../../CLAUDE.md) for full architecture, build instructions, launch arguments, and the verification checklist.

## Quick start

```bash
# Build
cd ~/Documents/CPSL_ROS2_ArduPilot
colcon build --symlink-install
source install/setup.zsh

# Launch with namespace
ros2 launch aurelia_x6_mavlink aurelia_x6.launch.py namespace:=cpsl_uav_1
```
