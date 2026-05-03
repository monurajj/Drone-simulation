# Drone Simulation

ROS 2 workspace with Python nodes that talk to **PX4** over DDS using `px4_msgs`. The intent is autonomous and semi-autonomous quadrotor behaviour in simulation (takeoff, teleoperation, LiDAR-based sector distances, and simple forward-wall avoidance).

The repository was tested on **Ubuntu 22.04** with **ROS 2 Humble**. macOS hosts can clone and edit the workspace, but PX4 simulation and bridging are easiest on Linux/WSL following the PX4 toolchain.

---

## What's included

| Component | Executable | Purpose |
|-----------|-------------|---------|
| **Takeoff** | `takeoff_node` | Publishes offboard heartbeat + position setpoints for a smoothed climb to ~5 m altitude; switches to OFFBOARD and arms via `VehicleCommand`. |
| **Keyboard control** | `keyboard_control` | Arrow keys move in the horizontal plane (`W/S` altitude, `A/D` yaw). Sends land, RTL, disarm, etc. Uses `/scan` to reduce or block forward motion when obstacles are ahead. |
| **LiDAR processor** | `lidar_processor` | Subscribes to `sensor_msgs/LaserScan` on `/scan` and publishes eight sector minimum ranges on `/sector_distances`. |
| **Avoidance / safe-stop** | `avoidance_node` | Forward velocity offboard mode: cruises until the forward sector (±30°, ignores readings under ~0.5 m from the drone body) is closer than ~2 m, then stops; resumes when clearance exceeds ~2.5 m. |

### PX4 interfaces

Nodes publish/subscribe PX4-aligned topics:

- **`/fmu/in/offboard_control_mode`**, **`/fmu/in/trajectory_setpoint`**, **`/fmu/in/vehicle_command`**
- **`/scan`** — `LaserScan` (e.g. from Gazebo or a bridged simulator)

The takeoff node uses **BEST_EFFORT** reliability and **TRANSIENT_LOCAL** durability on outgoing PX4 pubs, which matches typical PX4 subscriber expectations.

---

## Repository layout

```
Drone-simulation/
├── README.md
├── PX4-Autopilot/       # placeholder; clone official PX4-Autopilot for SITL
├── drone_ws/
│   └── src/
│       ├── drone_control/   # this package (ament_python)
│       └── px4_msgs/       # clone from PX4 — see below
```

`PX4-Autopilot/` and `src/px4_msgs/` may be empty in a fresh checkout; populate them before building.

---

## Prerequisites

1. **ROS 2 Humble** (desktop install is enough): [Ubuntu (binary) ROS 2 Humble installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
2. **`px4_msgs` in your workspace**, from the same ROS 2 distro / PX4 line you use ([PX4/px4_msgs](https://github.com/PX4/px4_msgs)). Example:

   ```bash
   cd drone_ws/src
   git clone https://github.com/PX4/px4_msgs.git
   ```

   Pick a branch that matches your **PX4-Autopilot** version (avoid mixing incompatible message IDs).

3. **PX4 SITL + Gazebo** and the **XRCE-DDS / Micro XRCE Agent** bridge so ROS 2 and the simulated FCU share the DDS domain. Official setup: [PX4 ROS 2 user guide — development environment](https://docs.px4.io/main/en/ros2/).

4. A simulated **LaserScan on `/scan`** if you run `lidar_processor`, `avoidance_node`, or `keyboard_control` with safety features enabled.

Workspace dependencies declared in [`drone_control/package.xml`](drone_ws/src/drone_control/package.xml): `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `px4_msgs`.

---

## Build

```bash
cd drone_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # optional, if rosdep configured
colcon build --symlink-install
source install/setup.bash
```

---

## Run

With PX4 bridged into the same DDS domain as your ROS shell (per PX4 docs), in separate terminals:

```bash
source /opt/ros/humble/setup.bash
source ~/path/to/drone_ws/install/setup.bash
```

Examples:

```bash
ros2 run drone_control takeoff_node
ros2 run drone_control keyboard_control
ros2 run drone_control lidar_processor
ros2 run drone_control avoidance_node
```

**Keyboard terminal:** run `keyboard_control` in a raw TTY-capable terminal (same machine as ROS); it adjusts `termios` for arrow-key input.

---

## Parameters you might tune

- **Takeoff:** climb profile and timings in [`takeoff_node.py`](drone_ws/src/drone_control/drone_control/takeoff_node.py) (`counter` thresholds and target `z`).
- **Avoidance:** `D_STOP`, `D_RESUME`, `V_FORWARD` at the top of [`avoidance_node.py`](drone_ws/src/drone_control/drone_control/avoidance_node.py).
- **Keyboard:** movement steps (`STEP_XY`, …) and `BRAKE_START` / `BRAKE_STOP` in [`keyboard_control.py`](drone_ws/src/drone_control/drone_control/keyboard_control.py).

---

## License

The `drone_control` package is marked **Apache-2.0** in `package.xml`. `px4_msgs` and PX4-Autopilot have their own licenses.

---

## See also

- [PX4 ROS 2 overview](https://docs.px4.io/main/en/ros2/overview.html)
- [Offboard mode](https://docs.px4.io/main/en/flight_modes/offboard.html)

If you extend this repo with launch files or a pinned PX4/`px4_msgs` branch, document those versions here so SITL and messages stay aligned.
