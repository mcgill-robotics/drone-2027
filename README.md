# drone-2027

McGill Robotics drone companion-computer code (Jetson), ported from drone-2026.

It talks to the PX4 flight controller through PX4's **uXRCE-DDS bridge**:
`MicroXRCEAgent` runs on the Jetson, and PX4's internal messages appear as ROS 2
topics under `/fmu/out` and `/fmu/in` with `px4_msgs` types. MAVROS is gone.

> **Before flying:** do step 0 in `docs/px4_setup.md`. The exact PX4 firmware version
> and whether the Jetson reaches the flight controller over serial or Ethernet are not
> confirmed yet.

## Layout

```
docs/                     px4_setup.md, arming.md, actuators.md, sitl.md
docker/Dockerfile         ROS 2 Humble + Micro-XRCE-DDS-Agent
src/px4_msgs/             PX4 message definitions (git submodule, pinned to firmware release/1.17)
src/drone/                ROS 2 package with the core flight engine
  drone/px4/              PX4 interface: telemetry, commands, offboard, frames, agent
  drone/flight_checks/    The 9-step testing ladder (check_link -> check_lap)
  launch/                 agent.launch.py
  test/                   Pure logic unit tests (no ROS needed)
```

## Requirements (Jetson or Ubuntu 22.04)

- ROS 2 Humble
- Micro-XRCE-DDS-Agent **v2.4.2**. v3.x is not compatible with PX4's client.

  ```sh
  git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  cd Micro-XRCE-DDS-Agent && mkdir build && cd build
  cmake .. && make && sudo make install && sudo ldconfig /usr/local/lib/
  ```

## Build

```sh
git clone --recurse-submodules git@github.com:mcgill-robotics/drone-2027.git
cd drone-2027
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means Python edits take effect without rebuilding. Rebuild after
adding files, entry points or launch files.

## Flight Checks Ladder

Every check takes a link flag: `--sitl`, `--serial [DEVICE]` or `--udp [PORT]`, and
starts the agent itself unless `--no-agent` is given.

Run the checks in numerical order from Step 1 to Step 9:

| Step | Script | Purpose | Arming / Motors |
| :--- | :--- | :--- | :--- |
| **1** | `ros2 run drone check_link --sitl` | Verifies MicroXRCEAgent & PX4 FMU bridge connection | Zero risk (Motors off) |
| **2** | `ros2 run drone check_telemetry --sitl` | Validates sensor health, GPS fix, EKF state, battery | Zero risk (Motors off) |
| **3** | `ros2 run drone check_setpoints --sitl` | Tests ENU $\leftrightarrow$ NED setpoint coordinate calculations | Zero risk (Dry-run math) |
| **4** | `ros2 run drone check_offboard --sitl` | Tests 10 Hz OFFBOARD heartbeat handshake with PX4 | Zero risk (Motors disarmed) |
| **5** | `ros2 run drone check_arm --sitl --api` | Tests arming/disarming interlock | Low risk (Props removed!) |
| **6** | `ros2 run drone check_hover --sitl --api --altitude 3` | Arms, takes off to 3m, hovers rock-steady, lands | First flight test |
| **7** | `ros2 run drone check_goto_gps --sitl --api` | Commands navigation to a GPS waypoint and holds | Position navigation |
| **8** | `ros2 run drone check_gps_movement --sitl --api` | Flies sequential GPS path waypoints | Trajectory tracking |
| **9** | `ros2 run drone check_lap --sitl --api` | Executes full perimeter lap and Return-to-Launch | Complete flight loop |

`--api` arms from code and is strictly refused without `--sitl`: on the real
drone the safety pilot arms exclusively with the physical RC transmitter switch (`docs/arming.md`).

Simulator setup: `docs/sitl.md`. Drone hardware setup: `docs/px4_setup.md`.

## Coordinate frames

All high-level code works in **ENU** (x East, y North, z Up), as standard in robotics.
PX4's internal topics work in aviation **NED** (North, East, Down). Conversions are
strictly isolated to `src/drone/drone/px4/frames.py` and `setpoints.py`.

## Tests

```sh
pip install pytest
pytest                         # ROS-free unit tests
```
