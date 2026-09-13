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
ui/                       browser control panel (talks to api_server on port 5000)
config/mediamtx.yml       camera streaming config
missions/mission1/        lap points, coverage area and progress (read and written by Mission 1)
docker/Dockerfile         ROS 2 Humble + Micro-XRCE-DDS-Agent
src/px4_msgs/             PX4 message definitions (git submodule, pinned to the firmware release)
src/drone/                ROS 2 package with all our code
  drone/px4/              PX4 interface: telemetry, commands, offboard, frames, agent
  drone/flight_checks/    check_link, check_telemetry, check_offboard, check_arm, check_hover, ...
  drone/mission1/         Mission 1 runner: laps, boustrophedon coverage, return
  drone/pathing/          lap TSP and coverage planners
  drone/api/              API server, target georeferencing, target registry
  drone/detection/        wet/dry target detector
  drone/avoidance/        obstacle avoidance node and core
  launch/                 agent.launch.py, oa_gazebo.launch.py
  test/                   unit tests (no ROS needed)
```

## Requirements (Jetson or Ubuntu 22.04)

- ROS 2 Humble
- Micro-XRCE-DDS-Agent **v2.4.2**. v3.x is not compatible with PX4's client.

  ```sh
  git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  cd Micro-XRCE-DDS-Agent && mkdir build && cd build
  cmake .. && make && sudo make install && sudo ldconfig /usr/local/lib/
  ```

- On the Jetson, `pyrealsense2` for the depth camera (build librealsense with Python bindings).
- MediaMTX for camera streaming (see below).

## Build

```sh
git clone --recurse-submodules git@github.com:mcgill-robotics/drone-2027.git
cd drone-2027
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
sudo apt install python3-flask-cors
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means Python edits take effect without rebuilding. Rebuild after
adding files, entry points or launch files.

## Run

Every tool takes a link flag: `--sitl`, `--serial [DEVICE]` or `--udp [PORT]`, and
starts the agent itself unless `--no-agent` is given.

| What | Command |
| --- | --- |
| Is PX4 reachable? | `ros2 run drone check_link --sitl` |
| Print telemetry | `ros2 run drone check_telemetry --sitl` |
| OFFBOARD without arming | `ros2 run drone check_offboard --sitl` |
| Hover test | `ros2 run drone check_hover --sitl --api` |
| Mission 1 | `ros2 run drone mission1 --sitl --api-arm` (drone: `--serial` or `--udp`, pilot arms) |
| Control panel backend | `ros2 run drone api_server --sitl`, then open `ui/index.html` |
| Obstacle avoidance (Gazebo) | `ros2 launch drone oa_gazebo.launch.py` |

`--api` / `--api-arm` arm from code and are refused without `--sitl`: on the real
drone the pilot arms with the RC switch (`docs/arming.md`).

Simulator setup: `docs/sitl.md`. Drone setup: `docs/px4_setup.md`.

## Coordinate frames

All mission code works in **ENU** (x East, y North, z Up), as it did with MAVROS.
PX4's topics are NED; the only conversion is in `src/drone/drone/px4/frames.py`.

## Tests and formatting

```sh
pip install pytest pyyaml
pytest                         # ROS-free unit tests, also run in CI
pre-commit run --all-files     # ruff format
```

## Camera feed

The Intel RealSense depth camera is streamed over WebRTC via MediaMTX. `api_server`
starts MediaMTX and the ffmpeg publishers automatically. The MediaMTX binary is not
committed; download the arm64 build into the repo root (or put it on `PATH`):

```sh
wget https://github.com/bluenviron/mediamtx/releases/download/v1.18.2/mediamtx_v1.18.2_linux_arm64v8.tar.gz
tar -xzf mediamtx_v1.18.2_linux_arm64v8.tar.gz mediamtx
rm mediamtx_v1.18.2_linux_arm64v8.tar.gz
```

`config/mediamtx.yml` configures the `depth`, `rgb` and `front_clean` streams.
Screenshots, the target registry and the MediaMTX log are written to `runtime/`
(override with `DRONE_RUNTIME_DIR`).
