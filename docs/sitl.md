# Simulator: PX4 SITL + Gazebo

Run on Ubuntu 22.04 with ROS 2 Humble (a lab PC, the Jetson, or `docker/Dockerfile`).
macOS is not a practical host for PX4 SITL with Gazebo.

## One-time setup

1. PX4-Autopilot at the same release as `src/px4_msgs` (currently 1.17):

   ```sh
   git clone --recursive -b release/1.17 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
   bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
   ```

2. Micro-XRCE-DDS-Agent and this workspace: see the README.

SITL starts PX4's DDS client automatically and connects to UDP port 8888 on
localhost, which is what `--sitl` uses. Nothing needs to be configured in PX4.

## Flight checks

Terminal 1:

```sh
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Terminal 2:

```sh
cd drone-2027 && source install/setup.bash
ros2 run drone check_link --sitl
ros2 run drone check_telemetry --sitl --duration 10
ros2 run drone check_offboard --sitl
ros2 run drone check_arm --sitl --api
ros2 run drone check_hover --sitl --api
ros2 run drone check_goto_gps --sitl --api --lat 47.39785 --lon 8.54573 --alt 10
```

Each tool starts the agent itself. To run two tools at the same time, start the agent
once (`ros2 launch drone agent.launch.py`) and give both `--no-agent`.

`check_gps_movement` also flies to a GPS target; the Zurich coordinates above work
with SITL's default spawn:

```sh
ros2 run drone check_gps_movement --sitl --api --target 47.39785,8.54573,5
```

`check_lap` uses Montreal coordinates (`LAP_WAYPOINTS` in `check_lap.py`). SITL spawns
in Zurich by default, so move the simulated home there first:

```sh
PX4_HOME_LAT=45.5048 PX4_HOME_LON=-73.5772 PX4_HOME_ALT=30 make px4_sitl gz_x500
ros2 run drone check_lap --sitl --api
```

## Coordinate frames

Every getter, setpoint and target in `drone.px4` and the flight checks is **ENU**:
x = East, y = North, z = Up, relative to PX4's local origin. PX4's own topics are
NED; `src/drone/drone/px4/frames.py` converts between them. `send_position_setpoint(20, 0, 3)`
means 20 m East of the local origin, 3 m up.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `Failed to connect to PX4` | Is SITL running? Is the agent running (or was `--no-agent` passed by mistake)? `ros2 topic list \| grep fmu` |
| Topics listed but a callback never fires | QoS mismatch (use `drone.px4.qos.PX4_QOS`) or px4_msgs/firmware mismatch (e.g. `vehicle_status` vs `vehicle_status_v1`) |
| OFFBOARD rejected | Setpoints not flowing first, or no valid local position yet (wait for EKF) |
| Arms but won't move | `ros2 topic hz /fmu/in/offboard_control_mode` should be ~10 Hz |
| Takeoff times out | Local position invalid; `ros2 run drone check_telemetry --sitl` |
